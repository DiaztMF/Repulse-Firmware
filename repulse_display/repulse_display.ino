/*
 * RePulse Display - Waveshare ESP32-C6-Touch-LCD-1.69
 *
 * Receives the RePulse Band snapshot over UART and renders a single-page UI.
 *
 * Wiring:
 * - ESP32-C3 GPIO7 (TX) -> ESP32-C6 GPIO17 (RX)
 * - ESP32-C3 GPIO6 (RX) <- ESP32-C6 GPIO16 (TX), optional
 * - ESP32-C3 GND         -> ESP32-C6 GND
 *
 * Do not use GPIO7 or GPIO6 for UART on this C6 board:
 * - C6 GPIO7 is the shared I2C clock.
 * - C6 GPIO6 is the LCD backlight.
 *
 * Required library:
 * - GFX Library for Arduino
 *
 * Board:
 * - ESP32C6 Dev Module
 * - USB CDC On Boot: Enabled
 */

#include <Arduino.h>
#include <Arduino_GFX_Library.h>

// =============================================================
// WAVESHARE ESP32-C6-TOUCH-LCD-1.69
// =============================================================

#define LCD_SCK             1
#define LCD_DIN             2
#define LCD_DC              3
#define LCD_RST             4
#define LCD_CS              5
#define LCD_BL              6

#define LCD_WIDTH         240
#define LCD_HEIGHT        280

// UART pads exposed by the C6 board.
#define PIN_BAND_RX         17
#define PIN_BAND_TX         16

#define USB_BAUD        115200
#define BAND_BAUD       57600

#define BAND_TIMEOUT_MS    3000
#define LINE_BUFFER_SIZE    160

static Arduino_DataBus *displayBus = new Arduino_HWSPI(
    LCD_DC,
    LCD_CS,
    LCD_SCK,
    LCD_DIN
);

static Arduino_GFX *display = new Arduino_ST7789(
    displayBus,
    LCD_RST,
    0,      // portrait rotation
    true,   // IPS
    LCD_WIDTH,
    LCD_HEIGHT,
    0,
    20,     // ST7789V2 row offset
    0,
    20
);

static HardwareSerial BandUART(1);

// =============================================================
// COLORS
// =============================================================

static constexpr uint16_t rgb565(
    uint8_t red,
    uint8_t green,
    uint8_t blue
) {
    return
        ((red & 0xF8) << 8) |
        ((green & 0xFC) << 3) |
        (blue >> 3);
}

static constexpr uint16_t COLOR_BG =
    rgb565(5, 14, 13);

static constexpr uint16_t COLOR_CARD =
    rgb565(14, 31, 28);

static constexpr uint16_t COLOR_CARD_ACTIVE =
    rgb565(17, 42, 37);

static constexpr uint16_t COLOR_BORDER =
    rgb565(34, 69, 61);

static constexpr uint16_t COLOR_MINT =
    rgb565(85, 226, 191);

static constexpr uint16_t COLOR_TEAL =
    rgb565(42, 137, 117);

static constexpr uint16_t COLOR_WHITE =
    rgb565(237, 246, 243);

static constexpr uint16_t COLOR_MUTED =
    rgb565(120, 151, 142);

static constexpr uint16_t COLOR_RED =
    rgb565(255, 82, 92);

static constexpr uint16_t COLOR_AMBER =
    rgb565(255, 188, 76);

static constexpr uint16_t COLOR_BLUE =
    rgb565(91, 174, 255);

static constexpr uint16_t COLOR_OFF =
    rgb565(56, 73, 69);

// =============================================================
// BAND DATA
// =============================================================

struct BandData {
    uint16_t bpm = 0;
    uint16_t rrMs = 0;
    uint8_t spo2 = 0;

    uint16_t motionMg = 0;
    uint8_t position = 255;

    bool worn = false;
    uint8_t quality = 0;

    uint8_t stage = 0;
    uint8_t reason = 0;

    uint8_t battery = 255;
    bool bleConnected = false;

    bool bpmValid = false;
    bool rrValid = false;
    bool spo2Valid = false;
};

static bool parseBandPacket(
    const char *line,
    BandData &result
);

static BandData band;

static char lineBuffer[LINE_BUFFER_SIZE];
static size_t lineLength = 0;

static uint32_t lastPacketMs = 0;
static uint32_t packetCount = 0;

static bool uartOnline = false;
static bool timeoutReported = false;

// =============================================================
// TEXT HELPERS
// =============================================================

static void drawText(
    int16_t x,
    int16_t y,
    const char *text,
    uint16_t color,
    uint8_t size = 1
) {
    display->setTextSize(size);
    display->setTextColor(color);
    display->setCursor(x, y);
    display->print(text);
}

static const char *positionName(
    uint8_t position
) {
    switch (position) {
        case 0:
            return "TELENTANG";

        case 1:
            return "MIRING KIRI";

        case 2:
            return "MIRING KANAN";

        case 3:
            return "TENGKURAP";

        default:
            return "POSISI --";
    }
}

static const char *stageName(
    uint8_t stage
) {
    switch (stage) {
        case 0:
            return "NORMAL";

        case 1:
            return "KONFIRMASI";

        case 2:
            return "GETAR HALUS";

        case 3:
            return "GETAR KERAS";

        case 4:
            return "SOS";

        default:
            return "STATUS --";
    }
}

static const char *reasonName(
    uint8_t reason
) {
    switch (reason) {
        case 1:
            return "IRAMA";

        case 2:
            return "AMBANG HR";

        case 3:
            return "MANUAL";

        default:
            return "";
    }
}

static uint16_t stageColor(
    uint8_t stage
) {
    switch (stage) {
        case 1:
            return COLOR_AMBER;

        case 2:
            return COLOR_AMBER;

        case 3:
        case 4:
            return COLOR_RED;

        default:
            return COLOR_MINT;
    }
}

// =============================================================
// UI PRIMITIVES
// =============================================================

static void drawHeart(
    int16_t x,
    int16_t y,
    uint16_t color
) {
    display->fillCircle(x - 4, y - 3, 5, color);
    display->fillCircle(x + 4, y - 3, 5, color);
    display->fillTriangle(
        x - 9,
        y - 1,
        x + 9,
        y - 1,
        x,
        y + 10,
        color
    );
}

static void drawValidityDot(
    int16_t x,
    int16_t y,
    bool valid
) {
    display->fillCircle(
        x,
        y,
        3,
        valid ? COLOR_MINT : COLOR_OFF
    );
}

static void drawBattery(
    int16_t x,
    int16_t y,
    uint8_t percent
) {
    display->drawRoundRect(
        x,
        y,
        27,
        13,
        3,
        COLOR_MUTED
    );

    display->fillRect(
        x + 27,
        y + 4,
        2,
        5,
        COLOR_MUTED
    );

    display->fillRoundRect(
        x + 2,
        y + 2,
        23,
        9,
        2,
        COLOR_BG
    );

    if (percent <= 100) {
        uint16_t color =
            percent <= 20
            ? COLOR_RED
            : COLOR_MINT;

        int16_t width =
            (int16_t)(21UL * percent / 100UL);

        if (width > 0) {
            display->fillRoundRect(
                x + 3,
                y + 3,
                width,
                7,
                1,
                color
            );
        }
    }
}

// =============================================================
// STATIC UI
// =============================================================

static void drawStaticUi() {
    display->fillScreen(
        COLOR_BG
    );

    drawHeart(
        20,
        17,
        COLOR_RED
    );

    drawText(
        35,
        8,
        "RePulse",
        COLOR_WHITE,
        2
    );

    drawText(
        35,
        25,
        "LIVE BAND",
        COLOR_MUTED,
        1
    );

    display->drawFastHLine(
        8,
        37,
        224,
        COLOR_BORDER
    );

    display->fillRoundRect(
        8,
        44,
        224,
        88,
        14,
        COLOR_CARD
    );

    display->drawRoundRect(
        8,
        44,
        224,
        88,
        14,
        COLOR_BORDER
    );

    drawText(
        18,
        52,
        "HEART RATE",
        COLOR_MUTED,
        1
    );

    drawText(
        164,
        52,
        "RR",
        COLOR_MUTED,
        1
    );

    display->fillRoundRect(
        8,
        140,
        108,
        58,
        12,
        COLOR_CARD
    );

    display->drawRoundRect(
        8,
        140,
        108,
        58,
        12,
        COLOR_BORDER
    );

    display->fillRoundRect(
        124,
        140,
        108,
        58,
        12,
        COLOR_CARD
    );

    display->drawRoundRect(
        124,
        140,
        108,
        58,
        12,
        COLOR_BORDER
    );

    drawText(
        18,
        148,
        "SpO2",
        COLOR_MUTED,
        1
    );

    drawText(
        134,
        148,
        "MOTION",
        COLOR_MUTED,
        1
    );

    display->fillRoundRect(
        8,
        206,
        224,
        34,
        10,
        COLOR_CARD
    );

    display->drawRoundRect(
        8,
        206,
        224,
        34,
        10,
        COLOR_BORDER
    );

    display->fillRoundRect(
        8,
        248,
        224,
        24,
        8,
        COLOR_CARD_ACTIVE
    );
}

// =============================================================
// DYNAMIC UI
// =============================================================

static void drawHeaderStatus() {
    display->fillRect(
        145,
        5,
        87,
        29,
        COLOR_BG
    );

    display->fillCircle(
        153,
        14,
        4,
        uartOnline ? COLOR_MINT : COLOR_RED
    );

    drawText(
        162,
        10,
        uartOnline ? "UART" : "OFF",
        uartOnline ? COLOR_MINT : COLOR_RED,
        1
    );

    drawBattery(
        201,
        8,
        band.battery
    );

    char batteryText[8];

    if (band.battery <= 100) {
        snprintf(
            batteryText,
            sizeof(batteryText),
            "%u%%",
            band.battery
        );
    } else {
        snprintf(
            batteryText,
            sizeof(batteryText),
            "--"
        );
    }

    drawText(
        203,
        24,
        batteryText,
        COLOR_MUTED,
        1
    );
}

static void drawHeartCard() {
    display->fillRect(
        16,
        64,
        208,
        59,
        COLOR_CARD
    );

    drawValidityDot(
        19,
        56,
        band.bpmValid
    );

    drawValidityDot(
        155,
        56,
        band.rrValid
    );

    char text[16];

    if (band.bpmValid) {
        snprintf(
            text,
            sizeof(text),
            "%u",
            band.bpm
        );
    } else {
        snprintf(
            text,
            sizeof(text),
            "--"
        );
    }

    drawText(
        18,
        70,
        text,
        band.bpmValid ? COLOR_WHITE : COLOR_MUTED,
        5
    );

    drawText(
        105,
        96,
        "BPM",
        COLOR_MUTED,
        1
    );

    if (band.rrValid) {
        snprintf(
            text,
            sizeof(text),
            "%u",
            band.rrMs
        );
    } else {
        snprintf(
            text,
            sizeof(text),
            "--"
        );
    }

    drawText(
        155,
        72,
        text,
        band.rrValid ? COLOR_BLUE : COLOR_MUTED,
        2
    );

    drawText(
        170,
        94,
        "ms",
        COLOR_MUTED,
        1
    );

    char qualityText[18];

    snprintf(
        qualityText,
        sizeof(qualityText),
        "SIGNAL %u/15",
        band.quality
    );

    drawText(
        18,
        116,
        qualityText,
        band.worn ? COLOR_TEAL : COLOR_MUTED,
        1
    );
}

static void drawMetricCards() {
    display->fillRect(
        16,
        162,
        92,
        29,
        COLOR_CARD
    );

    display->fillRect(
        132,
        162,
        92,
        29,
        COLOR_CARD
    );

    drawValidityDot(
        104,
        151,
        band.spo2Valid
    );

    char text[20];

    if (band.spo2Valid) {
        snprintf(
            text,
            sizeof(text),
            "%u%%",
            band.spo2
        );
    } else {
        snprintf(
            text,
            sizeof(text),
            "--%%"
        );
    }

    drawText(
        18,
        166,
        text,
        band.spo2Valid ? COLOR_BLUE : COLOR_MUTED,
        3
    );

    snprintf(
        text,
        sizeof(text),
        "%u",
        band.motionMg
    );

    drawText(
        134,
        166,
        text,
        COLOR_WHITE,
        2
    );

    drawText(
        190,
        174,
        "mg",
        COLOR_MUTED,
        1
    );
}

static void drawPositionCard() {
    display->fillRect(
        15,
        212,
        210,
        21,
        COLOR_CARD
    );

    drawText(
        16,
        216,
        positionName(band.position),
        COLOR_WHITE,
        1
    );

    char statusText[28];

    snprintf(
        statusText,
        sizeof(statusText),
        "%s  BLE %s",
        band.worn ? "DIPAKAI" : "DILEPAS",
        band.bleConnected ? "ON" : "OFF"
    );

    drawText(
        111,
        216,
        statusText,
        band.worn ? COLOR_MINT : COLOR_MUTED,
        1
    );
}

static void drawStageCard() {
    uint16_t color =
        uartOnline
        ? stageColor(band.stage)
        : COLOR_RED;

    display->fillRoundRect(
        8,
        248,
        224,
        24,
        8,
        uartOnline
            ? COLOR_CARD_ACTIVE
            : COLOR_CARD
    );

    display->fillCircle(
        20,
        260,
        4,
        color
    );

    const char *label =
        uartOnline
        ? stageName(band.stage)
        : "MENUNGGU BAND";

    drawText(
        31,
        256,
        label,
        color,
        1
    );

    if (
        uartOnline &&
        band.reason != 0
    ) {
        drawText(
            151,
            256,
            reasonName(band.reason),
            color,
            1
        );
    }
}

static void drawDynamicUi() {
    drawHeaderStatus();
    drawHeartCard();
    drawMetricCards();
    drawPositionCard();
    drawStageCard();
}

// =============================================================
// UART PARSER
// =============================================================

static bool parseBandPacket(
    const char *line,
    BandData &result
) {
    unsigned int value[14];

    int parsed = sscanf(
        line,
        "RP,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u",
        &value[0],
        &value[1],
        &value[2],
        &value[3],
        &value[4],
        &value[5],
        &value[6],
        &value[7],
        &value[8],
        &value[9],
        &value[10],
        &value[11],
        &value[12],
        &value[13]
    );

    if (parsed != 14) {
        return false;
    }

    bool batteryValid =
        value[9] <= 100 ||
        value[9] == 255;

    if (
        value[0] > 255 ||
        value[1] > 65535 ||
        value[2] > 100 ||
        value[3] > 65535 ||
        value[4] > 255 ||
        value[5] > 1 ||
        value[6] > 15 ||
        value[7] > 4 ||
        value[8] > 3 ||
        !batteryValid ||
        value[10] > 1 ||
        value[11] > 1 ||
        value[12] > 1 ||
        value[13] > 1
    ) {
        return false;
    }

    result.bpm =
        (uint16_t)value[0];

    result.rrMs =
        (uint16_t)value[1];

    result.spo2 =
        (uint8_t)value[2];

    result.motionMg =
        (uint16_t)value[3];

    result.position =
        (uint8_t)value[4];

    result.worn =
        value[5] == 1;

    result.quality =
        (uint8_t)value[6];

    result.stage =
        (uint8_t)value[7];

    result.reason =
        (uint8_t)value[8];

    result.battery =
        (uint8_t)value[9];

    result.bleConnected =
        value[10] == 1;

    result.bpmValid =
        value[11] == 1;

    result.rrValid =
        value[12] == 1;

    result.spo2Valid =
        value[13] == 1;

    return true;
}

static void handleBandLine(
    const char *line
) {
    BandData received;

    if (!parseBandPacket(
            line,
            received
        )) {
        Serial.printf(
            "[UART] Paket tidak valid: %s\n",
            line
        );

        return;
    }

    band = received;
    packetCount++;

    lastPacketMs = millis();
    uartOnline = true;
    timeoutReported = false;

    drawDynamicUi();

    Serial.printf(
        "[RX #%lu] BPM=%u/%u RR=%u/%u SpO2=%u/%u motion=%u pos=%u stage=%u reason=%u\n",
        (unsigned long)packetCount,
        band.bpm,
        band.bpmValid ? 1U : 0U,
        band.rrMs,
        band.rrValid ? 1U : 0U,
        band.spo2,
        band.spo2Valid ? 1U : 0U,
        band.motionMg,
        band.position,
        band.stage,
        band.reason
    );
}

static void uartLoop() {
    while (BandUART.available() > 0) {
        char value =
            (char)BandUART.read();

        if (value == '\n') {
            if (lineLength > 0) {
                lineBuffer[lineLength] = '\0';

                handleBandLine(
                    lineBuffer
                );
            }

            lineLength = 0;
            continue;
        }

        if (value == '\r') {
            continue;
        }

        if (
            lineLength <
            LINE_BUFFER_SIZE - 1
        ) {
            lineBuffer[lineLength++] = value;
        } else {
            lineLength = 0;

            Serial.println(
                "[UART] Buffer baris penuh"
            );
        }
    }
}

static void timeoutLoop() {
    if (
        !uartOnline ||
        millis() - lastPacketMs <=
            BAND_TIMEOUT_MS
    ) {
        return;
    }

    uartOnline = false;

    band.bpmValid = false;
    band.rrValid = false;
    band.spo2Valid = false;

    drawDynamicUi();

    if (!timeoutReported) {
        timeoutReported = true;

        Serial.println(
            "[UART] Timeout: tidak ada paket RP dari C3"
        );
    }
}

// =============================================================
// SETUP / LOOP
// =============================================================

void setup() {
    Serial.begin(
        USB_BAUD
    );

    for (
        uint32_t startedAt = millis();
        !Serial &&
        millis() - startedAt < 2000;
    ) {
        delay(10);
    }

    Serial.println(
        "\n=== RePulse Display ESP32-C6 ==="
    );

    BandUART.begin(
        BAND_BAUD,
        SERIAL_8N1,
        PIN_BAND_RX,
        PIN_BAND_TX
    );

    if (!display->begin()) {
        Serial.println(
            "[LCD] Inisialisasi gagal"
        );
    }

    pinMode(
        LCD_BL,
        OUTPUT
    );

    digitalWrite(
        LCD_BL,
        HIGH
    );

    drawStaticUi();
    drawDynamicUi();

    Serial.printf(
        "[UART] RX=GPIO%d TX=GPIO%d baud=%lu\n",
        PIN_BAND_RX,
        PIN_BAND_TX,
        (unsigned long)BAND_BAUD
    );

    Serial.println(
        "[BOOT] UI RePulse siap, menunggu paket RP"
    );
}

void loop() {
    uartLoop();
    timeoutLoop();

    delay(1);
}
