/*
 * ═══════════════════════════════════════════════════════════════
 *  RePulse Bedside — ESP32-C3
 * ═══════════════════════════════════════════════════════════════
 *
 *  Peran: bedside adalah "RePulse Bedside" di BLE_GATT_CONTRACT.md §4.
 *  Ia mengukur kamar, menjalankan aktuator yang diperintahkan aplikasi, dan
 *  melapor selesai. Ia tidak pernah memutuskan apa pun — dengan satu
 *  pengecualian di §2.1: kalau gelang menyiarkan tahap >= 3 dan tidak ada HP
 *  di mana pun, bedside membunyikan sirene atas kemauannya sendiri.
 *
 *  Sensor  : BH1750 (lux), DHT11 (suhu/RH), INMP441 (mikrofon I2S)
 *  Aktuator: WS2812 (lampu), DFPlayer Mini (white noise + sirene),
 *            diffuser aroma BELUM punya pin kendali (lihat PIN_AROMA)
 *
 *  Library (Arduino Library Manager):
 *    1. NimBLE-Arduino  >= 2.0   — h2zero
 *    2. ArduinoJson     >= 7.0   — Benoit Blanchon
 *    3. BH1750                   — Christopher Laws
 *    4. DHT sensor library       — Adafruit
 *    5. Adafruit NeoPixel        — Adafruit
 *    6. DFRobotDFPlayerMini      — DFRobot
 *
 *  Board: ESP32C3 Dev Module — USB CDC On Boot WAJIB Enabled
 * ═══════════════════════════════════════════════════════════════
 */

#include <Arduino.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <BH1750.h>
#include <DHT.h>
#include <Adafruit_NeoPixel.h>
#include <DFRobotDFPlayerMini.h>
#include <ESP_I2S.h>

#include "ble_bedside.h"
#include "snore.h"
#include "aroma.h"
#include "light.h"

// ═══════════════════════════════════════════════════════════════
//  PIN — sesuai tabel wiring bedside
// ═══════════════════════════════════════════════════════════════

#define PIN_MIC_BCLK       4    // INMP441 SCK
#define PIN_MIC_WS         5    // INMP441 WS / LRCK
#define PIN_MIC_DIN        6    // INMP441 SD / DOUT

#define PIN_I2C_SCL        7    // BH1750
#define PIN_I2C_SDA        9    // BH1750 — strapping pin, lihat catatan bawah

#define PIN_LED            8    // WS2812 DIN — strapping pin, lihat catatan
#define PIN_DHT           10    // DHT11 DATA, pull-up 10k ke 3V3

#define PIN_DFPLAYER_RX    2    // ke TX DFPlayer
#define PIN_DFPLAYER_TX    3    // ke RX DFPlayer, lewat resistor 1k

/* -1 = diffuser tidak punya pin kendali, sesuai keputusan 19 Agustus 2026.
 * Perintah `aroma` dibalas status = 1 (gagal), bukan status = 0 — firmware
 * tidak boleh melapor "selesai" untuk sesuatu yang tidak pernah ia jalankan.
 *
 * ⚠ Konsekuensi keselamatan yang perlu dicatat: diffuser yang dicolok
 * langsung ke 5V menyala sepanjang malam. Justru itu yang dilarang §4.3 —
 * ruangan jadi lembap dan berisiko mengiritasi saluran napas. Selama belum
 * ada MOSFET/relay, cabut diffusernya saat tidur.
 *
 * Untuk mengaktifkan nanti: isi pin ini (GPIO21 kalau pad 20/21 keluar, atau
 * GPIO1 menggantikan modul LM393 yang tidak dipakai firmware ini), lalu set
 * AROMA_ACTIVE_LEVEL — modul relay biasanya aktif LOW, MOSFET aktif HIGH. */
#define PIN_AROMA         -1
#define AROMA_ACTIVE_LEVEL HIGH

/* ⚠ CATATAN STRAPPING PIN ESP32-C3: GPIO2, GPIO8, dan GPIO9 ikut menentukan
 * mode boot. GPIO9 aman karena pull-up I2C BH1750 menahannya HIGH. GPIO8
 * (WS2812 DIN) dan GPIO2 (RX dari DFPlayer) mengambang saat boot — pasang
 * resistor pull-up 10k ke 3V3 di keduanya, supaya papan tidak gagal boot
 * tergantung modul mana yang menyala lebih dulu. */

// ═══════════════════════════════════════════════════════════════
//  KNOB KALIBRASI
// ═══════════════════════════════════════════════════════════════

#define LED_COUNT             60
/* 60 LED putih penuh menarik ~3,6 A — adaptor 5V 2A akan drop dan papan
 * ikut reboot. Batas 100 menahannya di sekitar 1,4 A. Naikkan hanya setelah
 * mengukur arus sungguhan, atau setelah ganti adaptor 5V 5A. */
#define LED_MAX_BRIGHTNESS   100

#define MIC_SAMPLE_RATE    16000
#define MIC_BLOCK_SAMPLES    256
/* Offset kalibrasi dB. Ukur dengan aplikasi sound-meter di HP di kamar
 * sungguhan lalu geser angka ini sampai cocok. Tanpa ini, ambang dengkuran
 * di snore.h tidak berarti apa-apa. */
#define MIC_DB_OFFSET         26.0f

#define ROOM_INTERVAL_MS   60000    // §4.1 lux tiap 1 menit
#define DHT_INTERVAL_MS   300000    // §4.1 suhu/RH tiap 5 menit
#define SNORE_FEED_MS        100    // amplop ~10 Hz

/* Trek di microSD DFPlayer. 0001-0003 white noise, 0004 sirene. */
#define TRACK_WHITE_NOISE_BASE  1
#define TRACK_SIREN             4
#define VOLUME_SIREN           30    // maksimum, §PRD 6.3

// ═══════════════════════════════════════════════════════════════
//  Keadaan
// ═══════════════════════════════════════════════════════════════

static BH1750             lightMeter;
/* DHT11: resolusi cuma derajat bulat, jadi nilai °C×10 di characteristic
 * 0001 selalu berakhiran 0. Kontrak menerimanya, tapi tren suhu semalam
 * jadi bertangga. Ganti ke DHT22 di sini kalau sensornya diupgrade. */
static DHT                dht(PIN_DHT, DHT11);
static Adafruit_NeoPixel  strip(LED_COUNT, PIN_LED, NEO_GRB + NEO_KHZ800);
static DFRobotDFPlayerMini dfplayer;
static HardwareSerial     DfSerial(1);
static I2SClass           i2s;

static SnoreDetector snore;
static AromaLimiter  aromaLimit;

static bool  bh1750Ready = false;
static bool  dfReady     = false;
static bool  micReady    = false;

// Kamar
static float    tempC     = 0.0f;
static float    humidity  = 0.0f;
static uint32_t lux_x100  = 0;
static uint8_t  roomDb    = 0;

// Lampu — §4.3 peredupan eksponensial
static char     lightMode[12] = "off";
static uint16_t lightKelvin   = 2700;
static uint8_t  lightFrom     = 0;
static uint8_t  lightTo       = 0;
static uint32_t lightStartMs  = 0;
static uint32_t lightRampMs   = 0;

// Aroma
static uint32_t aromaOffAtMs = 0;

// Sirene
static bool sirenOn      = false;
static bool sirenIsOurs  = false;   // dinyalakan bedside sendiri, §2.1

// Dengkuran
static bool lastSnoreFlag = false;

// Tempo
static uint32_t lastRoomMs = 0, lastDhtMs = 0, lastSnoreFeedMs = 0;
static uint32_t lastBandSeenMs = 0;

// ═══════════════════════════════════════════════════════════════
//  Lampu
// ═══════════════════════════════════════════════════════════════

static void lightApply(uint8_t brightness) {
    Rgb c = light_color(lightMode, lightKelvin, brightness);
    for (uint16_t i = 0; i < LED_COUNT; i++) strip.setPixelColor(i, c.r, c.g, c.b);
    strip.show();
}

static void lightSet(const char *mode, uint16_t kelvin, uint8_t brightness, uint32_t ramp_s) {
    strncpy(lightMode, mode, sizeof(lightMode) - 1);
    lightMode[sizeof(lightMode) - 1] = '\0';
    lightKelvin  = kelvin;
    lightFrom    = lightTo;
    lightTo      = brightness > LED_MAX_BRIGHTNESS ? LED_MAX_BRIGHTNESS : brightness;
    lightStartMs = millis();
    lightRampMs  = ramp_s * 1000u;
    if (lightRampMs == 0) lightApply(lightTo);
}

/* Menulis 60 LED memakan ~1,8 ms dan memblok. Peredupan 25 menit tidak butuh
 * pembaruan seribu kali sedetik — 25 Hz sudah jauh lebih halus daripada yang
 * bisa dilihat mata, dan menyisakan loop untuk mikrofon dan BLE. */
#define LIGHT_REFRESH_MS 40

static void lightLoop() {
    if (lightRampMs == 0) return;
    /* Darurat memiliki strip, tanpa berbagi. Peredupan sunset berjalan 25
     * menit, jadi anomali di tengahnya menemukan ramp yang masih hidup —
     * dan keduanya menulis strip yang sama. lightLoop menyala tiap 40 ms,
     * kedipan darurat tiap 400 ms, jadi yang menang adalah yang meredup:
     * kedipan putih §PRD 6.3 menyusut jadi kedipan 40 ms sekali sedetik,
     * nyaris tak terlihat, tepat pada saat ia paling harus terlihat. */
    if (sirenOn) return;
    static uint32_t lastPaintMs = 0;
    if (millis() - lastPaintMs < LIGHT_REFRESH_MS) return;
    lastPaintMs = millis();

    uint32_t elapsed = millis() - lightStartMs;
    float    progress = (float)elapsed / (float)lightRampMs;
    if (progress >= 1.0f) {
        progress    = 1.0f;
        lightRampMs = 0;
    }
    lightApply(light_ramp(lightFrom, lightTo, progress));
}

/* §PRD 6.3: saat darurat, lampu putih 100% berkedip. */
static void alertFlashLoop() {
    if (!sirenOn) return;
    bool on = (millis() / 400) % 2 == 0;

    // Sama seperti lightLoop: tulis hanya saat keadaannya benar-benar berubah,
    // bukan 250 kali per kedipan.
    static int8_t lastFlash = -1;
    if (on == (lastFlash == 1)) return;
    lastFlash = on ? 1 : 0;

    for (uint16_t i = 0; i < LED_COUNT; i++) {
        strip.setPixelColor(i, on ? LED_MAX_BRIGHTNESS : 0,
                               on ? LED_MAX_BRIGHTNESS : 0,
                               on ? LED_MAX_BRIGHTNESS : 0);
    }
    strip.show();
}

// ═══════════════════════════════════════════════════════════════
//  Sirene — tidak punya pin sendiri
// ═══════════════════════════════════════════════════════════════
/* Sirene diperankan DFPlayer pada volume maksimum ditambah lampu putih
 * berkedip. ponytail: tidak ada GPIO tersisa di tabel wiring dan speaker
 * 3W pada volume 30 sudah cukup keras untuk membangunkan orang. Kalau
 * belakangan dipasang sirene 12V terpisah, ia butuh satu pin lagi.
 *
 * Ini juga alasan `white_noise` dan `siren` tidak bisa berbunyi bersamaan —
 * dan itu justru benar: §PRD 6.3 memang mematikan white noise saat darurat. */
static void sirenSet(bool on, bool ours) {
    if (on == sirenOn) return;
    sirenOn     = on;
    sirenIsOurs = on ? ours : false;

    if (!dfReady) {
        Serial.println("[SIREN] DFPlayer tidak siap — sirene tidak berbunyi");
        return;
    }
    if (on) {
        dfplayer.volume(VOLUME_SIREN);
        dfplayer.loop(TRACK_SIREN);
        Serial.printf("[SIREN] MENYALA (%s)\n", ours ? "mandiri §2.1" : "perintah aplikasi");
    } else {
        dfplayer.stop();
        lightApply(lightTo);
        Serial.println("[SIREN] mati");
    }
}

// ═══════════════════════════════════════════════════════════════
//  Aroma
// ═══════════════════════════════════════════════════════════════

static void aromaOff() {
    if (PIN_AROMA >= 0) {
        digitalWrite(PIN_AROMA, AROMA_ACTIVE_LEVEL == HIGH ? LOW : HIGH);
    }
    aromaOffAtMs = 0;
}

static void aromaLoop() {
    if (aromaOffAtMs != 0 && millis() >= aromaOffAtMs) {
        aromaOff();
        Serial.println("[AROMA] selesai");
    }
}

// ═══════════════════════════════════════════════════════════════
//  §4.3 Perintah aktuator
// ═══════════════════════════════════════════════════════════════

static void onActuator(const char *json) {
    Serial.printf("[ACT] %s\n", json);

    JsonDocument doc;
    if (deserializeJson(doc, json)) {
        Serial.println("[ACT] JSON tidak valid");
        return;
    }
    uint8_t command_id = doc["command_id"] | 0;
    uint8_t status     = 0;                       // §4.4: 0 selesai

    if (doc["light"].is<JsonObject>()) {
        JsonObject l = doc["light"];
        lightSet(l["mode"] | "off", l["kelvin"] | 2700,
                 l["brightness"] | 0, l["ramp_s"] | 0);
    }

    if (doc["white_noise"].is<JsonObject>()) {
        JsonObject w = doc["white_noise"];
        if (!dfReady) {
            status = 1;                           // §4.4: 1 gagal
        } else if (w["on"] | false) {
            dfplayer.volume((uint8_t)((w["volume"] | 2) * 10));   // 0-3 → 0-30
            dfplayer.loop(TRACK_WHITE_NOISE_BASE + (uint8_t)(w["track"] | 1) - 1);
        } else {
            dfplayer.stop();
        }
    }

    if (doc["aroma"].is<JsonObject>()) {
        JsonObject a = doc["aroma"];
        uint16_t seconds = a["duration_s"] | 0;

        if (!(a["on"] | false)) {
            aromaOff();
        } else if (seconds == 0 || seconds > AROMA_MAX_SECONDS) {
            /* §4.3: permintaan yang melampaui batas dibalas status = 2, BUKAN
             * diam-diam dijalankan lebih pendek. Uji §6 no. 4 memang dirancang
             * untuk menangkap firmware yang memotong diam-diam.
             *
             * Batas ini diperiksa lebih dulu daripada keberadaan pin: menolak
             * 60 detik adalah keputusan kebijakan, dan jawabannya sama saja
             * apakah diffusernya terpasang atau tidak. */
            status = 2;
            Serial.printf("[AROMA] DITOLAK — %u dtk melampaui batas %u dtk\n",
                          seconds, AROMA_MAX_SECONDS);
        } else if (PIN_AROMA < 0) {
            status = 1;                           // §4.4: 1 gagal
            Serial.println("[AROMA] tidak ada pin kendali — perintah gagal");
        } else {
            uint16_t granted = aromaLimit.request(millis(), seconds);
            if (granted == 0) {
                status = 2;
                Serial.println("[AROMA] DITOLAK — kuota malam ini habis");
            } else {
                digitalWrite(PIN_AROMA, AROMA_ACTIVE_LEVEL);
                aromaOffAtMs = millis() + (uint32_t)granted * 1000u;
                Serial.printf("[AROMA] %u dtk (kejadian ke-%u malam ini)\n",
                              granted, aromaLimit.eventsTonight());
            }
        }
    }

    if (doc["siren"].is<JsonObject>()) {
        sirenSet(doc["siren"]["on"] | false, false);
    }

    BLE_IndicateAck(command_id, status);
}

// ═══════════════════════════════════════════════════════════════
//  §2.1 Siaran gelang → sirene mandiri
// ═══════════════════════════════════════════════════════════════

/* Keempat syarat harus benar semua. Syarat 3 dan 4 terlihat mubazir tapi
 * menjaga kegagalan yang berbeda — yang satu "bedside kehilangan HP", yang
 * lain "gelang kehilangan HP". Bila salah satu masih punya HP, HP yang
 * berkuasa. Ini yang mencegah sirene diperintah dua tuan. */
static void onBandSeen(uint8_t stage, bool phone_connected, bool paired) {
    lastBandSeenMs = millis();

    bool should_sound = stage >= 3            // 1
                     && paired                // 2
                     && !BLE_Connected()      // 3
                     && !phone_connected;     // 4

    if (should_sound && !sirenOn) {
        sirenSet(true, true);
    } else if (!should_sound && sirenOn && sirenIsOurs) {
        // Hanya sirene yang kita nyalakan sendiri yang boleh kita matikan
        // sendiri. Sirene atas perintah aplikasi tetap milik aplikasi.
        sirenSet(false, false);
    }
}

// ═══════════════════════════════════════════════════════════════
//  Mikrofon — §4.2 dengkuran, dan dB untuk §4.1
// ═══════════════════════════════════════════════════════════════

/* DMA I2S harus dikuras setiap loop, bukan sekali per 100 ms. Membaca satu
 * blok tiap 100 ms berarti 84% audio menumpuk di buffer sampai meluap, dan
 * yang terbaca justru suara 100 ms yang lalu — amplop dengkuran jadi mundur
 * dan tidak beraturan. Jadi: kuras terus, dan yang dilaporkan ke detektor
 * adalah PUNCAK selama jendela 100 ms, karena §4.2 mencari puncak berulang,
 * bukan rata-rata. */
static void micLoop() {
    if (!micReady) return;

    static int32_t block[MIC_BLOCK_SAMPLES];
    static uint8_t windowPeakDb = 0;

    while (i2s.available() >= (int)sizeof(block)) {
        if (i2s.readBytes((char *)block, sizeof(block)) < sizeof(block)) break;

        // INMP441 mengirim 24 bit yang rata kiri di dalam slot 32 bit.
        double sum_sq = 0;
        for (size_t i = 0; i < MIC_BLOCK_SAMPLES; i++) {
            double s = (double)(block[i] >> 14);
            sum_sq += s * s;
        }
        double rms = sqrt(sum_sq / MIC_BLOCK_SAMPLES);
        double db  = rms > 1.0 ? 20.0 * log10(rms) + MIC_DB_OFFSET : 0.0;
        if (db < 0)   db = 0;
        if (db > 255) db = 255;
        if ((uint8_t)db > windowPeakDb) windowPeakDb = (uint8_t)db;
    }

    if (millis() - lastSnoreFeedMs < SNORE_FEED_MS) return;
    lastSnoreFeedMs = millis();

    roomDb       = windowPeakDb;
    windowPeakDb = 0;

    snore.feed(millis(), roomDb);
    if (snore.flagged() != lastSnoreFlag) {
        lastSnoreFlag = snore.flagged();
        BLE_NotifySnore(lastSnoreFlag, snore.intensity());
        Serial.printf("[SNORE] %s intensitas=%u\n",
                      lastSnoreFlag ? "terdeteksi" : "berhenti", snore.intensity());
    }
}

// ═══════════════════════════════════════════════════════════════
//  §4.1 Sensor kamar
// ═══════════════════════════════════════════════════════════════

static void roomLoop() {
    /* Sebelum ada satu pun bacaan sah, coba tiap 2 detik. DHT11 butuh sekitar
     * sedetik setelah diberi daya, dan menunggu 5 menit penuh berarti demo
     * dimulai dengan 0,0°C di layar. Setelah dapat, baru turun ke 5 menit. */
    bool haveReading = humidity > 0.0f;
    uint32_t dhtInterval = haveReading ? DHT_INTERVAL_MS : 2000;

    if (lastDhtMs == 0 || millis() - lastDhtMs >= dhtInterval) {
        lastDhtMs = millis();
        float t = dht.readTemperature();
        float h = dht.readHumidity();
        if (!isnan(t)) tempC    = t;
        if (!isnan(h)) humidity = h;
    }

    if (millis() - lastRoomMs < ROOM_INTERVAL_MS && lastRoomMs != 0) return;
    lastRoomMs = millis();

    if (bh1750Ready) {
        float lx = lightMeter.readLightLevel();
        /* Dibaca sepanjang malam meski lampu sudah mati — ini dasar penandaan
         * gelap optimal, dan ambangnya < 3 lux. Karena itu dikirim ×100. */
        if (lx >= 0) lux_x100 = (uint32_t)(lx * 100.0f);
    }

    BLE_NotifyRoom((int16_t)(tempC * 10.0f), (uint16_t)(humidity * 10.0f),
                   lux_x100, roomDb);
    Serial.printf("[ROOM] %.1f°C %.1f%% %.2f lux %u dB\n",
                  tempC, humidity, lux_x100 / 100.0f, roomDb);
}

// ═══════════════════════════════════════════════════════════════
//  setup / loop
// ═══════════════════════════════════════════════════════════════

void setup() {
    Serial.begin(115200);
    /* Sama seperti di gelang: port USB-CDC baru muncul di PC satu sampai
     * dua detik setelah reset, jadi delay(300) membuang seluruh banner boot
     * — termasuk baris TIDAK DITEMUKAN yang justru paling ingin kamu baca
     * saat menelusuri sensor yang belum terpasang. */
    for (uint32_t t0 = millis(); !Serial && millis() - t0 < 2000; ) delay(10);
    Serial.println(F("\n=== RePulse Bedside (ESP32-C3) ==="));

    if (PIN_AROMA >= 0) {
        pinMode(PIN_AROMA, OUTPUT);
        aromaOff();
    } else {
        Serial.println("[AROMA] pin kendali belum ada — perintah aroma akan gagal");
    }

    strip.begin();
    strip.setBrightness(255);          // kecerahan diatur per-warna, bukan global
    strip.clear();
    strip.show();

    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    bh1750Ready = lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE);
    Serial.printf("[BH1750] %s\n", bh1750Ready ? "OK" : "TIDAK DITEMUKAN");

    dht.begin();

    DfSerial.begin(9600, SERIAL_8N1, PIN_DFPLAYER_RX, PIN_DFPLAYER_TX);

    /* Dua percobaan, karena "tidak menjawab" dan "tidak ada" bukan hal yang
     * sama.
     *
     * begin(..., isACK, doReset) mengirim perintah reset lalu menunggu
     * balasannya. Klon DFPlayer yang beredar — MH2024K, GD3200B — sering
     * tidak pernah membalas reset meski memutar berkas dengan sempurna,
     * dan dengan isACK menyala kegagalan itu jadi mutlak: modul sehat
     * dilaporkan hilang, dan sirene tidak akan pernah berbunyi.
     *
     * Percobaan kedua mematikan keduanya. Harganya nyata — tanpa ACK kita
     * tidak lagi tahu apakah sebuah perintah benar-benar diterima — jadi
     * ia hanya dipakai setelah cara yang jujur gagal, dan log mengatakan
     * mana yang berhasil supaya tidak ada yang salah menduga. */
    dfReady = dfplayer.begin(DfSerial, /*isACK=*/true, /*doReset=*/true);
    if (!dfReady) {
        Serial.println(F("[DFPLAYER] tidak membalas reset — coba tanpa ACK"));
        delay(200);
        dfReady = dfplayer.begin(DfSerial, /*isACK=*/false, /*doReset=*/false);
        if (dfReady) Serial.println(F("[DFPLAYER] menjawab tanpa ACK — perintah tidak dikonfirmasi"));
    }
    Serial.printf("[DFPLAYER] %s\n", dfReady ? "OK" : "TIDAK DITEMUKAN");
    if (dfReady) dfplayer.volume(20);

    i2s.setPins(PIN_MIC_BCLK, PIN_MIC_WS, -1, PIN_MIC_DIN);
    micReady = i2s.begin(I2S_MODE_STD, MIC_SAMPLE_RATE,
                         I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO);
    Serial.printf("[INMP441] %s\n", micReady ? "OK" : "GAGAL");

    BedsideCallbacks cb = { onActuator, onBandSeen };
    BLE_Init(cb);

    Serial.println(F("[BOOT] Siap"));
}

void loop() {
    micLoop();
    roomLoop();
    lightLoop();
    aromaLoop();
    alertFlashLoop();
    BLE_ScanLoop();

    /* Gelang menghilang dari udara sama sekali. Sirene mandiri dimatikan —
     * tanpa siaran, syarat 1 dan 2 di §2.1 tidak bisa dibuktikan lagi. */
    if (sirenIsOurs && millis() - lastBandSeenMs > 15000) {
        sirenSet(false, false);
    }
}
