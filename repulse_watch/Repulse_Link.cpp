#include "Repulse_Link.h"
#include "Repulse_UI.h"

#include <Arduino.h>

/* ⚠ COCOKKAN DENGAN SKEMA SEBELUM FLASH.
 *
 * GPIO yang sudah terpakai di papan ini: 4 5 6 7 8 10 11 14 16 17 18 21 40
 * 41 42 45 46. GPIO 43/44 adalah UART0 bawaan ESP32-S3 dan bebas selama log
 * Serial berjalan lewat USB-CDC — itu yang dipakai di sini. */
#define PIN_BAND_RX  44
#define PIN_BAND_TX  43
#define BAND_BAUD    115200

static HardwareSerial Band(1);
static char    line[96];
static uint8_t line_length = 0;

/* Format satu baris dari gelang (repulse_band.ino, s3Loop):
 *   RP,<bpm>,<spo2>,<milli_g>,<worn>,<quality>,<stage>,<reason>,<batt>,<link>
 * ASCII supaya bisa dibaca langsung di serial monitor saat menelusuri
 * masalah — sepuluh angka sedetik tidak butuh protokol biner. */
static void handle_line(const char *text)
{
    unsigned bpm, spo2, milli_g, worn, quality, stage, reason, battery, linked;
    if(sscanf(text, "RP,%u,%u,%u,%u,%u,%u,%u,%u,%u",
              &bpm, &spo2, &milli_g, &worn, &quality,
              &stage, &reason, &battery, &linked) != 9) {
        return;
    }

    /* milli_g, quality, reason, dan battery ikut di baris karena berguna saat
     * menelusuri gelang lewat serial monitor layar — tapi tidak ada tempatnya
     * di UI, jadi tidak diteruskan. Baterai yang tampil adalah baterai jam. */
    (void)milli_g; (void)quality; (void)reason; (void)battery;

    Repulse_UI_Feed((uint8_t)bpm, (uint8_t)spo2, worn != 0,
                    (uint8_t)stage, linked != 0);
}

void Repulse_Link_Init(void)
{
    Band.begin(BAND_BAUD, SERIAL_8N1, PIN_BAND_RX, PIN_BAND_TX);
    Serial.printf("[LINK] UART gelang di RX=%d TX=%d\r\n", PIN_BAND_RX, PIN_BAND_TX);
}

void Repulse_Link_Loop(void)
{
    while(Band.available() > 0) {
        char input = Band.read();
        if(input == '\r') {
            continue;
        }
        if(input == '\n') {
            line[line_length] = '\0';
            if(line_length > 0) {
                handle_line(line);
            }
            line_length = 0;
            continue;
        }
        if(line_length < sizeof(line) - 1) {
            line[line_length++] = input;
        } else {
            line_length = 0;   // baris rusak, buang seluruhnya
        }
    }
}
