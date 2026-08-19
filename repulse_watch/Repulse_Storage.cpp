#include "Repulse_Storage.h"

#include "TCA9554PWR.h"

#include <Arduino.h>
#include <FS.h>
#include <SD_MMC.h>

static const int SD_CLK_PIN = 14;
static const int SD_CMD_PIN = 17;
static const int SD_D0_PIN = 16;
static const char *LOG_DIRECTORY = "/repulse";
static const char *LOG_PATH = "/repulse/health_log.csv";

static bool storage_ready = false;
static uint32_t card_size_mb = 0;

bool Repulse_Storage_Init(void)
{
    storage_ready = false;
    card_size_mb = 0;

    Set_EXIO(EXIO_PIN4, High);
    vTaskDelay(pdMS_TO_TICKS(10));

    if(!SD_MMC.setPins(SD_CLK_PIN, SD_CMD_PIN, SD_D0_PIN, -1, -1, -1)) {
        Serial.println("[SD] Onboard pin configuration failed");
        return false;
    }

    // One-bit mode matches the onboard wiring. Never format a user's card automatically.
    if(!SD_MMC.begin("/sdcard", true, false)) {
        Serial.println("[SD] No onboard microSD card mounted");
        return false;
    }

    if(SD_MMC.cardType() == CARD_NONE) {
        Serial.println("[SD] No onboard microSD card inserted");
        SD_MMC.end();
        return false;
    }

    if(!SD_MMC.exists(LOG_DIRECTORY) && !SD_MMC.mkdir(LOG_DIRECTORY)) {
        Serial.println("[SD] Cannot create /repulse directory");
        SD_MMC.end();
        return false;
    }

    if(!SD_MMC.exists(LOG_PATH)) {
        File log_file = SD_MMC.open(LOG_PATH, FILE_WRITE);
        if(!log_file) {
            Serial.println("[SD] Cannot create health_log.csv");
            SD_MMC.end();
            return false;
        }
        log_file.println(
            "timestamp,bpm,spo2,rr_ms,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,motion,position,signal,alert_stage");
        log_file.close();
    }

    card_size_mb = static_cast<uint32_t>(SD_MMC.cardSize() / (1024ULL * 1024ULL));
    storage_ready = true;
    Serial.printf("[SD] Onboard microSD ready (%lu MB)\r\n",
                  static_cast<unsigned long>(card_size_mb));
    return true;
}

bool Repulse_Storage_Ready(void)
{
    return storage_ready;
}

uint32_t Repulse_Storage_Card_Size_MB(void)
{
    return card_size_mb;
}

bool Repulse_Storage_Append(const char *csv_line)
{
    if(!storage_ready || csv_line == nullptr || csv_line[0] == '\0') {
        return false;
    }

    File log_file = SD_MMC.open(LOG_PATH, FILE_APPEND);
    if(!log_file) {
        storage_ready = false;
        Serial.println("[SD] Cannot append health log");
        return false;
    }

    bool written = log_file.println(csv_line) > 0;
    log_file.close();
    if(!written) {
        storage_ready = false;
        Serial.println("[SD] Health log write failed");
    }
    return written;
}
