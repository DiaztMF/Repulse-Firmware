#pragma once
#include <stdint.h>

/*
 * Batas keselamatan diffuser — BLE_GATT_CONTRACT.md §4.3.
 *
 * "Batas keselamatan wajib ditegakkan firmware, bukan hanya aplikasi."
 * Diffuser ultrasonik yang menyala terus-menerus membuat ruangan lembap dan
 * berisiko mengiritasi saluran napas, terutama bagi penderita asma.
 *
 * Satu hal yang paling mudah salah di sini: permintaan yang melampaui batas
 * harus DITOLAK (status 2), bukan dipotong diam-diam jadi 30 detik. Firmware
 * yang diam-diam menjalankan 30 detik saat diminta 60 terlihat identik dengan
 * firmware yang benar dari sisi aplikasi — dan uji nomor 4 di §6 memang
 * dirancang untuk menangkap persis itu.
 */

#define AROMA_MAX_SECONDS       30
#define AROMA_MAX_EVENTS_NIGHT   4
/* Tanpa RTC, "malam baru" ditandai jeda panjang antar permintaan.
 * ponytail: delapan jam sunyi; ganti ke jam sungguhan kalau bedside kelak
 * dapat NTP atau sinkronisasi waktu dari aplikasi. */
#define AROMA_NIGHT_GAP_MS  (8UL * 60UL * 60UL * 1000UL)

class AromaLimiter {
public:
    uint16_t max_seconds = AROMA_MAX_SECONDS;
    uint8_t  max_events  = AROMA_MAX_EVENTS_NIGHT;

    /* Mengembalikan durasi yang diizinkan, atau 0 bila ditolak. Nol berarti
     * balas status = 2 di characteristic 0004, bukan status = 0. */
    uint16_t request(uint32_t now_ms, uint16_t seconds) {
        if (last_request_ms_ != 0 && now_ms - last_request_ms_ > AROMA_NIGHT_GAP_MS) {
            events_ = 0;
        }
        if (seconds == 0 || seconds > max_seconds) return 0;
        if (events_ >= max_events)                 return 0;

        last_request_ms_ = now_ms;
        events_++;
        return seconds;
    }

    uint8_t eventsTonight() const { return events_; }
    void    resetNight()          { events_ = 0; last_request_ms_ = 0; }

private:
    uint8_t  events_          = 0;
    uint32_t last_request_ms_ = 0;
};
