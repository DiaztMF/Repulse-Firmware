#pragma once
#include <stdint.h>

/*
 * Deteksi pola dengkuran — BLE_GATT_CONTRACT.md §4.2.
 *
 * Kontrak melarang keras cara yang paling gampang: "Deteksi mencari pola
 * berulang 0,2-0,5 Hz mengikuti irama napas di atas ambang volume, BUKAN
 * rata-rata desibel." Kipas dan AC punya rata-rata desibel tinggi tapi rata;
 * dengkuran punya puncak berulang tiap 2-5 detik.
 *
 * Murni, tanpa Arduino — lihat test/snore_test.cpp.
 */

struct SnoreConfig {
    /* Ambang volume, dalam dB yang sama dengan yang dilaporkan characteristic
     * 0001. WAJIB dikalibrasi di kamar sungguhan (§4.2) — kipas, AC, suara
     * jalan, dan orang lain di kamar semuanya menggeser lantai kebisingan. */
    uint8_t threshold_db = 45;
    /* Histeresis, supaya kebisingan yang menggantung persis di ambang tidak
     * menghasilkan puluhan tepi naik palsu per detik. */
    uint8_t hysteresis_db = 3;
    /* Berapa siklus berjarak benar sebelum dipercaya. Tiga siklus ≈ 6-15
     * detik napas berdengkur; satu batuk tidak akan lolos. */
    uint8_t min_cycles = 3;
};

/* 0,2-0,5 Hz = satu puncak tiap 2 sampai 5 detik. */
#define SNORE_PERIOD_MIN_MS  2000
#define SNORE_PERIOD_MAX_MS  5000
/* Sunyi selama ini menghapus hitungan — orangnya berhenti mendengkur. */
#define SNORE_IDLE_RESET_MS  9000
/* Rentang dB di atas ambang yang dipetakan ke intensitas 0-100. */
#define SNORE_INTENSITY_SPAN 30

class SnoreDetector {
public:
    SnoreConfig cfg;

    /* db = level amplop kebisingan sekarang. Dipanggil berkala, sekitar
     * 10 Hz; laju persisnya tidak penting karena jarak antar puncak diukur
     * dari cap waktu, bukan dari jumlah pemanggilan. */
    void feed(uint32_t now_ms, uint8_t db) {
        if (db > peak_db_) peak_db_ = db;

        if (above_) {
            if (db < (int)cfg.threshold_db - (int)cfg.hysteresis_db) above_ = false;
            return;
        }
        if (db < (int)cfg.threshold_db + (int)cfg.hysteresis_db) {
            // Masih di bawah ambang. Sunyi cukup lama = polanya putus.
            if (last_rise_ms_ != 0 && now_ms - last_rise_ms_ > SNORE_IDLE_RESET_MS) {
                reset();
            }
            return;
        }

        above_ = true;
        uint32_t period = now_ms - last_rise_ms_;
        if (last_rise_ms_ != 0 &&
            period >= SNORE_PERIOD_MIN_MS && period <= SNORE_PERIOD_MAX_MS) {
            if (cycles_ < 255) cycles_++;
        } else {
            // Jarak salah — mulai hitung dari nol, puncak ini jadi patokan baru.
            cycles_   = 0;
            peak_db_  = db;
        }
        last_rise_ms_ = now_ms;
    }

    bool flagged() const { return cycles_ >= cfg.min_cycles; }

    uint8_t intensity() const {
        if (!flagged() || peak_db_ <= cfg.threshold_db) return 0;
        uint16_t above = (uint16_t)(peak_db_ - cfg.threshold_db);
        if (above >= SNORE_INTENSITY_SPAN) return 100;
        return (uint8_t)(above * 100 / SNORE_INTENSITY_SPAN);
    }

    void reset() {
        cycles_       = 0;
        peak_db_      = 0;
        last_rise_ms_ = 0;
        above_        = false;
    }

private:
    uint8_t  cycles_       = 0;
    uint8_t  peak_db_      = 0;
    uint32_t last_rise_ms_ = 0;
    bool     above_        = false;
};
