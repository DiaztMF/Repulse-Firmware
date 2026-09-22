#pragma once
#include <stdint.h>

/*
 * Tangga eskalasi — BLE_GATT_CONTRACT.md §3.5.
 *
 * Sengaja tidak menyentuh Arduino apa pun: tangga ini satu-satunya bagian
 * firmware yang salahnya tidak kelihatan sampai jam 3 pagi, jadi ia harus
 * bisa dijalankan di laptop. Lihat ladder_test.cpp.
 *
 * Kontrak §3.5: tangga berjalan penuh di firmware, tersambung maupun tidak.
 * Aplikasi adalah cermin, bukan pengendali.
 */

enum Reason : uint8_t {
    REASON_NONE      = 0,
    REASON_IRREGULAR = 1,  // irama RR tidak teratur
    REASON_THRESHOLD = 2,  // HR di luar ambang personal
    REASON_MANUAL    = 3,  // tombol SOS
};

struct LadderConfig {
    // Durasi tiap tahap, bukan waktu absolut. Default = tabel §3.5
    // (0-20 dtk tahap 1, 20-35 tahap 2, 35-65 tahap 3, tahap 4 pada 65).
    uint16_t stage1_s = 20;
    uint16_t stage2_s = 15;
    uint16_t stage3_s = 30;
    /* Berapa lama anomali diabaikan setelah seseorang menekan "saya
     * baik-baik saja".
     *
     * Tanpa ini tangga langsung naik lagi pada iterasi loop berikutnya —
     * dalam hitungan milidetik — karena keadaan yang memicunya (denyut di
     * luar ambang) masih sama persis. Orangnya menekan tombol, layarnya
     * hilang sekejap, lalu kembali. Berulang-ulang.
     *
     * Hanya berlaku untuk penolakan yang DISENGAJA oleh manusia. Anomali
     * yang hilang sendiri (clear) dan yang dibatalkan gerakan tubuh
     * (motion) tidak mendapat jeda ini — keduanya bukan pernyataan.
     *
     * Tombol SOS menembusnya tanpa syarat. Jeda yang bisa menghalangi
     * seseorang meminta tolong bukan jeda, itu kerusakan. */
    uint16_t standdown_cooldown_s = 180;

    // Gerakan di atas ini saat tahap >= 2 berarti tubuh merespons.
    // ponytail: satu ambang global; jadikan per-pengguna kalau kalibrasi
    // menunjukkan orang tidur gelisah memicu batal terus.
    uint16_t motion_response_mg = 150;
};

class Ladder {
public:
    LadderConfig cfg;

    uint8_t stage() const  { return stage_; }
    uint8_t reason() const { return reason_; }

    /* Ada transisi yang belum dilaporkan? Kontrak §3.5 mewajibkan SETIAP
     * transisi dikirim, termasuk turun ke 0 — itu yang membedakan "tubuh
     * merespons" dari "alarm hilang begitu saja". */
    bool takeChange() { bool c = changed_; changed_ = false; return c; }

    /* Anomali terdeteksi. Hanya menaikkan dari 0; anomali kedua saat tangga
     * sudah jalan tidak me-restart timer. */
    void anomaly(uint32_t now_ms, uint8_t reason) {
        if (stage_ != 0) return;
        if (muted(now_ms)) return;
        set(1, reason, now_ms);
    }

    /* Masih dalam jeda setelah seseorang menyatakan dirinya baik-baik saja? */
    bool muted(uint32_t now_ms) const {
        return muted_until_ms_ != 0 &&
               (int32_t)(muted_until_ms_ - now_ms) > 0;
    }

    /* Sisa jeda dalam detik, untuk dicetak. 0 kalau tidak sedang dijeda. */
    uint16_t muteLeft_s(uint32_t now_ms) const {
        if (!muted(now_ms)) return 0;
        return (uint16_t)((muted_until_ms_ - now_ms) / 1000u);
    }

    /* Tombol SOS. Langsung tahap 4 — pengguna sudah minta tolong, tidak ada
     * yang perlu dikonfirmasi selama 65 detik. */
    void manualSos(uint32_t now_ms) {
        muted_until_ms_ = 0;   // permintaan tolong tidak pernah dijeda
        set(4, REASON_MANUAL, now_ms);
    }

    /* Anomali hilang dengan sendirinya. Tahap 4 tidak ikut batal — begitu
     * SOS terkirim, hanya aplikasi yang boleh menutupnya. */
    void clear(uint32_t now_ms) {
        if (stage_ >= 1 && stage_ <= 3) set(0, REASON_NONE, now_ms);
    }

    /* Level gerakan terbaru, sudah di-gate terhadap motor getar oleh
     * pemanggil (§3.5: motor koin ada di pergelangan yang sama dengan IMU,
     * jadi alarm akan membatalkan dirinya sendiri kalau tidak di-gate). */
    void motion(uint32_t now_ms, uint16_t milli_g) {
        if (stage_ >= 2 && stage_ <= 3 && milli_g > cfg.motion_response_mg) {
            set(0, REASON_NONE, now_ms);
        }
    }

    /* Dipanggil sesering mungkin. Menaikkan tahap saat durasinya habis. */
    void tick(uint32_t now_ms) {
        if (stage_ == 0 || stage_ == 4) return;
        uint32_t elapsed_s = (now_ms - entered_ms_) / 1000u;
        uint16_t limit = stage_ == 1 ? cfg.stage1_s
                       : stage_ == 2 ? cfg.stage2_s
                                     : cfg.stage3_s;
        if (elapsed_s >= limit) set(stage_ + 1, reason_, now_ms);
    }

    /* Aplikasi menutup kejadian — seseorang menekan "saya baik-baik saja".
     *
     * Jedanya dipasang di sini dan hanya di sini. Ini satu-satunya jalan
     * masuk yang berarti sebuah PERNYATAAN dari manusia, dan pernyataan itu
     * layak dipercaya untuk beberapa menit ke depan. */
    void reset(uint32_t now_ms) {
        if (stage_ != 0) set(0, REASON_NONE, now_ms);
        muted_until_ms_ = now_ms + (uint32_t)cfg.standdown_cooldown_s * 1000u;
        if (muted_until_ms_ == 0) muted_until_ms_ = 1;   // 0 berarti "tidak dijeda"
    }

private:
    uint8_t  stage_      = 0;
    uint8_t  reason_     = REASON_NONE;
    uint32_t entered_ms_ = 0;
    uint32_t muted_until_ms_ = 0;
    bool     changed_    = false;

    void set(uint8_t stage, uint8_t reason, uint32_t now_ms) {
        stage_      = stage;
        reason_     = reason;
        entered_ms_ = now_ms;
        changed_    = true;
    }
};
