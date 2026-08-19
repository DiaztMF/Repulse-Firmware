#pragma once
#include <stdint.h>
#include <string.h>

/*
 * Ring buffer offline — BLE_GATT_CONTRACT.md §3.9.
 *
 * Juga murni, juga bisa dites di laptop. Aturan yang paling mudah salah ada
 * di sini: buffer BARU dihapus setelah ACK diterima (§3.9 langkah 5), supaya
 * putus di tengah flush tidak menghilangkan kejadian semalam.
 *
 * Kapasitas 256 entri = 2,3 KB RAM. Ini jawaban untuk kontrak §7 no. 3.
 * Pada laju "ringkasan vital berkala" tiap 60 detik itu ~4 jam offline;
 * transisi tangga dan SOS jauh lebih jarang.
 */

#define EVTBUF_CAPACITY   256
#define EVTBUF_ENTRY_LEN  9
#define EVTBUF_PER_PACKET 19    // §3.9: maksimum 19 pada MTU 185
#define EVTBUF_SENTINEL   0xFFFF

enum EventType : uint8_t {
    EVT_ANOMALY   = 1,  // [reason, 0, 0, 0]
    EVT_STAGE     = 2,  // [stage, reason, 0, 0]
    EVT_SOS       = 3,  // [0, 0, 0, 0]
    EVT_VITALS    = 4,  // [bpm, spo2, milli_g >> 8, milli_g & 0xFF]
    EVT_LINK      = 5,  // [0=putus 1=sambung, 0, 0, 0]
};

class EventBuffer {
public:
    /* epoch_s = waktu kejadian sebenarnya, bukan waktu pengiriman (§3.9).
     * Kalau jam gelang belum pernah disinkronkan, epoch 0 ikut tersimpan
     * apa adanya — aplikasi yang memutuskan apa artinya, bukan kita. */
    void push(uint32_t epoch_s, uint8_t type,
              uint8_t p0 = 0, uint8_t p1 = 0, uint8_t p2 = 0, uint8_t p3 = 0) {
        uint8_t *e = slot(head_);
        e[0] = epoch_s & 0xFF;          // little-endian, §1
        e[1] = (epoch_s >> 8)  & 0xFF;
        e[2] = (epoch_s >> 16) & 0xFF;
        e[3] = (epoch_s >> 24) & 0xFF;
        e[4] = type;
        e[5] = p0; e[6] = p1; e[7] = p2; e[8] = p3;

        head_ = (head_ + 1) % EVTBUF_CAPACITY;
        if (count_ < EVTBUF_CAPACITY) count_++;
        else                          tail_ = head_;   // timpa yang tertua
    }

    uint16_t count() const { return count_; }

    /* §3.9 langkah 1. Membekukan apa yang akan dikirim, supaya kejadian yang
     * datang di tengah flush tidak menggeser penomoran seq. */
    void beginFlush() {
        flushing_  = true;
        snapshot_  = count_;
        next_seq_  = 0;
    }

    bool flushing() const { return flushing_; }

    /* Mengisi paket berikutnya. Mengembalikan false bila sudah habis —
     * saat itu pemanggil mengirim paket sentinel. */
    bool nextPacket(uint8_t *out, uint16_t *out_len) {
        uint16_t sent = (uint16_t)next_seq_ * EVTBUF_PER_PACKET;
        if (!flushing_ || sent >= snapshot_) return false;

        uint8_t n = (uint8_t)((snapshot_ - sent) < EVTBUF_PER_PACKET
                              ? (snapshot_ - sent) : EVTBUF_PER_PACKET);
        out[0] = next_seq_ & 0xFF;
        out[1] = (next_seq_ >> 8) & 0xFF;
        out[2] = n;
        for (uint8_t i = 0; i < n; i++) {
            memcpy(out + 3 + i * EVTBUF_ENTRY_LEN,
                   slot((tail_ + sent + i) % EVTBUF_CAPACITY),
                   EVTBUF_ENTRY_LEN);
        }
        *out_len = 3 + (uint16_t)n * EVTBUF_ENTRY_LEN;
        next_seq_++;
        return true;
    }

    static uint16_t sentinel(uint8_t *out) {
        out[0] = EVTBUF_SENTINEL & 0xFF;
        out[1] = EVTBUF_SENTINEL >> 8;
        out[2] = 0;
        return 3;
    }

    /* §3.9 langkah 5. Baru di sinilah entri hilang. */
    void ack(uint16_t last_seq) {
        if (!flushing_) return;
        uint32_t drop = (uint32_t)(last_seq + 1) * EVTBUF_PER_PACKET;
        if (drop > snapshot_) drop = snapshot_;
        tail_   = (uint16_t)((tail_ + drop) % EVTBUF_CAPACITY);
        count_ -= (uint16_t)drop;
        flushing_ = false;
    }

    /* Flush gagal di tengah jalan (koneksi putus). Tidak ada yang dihapus. */
    void abortFlush() { flushing_ = false; }

private:
    uint8_t  data_[EVTBUF_CAPACITY * EVTBUF_ENTRY_LEN] = {0};
    uint16_t head_ = 0, tail_ = 0, count_ = 0;
    bool     flushing_ = false;
    uint16_t snapshot_ = 0, next_seq_ = 0;

    uint8_t *slot(uint16_t i) { return data_ + (size_t)i * EVTBUF_ENTRY_LEN; }
};
