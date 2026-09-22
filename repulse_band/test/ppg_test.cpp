/*
 * Jalankan: g++ -std=c++17 -o ppg_test ppg_test.cpp && ./ppg_test
 *
 * Detektor denyut tidak bisa diuji dengan menempelkan jari ke papan lalu
 * melihat apakah angkanya wajar — itu menguji jari, bukan kode. Di sini
 * gelombangnya dibuat sendiri, jadi BPM yang benar sudah diketahui sebelum
 * detektornya menjawab.
 *
 * Yang paling penting dari berkas ini: gelombangnya membawa modulasi napas
 * yang LIMA KALI lebih besar daripada denyutnya. Itu bukan untuk menyulitkan
 * — itu kondisi pergelangan yang sebenarnya, dan justru kondisi yang membuat
 * detektor lama diam: ambang adaptif yang tidak membuang napas akan mengunci
 * ke ayunan napas, dan denyut aslinya tidak pernah melewatinya.
 */
#include <cassert>
#include <initializer_list>
#include <cmath>
#include <cstdio>
#include <cstdint>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "../ppg.h"

static const float SPS = 25.0f;              /* 100 Hz / rata-rata 4 */
static const uint32_t STEP_MS = (uint32_t)(1000.0f / SPS);

/* Pergelangan, bukan ujung jari: DC tinggi, AC cuma sepersekian persen. */
static const float DC_IR = 60000.0f;
static const float DC_RED = 50000.0f;
static const float PULSE_FRAC = 0.004f;      /* 0,4% — ujung bawah pergelangan */
static const float BREATH_FRAC = 0.020f;     /* 5x denyut, dan itu wajar */

struct Sample { uint32_t ir, red; };

static Sample synth(float t_s, float bpm) {
    const float hr = bpm / 60.0f;
    const float pulse = sinf(2.0f * (float)M_PI * hr * t_s);
    const float breath = sinf(2.0f * (float)M_PI * 0.25f * t_s);

    /* Modulasi relatif yang sama di kedua kanal -> R sekitar 1,0, yang
     * setelah koreksi pergelangan jatuh di sekitar 97%. */
    const float mod = PULSE_FRAC * pulse + BREATH_FRAC * breath;
    return { (uint32_t)(DC_IR * (1.0f + mod)),
             (uint32_t)(DC_RED * (1.0f + mod)) };
}

/* Gelombang yang sama, tetapi dengan dicrotic notch — puncak kedua setinggi
 * 40% yang datang sekitar sepertiga siklus setelah puncak sistolik.
 *
 * BATAS YANG DIAKUI: detektor ini TIDAK akurat pada bentuk ini. Diukur,
 * bukan dikira — galat BPM terburuknya 25% sampai 100%, dan tidak ada
 * pasangan tetapan waktu atau ambang yang kusapu memperbaikinya. Dua
 * penggantian detektor sudah dicoba dan keduanya lebih buruk; catatannya
 * ada di ppg.h di atas pemicunya.
 *
 * Jadi bentuk ini TIDAK dipakai menguji akurasi BPM. Ia dipakai untuk satu
 * hal saja, di bawah: bahwa notch tidak pernah bisa menghasilkan RR pendek
 * palsu. Itu jaminan yang memang bisa diberikan refraktori adaptif, dan itu
 * cacat yang benar-benar terlihat di perangkat — RR 321 ms muncul di antara
 * denyut-denyut 800 ms, dan RR palsu masuk ke riwayat yang memutuskan
 * apakah malam seseorang dianggap anomali. */
static Sample synthNotch(float t_s, float bpm) {
    const float hr = bpm / 60.0f;
    float phase = t_s * hr;
    phase -= (float)(long)phase;

    const float d1 = (phase - 0.20f) / 0.15f;
    const float d2 = (phase - 0.55f) / 0.13f;
    const float pulse = expf(-d1 * d1) + 0.40f * expf(-d2 * d2);
    const float breath = sinf(2.0f * (float)M_PI * 0.25f * t_s);

    const float mod = PULSE_FRAC * pulse + BREATH_FRAC * breath;
    return { (uint32_t)(DC_IR * (1.0f + mod)),
             (uint32_t)(DC_RED * (1.0f + mod)) };
}

/** Jalankan detik demi detik, kembalikan ringkasan bagian akhirnya. */
struct Run {
    int beats = 0;
    float last_bpm = 0;
    bool bpm_valid = false;
    int spo2_windows = 0;
    float last_spo2 = 0;
    float last_r = 0;
    uint8_t last_quality = 0;
    bool worn_end = false;
    uint16_t shortest_rr = 0xFFFF;
};

static Run drive(Ppg &p, float bpm, float seconds, uint32_t &now_ms,
                 bool contact = true, bool count_after_settle = true) {
    Run r;
    const int n = (int)(seconds * SPS);
    for (int i = 0; i < n; i++) {
        const float t = (float)now_ms / 1000.0f;
        Sample s = contact ? synth(t, bpm) : Sample{ 1000, 900 };
        PpgOut o = p.feed(s.ir, s.red, now_ms);

        /* Empat detik pertama dibuang: 3 detik settle DC ditambah satu detik
         * bagi amplop puncak menemukan tingginya. Menghitung denyut dari
         * dalam masa itu berarti menguji transien, bukan detektor. */
        const bool counting = !count_after_settle || t > 4.0f;
        if (o.beat && counting) {
            r.beats++;
            if (o.rr_ms && o.rr_ms < r.shortest_rr) r.shortest_rr = o.rr_ms;
        }
        if (o.spo2_ready) { r.spo2_windows++; r.last_spo2 = o.spo2; r.last_r = o.r; }
        r.last_bpm = o.bpm;
        r.bpm_valid = o.bpm_valid;
        r.last_quality = o.quality;
        r.worn_end = o.worn;
        now_ms += STEP_MS;
    }
    return r;
}

int main() {
    // --- kontak: ambang, dan debounce yang menjaganya ---------------------
    {
        Ppg p(SPS);
        uint32_t now = 1000;

        /* Di atas meja IR kecil. Tidak dipakai, dan tidak boleh ada denyut
         * apa pun dilaporkan — gelang di atas nakas yang melaporkan nadi
         * adalah kegagalan yang paling mahal di produk ini. */
        Run off = drive(p, 75.0f, 3.0f, now, /*contact=*/false);
        assert(!off.worn_end);
        assert(off.beats == 0);
        assert(!off.bpm_valid);

        /* Menempel: tidak langsung diakui, harus bertahan dulu. */
        PpgOut first = p.feed((uint32_t)DC_IR, (uint32_t)DC_RED, now);
        assert(!first.worn && "kontak sesaat belum kontak");

        now += 100;
        PpgOut mid = p.feed((uint32_t)DC_IR, (uint32_t)DC_RED, now);
        assert(!mid.worn && "100 ms masih di dalam debounce 250 ms");

        now += 200;
        PpgOut settled = p.feed((uint32_t)DC_IR, (uint32_t)DC_RED, now);
        assert(settled.worn && "300 ms sudah melewati debounce");
        assert(settled.worn_changed);
    }

    // --- denyut, dengan napas lima kali lebih besar ------------------------
    {
        Ppg p(SPS);
        uint32_t now = 1000;
        Run r = drive(p, 75.0f, 40.0f, now);

        assert(r.worn_end);
        assert(r.bpm_valid && "40 detik kulit menempel harus menghasilkan BPM");

        /* Inilah kegagalan lama, dinyatakan sebagai angka: detektor yang
         * tidak membuang napas berhenti di 0. */
        assert(r.last_bpm > 0.0f);
        assert(fabsf(r.last_bpm - 75.0f) < 4.0f && "BPM dalam 4 dari yang sebenarnya");

        /* 36 detik dihitung pada 75 bpm = 45 denyut. Toleransinya lebar
         * karena amplop butuh beberapa denyut untuk menemukan tingginya. */
        assert(r.beats >= 35 && r.beats <= 50);

        /* Cahaya cukup dan denyut baru saja terjadi -> kualitas tinggi. */
        assert(r.last_quality > 4 && "denyut segar tidak boleh dijepit di 4");


    }

    // --- dua detak berbeda, supaya bukan kebetulan satu angka -------------
    for (float bpm : { 52.0f, 96.0f, 130.0f }) {
        Ppg p(SPS);
        uint32_t now = 1000;
        Run r = drive(p, bpm, 40.0f, now);
        assert(r.bpm_valid);
        char msg[96];
        snprintf(msg, sizeof(msg), "BPM %.0f", (double)bpm);
        assert(fabsf(r.last_bpm - bpm) < bpm * 0.06f && msg);
    }

    // --- SpO2: masuk akal, dan tidak pernah mengarang ----------------------
    {
        Ppg p(SPS);
        uint32_t now = 1000;
        Run r = drive(p, 75.0f, 40.0f, now);

        assert(r.spo2_windows > 10 && "satu jendela per detik setelah settle");
        assert(r.last_spo2 >= 70.0f && "di bawah 70 bukan bacaan rendah, itu bacaan gagal");
        assert(r.last_spo2 <= 100.0f && "tidak ada yang di atas 100");
        assert(r.last_r > 0.3f && r.last_r < 1.15f);
    }

    // --- sinyal mati: tidak ada denyut yang dikarang ----------------------
    {
        Ppg p(SPS);
        uint32_t now = 1000;
        /* Kulit menempel — DC penuh — tetapi sama sekali tidak ada denyut di
         * dalamnya. Detektor harus diam, bukan mengejar derau. */
        for (int i = 0; i < (int)(30 * SPS); i++) {
            p.feed((uint32_t)DC_IR, (uint32_t)DC_RED, now);
            now += STEP_MS;
        }
        PpgOut o = p.feed((uint32_t)DC_IR, (uint32_t)DC_RED, now);
        assert(!o.bpm_valid && "DC rata bukan denyut");
        assert(o.quality <= 4 && "tanpa denyut, kualitas dijepit di 4");
    }

    // --- kontak lepas menghapus semuanya ----------------------------------
    {
        Ppg p(SPS);
        uint32_t now = 1000;
        Run warm = drive(p, 75.0f, 40.0f, now);
        assert(warm.bpm_valid);

        Run gone = drive(p, 75.0f, 3.0f, now, /*contact=*/false);
        assert(!gone.worn_end);
        assert(!gone.bpm_valid && "BPM lama tidak boleh bertahan setelah dilepas");
    }

    // --- laju cuplik lain tidak diam-diam merusak tetapan waktu -----------
    {
        /* Inilah yang membuat tetapan waktu ditulis dalam detik. Pada 12,5
         * sampel/detik — rata-rata perangkat keras 8, seperti AsaWatch —
         * jawabannya harus tetap sama. */
        const float sps = 12.5f;
        Ppg p(sps);
        uint32_t now = 1000;
        const uint32_t step = (uint32_t)(1000.0f / sps);
        float bpm_end = 0;
        bool valid = false;
        for (int i = 0; i < (int)(40 * sps); i++) {
            Sample s = synth((float)now / 1000.0f, 75.0f);
            PpgOut o = p.feed(s.ir, s.red, now);
            bpm_end = o.bpm;
            valid = o.bpm_valid;
            now += step;
        }
        assert(valid && "12,5 sampel/detik juga harus menemukan denyut");
        assert(fabsf(bpm_end - 75.0f) < 4.0f);
    }

    // --- dicrotic notch tidak boleh menjadi RR palsu ----------------------
    {
        /* Satu-satunya hal yang diuji dari gelombang dua puncak. Lihat
         * catatan panjang di synthNotch(). */
        Ppg p(SPS);
        uint32_t now = 1000;
        uint16_t shortest = 0xFFFF;
        int beats = 0;

        for (int i = 0; i < (int)(60 * SPS); i++) {
            PpgOut o = p.feed(synthNotch((float)now / 1000.0f, 75.0f).ir,
                              synthNotch((float)now / 1000.0f, 75.0f).red, now);
            if (o.beat && o.rr_ms) {
                beats++;
                if (o.rr_ms < shortest) shortest = o.rr_ms;
            }
            now += STEP_MS;
        }

        assert(beats > 0 && "harus tetap menemukan denyut, seakurat apa pun");

        /* Pada 75 bpm rata-rata jaraknya 800 ms, jadi refraktori adaptif
         * berada di sekitar 480 ms. Notch jatuh sekitar 280 ms setelah
         * puncaknya — jauh di bawah itu, dan tidak boleh pernah lolos. */
        assert(shortest >= 400 &&
               "notch tidak boleh menghasilkan RR pendek palsu");
    }

    // --- ADC jenuh: AGC harus menurunkan arus sampai keluar dari atap ----
    {
        /* Persis angka dari jari di atas meja pada arus penuh: DC 254000
         * dari atap 262143, AC terpotong habis. Sebelum AGC ada, keadaan ini
         * abadi — tanpa AC tidak ada denyut, tanpa denyut gerbangnya tidak
         * pernah terbuka, dan tidak ada yang pernah menurunkan arusnya. */
        Ppg p(SPS);
        uint32_t now = 1000;
        int changes = 0;
        const uint8_t first = p.ledIr();

        for (int i = 0; i < (int)(30 * SPS); i++) {
            /* DC mengikuti arus: menurunkan arus menurunkan DC sebanding.
             * Tanpa pemodelan ini AGC akan terlihat "tidak berhasil" hanya
             * karena gelombang ujinya tidak ikut menanggapi. */
            const uint32_t ir  = (uint32_t)(254000.0f * ((float)p.ledIr()  / 255.0f));
            const uint32_t red = (uint32_t)(250000.0f * ((float)p.ledRed() / 255.0f));
            PpgOut o = p.feed(ir, red, now);
            if (o.led_changed) changes++;
            now += STEP_MS;
        }

        assert(first == 0xFF && "mulai dari arus penuh, yang benar untuk pergelangan");
        assert(changes > 0 && "AGC harus turun tangan saat DC menyentuh atap");
        assert(p.ledIr() < 0xFF && "arus IR harus turun");
        assert(p.ledIr() >= 0x40 && "tapi tidak sampai di bawah lantainya");

        const float settled = 254000.0f * ((float)p.ledIr() / 255.0f);
        assert(settled <= 200000.0f && "harus keluar dari atap");
        assert(settled >= 60000.0f && "dan tidak jatuh terus sampai lantai");
    }

    // --- sinyal lemah: AGC menaikkan lagi --------------------------------
    {
        /* Kebalikannya, dan alasan lantai AGC_DC_LOW ada: kulit gelap atau
         * kontak longgar memberi DC kecil, dan arusnya harus naik lagi. */
        Ppg p(SPS);
        uint32_t now = 1000;
        for (int i = 0; i < (int)(12 * SPS); i++) {
            p.feed(254000, 250000, now);
            now += STEP_MS;
        }
        const uint8_t lowered = p.ledIr();
        assert(lowered < 0xFF);

        for (int i = 0; i < (int)(30 * SPS); i++) {
            p.feed(40000, 38000, now);   /* di atas ambang kontak, di bawah AGC_DC_LOW */
            now += STEP_MS;
        }
        assert(p.ledIr() > lowered && "arus harus naik lagi saat sinyalnya lemah");
    }

    printf("ok — denyut di bawah napas 5x lebih besar, notch tidak jadi RR palsu, "
           "gelang di atas meja diam, dan ADC jenuh menyembuhkan dirinya\n");
    return 0;
}
