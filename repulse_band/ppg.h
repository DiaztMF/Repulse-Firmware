#pragma once
#include <stdint.h>
#include <math.h>
#include <string.h>

/*
 * PPG: sampel Red/IR mentah -> kontak kulit, denyut, dan SpO2.
 *
 * Diadaptasi dari AsaWatch (github.com/MrGengs/Asawatch, no_touch/ppg.cpp),
 * sebuah jam tangan MAX30102 yang sudah diuji di pergelangan pada belasan
 * subjek. Yang diambil adalah ALGORITMANYA, bukan angka kalibrasinya:
 * glukosa dan tekanan darah sengaja tidak dibawa sama sekali. Keduanya di
 * sana pun ditandai belum tervalidasi, dan produk ini tidak membuat klaim
 * medis.
 *
 * Kenapa diganti. Detektor lama memakai checkForBeat() milik SparkFun, yang
 * di dalamnya bekerja pada int16_t: averageDCEstimator mengembalikan
 * int16_t, jadi DC di atas 32767 kembali negatif dan sinyal AC menjadi nol
 * selamanya. Kode lama mengakalinya dengan menggeser sinyal ke titik tengah
 * buatan (BEAT_CENTRE) lalu menjepitnya di 60000 — sebuah tambalan yang
 * komentarnya sendiri akui sebagai tambalan. Detektor di bawah ini bekerja
 * penuh di titik mengambang: tidak ada batas 16 bit untuk diakali.
 *
 * Tiga lapis, dan masing-masing menjawab satu kegagalan nyata:
 *
 *  1. DC dibuang dengan EMA, bukan pembagian. Membagi sinyal agar muat di
 *     16 bit ikut membagi AC yang justru dicari — di pergelangan AC cuma
 *     0,1-0,5% dari DC dan tidak ada ruang untuk dikorbankan.
 *
 *  2. Ayunan napas dibuang dengan EMA kedua yang lebih lambat. Napas
 *     (~0,2-0,4 Hz) sering berayun jauh lebih besar daripada denyut
 *     (~1-2 Hz); tanpa dibuang, ambang adaptif mengunci ke napas dan denyut
 *     asli tidak pernah melewatinya.
 *
 *  3. Ambangnya mengikuti amplop puncak, bukan angka tetap. Kulit setiap
 *     orang berbeda dan satu angka tetap hanya cocok untuk satu pergelangan.
 *
 * Murni: tidak ada Arduino.h, tidak ada Wire, tidak ada MAX30105 di sini.
 * Pemanggil yang mengurus perangkat kerasnya dan menyuapkan sampel. Itu yang
 * membuat berkas ini bisa diuji di komputer — lihat test/ppg_test.cpp — sama
 * seperti ladder.h dan evtbuf.h.
 */

/* Ambang IR untuk "kulit menempel", pada arus LED penuh (0xFF).
 *
 * Angka AsaWatch, diukur di pergelangan, bukan di ujung jari. Nilai lama di
 * sini adalah 5000 dengan komentar yang mengakui itu tebakan — dan tebakan
 * itu diambil saat arus LED masih 0x7F. Arus naik berarti IR naik, jadi
 * keduanya harus berubah bersama.
 *
 * ponytail: tetap perlu diukur di pergelangan yang akan memakainya.
 * PpgOut.ir dicetak oleh [CAL]; baca di pergelangan DAN di atas meja, lalu
 * taruh ambangnya di tengah. */
#ifndef PPG_IR_PRESENT
#define PPG_IR_PRESENT 30000.0f
#endif

/** IR yang dianggap kualitas penuh (15/15). */
#ifndef PPG_IR_QUALITY_FULL
#define PPG_IR_QUALITY_FULL 120000.0f
#endif

/** Hasil satu sampel. Semua bendera hanya benar pada sampel yang memicunya. */
struct PpgOut {
    /** Denyut terdeteksi pada sampel ini. */
    bool beat = false;
    /** Jarak ke denyut sebelumnya, ms. 0 pada denyut pertama sesi. */
    uint16_t rr_ms = 0;

    bool worn = false;
    /** `worn` berubah pada sampel ini — pemanggil mereset apa yang perlu. */
    bool worn_changed = false;

    float bpm = 0.0f;
    bool bpm_valid = false;

    /** Satu jendela SpO2 selesai pada sampel ini, dan hasilnya masuk akal. */
    bool spo2_ready = false;
    /** Persen. §3.2 mengirim 0 untuk "tidak valid". */
    float spo2 = 0.0f;

    /** Dua fitur sinyal yang dipakai untuk membuang jendela artefak, dan
     *  satu-satunya angka yang berguna saat mengkalibrasi ambang. */
    float pi = 0.0f;
    float r = 0.0f;

    /** 0-15, §3.1. Turun ke 4 kalau sudah lama tidak ada denyut. */
    uint8_t quality = 0;

    /** AGC memutuskan arus LED baru. Pemanggil wajib menuliskannya ke
     *  sensor — ppg.h tidak menyentuh perangkat keras. */
    bool led_changed = false;
};

class Ppg {
public:
    /**
     * @param sps  Sampel per detik yang benar-benar keluar dari FIFO —
     *             SAMPLE_RATE dibagi SAMPLE_AVERAGE, bukan SAMPLE_RATE saja.
     *
     * Semua tetapan waktu di bawah ditulis dalam DETIK lalu diubah menjadi
     * alpha di sini. Aslinya ditulis sebagai alpha telanjang (0,95 / 0,97 /
     * 0,98) yang benar hanya pada 12,5 sampel/detik — rata-rata perangkat
     * keras 8. Menyalinnya apa adadanya ke laju lain akan menggeser setiap
     * tetapan waktu secara diam-diam: pada 25 sampel/detik, DC_ALPHA 0,95
     * berarti garis dasar mengejar dalam 0,8 detik, cukup cepat untuk ikut
     * memakan denyut yang sedang dicari. Ditulis begini, mengubah
     * SAMPLE_AVERAGE tidak diam-diam merusak apa pun.
     */
    explicit Ppg(float sps = 25.0f) { configure(sps); }

    void configure(float sps) {
        sps_ = sps > 1.0f ? sps : 1.0f;
        dc_a_     = alphaFor(DC_TC_S);
        slow_a_   = alphaFor(SLOW_TC_S);
        smooth_a_ = alphaFor(SMOOTH_TC_S);
        env_a_    = alphaFor(ENV_TC_S);
        reset();
    }

    /** Satu pasang sampel mentah dari FIFO. `now_ms` adalah millis(). */
    PpgOut feed(uint32_t ir, uint32_t red, uint32_t now_ms) {
        PpgOut o;
        last_ir_ = ir;

        o.worn = contact(ir, now_ms);
        o.worn_changed = (o.worn != worn_prev_);
        if (o.worn_changed) {
            if (o.worn) contact_at_ms_ = now_ms;
            resetDetector();
            worn_prev_ = o.worn;
        }

        if (!o.worn) {
            dc_init_ = false;
            win_n_ = 0;
            return o;
        }

        if (!dc_init_) {
            dc_ir_ = (float)ir;
            dc_red_ = (float)red;
            dc_init_ = true;
        }
        dc_ir_  = dc_ir_  * dc_a_ + (float)ir  * (1.0f - dc_a_);
        dc_red_ = dc_red_ * dc_a_ + (float)red * (1.0f - dc_a_);

        const float ac_ir  = (float)ir  - dc_ir_;
        const float ac_red = (float)red - dc_red_;

        /* Selama DC masih mengejar garis dasar, yang terlihat sebagai "AC"
         * sebagian besar adalah transien filter, bukan denyut. Menyuapkannya
         * ke detektor mencemari amplop puncaknya — dan amplop itu hanya
         * naik, jadi pencemarannya menetap sampai kontak lepas. */
        const bool settled = (now_ms - contact_at_ms_) >= DC_SETTLE_MS;
        if (settled) detectBeat(ac_ir, now_ms, o);

        accumulate(ac_red, ac_ir, dc_red_, dc_ir_, now_ms, settled, o);

        o.bpm = bpm_;
        o.bpm_valid = bpm_valid_;
        o.quality = quality(ir, now_ms);
        return o;
    }

    /** Untuk baris [CAL] — membedakan "tidak dipakai" dari "FIFO diam". */
    uint32_t ir() const { return last_ir_; }
    /** Arus LED yang sedang diminta AGC. Tulis ini ke sensor setiap kali
     *  PpgOut.led_changed benar. */
    uint8_t ledRed() const { return led_red_; }
    uint8_t ledIr() const { return led_ir_; }
    float threshold() const { return PPG_IR_PRESENT; }
    /* Dicetak saat boot. Tanpa ini, satu-satunya cara tahu apakah
     * PPG_WRIST_OFFSET benar-benar tertimpa adalah menebak dari angkanya —
     * dan pada jari angka yang salah terjepit di 100, yang tidak bisa
     * dibedakan dari jari yang memang sehat. */
    float wristOffset() const { return WRIST_OFFSET; }

    void reset() {
        worn_prev_ = false;
        raw_cand_ = false;
        raw_cand_at_ms_ = 0;
        contact_at_ms_ = 0;
        led_red_ = led_ir_ = AGC_LED_MAX;
        agc_at_ms_ = 0;
        dc_init_ = false;
        dc_ir_ = dc_red_ = 0.0f;
        last_ir_ = 0;
        resetDetector();
    }

private:
    /* Tetapan waktu, dalam detik.
     *
     * DI SINI ANGKANYA BERBEDA DARI ASAWATCH, dan itu disengaja. Aslinya
     * memakai DC 1,6 s, slow 2,67 s, dan SATU kutub. Diukur dengan gelombang
     * buatan yang membawa napas lima kali lebih besar daripada denyut —
     * lihat test/ppg_test.cpp — susunan itu menyisakan napas sebesar
     * SELURUH sinyalnya: rasio denyut terhadap sisa napas cuma 1,04. Ambang
     * adaptif lalu mengunci ke ayunan napas, jarak antar "denyut" menjadi
     * empat detik, dan MAX_IBI_MS membuang semuanya — BPM tidak pernah sah.
     * Itu persis kegagalan yang hendak dicegah oleh komentar aslinya.
     *
     * Sebabnya bisa dihitung: 0,25 Hz ke 1,25 Hz hanya 2,3 oktaf, dan satu
     * kutub melemahkan 6 dB per oktaf — sekitar lima kali, yang pas habis
     * dimakan napas yang memang lima kali lebih besar. Dua kutub memberi 12
     * dB per oktaf, dan rasionya naik ke 2,33: denyut menang telak. Diperiksa
     * dari 50 sampai 180 bpm dan pada 12,5 maupun 25 sampel/detik.
     *
     * Susunannya jadi band-pass yang benar-benar mengelilingi pita denyut:
     * DC 0,9 s membuang geser garis dasar, dua kutub 0,15 s membuang napas,
     * dan smooth 0,10 s membuang derau satu sampel. Ketiganya disapu
     * terhadap gelombang buatan, bukan dikira: galat BPM terburuk 3,8%
     * dari 50 sampai 170 bpm, dan itu bertahan pada 12,5 / 25 / 50 / 100
     * sampel per detik. Pada 0,25 s galatnya 32% di 100 sampel/detik, jadi
     * jangan dikembalikan ke sana tanpa menyapunya lagi. */
    static constexpr float DC_TC_S     = 0.9f;
#ifndef PPG_SLOW_TC_S
#define PPG_SLOW_TC_S 0.15f
#endif
    static constexpr float SLOW_TC_S   = PPG_SLOW_TC_S;
    static constexpr float SMOOTH_TC_S = 0.10f;
    /* Amplop puncak, dan pecahan amplop yang dianggap denyut. Keduanya
     * bisa ditimpa saat kompilasi supaya bisa disapu terhadap gelombang
     * buatan tanpa menyunting berkas ini. */
#ifndef PPG_ENV_TC_S
#define PPG_ENV_TC_S 4.0f
#endif
#ifndef PPG_THRESH_FRAC
#define PPG_THRESH_FRAC 0.40f
#endif
    static constexpr float ENV_TC_S = PPG_ENV_TC_S;
    static constexpr float THRESH_FRAC = PPG_THRESH_FRAC;

    static constexpr uint32_t DC_SETTLE_MS = 3000;
    static constexpr uint32_t CONTACT_DEBOUNCE_MS = 250;
    /* Refraktori. 350 ms, bukan 300: pada gelombang sinus keduanya
     * sama-sama memberi galat terburuk 3,8%, tetapi 350 ms menolak dicrotic
     * notch yang terlihat di perangkat sebagai RR 321 ms. Di 400 ms rusak —
     * 170 bpm berperiode 353 ms, jadi refraktori 400 ms memblokirnya dan
     * BPM terkunci di setengah laju. Batas atas yang bisa dibaca karenanya
     * 171 bpm. */
#ifndef PPG_MIN_IBI_MS
#define PPG_MIN_IBI_MS 350
#endif
    static constexpr uint32_t MIN_IBI_MS = PPG_MIN_IBI_MS;
    static constexpr uint32_t MAX_IBI_MS = 2000;  /* buang di bawah 30 bpm */
    static constexpr uint32_t BEAT_STALE_MS = 4000;

    static constexpr uint32_t SPO2_WINDOW_MS = 1000;
    static constexpr int SPO2_WINDOW_MIN = 8;

    /* ---- Auto-gain control, arus LED per kanal ----
     *
     * Arus penuh adalah titik awal yang benar untuk pergelangan — sinyalnya
     * lemah dan butuh semua cahaya yang bisa didapat. Tapi arus yang sama
     * pada JARI, atau pada kulit tipis, mendorong DC ke atap ADC 18 bit
     * (262143). Sinyal yang menyentuh atap terpotong rata, dan yang terpotong
     * itu justru puncak-puncak AC yang sedang dicari: DC terlihat sempurna,
     * kualitas 15/15, dan BPM tetap nol selamanya.
     *
     * Itu bukan hipotesis. Jari di atas meja pada arus penuh membaca 254521
     * dari 262143 — 97% skala penuh — dengan PI 0,000 dan R 0,0000.
     *
     * Red dan IR diatur TERPISAH. Jaringan menyerap keduanya berbeda, dan
     * R = (AC/DC)merah / (AC/DC)infra sudah menormalkan tiap kanal terhadap
     * DC-nya sendiri — mengubah salah satu saja tidak merusak arti rasionya,
     * hanya menggeser titik kerjanya.
     *
     * Diperiksa JARANG. Tiap perubahan arus memaksa DC dan detektor mengejar
     * ulang dari awal, jadi memeriksa tiap jendela berarti kulit yang
     * kebetulan berada persis di tepi ambang tidak akan pernah sempat
     * stabil — ia akan naik-turun selamanya. */
    static constexpr float AGC_DC_HIGH = 200000.0f;
    static constexpr float AGC_DC_LOW  = 60000.0f;
    static constexpr uint8_t AGC_STEP = 32;
    static constexpr uint8_t AGC_LED_MAX = 0xFF;
    /* Jangan turun sampai mendekati ambang kehadiran kulit: arus yang
     * terlalu kecil membuat kulit sungguhan terbaca sebagai meja kosong. */
    static constexpr uint8_t AGC_LED_MIN = 0x40;
    static constexpr uint32_t AGC_CHECK_MS = 4000;

    /* Perburuan pertama tidak boleh selambat perawatannya.
     *
     * Sebelum ada denyut sama sekali, tiap langkah AGC menunggu empat detik
     * penuh - dan setiap langkah juga memulai ulang settle di bawah, karena
     * arus yang berubah membuat DC lama tak berarti. Turun dari 0xFF ke
     * 0xBF itu dua langkah: delapan detik hanya untuk menemukan arus yang
     * benar, sebelum denyut pertama boleh dihitung. Itulah diam panjang saat
     * gelang baru dipasang.
     *
     * Setengah detik selama belum ada denyut membuat arusnya bertemu dalam
     * sekitar satu detik. Begitu ada denyut, kembali ke empat detik: di
     * situ tugasnya bukan lagi mencari, melainkan mengikuti perubahan
     * lambat, dan mengganggu detektor tiap setengah detik sepanjang malam
     * akan merusak tepat hal yang baru saja ditemukan. */
    static constexpr uint32_t AGC_FAST_MS = 500;

    /* Batas kewajaran, AN6409. Di luar ini yang terbaca artefak gerak atau
     * kontak longgar, dan memaksanya lewat formula menghasilkan angka
     * mustahil, bukan angka yang sekadar kurang tepat. */
    static constexpr float R_MIN = 0.3f;
    static constexpr float R_MAX = 1.15f;
    static constexpr float PI_MIN = 0.02f;
    static constexpr float PI_MAX = 15.0f;
    static constexpr float SPO2_MIN = 70.0f;

    /* Koreksi bias pergelangan. AN6409 divalidasi untuk geometri ujung jari;
     * di pergelangan hasilnya jatuh secara sistematis. AsaWatch mengukur
     * selisihnya sekali (wrist 83,1% vs jari 99,9% berdekatan) dan memakai
     * selisih itu.
     *
     * ponytail: satu titik data, bukan kalibrasi. Kalau ada oksimeter medis,
     * ukur di pergelangan yang sama dan ganti angka ini.
     *
     * SAAT MENGUJI DENGAN JARI DI ATAS MEJA, NOLKAN INI:
     *
     *     #define PPG_WRIST_OFFSET 0.0f   // sebelum #include "ppg.h"
     *
     * AN6409 memang divalidasi untuk geometri jari, jadi di jari rumusnya
     * sudah benar tanpa koreksi. Membiarkan +16,8 di jari mendorong setiap
     * bacaan melewati 100 dan terjepit di sana — SpO2 akan terbaca 100%
     * terus, dan itu bukan sensor yang bagus, itu angka yang tidak bisa
     * dipakai menilai apa pun. */
#ifndef PPG_WRIST_OFFSET
#define PPG_WRIST_OFFSET 16.8f
#endif
    static constexpr float WRIST_OFFSET = PPG_WRIST_OFFSET;

    float alphaFor(float tc_s) const {
        const float n = tc_s * sps_;
        return n <= 1.0f ? 0.0f : 1.0f - 1.0f / n;
    }

    /* Kontak dengan debounce waktu, bukan histeresis nilai. Kontak yang
     * goyang sesaat melintasi ambang bolak-balik; histeresis hanya
     * menggeser ambangnya, sedangkan debounce menuntut keadaan barunya
     * bertahan dulu. Tanpa ini satu sampel nakal membuang seluruh kemajuan
     * settle yang baru saja dikumpulkan. */
    bool contact(uint32_t ir, uint32_t now_ms) {
        const bool raw = (float)ir >= PPG_IR_PRESENT;
        if (raw != raw_cand_) {
            raw_cand_ = raw;
            raw_cand_at_ms_ = now_ms;
        }
        if (raw_cand_ != worn_prev_ &&
            (now_ms - raw_cand_at_ms_) >= CONTACT_DEBOUNCE_MS) {
            return raw_cand_;
        }
        return worn_prev_;
    }

    /* Jarak minimum ke denyut berikutnya, TETAP.
     *
     * Refraktori adaptif sudah dicoba dua kali dan keduanya terukur lebih
     * buruk. Berbasis rata-rata jarak, satu denyut yang terlewat
     * menggandakan satu jarak, rata-ratanya naik, refraktori melewati satu
     * periode penuh, dan denyut berikutnya ikut terblokir — BPM terkunci di
     * TEPAT setengah laju sebenarnya (75 -> 37,7; 130 -> 65,2; 170 -> 84,5).
     * Berbasis jarak terpendek, yang seharusnya tidak bisa membengkak,
     * hasilnya sama saja.
     *
     * Jadi tetap. Dan angkanya disapu, bukan dikira — lihat tabel di
     * test/ppg_test.cpp. */
    uint32_t refractory() const { return MIN_IBI_MS; }

    void detectBeat(float ac_ir, uint32_t now_ms, PpgOut &o) {
        /* Haluskan sedikit dulu, supaya satu sampel berisik tidak
         * menyeberangi ambang sendirian. */
        smooth_ = smooth_ * smooth_a_ + ac_ir * (1.0f - smooth_a_);

        /* Lapis 2: buang ayunan napas. Dua kutub, bukan satu — satu kutub
         * tidak cukup curam untuk memisahkan 0,25 Hz dari 1,25 Hz ketika
         * yang pertama lebih besar. */
        slow_ = slow_ * slow_a_ + smooth_ * (1.0f - slow_a_);
        const float hp1 = smooth_ - slow_;
        slow2_ = slow2_ * slow_a_ + hp1 * (1.0f - slow_a_);
        const float cardiac = hp1 - slow2_;

        /* Ambang dihitung dari amplop SEBELUM sampel ini ikut memperbaruinya.
         * Kalau amplopnya ikut naik memakai puncak yang sedang dinilai, ia
         * mengejar persis puncak itu dan penyeberangan tidak pernah terjadi. */
        const float threshold = env_ * THRESH_FRAC;

        /* Pemicunya penyeberangan ambang adaptif ke atas, dengan
         * refraktori yang mengikuti denyut orangnya.
         *
         * Dua penggantian yang lebih pintar sudah dicoba di sini dan
         * keduanya lebih buruk, jadi keduanya dicatat supaya tidak diulang:
         *
         *   Gerbang tinggi puncak + pemicu penyeberangan -> SATU denyut
         *   lalu diam selamanya. Sisa napas menahan sinyal di atas ambang
         *   selama setengah siklus napas, jadi penyeberangan naik
         *   berikutnya tidak pernah terjadi.
         *
         *   Pencarian puncak lokal + gerbang tinggi -> nol denyut. Puncak
         *   pertama setelah settle masih transien filter dan jauh lebih
         *   tinggi daripada denyut mana pun sesudahnya; rata-rata tinggi
         *   denyut terkunci ke sana, dan semua denyut asli tertolak.
         *
         * Yang di bawah ini sederhana dan SUDAH menghasilkan BPM di
         * perangkat sungguhan. Kalau nanti diganti lagi, buktinya harus
         * datang dari gelang di pergelangan, bukan dari gelombang buatan —
         * dua kegagalan di atas lolos setiap gelombang buatan yang kupunya. */
        if (cardiac > threshold && cardiac_prev_ <= threshold &&
            (now_ms - last_beat_ms_) > refractory()) {
            const uint32_t ibi = now_ms - last_beat_ms_;
            if (last_beat_ms_ != 0 && ibi < MAX_IBI_MS) {
                ibi_[ibi_i_] = (float)ibi;
                ibi_i_ = (ibi_i_ + 1) % IBI_N;
                if (ibi_n_ < IBI_N) ibi_n_++;

                float sum = 0.0f;
                for (uint8_t i = 0; i < ibi_n_; i++) sum += ibi_[i];
                bpm_ = 60000.0f / (sum / (float)ibi_n_);
                bpm_valid_ = true;

                o.beat = true;
                o.rr_ms = (uint16_t)ibi;
                if (stable_beats_ < MIN_STABLE_BEATS) stable_beats_++;
            }
            last_beat_ms_ = now_ms;
        }

        const float mag = cardiac < 0 ? -cardiac : cardiac;
        env_ = env_ * env_a_ + mag * (1.0f - env_a_);
        cardiac_prev_ = cardiac;
    }


    void accumulate(float ac_red, float ac_ir, float dc_red, float dc_ir,
                    uint32_t now_ms, bool settled, PpgOut &o) {
        if (win_n_ == 0) win_at_ms_ = now_ms;
        sq_red_ += (double)ac_red * ac_red;
        sq_ir_  += (double)ac_ir  * ac_ir;
        sum_dc_red_ += dc_red;
        sum_dc_ir_  += dc_ir;
        win_n_++;

        if (win_n_ < SPO2_WINDOW_MIN) return;
        if ((now_ms - win_at_ms_) < SPO2_WINDOW_MS) return;

        const float rms_red = sqrtf((float)(sq_red_ / win_n_));
        const float rms_ir  = sqrtf((float)(sq_ir_ / win_n_));
        const float mean_red = (float)(sum_dc_red_ / win_n_);
        const float mean_ir  = (float)(sum_dc_ir_ / win_n_);

        sq_red_ = sq_ir_ = 0.0;
        sum_dc_red_ = sum_dc_ir_ = 0.0;
        win_n_ = 0;

        /* AGC dulu, SEBELUM gerbang mana pun.
         *
         * Urutan ini bukan gaya. Jenuh menghapus AC, tanpa AC tidak ada
         * denyut, tanpa denyut `bpm_valid_` tidak pernah benar — jadi
         * menaruh AGC di balik gerbang itu berarti satu-satunya yang bisa
         * MENYEMBUHKAN kejenuhan hanya berjalan kalau kejenuhannya sudah
         * sembuh. Jari di atas meja akan terjebak di sana selamanya. */
        agc(mean_red, mean_ir, now_ms, o);

        /* bpm_valid disyaratkan bukan karena SpO2 butuh denyut, tapi karena
         * sebuah jendela tanpa satu denyut pun di dalamnya tidak punya AC
         * yang berarti untuk dirasiokan. */
        if (!settled || !bpm_valid_) return;
        if (!(mean_red > 0.0f && mean_ir > 0.0f && rms_ir > 0.0f)) return;

        const float r = (rms_red / mean_red) / (rms_ir / mean_ir);
        const float pi = (rms_ir / mean_ir) * 100.0f;
        if (r < R_MIN || r > R_MAX || pi < PI_MIN || pi > PI_MAX) return;

        float spo2 = -45.060f * r * r + 30.354f * r + 94.845f + WRIST_OFFSET;
        if (spo2 > 100.0f) spo2 = 100.0f;
        if (spo2 < SPO2_MIN) return;

        o.spo2_ready = true;
        o.spo2 = spo2;
        o.pi = pi;
        o.r = r;
    }

    /** Menimbang DC terhadap atap dan lantainya, lalu menggeser arus satu
     *  langkah. Tidak menyentuh perangkat keras — itu tugas pemanggil. */
    void agc(float mean_red, float mean_ir, uint32_t now_ms, PpgOut &o) {
        const uint32_t wait = bpm_valid_ ? AGC_CHECK_MS : AGC_FAST_MS;
        if (agc_at_ms_ != 0 && (now_ms - agc_at_ms_) < wait) return;
        agc_at_ms_ = now_ms;

        bool moved = false;
        moved |= step(led_ir_, mean_ir);
        moved |= step(led_red_, mean_red);
        if (!moved) return;

        o.led_changed = true;

        /* Arus baru berarti DC lama tidak berarti apa-apa lagi, dan amplop
         * puncak detektor diukur dalam satuan AC yang barusan berubah
         * skalanya. Keduanya harus mengejar ulang, dan settle berlaku lagi
         * dari titik ini. */
        dc_init_ = false;
        resetDetector();
        contact_at_ms_ = now_ms;
    }

    static bool step(uint8_t &led, float dc) {
        if (dc > AGC_DC_HIGH && led > AGC_LED_MIN) {
            led = (uint8_t)(led - AGC_STEP < AGC_LED_MIN
                                ? AGC_LED_MIN
                                : led - AGC_STEP);
            return true;
        }
        if (dc < AGC_DC_LOW && led < AGC_LED_MAX) {
            led = (uint8_t)((int)led + AGC_STEP > AGC_LED_MAX
                                ? AGC_LED_MAX
                                : led + AGC_STEP);
            return true;
        }
        return false;
    }

    uint8_t quality(uint32_t ir, uint32_t now_ms) const {
        const float span = PPG_IR_QUALITY_FULL - PPG_IR_PRESENT;
        const float above = (float)ir - PPG_IR_PRESENT;
        uint8_t q = above >= span ? 15
                                  : (uint8_t)((above * 15.0f) / span);
        /* Cahaya banyak bukan sinyal bagus kalau tidak ada denyut di
         * dalamnya. §3.1: angka inilah yang memutuskan apakah nadi yang
         * hilang layak jadi keadaan darurat. */
        const bool stale =
            last_beat_ms_ == 0 || (now_ms - last_beat_ms_) > BEAT_STALE_MS;
        if (stale && q > 4) q = 4;
        return q;
    }

    void resetDetector() {
        smooth_ = slow_ = slow2_ = cardiac_prev_ = 0.0f;
        /* Bukan nol: amplop nol berarti ambang nol, dan sampel derau pertama
         * langsung terhitung sebagai denyut. */
        env_ = 100.0f;
        last_beat_ms_ = 0;
        for (uint8_t i = 0; i < IBI_N; i++) ibi_[i] = 600.0f;
        ibi_i_ = 0;
        ibi_n_ = 0;
        bpm_ = 0.0f;
        bpm_valid_ = false;
        stable_beats_ = 0;
        sq_red_ = sq_ir_ = 0.0;
        sum_dc_red_ = sum_dc_ir_ = 0.0;
        win_n_ = 0;
        win_at_ms_ = 0;
    }

    static constexpr uint8_t IBI_N = 4;
    static constexpr uint8_t MIN_STABLE_BEATS = 3;

    float sps_ = 25.0f;
    float dc_a_ = 0, slow_a_ = 0, smooth_a_ = 0, env_a_ = 0;

    bool worn_prev_ = false, raw_cand_ = false;
    uint32_t raw_cand_at_ms_ = 0, contact_at_ms_ = 0;

    uint8_t led_red_ = AGC_LED_MAX, led_ir_ = AGC_LED_MAX;
    uint32_t agc_at_ms_ = 0;

    bool dc_init_ = false;
    float dc_ir_ = 0, dc_red_ = 0;
    uint32_t last_ir_ = 0;

    float smooth_ = 0, slow_ = 0, slow2_ = 0, cardiac_prev_ = 0, env_ = 100.0f;
    uint32_t last_beat_ms_ = 0;
    float ibi_[IBI_N] = { 600, 600, 600, 600 };
    uint8_t ibi_i_ = 0, ibi_n_ = 0, stable_beats_ = 0;
    float bpm_ = 0;
    bool bpm_valid_ = false;

    double sq_red_ = 0, sq_ir_ = 0, sum_dc_red_ = 0, sum_dc_ir_ = 0;
    int win_n_ = 0;
    uint32_t win_at_ms_ = 0;
};
