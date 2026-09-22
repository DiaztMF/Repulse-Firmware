/*
 * RePulse Band - ESP32-C3
 *
 * Sensor:
 * - MAX30102: BPM, RR, SpO2
 * - MPU6050: gerakan dan posisi tubuh
 *
 * Output:
 * - Motor getar pada GPIO1
 *
 * Input:
 * - Tombol SOS pada GPIO3
 *
 * Board:
 * - ESP32C3 Dev Module
 * - USB CDC On Boot: Enabled
 */

#include <Arduino.h>
#include <Wire.h>
#include <ArduinoJson.h>

#include <MAX30105.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#include "ble_band.h"
#include "ladder.h"
#include "evtbuf.h"
#include "ppg.h"

// =============================================================
// PIN ESP32-C3
// =============================================================

#define PIN_I2C_SDA        10
#define PIN_I2C_SCL         0

#define PIN_BUTTON          3
#define PIN_MOTOR           1

/* GPIO6 dan GPIO7 dulu UART ke layar ESP32-C6. Rangkaian final tidak punya
 * layar — gelang melaporkan lewat BLE saja — jadi keduanya bebas. Kalau
 * nanti dipakai, JANGAN pindah ke GPIO2, GPIO8, atau GPIO9: tiga itu
 * strapping pin, dan GPIO9 rendah saat reset melempar chip ke download mode
 * (tidak ada firmware jalan, serial kosong, LED biru mati). */

#define PIN_ECG_OUT        -1
#define PIN_ECG_LO_P       -1
#define PIN_ECG_LO_N       -1

#define PIN_BATTERY_ADC    -1

// =============================================================
// KOMUNIKASI
// =============================================================

#define SERIAL_BAUD             115200

/* Laju cuplik yang BENAR-BENAR keluar dari FIFO: SAMPLE_RATE / SAMPLE_AVERAGE.
 * ppg.h menurunkan seluruh tetapan waktunya dari angka ini, jadi mengubah
 * rata-rata perangkat keras di initMAX30102() tidak diam-diam menggeser
 * filter mana pun — asalkan angka ini ikut diubah. */
#define PPG_SAMPLE_RATE         100
#define PPG_SAMPLE_AVERAGE        4
#define PPG_SPS                 ((float)PPG_SAMPLE_RATE / PPG_SAMPLE_AVERAGE)

#define SERIAL_HEARTBEAT             1

#define VITALS_INTERVAL_MS        1000
#define MOTION_INTERVAL_MS        1000
#define STATUS_INTERVAL_MS      300000
#define OFFLINE_SUMMARY_MS       60000
#define LOG_INTERVAL_MS           1000

#define SOS_HOLD_MS               2000
#define DISCONNECT_GRACE_MS      30000

// =============================================================
// MAX30102
// =============================================================

/* Deteksi kontak, denyut, dan SpO2 sekarang ada di ppg.h, diadaptasi dari
 * AsaWatch. Ambang IR dan seluruh tetapan waktu ikut pindah ke sana supaya
 * satu tempat saja yang memilikinya — dulu ambangnya di sini sementara
 * filter yang memakainya di bawah, dan keduanya bergeser sendiri-sendiri.
 *
 * Yang hilang dari sini beserta alasannya:
 *   BEAT_CENTRE / BEAT_BASELINE_SHIFT  tambalan untuk batas 16 bit di dalam
 *                                      checkForBeat. Detektor baru bekerja
 *                                      di titik mengambang, jadi tidak ada
 *                                      batas untuk diakali.
 *   IR_WORN_THRESHOLD 5000             tebakan, dan diambil saat arus LED
 *                                      masih 0x7F. Sekarang PPG_IR_PRESENT
 *                                      di ppg.h, pada arus penuh.
 *   SPO2_BUFFER_LEN 100                penyangga untuk algoritma Maxim, yang
 *                                      menaikkan spo2Valid pada derau. Diganti
 *                                      jendela AC-RMS satu detik dengan
 *                                      gerbang kewajaran R dan PI.
 *
 * Untuk mengubah ambang kontak tanpa menyunting ppg.h:
 *   #define PPG_IR_PRESENT 45000.0f   sebelum #include "ppg.h" */
#define IR_CALIBRATION               1

/* Berapa lama BPM dan RR terakhir masih dianggap mewakili sekarang. Milik
 * .ino, bukan ppg.h: yang di sana menjepit kualitas, yang ini memutuskan
 * apakah §3.1 boleh mengirim angkanya sama sekali. */
#define BEAT_STALE_MS             4000

/* Denyut terakhir yang terukur ditahan SELAMA GELANG TERPASANG, dan hilang
 * saat dilepas. Tidak ada batas waktu.
 *
 * Batas tiga puluh detik dulu ada di sini dan sudah dihapus atas permintaan
 * yang jelas: selama gelang di pergelangan, layar harus punya angka. Yang
 * dikorbankan nyata dan perlu ditulis - kalau optiknya berhenti membaca
 * sementara gelangnya tetap terpasang, layar menunjukkan angka lama tanpa
 * mengatakan berapa lama, dan tidak ada apa pun di layar yang membedakan
 * denyut semenit lalu dari denyut sejam lalu.
 *
 * Yang membuatnya masih layak dikerjakan: tidak ada satu pun keputusan yang
 * memakainya. Paket §3.1 menandainya dengan bit 6, jadi aplikasi tahu; skor
 * malam dan denyut istirahat menolak sampel tahanan seluruhnya; dan tangga
 * eskalasi di dalam gelang memakai bpmIsValid() yang tetap ketat dan tidak
 * pernah melihat nilai tahanan sama sekali. Angka ini hanya untuk dilihat
 * manusia, tidak untuk dipercaya mesin. */

// =============================================================
// VALIDASI RR
// =============================================================

#define RR_HISTORY                    8
#define RR_REFERENCE_MIN_COUNT        3
#define RR_IRREGULAR_MIN_COUNT        6

// =============================================================
// MOTOR
// =============================================================

#define MOTOR_GATE_MS               200
#define MOTOR_SOFT_DUTY             110
#define MOTOR_HARD_DUTY             255

/* Sentakan awal, dan berapa lama ia bertahan.
 *
 * Getaran halus tidak pernah terasa sementara yang keras selalu terasa,
 * dan alasannya bukan di logika: 110 dari 255 pada 8 bit adalah 43%,
 * yang di rel 3,3 V rata-ratanya sekitar 1,4 V. Motor ERM koin butuh
 * kira-kira 2,3 V untuk MULAI berputar dari diam. Jadi perintahnya
 * terkirim, PWM-nya benar, dan porosnya tidak pernah lepas dari gesekan
 * statis. Yang keras jalan hanya karena 255 berarti tegangan penuh.
 *
 * Sekali berputar, motor bertahan di duty yang jauh lebih rendah daripada
 * yang dibutuhkannya untuk mulai. Jadi setiap getaran kini dimulai dengan
 * sentakan penuh, lalu turun ke dutynya yang sebenarnya.
 *
 * ponytail: 60 ms cukup untuk motor koin yang biasa; kalau motormu lebih
 * berat dan masih diam, naikkan ini dulu sebelum menyentuh SOFT_DUTY. */
#define MOTOR_KICK_MS                60
#define MOTOR_PWM_FREQ             5000
#define MOTOR_PWM_BITS                8

// =============================================================
// ECG
// =============================================================

#define ECG_SAMPLE_HZ               250
#define ECG_SAMPLES_PER_PACKET       90

// =============================================================
// BUFFER OFFLINE
// =============================================================

#define FLUSH_PACKET_GAP_MS           30
#define FLUSH_SENTINEL_RETRY_MS     2000

// =============================================================
// KONFIGURASI BAND
// =============================================================

struct BandConfig {
    uint8_t baseline_bpm = 62;
    uint8_t hr_threshold_delta = 16;

    float rr_variability_threshold = 0.18f;

    uint16_t spo2_sample_interval_s = 20;

    // Batas fisik interval RR yang diterima.
    uint16_t rr_min_ms = 300;
    uint16_t rr_max_ms = 2000;

    /* Berapa lama keadaan di luar ambang harus BERTAHAN sebelum tangga
     * dinaikkan.
     *
     * Sebelumnya nol: satu sampel di atas ambang langsung memulai tahap 1.
     * Di pergelangan yang ikatannya belum kencang, avgBPM bergoyang
     * beberapa denyut setiap beberapa detik, jadi satu lonjakan sesaat
     * cukup untuk membangunkan seluruh sistem. Alarm klinis mana pun
     * menuntut keadaannya bertahan dulu, justru karena sensor selalu lebih
     * berisik daripada tubuh yang diukurnya. */
    uint16_t anomaly_hold_s = 12;

    /* Kualitas sinyal minimum sebelum angka denyut boleh dipakai untuk
     * MEMUTUSKAN. Lebih ketat daripada rr_min_quality, yang hanya menjaga
     * satu interval RR: keputusan ini membangunkan orang.
     *
     * Kenapa 5 dan bukan angka lain: quality() menahan nilainya di 4 kalau
     * tidak ada denyut dalam 4 detik terakhir. Jadi 5 adalah angka
     * TERKECIL yang tidak bisa dipalsukan oleh cahaya terang tanpa denyut
     * di dalamnya — ia otomatis berarti "ada denyut baru DAN perfusinya
     * cukup". Menaikkannya lebih tinggi menuntut IR yang di pergelangan
     * belum tentu tercapai, dan gelang yang diam sepanjang malam adalah
     * kegagalan yang jauh lebih berbahaya daripada alarm palsu. Bisa
     * disetel dari aplikasi kalau kamar dan kulitnya bicara lain. */
    uint8_t anomaly_min_quality = 5;

    /* 0 = deteksi anomali dimatikan sementara. Tombol SOS dan seluruh
     * perekaman malam tetap jalan; yang berhenti hanya tangga yang naik
     * sendiri. Ada untuk demo dan untuk pemasangan ulang sensor, dan
     * aplikasi menampilkan peringatan selama ini menyala. */
    uint8_t anomaly_enabled = 1;

    // Syarat kualitas pembacaan RR.
    uint8_t rr_min_quality = 3;
    uint16_t rr_motion_limit_mg = 200;

    // Setelah ada referensi RR, interval baru harus berada
    // di antara 60% sampai 160% median RR sebelumnya.
    uint8_t rr_min_ratio_percent = 60;
    uint8_t rr_max_ratio_percent = 160;
};

static BandConfig cfg;

// =============================================================
// OBJEK HARDWARE
// =============================================================

static MAX30105 maxSensor;

/* Seluruh DSP optik. Murni dan teruji di komputer — test/ppg_test.cpp. */
static Ppg ppg(PPG_SPS);
static Adafruit_MPU6050 mpu6050;

static Ladder ladder;
static EventBuffer offline;

// =============================================================
// STATUS HARDWARE
// =============================================================

static bool max30102Ready = false;
static bool mpu6050Ready = false;

// =============================================================
// WAKTU
// =============================================================

static uint32_t epoch_at_sync = 0;
static uint32_t millis_at_sync = 0;

// =============================================================
// DATA VITAL
// =============================================================

static bool worn = false;

static uint8_t opticalQuality = 0;
static uint8_t signalQuality = 0;

// Puncak terakhir digunakan untuk menghitung interval kandidat.
static uint32_t previousPeakMs = 0;

// Detak terakhir yang lolos pemeriksaan dasar.
static uint32_t lastBeatMs = 0;

static float avgBPM = 0.0f;

/* Denyut terakhir yang benar-benar lolos seluruh rantai pemeriksaan, dan
 * kapan ia lolos. Hanya untuk ditampilkan.
 *
 * `heldAtMs` tidak lagi dipakai sebagai kedaluwarsa - lihat catatan di
 * bagian atas. Disimpan karena log kalibrasi memakainya untuk menjawab
 * "sudah berapa lama angka ini tidak berubah", yang tepat pertanyaan yang
 * tidak bisa dijawab dari layar. */
static uint8_t heldBPM = 0;
static uint32_t heldAtMs = 0;

/* Satu paket punya satu byte status, jadi bendera held berlaku untuk
 * seluruh paket. Ini menandai bahwa ada sampel di dalamnya yang denyutnya
 * diingat, bukan diukur. */
static bool heldInBatch = false;
static uint8_t bpmBeatCount = 0;

static uint16_t lastRRms = 0;
static uint32_t lastValidRRAtMs = 0;
static bool rrValid = false;

static uint16_t rrHistory[RR_HISTORY] = {0};
static uint8_t rrCount = 0;
static uint8_t rrIdx = 0;

// Maksimum lima sampel detak per paket BLE.
static uint8_t vitalsBatch[5][3];
static uint8_t vitalsCount = 0;

// =============================================================
// DATA SPO2
// =============================================================

/* Garis dasar DC lambat untuk detektor denyut. 0 = belum ada sampel. */

/* Kapan jendela SpO2 terakhir lahir. Jarak kirim 20 detik lebih panjang
 * daripada umur satu jendela, jadi tanpa ini angka lama bisa terkirim
 * sebagai angka sekarang. */
static uint32_t lastSpo2WindowMs = 0;
#define SPO2_FRESH_MS 5000


static uint8_t spo2Value = 0;
static bool spo2ValueValid = false;

// =============================================================
// POSISI
// =============================================================

// 0 = telentang
// 1 = miring kiri
// 2 = miring kanan
// 3 = tengkurap
// 255 = tidak diketahui
static uint8_t bodyPosition = 255;

// =============================================================
// DATA GERAKAN
// =============================================================

static uint16_t motionMilliG = 0;

// =============================================================
// MOTOR
// =============================================================

static uint32_t motorOffAtMs = 0;
static bool motorOn = false;

// =============================================================
// TOMBOL SOS
// =============================================================

/* Tidak lagi volatile: tidak ada interrupt yang menyentuhnya. Lihat
 * buttonLoop() untuk alasannya. */
static bool     buttonRaw    = false;   // bacaan pin apa adanya
static uint32_t buttonEdgeMs = 0;       // kapan bacaan mentah terakhir berubah
static bool     buttonHeld   = false;   // keadaan yang sudah tenang
static uint32_t buttonDownMs = 0;       // kapan tekanan yang tenang dimulai
static bool     sosLatched   = false;

// =============================================================
// ECG
// =============================================================

static bool ecgActive = false;
static uint32_t ecgStopAtMs = 0;
static uint32_t ecgNextSampleUs = 0;

static int16_t ecgSamples[ECG_SAMPLES_PER_PACKET];

static uint8_t ecgFilled = 0;
static uint8_t ecgSeq = 0;

static bool ecgLeadOn = false;

// =============================================================
// TIMER
// =============================================================

static uint32_t lastVitalsMs = 0;
static uint32_t lastMotionMs = 0;
static uint32_t lastStatusMs = 0;
static uint32_t lastSummaryMs = 0;
static uint32_t lastLogMs = 0;
static uint32_t lastSpo2Ms = 0;

static uint32_t disconnectedSinceMs = 0;
static bool wasConnected = false;

// =============================================================
// STATUS VALIDITAS
// =============================================================

static bool bpmIsValid() {
    return
        worn &&
        bpmBeatCount >= 3 &&
        avgBPM >= 1.0f &&
        lastBeatMs != 0 &&
        millis() - lastBeatMs <= BEAT_STALE_MS;
}

static bool rrIsValid() {
    return
        worn &&
        rrValid &&
        lastRRms != 0 &&
        lastValidRRAtMs != 0 &&
        millis() - lastValidRRAtMs <= BEAT_STALE_MS;
}

// =============================================================
// RESET DATA VITAL
// =============================================================

static void resetVitalData() {
    previousPeakMs = 0;
    lastBeatMs = 0;

    avgBPM = 0.0f;
    bpmBeatCount = 0;

    lastRRms = 0;
    lastValidRRAtMs = 0;
    rrValid = false;

    memset(
        rrHistory,
        0,
        sizeof(rrHistory)
    );

    rrCount = 0;
    rrIdx = 0;

    vitalsCount = 0;

    /* Kontak hilang berarti tidak ada lagi yang layak ditahan. Menahan
     * denyut seseorang setelah gelangnya dilepas adalah angka yang bukan
     * milik siapa pun. */
    heldBPM = 0;
    heldAtMs = 0;
    heldInBatch = false;
}

/**
 * Irama hilang, denyutnya tidak.
 *
 * Satu jarak yang terlalu renggang dulu memanggil resetVitalData() dan
 * membuang SEMUANYA: rata-rata, hitungan denyut, seluruh riwayat RR. Lalu
 * bpmIsValid() menuntut tiga denyut baru sebelum mau menjawab lagi.
 *
 * Di pergelangan, denyut yang terlewat itu peristiwa rutin - satu puncak
 * tertelan gerakan dan jaraknya langsung jadi dua kali lipat. Jadi satu
 * kedipan optik membuang pengetahuan satu menit, tiga denyut kemudian ia
 * pulih, lalu terlewat lagi. Itulah diam yang datang dan pergi, dan itu
 * bukan sensornya - itu baris ini.
 *
 * Yang benar-benar tidak lagi dipercaya setelah jeda panjang adalah
 * IRAMANYA: median RR dan riwayatnya tidak lagi menggambarkan apa pun.
 * Rata-rata denyutnya masih menggambarkan orang yang sama.
 */
static void resyncRhythm() {
    lastRRms = 0;
    lastValidRRAtMs = 0;
    rrValid = false;

    memset(
        rrHistory,
        0,
        sizeof(rrHistory)
    );

    rrCount = 0;
    rrIdx = 0;
}

// =============================================================
// WAKTU EPOCH
// =============================================================

static uint32_t nowEpoch() {
    if (epoch_at_sync == 0) {
        return 0;
    }

    return epoch_at_sync +
           (millis() - millis_at_sync) / 1000u;
}

// =============================================================
// MOTOR GETAR
// =============================================================

static uint8_t  motorHoldDuty   = 0;
static uint32_t motorKickAtMs   = 0;

static void vibrate(
    bool hard,
    uint32_t duration_ms
) {
    motorHoldDuty = hard
        ? MOTOR_HARD_DUTY
        : MOTOR_SOFT_DUTY;

    // Selalu berangkat dari tegangan penuh, apa pun duty tujuannya.
    ledcWrite(
        PIN_MOTOR,
        MOTOR_HARD_DUTY
    );

    motorOn = true;
    motorKickAtMs = millis() + MOTOR_KICK_MS;
    motorOffAtMs = millis() + duration_ms;
}

static void motorLoop() {
    if (!motorOn) {
        return;
    }

    /* Turun dari sentakan ke duty sebenarnya. Nol berarti sudah turun,
     * jadi getaran keras melewati langkah ini tanpa menulis apa pun. */
    if (
        motorKickAtMs != 0 &&
        (int32_t)(millis() - motorKickAtMs) >= 0
    ) {
        motorKickAtMs = 0;

        if (motorHoldDuty != MOTOR_HARD_DUTY) {
            ledcWrite(
                PIN_MOTOR,
                motorHoldDuty
            );
        }
    }

    if ((int32_t)(millis() - motorOffAtMs) < 0) {
        return;
    }

    ledcWrite(
        PIN_MOTOR,
        0
    );

    motorOn = false;
    motorKickAtMs = 0;
}

static bool motionTrusted() {
    if (motorOn) {
        return false;
    }

    return
        (uint32_t)(millis() - motorOffAtMs) >
        MOTOR_GATE_MS;
}

// =============================================================
// TOMBOL SOS
// =============================================================

/* Dulu ini interrupt, dan itulah sebabnya tombolnya kadang "ditekan tapi
 * tidak terjadi apa-apa".
 *
 * ISR-nya memanggil digitalRead() saat ISR BERJALAN, bukan membaca tepi yang
 * memicunya. Sakelar mekanis memantul 1-20 ms, jadi interrupt bisa terpicu
 * oleh tepi turun lalu membaca pin yang sudah memantul kembali ke HIGH:
 * buttonHeld tertulis false padahal jari masih menekan. Tidak ada tepi lain
 * yang akan datang untuk membetulkannya - tombol itu mati sampai dilepas
 * dan ditekan ulang, dan orang yang menekannya tidak punya cara tahu.
 *
 * Polling tidak bisa desinkron dengan pin, karena ia MEMBACA pin. loop()
 * berputar jauh lebih cepat dari 25 ms, dan sekarang tidak ada lagi
 * volatile, IRAM_ATTR, atau millis() di dalam interrupt. */
#define BUTTON_DEBOUNCE_MS 25

static void buttonLoop() {
    const uint32_t now = millis();

    const bool raw =
        digitalRead(PIN_BUTTON) == LOW;

    if (raw != buttonRaw) {
        buttonRaw = raw;
        buttonEdgeMs = now;
    }

    // Masih memantul; bacaannya belum layak dipercaya.
    const bool settled =
        (now - buttonEdgeMs) >= BUTTON_DEBOUNCE_MS;

    if (settled && raw != buttonHeld) {
        buttonHeld = raw;

        if (buttonHeld) {
            buttonDownMs = now;

            /* Satu ketukan pendek supaya jari tahu tekanannya terbaca dan
             * layak ditahan. Tanpa ini, dua detik pertama tidak bisa
             * dibedakan dari tombol rusak - dan orang melepasnya di detik
             * pertama, persis seperti yang terjadi. */
            vibrate(
                false,
                80
            );

            /* Dicetak di sini, bukan hanya saat SOS berangkat: kalau
             * baris ini tidak pernah muncul, masalahnya kabel atau pin,
             * bukan lama tahan. */
            Serial.println(
                "[SOS] Tombol ditekan, tahan 2 detik"
            );
        } else {
            if (!sosLatched) {
                Serial.printf(
                    "[SOS] Dilepas setelah %lu ms, belum cukup\n",
                    (unsigned long)(now - buttonDownMs)
                );
            }

            sosLatched = false;
        }
    }

    if (!buttonHeld || sosLatched) {
        return;
    }

    if (now - buttonDownMs < SOS_HOLD_MS) {
        return;
    }

    sosLatched = true;

    Serial.println(
        "[SOS] Tombol ditahan >= 2 detik"
    );

    BLE_IndicateSos();

    offline.push(
        nowEpoch(),
        EVT_SOS
    );

    ladder.manualSos(
        millis()
    );

    vibrate(
        true,
        400
    );
}

// =============================================================
// INISIALISASI MAX30102
// =============================================================

static void initMAX30102() {
    Serial.print(
        F("[HR] MAX30102...")
    );

    if (!maxSensor.begin(
            Wire,
            I2C_SPEED_FAST
        )) {
        Serial.println(
            F(" TIDAK DITEMUKAN")
        );

        return;
    }

    /* Setelan AsaWatch, yang diuji di pergelangan pada belasan subjek.
     *
     * Catatan lama di sini memperingatkan bahwa menaikkan arus LED ke 0xFF
     * pernah membuat `worn` jatuh ke nol — dan peringatan itu benar, tetapi
     * bukan karena arusnya. Empat hal diubah sekaligus waktu itu, dan salah
     * satunya memperkecil rentang ADC ke 4096 nA: pada arus penuh itu justru
     * menjenuhkan ADC, jadi sinyalnya terpotong rata dan tidak ada AC yang
     * tersisa untuk dideteksi. Di sini rentangnya 16384 nA — headroom
     * terbesar — justru KARENA arusnya penuh.
     *
     * Arus penuh memang yang dibutuhkan pergelangan: cahaya menembus kulit
     * lebih tebal, lewat tendon dan tulang, dan DC-nya turun satu orde
     * dibanding ujung jari. Ambang kontak di ppg.h (30000) mengandaikan
     * arus ini; keduanya harus berubah bersama. */
    maxSensor.setup(
        0xFF,                 // arus penuh — sinyal pergelangan lemah
        PPG_SAMPLE_AVERAGE,   // rata-rata perangkat keras
        2,                    // RED + IR
        PPG_SAMPLE_RATE,
        411,                  // lebar pulsa -> ADC 18 bit
        16384                 // rentang ADC: headroom terbesar
    );

    maxSensor.setPulseAmplitudeRed(0xFF);
    maxSensor.setPulseAmplitudeIR(0xFF);

    ppg.configure(PPG_SPS);

    max30102Ready = true;

    Serial.printf(
        " OK (%d sampel/detik, ambang kontak %.0f, koreksi SpO2 %+.1f%s)\n",
        (int)PPG_SPS,
        (double)ppg.threshold(),
        (double)ppg.wristOffset(),
        ppg.wristOffset() == 0.0f ? " = MODE JARI" : " = pergelangan"
    );
}


// =============================================================
// WORN DAN SIGNAL QUALITY
// =============================================================

static void applyPpg(const PpgOut &o) {
    worn = o.worn;

    /* Satu angka, dua pemakai. §3.1 mengirim signalQuality; gerbang RR di
     * bawah memakai opticalQuality. Dulu keduanya dihitung terpisah di sini
     * dari ambang yang sama, jadi keduanya selalu sama saja — sekarang
     * keduanya memang satu. */
    opticalQuality = o.quality;
    signalQuality = o.quality;

    if (o.worn_changed && !o.worn) {
        resetVitalData();
        spo2Value = 0;
        spo2ValueValid = false;
    }

#if IR_CALIBRATION
    /* Satu-satunya cara mengukur ambang kontak di pergelangan yang
     * sebenarnya akan memakainya, dan ppg.h menunjuk ke baris ini.
     *
     * Dicetak SELALU, bukan hanya saat menempel: angka saat TIDAK menempel
     * adalah setengah dari yang dibutuhkan. Baca IR di pergelangan dan di
     * atas meja, lalu taruh PPG_IR_PRESENT di tengah keduanya. */
    static uint32_t lastCalMs = 0;

    /* PI dan R hanya lahir sekali per detik, saat satu jendela SpO2 selesai;
     * baris ini dicetak dua kali sedetik. Tanpa menyimpannya, hampir setiap
     * baris menampilkan 0,000 — dan pembacanya akan menyimpulkan sinyalnya
     * mati padahal jendelanya cuma belum jatuh di sampel yang sama. */
    static float lastPi = 0.0f;
    static float lastR = 0.0f;

    if (o.spo2_ready) {
        lastPi = o.pi;
        lastR = o.r;
    }

    if (millis() - lastCalMs >= 500) {
        lastCalMs = millis();

        /* Umur angka tahanan ikut dicetak, dan itu bukan hiasan.
         *
         * Nilai tahanan sekarang bertahan selama gelang terpasang, tanpa
         * batas waktu - jadi layar tidak bisa lagi membedakan denyut
         * semenit lalu dari denyut sejam lalu. Baris ini bisa. Kalau
         * `ditahan` terus bertambah sementara gelangnya jelas terpasang,
         * itu berarti optiknya berhenti membaca dan angka di layar sudah
         * jadi kenangan. */
        Serial.printf(
            "[CAL] IR=%lu  ambang=%.0f  dipakai=%s  kualitas=%u/15  "
            "BPM=%.1f%s  tahan=%u(%lus)  PI=%.3f  R=%.4f\n",
            (unsigned long)ppg.ir(),
            (double)ppg.threshold(),
            o.worn ? "YA" : "TIDAK",
            o.quality,
            (double)o.bpm,
            o.bpm_valid ? "" : "(belum)",
            heldBPM,
            (unsigned long)(
                heldAtMs == 0
                ? 0
                : (millis() - heldAtMs) / 1000u
            ),
            (double)lastPi,
            (double)lastR
        );
    }
#endif
}

// =============================================================
// RIWAYAT RR
// =============================================================

static void pushRR(
    uint16_t rr_ms
) {
    rrHistory[rrIdx] = rr_ms;

    rrIdx =
        (rrIdx + 1) %
        RR_HISTORY;

    if (rrCount < RR_HISTORY) {
        rrCount++;
    }
}

static uint16_t rrMedian() {
    if (rrCount == 0) {
        return 0;
    }

    uint16_t values[RR_HISTORY];

    for (uint8_t i = 0; i < rrCount; i++) {
        values[i] = rrHistory[i];
    }

    for (uint8_t i = 1; i < rrCount; i++) {
        uint16_t key = values[i];
        int j = i - 1;

        while (
            j >= 0 &&
            values[j] > key
        ) {
            values[j + 1] = values[j];
            j--;
        }

        values[j + 1] = key;
    }

    if (rrCount & 1U) {
        return values[rrCount / 2];
    }

    return (uint16_t)(
        (
            (uint32_t)values[rrCount / 2 - 1] +
            (uint32_t)values[rrCount / 2]
        ) /
        2UL
    );
}

static bool rrConsistentWithHistory(
    uint16_t rr_ms
) {
    if (rrCount < RR_REFERENCE_MIN_COUNT) {
        return true;
    }

    uint16_t reference =
        rrMedian();

    if (reference == 0) {
        return true;
    }

    uint32_t ratio =
        (uint32_t)rr_ms *
        100UL /
        reference;

    return
        ratio >= cfg.rr_min_ratio_percent &&
        ratio <= cfg.rr_max_ratio_percent;
}

static float rrVariability() {
    if (rrCount < 3) {
        return 0.0f;
    }

    uint8_t start =
        rrCount == RR_HISTORY
        ? rrIdx
        : 0;

    uint32_t sum = 0;
    uint32_t difference = 0;

    uint16_t previous =
        rrHistory[start];

    sum += previous;

    for (uint8_t i = 1; i < rrCount; i++) {
        uint8_t index =
            (start + i) %
            RR_HISTORY;

        uint16_t current =
            rrHistory[index];

        sum += current;

        int32_t delta =
            (int32_t)current -
            (int32_t)previous;

        difference +=
            delta < 0
            ? (uint32_t)(-delta)
            : (uint32_t)delta;

        previous = current;
    }

    float mean =
        (float)sum /
        (float)rrCount;

    if (mean < 1.0f) {
        return 0.0f;
    }

    return
        (
            (float)difference /
            (float)(rrCount - 1)
        ) /
        mean;
}

// =============================================================
// PENGUMPULAN SAMPEL SPO2
// =============================================================


// =============================================================
// TAMBAH DATA BLE VITAL
// =============================================================

/** Ada denyut yang layak ditampilkan. Terpasang dan pernah terukur; lihat
 *  catatan di atas tentang apa yang sengaja TIDAK diperiksa di sini. */
static bool heldFresh() {
    return
        worn &&
        heldBPM > 0;
}

static void appendVitalSample(
    float bpm,
    uint16_t rr_ms
) {
    if (vitalsCount >= 5) {
        return;
    }

    uint8_t out =
        (uint8_t)(
            bpm > 255.0f
            ? 255
            : bpm
        );

    /* Lubang terakhir yang membuat angkanya berkedip.
     *
     * Kedua pemanggil mengirim o.bpm tanpa memeriksa o.bpm_valid, jadi di
     * detik-detik awal setelah kulit menyentuh sensor, denyut TERDETEKSI
     * tetapi rata-ratanya belum jadi - dan yang terkirim adalah sampel
     * SEGAR bernilai nol. Paketnya punya isi, jadi jalur tahanan tidak
     * pernah dipakai, dan nol itu sampai ke layar sebagai "tidak ada
     * denyut".
     *
     * Intervalnya tetap dikirim apa adanya: RR adalah pengukuran yang sah
     * walau rata-rata denyutnya belum ada, dan HRV memakainya. Yang
     * disubstitusi hanya angka denyutnya, dan paketnya mengaku. */
    if (out == 0 && heldFresh()) {
        out = heldBPM;
        heldInBatch = true;
    }

    vitalsBatch[vitalsCount][0] = out;

    vitalsBatch[vitalsCount][1] =
        (uint8_t)(
            rr_ms & 0xFF
        );

    vitalsBatch[vitalsCount][2] =
        (uint8_t)(
            rr_ms >> 8
        );

    vitalsCount++;
}

// =============================================================
// PEMROSESAN SAMPEL MAX30102
// =============================================================

static void handleSample(
    uint32_t ir,
    uint32_t red
) {
    const PpgOut o =
        ppg.feed(
            ir,
            red,
            millis()
        );

    applyPpg(o);

    /* AGC memutuskan, .ino yang menulis. ppg.h sengaja tidak tahu apa-apa
     * tentang MAX30105 — itu yang membuatnya bisa diuji di komputer. */
    if (o.led_changed) {
        maxSensor.setPulseAmplitudeRed(ppg.ledRed());
        maxSensor.setPulseAmplitudeIR(ppg.ledIr());

        Serial.printf(
            "[AGC] arus LED disesuaikan: Red=0x%02X IR=0x%02X"
            " (DC menyentuh atap ADC), menstabilkan ulang\n",
            ppg.ledRed(),
            ppg.ledIr()
        );
    }

    /* Satu jendela SpO2 selesai. Gerbang kewajaran R dan PI sudah dilewati
     * di dalam ppg.h, jadi apa pun yang sampai di sini layak dikirim —
     * berbeda dari algoritma Maxim sebelumnya, yang menaikkan spo2Valid
     * pada derau dan membuat saturasi terbaca 16% berjam-jam. */
    if (o.spo2_ready) {
        spo2Value =
            (uint8_t)(o.spo2 + 0.5f);

        spo2ValueValid = true;
        lastSpo2WindowMs = millis();
    }

    if (!o.beat) {
        return;
    }

    uint32_t now =
        millis();

    uint32_t rawDelta =
        o.rr_ms;

    /* Denyut pertama satu sesi tidak punya pasangan, jadi tidak ada jarak
     * untuk dinilai. ppg.h menandainya dengan rr_ms nol. */
    if (rawDelta == 0) {
        previousPeakMs = now;
        return;
    }

    previousPeakMs = now;

    // Puncak terlalu rapat kemungkinan puncak palsu.
    if (rawDelta < cfg.rr_min_ms) {
        rrValid = false;
        lastRRms = 0;

#if IR_CALIBRATION
        Serial.printf(
            "[DENYUT] jarak=%lu ms  DITOLAK: terlalu rapat\n",
            (unsigned long)rawDelta
        );
#endif
        return;
    }

    // Terlalu renggang: dipakai untuk sinkronisasi ulang, tidak untuk BPM.
    if (rawDelta > cfg.rr_max_ms) {
        resyncRhythm();

#if IR_CALIBRATION
        Serial.printf(
            "[DENYUT] jarak=%lu ms  DITOLAK: terlalu renggang\n",
            (unsigned long)rawDelta
        );
#endif
        return;
    }

    bool signalValid =
        opticalQuality >=
            cfg.rr_min_quality &&
        motionTrusted() &&
        motionMilliG <=
            cfg.rr_motion_limit_mg;

    if (!signalValid) {
        rrValid = false;
        lastRRms = 0;

#if IR_CALIBRATION
        Serial.printf(
            "[DENYUT] jarak=%lu ms  DITOLAK: sinyal kurang "
            "(kualitas=%u, gerak=%u mG)\n",
            (unsigned long)rawDelta,
            opticalQuality,
            motionMilliG
        );
#endif
        return;
    }

    lastBeatMs = now;

    if (bpmBeatCount < 255) {
        bpmBeatCount++;
    }

    float bpm =
        o.bpm;

    bool intervalValid =
        rrConsistentWithHistory(
            (uint16_t)rawDelta
        );

    /* BPM hanya bergerak dari jarak yang SUDAH lolos pemeriksaan riwayat.
     *
     * Dulu baris ini ada di atas pemeriksaan itu, dan akibatnya terlihat di
     * perangkat: detektor melewatkan satu denyut, jaraknya jadi dua kali
     * lipat, dan rata-rata empat denyut di ppg.h ikut terseret — BPM
     * tercetak 43,4 lalu 149,6 pada jari yang denyutnya sekitar 85. Jarak
     * seperti itu memang sudah ditolak untuk dikirim sebagai RR, tapi
     * angka BPM-nya terlanjur ikut tercemar sebelum penolakannya dibaca.
     *
     * Kalau sebuah jarak ditolak, BPM tidak bergerak sama sekali. Itu
     * jawaban yang benar: kami tidak tahu, dan diam lebih baik daripada
     * menebak. */
    if (intervalValid) {
        avgBPM =
            avgBPM < 1.0f
            ? bpm
            : (
                avgBPM * 0.8f +
                bpm * 0.2f
            );

        /* Diambil dari rata-rata, bukan dari denyut tunggal yang baru
         * masuk: yang ditahan nanti harus mewakili satu menit terakhir,
         * bukan satu puncak terakhir. */
        if (avgBPM >= 1.0f) {
            heldBPM =
                (uint8_t)(
                    avgBPM > 255.0f
                    ? 255
                    : avgBPM
                );

            heldAtMs = now;
        }
    }

    if (!intervalValid) {
        rrValid = false;
        lastRRms = 0;

        // BPM tetap dikirim; RR nol menandakan interval ini tidak dipakai.
        appendVitalSample(
            bpm,
            0
        );

#if IR_CALIBRATION
        Serial.printf(
            "[DENYUT] jarak=%lu ms  DITOLAK: menyimpang dari "
            "riwayat (tengah=%u ms)\n",
            (unsigned long)rawDelta,
            rrMedian()
        );
#endif
        return;
    }

    lastRRms =
        (uint16_t)rawDelta;

    lastValidRRAtMs = now;
    rrValid = true;

    pushRR(
        lastRRms
    );

    appendVitalSample(
        bpm,
        lastRRms
    );
}

static void pumpSensor() {
    if (!max30102Ready) {
        return;
    }

    maxSensor.check();

    while (maxSensor.available()) {
        uint32_t ir =
            maxSensor.getIR();

        uint32_t red =
            maxSensor.getRed();

        maxSensor.nextSample();

        handleSample(
            ir,
            red
        );
    }
}

// =============================================================
// SPO2
// =============================================================

static void spo2Loop() {
    if (!max30102Ready) {
        return;
    }

    /* §3.2 mengirim saturasi setiap `spo2_sample_interval_s`, dan yang
     * dikirim adalah jendela terbaru dari ppg.h.
     *
     * Dulu di sini ada mesin pengumpul: kumpulkan 100 sampel, panggil
     * algoritma Maxim, nilai hasilnya. Semua itu hilang karena ppg.h sudah
     * menghasilkan satu jendela per detik sepanjang kulit menempel — yang
     * tersisa tinggal memutuskan kapan mengirimkannya.
     *
     * Jendela yang basi tidak dikirim. Saturasi dari dua menit lalu bukan
     * saturasi sekarang, dan §3.2 tidak punya cara mengatakan "ini lama". */
    const uint32_t now = millis();

    if (now - lastSpo2Ms <
        (uint32_t)cfg.spo2_sample_interval_s * 1000UL) {
        return;
    }

    lastSpo2Ms = now;

    const bool fresh =
        spo2ValueValid &&
        worn &&
        now - lastSpo2WindowMs <= SPO2_FRESH_MS;

    if (!fresh) {
        spo2Value = 0;
        spo2ValueValid = false;
    }

    Serial.printf(
        "[SPO2] saturasi=%u%%  sah=%s\n",
        spo2Value,
        spo2ValueValid ? "YA" : "TIDAK"
    );

    BLE_NotifyOxygen(
        spo2Value,
        bodyPosition
    );
}

// =============================================================
// MPU6050
// =============================================================

static void initMPU6050() {
    Serial.print(
        F("[IMU] MPU6050...")
    );

    if (!mpu6050.begin(
            0x68,
            &Wire
        )) {
        Serial.println(
            F(" TIDAK DITEMUKAN")
        );

        return;
    }

    mpu6050.setAccelerometerRange(
        MPU6050_RANGE_4_G
    );

    mpu6050.setGyroRange(
        MPU6050_RANGE_500_DEG
    );

    mpu6050.setFilterBandwidth(
        MPU6050_BAND_5_HZ
    );

    mpu6050Ready = true;

    Serial.println(
        F(" OK")
    );
}

// =============================================================
// POSISI TUBUH
// =============================================================

static uint8_t positionFrom(
    float ax,
    float ay,
    float az
) {
    float absX = fabsf(ax);
    float absY = fabsf(ay);
    float absZ = fabsf(az);

    if (
        absZ > absX &&
        absZ > absY
    ) {
        return az > 0
            ? 0
            : 3;
    }

    if (absX > absY) {
        return ax > 0
            ? 1
            : 2;
    }

    return 255;
}

/* §3.2 kontrak GATT: 0 telentang, 1 miring kiri, 2 miring kanan,
 * 3 tengkurap, 255 tidak diketahui. */
static const char* positionLabel(
    uint8_t p
) {
    switch (p) {
        case 0:  return "telentang";
        case 1:  return "miring-kiri";
        case 2:  return "miring-kanan";
        case 3:  return "tengkurap";
        default: return "belum-tahu";
    }
}

// =============================================================
// LOOP GERAKAN MPU6050
// =============================================================

static void motionLoop() {
    if (
        !mpu6050Ready ||
        !motionTrusted()
    ) {
        return;
    }

    static uint32_t lastSampleMs = 0;

    if (
        millis() -
        lastSampleMs <
        20
    ) {
        return;
    }

    lastSampleMs = millis();

    sensors_event_t acceleration;
    sensors_event_t gyro;
    sensors_event_t temperature;

    mpu6050.getEvent(
        &acceleration,
        &gyro,
        &temperature
    );

    float ax =
        acceleration.acceleration.x /
        SENSORS_GRAVITY_STANDARD;

    float ay =
        acceleration.acceleration.y /
        SENSORS_GRAVITY_STANDARD;

    float az =
        acceleration.acceleration.z /
        SENSORS_GRAVITY_STANDARD;

    float magnitude =
        sqrtf(
            ax * ax +
            ay * ay +
            az * az
        );

    float deltaG =
        fabsf(
            magnitude -
            1.0f
        );

    uint32_t milliG =
        (uint32_t)(
            deltaG *
            1000.0f
        );

    motionMilliG =
        (uint16_t)(
            milliG > 65535
            ? 65535
            : milliG
        );

    bodyPosition =
        positionFrom(
            ax,
            ay,
            az
        );

    ladder.motion(
        millis(),
        motionMilliG
    );
}

// =============================================================
// DETEKSI ANOMALI
// =============================================================

/* Sejak kapan keadaan di luar ambang bertahan tanpa putus. 0 = tidak ada. */
static uint32_t anomalySinceMs = 0;

static void anomalyLoop() {
    /* Dimatikan dari aplikasi. Tangga yang sedang berjalan ikut diturunkan,
     * kalau tidak sakelarnya hanya mencegah alarm BERIKUTNYA dan
     * meninggalkan yang sekarang berbunyi tanpa penjelasan. */
    if (!cfg.anomaly_enabled) {
        anomalySinceMs = 0;
        ladder.clear(millis());
        return;
    }

    /* Kualitas sinyal ikut jadi syarat.
     *
     * bpmIsValid() menjawab "angkanya ada", bukan "angkanya layak dipercaya
     * untuk membangunkan orang". Gelang yang longgar tetap menghasilkan
     * angka; angka itu yang bergoyang. Keputusan ini menuntut sinyal yang
     * benar-benar bagus, dan diam saat sensornya sedang ragu. */
    if (!bpmIsValid() || signalQuality < cfg.anomaly_min_quality) {
        anomalySinceMs = 0;
        ladder.clear(
            millis()
        );

        return;
    }

    int16_t deviation =
        (int16_t)avgBPM -
        (int16_t)cfg.baseline_bpm;

    bool outOfBand =
        abs(deviation) >
        (int16_t)
        cfg.hr_threshold_delta;

    // Irregular hanya boleh diperiksa jika:
    // - jumlah RR valid sudah cukup;
    // - interval RR terakhir valid;
    // - data bukan data lama.
    bool irregular =
        rrCount >=
            RR_IRREGULAR_MIN_COUNT &&
        rrIsValid() &&
        rrVariability() >
            cfg.rr_variability_threshold;

    /* Harus bertahan, bukan sekadar terjadi.
     *
     * Jam mulai dipasang pada sampel pertama yang di luar ambang dan
     * dihapus oleh sampel pertama yang kembali normal, jadi apa pun yang
     * berkedip-kedip tidak pernah sampai ke tangga. Yang lolos hanyalah
     * keadaan yang bertahan penuh selama anomaly_hold_s. */
    if (outOfBand || irregular) {
        if (anomalySinceMs == 0) {
            anomalySinceMs = millis();
        }

        uint32_t held_s =
            (millis() - anomalySinceMs) / 1000u;

        if (held_s >= cfg.anomaly_hold_s) {
            ladder.anomaly(
                millis(),
                outOfBand ? REASON_THRESHOLD : REASON_IRREGULAR
            );
        }
    } else {
        anomalySinceMs = 0;
        ladder.clear(
            millis()
        );
    }
}

// =============================================================
// TANGGA ESKALASI
// =============================================================

static void escalationLoop() {
    ladder.tick(
        millis()
    );

    if (!ladder.takeChange()) {
        return;
    }

    uint8_t stage =
        ladder.stage();

    uint8_t reason =
        ladder.reason();

    Serial.printf(
        "[LADDER] stage=%u reason=%u\n",
        stage,
        reason
    );

    BLE_IndicateEscalation(
        stage,
        reason
    );

    BLE_UpdateAdvertising(
        stage,
        worn
    );

    offline.push(
        nowEpoch(),
        EVT_STAGE,
        stage,
        reason
    );

    if (stage == 2) {
        vibrate(
            false,
            600
        );
    }

    if (stage == 3) {
        vibrate(
            true,
            1200
        );
    }

    if (stage == 4) {
        vibrate(
            true,
            2000
        );
    }
}

// =============================================================
// BLE VITALS
// =============================================================

static void vitalsLoop() {
    if (
        millis() -
        lastVitalsMs <
        VITALS_INTERVAL_MS
    ) {
        return;
    }

    lastVitalsMs = millis();

    BLE_UpdateAdvertising(
        ladder.stage(),
        worn
    );

    uint8_t packet[16];

    uint8_t count =
        vitalsCount;

    /* Tidak ada sampel segar siklus ini. Dulu yang terkirim nol, dan nol
     * di layar berarti "tidak ada denyut" - padahal yang benar adalah
     * "belum ada yang baru". §3.1 bit 6 menyatakan bedanya, jadi penerima
     * boleh menampilkannya dan tetap menolak memakainya untuk keputusan. */
    bool holding =
        heldInBatch ||
        (count == 0 && heldFresh());

    heldInBatch = false;

    packet[0] =
        (uint8_t)(
            (worn ? 0x80 : 0x00) |
            (holding ? 0x40 : 0x00) |
            (signalQuality & 0x0F)
        );

    if (count == 0) {
        packet[1] =
            holding
            ? heldBPM
            : 0;

        /* RR tetap nol apa pun keadaannya. Satu interval adalah pengukuran
         * satu peristiwa; ia tidak punya versi yang diingat. */
        packet[2] = 0;
        packet[3] = 0;

        BLE_NotifyVitals(
            packet,
            4
        );

        return;
    }

    for (
        uint8_t i = 0;
        i < count;
        i++
    ) {
        memcpy(
            packet + 1 + i * 3,
            vitalsBatch[i],
            3
        );
    }

    vitalsCount = 0;

    BLE_NotifyVitals(
        packet,
        1 + count * 3
    );
}

// =============================================================
// BLE MOTION
// =============================================================

static void motionNotifyLoop() {
    if (
        millis() -
        lastMotionMs <
        MOTION_INTERVAL_MS
    ) {
        return;
    }

    lastMotionMs = millis();

    BLE_NotifyMotion(
        motionMilliG
    );
}

// =============================================================
// BATERAI
// =============================================================

static uint8_t batteryPercent() {
    if (PIN_BATTERY_ADC < 0) {
        static bool warned = false;

        if (!warned) {
            warned = true;

            Serial.println(
                "[BAT] Pembagi tegangan belum dipasang - lapor 255"
            );
        }

        return 255;
    }

    uint32_t milliVolt =
        analogReadMilliVolts(
            PIN_BATTERY_ADC
        ) * 2UL;

    if (milliVolt <= 3300) {
        return 0;
    }

    if (milliVolt >= 4200) {
        return 100;
    }

    return (uint8_t)(
        (milliVolt - 3300UL) *
        100UL /
        900UL
    );
}

static void statusLoop() {
    if (
        millis() -
        lastStatusMs <
        STATUS_INTERVAL_MS
    ) {
        return;
    }

    lastStatusMs = millis();

    BLE_NotifyStatus(
        batteryPercent(),
        false,
        nowEpoch()
    );
}

// =============================================================
// RINGKASAN OFFLINE
// =============================================================

static void offlineSummaryLoop() {
    if (BLE_Connected()) {
        return;
    }

    if (
        millis() -
        disconnectedSinceMs <
        DISCONNECT_GRACE_MS
    ) {
        return;
    }

    if (
        millis() -
        lastSummaryMs <
        OFFLINE_SUMMARY_MS
    ) {
        return;
    }

    lastSummaryMs = millis();

    uint8_t bpm =
        bpmIsValid()
        ? (uint8_t)(
            avgBPM > 255.0f
            ? 255
            : avgBPM
        )
        : 0;

    offline.push(
        nowEpoch(),
        EVT_VITALS,
        bpm,
        spo2ValueValid
            ? spo2Value
            : 0,
        (uint8_t)(
            motionMilliG >> 8
        ),
        (uint8_t)(
            motionMilliG & 0xFF
        )
    );
}

// =============================================================
// FLUSH BUFFER OFFLINE
// =============================================================

static void flushLoop() {
    static uint32_t lastPacketMs = 0;
    static uint32_t lastSentinelMs = 0;

    if (!offline.flushing()) {
        lastSentinelMs = 0;
        return;
    }

    if (!BLE_Connected()) {
        offline.abortFlush();
        lastSentinelMs = 0;
        return;
    }

    uint8_t packet[
        3 +
        EVTBUF_PER_PACKET *
        EVTBUF_ENTRY_LEN
    ];

    uint16_t length = 0;

    if (
        millis() -
        lastPacketMs <
        FLUSH_PACKET_GAP_MS
    ) {
        return;
    }

    if (
        offline.nextPacket(
            packet,
            &length
        )
    ) {
        lastPacketMs = millis();

        BLE_NotifyBuffer(
            packet,
            length
        );

        return;
    }

    if (
        lastSentinelMs != 0 &&
        millis() -
        lastSentinelMs <
        FLUSH_SENTINEL_RETRY_MS
    ) {
        return;
    }

    lastSentinelMs = millis();

    length =
        EventBuffer::sentinel(
            packet
        );

    BLE_NotifyBuffer(
        packet,
        length
    );
}

static void onFlushStart() {
    Serial.printf(
        "[BUF] flush_start, %u entri\n",
        offline.count()
    );

    offline.beginFlush();
}

static void onFlushAck(
    uint16_t lastSequence
) {
    Serial.printf(
        "[BUF] flush_ack last_seq=%u\n",
        lastSequence
    );

    offline.ack(
        lastSequence
    );
}

// =============================================================
// ECG
// =============================================================

static void ecgStart(
    uint16_t durationSeconds
) {
    if (PIN_ECG_OUT < 0) {
        Serial.println(
            "[ECG] AD8232 tidak terpasang"
        );

        return;
    }

    ecgActive = true;
    ecgFilled = 0;

    ecgStopAtMs =
        millis() +
        (uint32_t)
        durationSeconds *
        1000UL;

    ecgNextSampleUs =
        micros();

    Serial.printf(
        "[ECG] Mulai %u detik\n",
        durationSeconds
    );
}

static void ecgStop() {
    if (!ecgActive) {
        return;
    }

    ecgActive = false;

    Serial.println(
        "[ECG] Berhenti"
    );
}

static void ecgLoop() {
    if (!ecgActive) {
        return;
    }

    if (
        (int32_t)(
            millis() -
            ecgStopAtMs
        ) >= 0
    ) {
        ecgStop();
        return;
    }

    uint32_t now =
        micros();

    if (
        (int32_t)(
            now -
            ecgNextSampleUs
        ) < 0
    ) {
        return;
    }

    ecgNextSampleUs +=
        1000000UL /
        ECG_SAMPLE_HZ;

    ecgLeadOn =
        digitalRead(
            PIN_ECG_LO_P
        ) == LOW &&
        digitalRead(
            PIN_ECG_LO_N
        ) == LOW;

    ecgSamples[ecgFilled++] =
        (int16_t)
        analogRead(
            PIN_ECG_OUT
        );

    if (
        ecgFilled <
        ECG_SAMPLES_PER_PACKET
    ) {
        return;
    }

    uint8_t packet[
        2 +
        ECG_SAMPLES_PER_PACKET * 2
    ];

    packet[0] =
        ecgSeq++;

    packet[1] =
        ecgLeadOn
        ? 0x80
        : 0x00;

    memcpy(
        packet + 2,
        ecgSamples,
        sizeof(ecgSamples)
    );

    BLE_NotifyEcg(
        packet,
        sizeof(packet)
    );

    ecgFilled = 0;
}

// =============================================================
// KONFIGURASI DARI APLIKASI
// =============================================================

static void onConfig(
    const char *json
) {
    Serial.printf(
        "[CFG] %s\n",
        json
    );

    JsonDocument document;

    if (
        deserializeJson(
            document,
            json
        )
    ) {
        Serial.println(
            "[CFG] JSON tidak valid"
        );

        return;
    }

    cfg.baseline_bpm =
        document["baseline_bpm"] |
        cfg.baseline_bpm;

    cfg.hr_threshold_delta =
        document["hr_threshold_delta"] |
        cfg.hr_threshold_delta;

    cfg.rr_variability_threshold =
        document["rr_variability_threshold"] |
        cfg.rr_variability_threshold;

    cfg.spo2_sample_interval_s =
        document["spo2_sample_interval_s"] |
        cfg.spo2_sample_interval_s;

    cfg.rr_min_ms =
        document["rr_min_ms"] |
        cfg.rr_min_ms;

    cfg.rr_max_ms =
        document["rr_max_ms"] |
        cfg.rr_max_ms;

    cfg.anomaly_hold_s =
        document["anomaly_hold_s"] |
        cfg.anomaly_hold_s;

    cfg.anomaly_min_quality =
        document["anomaly_min_quality"] |
        cfg.anomaly_min_quality;

    cfg.anomaly_enabled =
        document["anomaly_enabled"] |
        cfg.anomaly_enabled;

    cfg.rr_min_quality =
        document["rr_min_quality"] |
        cfg.rr_min_quality;

    cfg.rr_motion_limit_mg =
        document["rr_motion_limit_mg"] |
        cfg.rr_motion_limit_mg;

    cfg.rr_min_ratio_percent =
        document["rr_min_ratio_percent"] |
        cfg.rr_min_ratio_percent;

    cfg.rr_max_ratio_percent =
        document["rr_max_ratio_percent"] |
        cfg.rr_max_ratio_percent;

    ladder.cfg.stage1_s =
        document["stage1_s"] |
        ladder.cfg.stage1_s;

    ladder.cfg.stage2_s =
        document["stage2_s"] |
        ladder.cfg.stage2_s;

    ladder.cfg.stage3_s =
        document["stage3_s"] |
        ladder.cfg.stage3_s;

    ladder.cfg.motion_response_mg =
        document["motion_response_threshold_mg"] |
        ladder.cfg.motion_response_mg;

    ladder.cfg.standdown_cooldown_s =
        document["standdown_cooldown_s"] |
        ladder.cfg.standdown_cooldown_s;

    // Perlindungan konfigurasi tidak masuk akal.
    if (cfg.spo2_sample_interval_s < 10) {
        cfg.spo2_sample_interval_s = 10;
    }

    if (cfg.rr_min_ms < 240) {
        cfg.rr_min_ms = 240;
    }

    if (cfg.rr_max_ms <= cfg.rr_min_ms) {
        cfg.rr_max_ms = 2000;
    }

    if (cfg.rr_min_quality > 15) {
        cfg.rr_min_quality = 15;
    }

    if (cfg.anomaly_min_quality > 15) {
        cfg.anomaly_min_quality = 15;
    }

    /* Tanpa batas atas, satu nilai salah dari aplikasi membuat gelang diam
     * sepanjang malam tanpa ada yang tahu. Lima menit sudah jauh lebih
     * lama daripada yang masuk akal untuk dipertahankan. */
    if (cfg.anomaly_hold_s > 300) {
        cfg.anomaly_hold_s = 300;
    }

    if (
        cfg.rr_min_ratio_percent >=
        cfg.rr_max_ratio_percent
    ) {
        cfg.rr_min_ratio_percent = 60;
        cfg.rr_max_ratio_percent = 160;
    }
}

// =============================================================
// PERINTAH DARI APLIKASI
// =============================================================

static void onCommand(
    const char *json
) {
    JsonDocument document;

    if (
        deserializeJson(
            document,
            json
        )
    ) {
        Serial.printf(
            "[CMD] JSON tidak valid: %s\n",
            json
        );

        return;
    }

    const char *command =
        document["cmd"] | "";

    // §2.1 heartbeat. Kedatangannya sudah dicatat ble_band.cpp; di sini
    // cukup tidak mengotori log setiap 10 detik.
    if (!strcmp(
            command,
            "ping"
        )) {
        return;
    }

    Serial.printf(
        "[CMD] %s\n",
        json
    );

    // §3.8. Satu-satunya jalan keluar dari tahap 4. escalationLoop yang
    // melaporkan tahap 0 lewat Indicate dan siaran, seperti transisi lain.
    if (!strcmp(
            command,
            "stand_down"
        )) {
        ladder.reset(
            millis()
        );

        return;
    }

    if (!strcmp(
            command,
            "vibrate"
        )) {
        const char *pattern =
            document["pattern"] |
            "soft";

        uint32_t duration =
            document["duration_ms"] |
            800UL;

        vibrate(
            !strcmp(
                pattern,
                "hard"
            ),
            duration
        );

        return;
    }

    if (!strcmp(
            command,
            "sync_time"
        )) {
        epoch_at_sync =
            document["epoch_s"] |
            0UL;

        millis_at_sync =
            millis();

        Serial.printf(
            "[TIME] epoch=%lu\n",
            (unsigned long)
            epoch_at_sync
        );

        return;
    }

    if (!strcmp(
            command,
            "record_baseline"
        )) {
        vibrate(
            false,
            200
        );

        Serial.printf(
            "[BASE] Mulai %u detik\n",
            (unsigned)(
                document["duration_s"] |
                180UL
            )
        );

        return;
    }

    if (!strcmp(
            command,
            "ecg_start"
        )) {
        ecgStart(
            document["duration_s"] |
            30UL
        );

        return;
    }

    if (!strcmp(
            command,
            "ecg_stop"
        )) {
        ecgStop();
    }
}

// =============================================================
// LOG SERIAL
// =============================================================

/*
 * Satu baris per detik ke USB serial. Dulu fungsi ini juga mengirim baris
 * "RP,81,752,97,..." lewat UART1 ke layar ESP32-C6 — empat belas angka
 * telanjang yang dihitung parser layar berdasarkan komanya. Rangkaian final
 * tidak punya layar, jadi yang tersisa hanya versi berlabel untuk mata
 * manusia. Angka yang sama tetap pergi ke aplikasi lewat BLE.
 */

static void logLoop() {
    if (
        millis() -
        lastLogMs <
        LOG_INTERVAL_MS
    ) {
        return;
    }

    lastLogMs = millis();

    bool bpmValid =
        bpmIsValid();

    bool currentRRValid =
        rrIsValid();

    uint8_t bpm =
        bpmValid
        ? (uint8_t)(
            avgBPM > 255.0f
            ? 255
            : avgBPM
        )
        : 0;

    uint16_t rr =
        currentRRValid
        ? lastRRms
        : 0;

    uint8_t spo2 =
        spo2ValueValid
        ? spo2Value
        : 0;

#if SERIAL_HEARTBEAT
    char batt[8];

    uint8_t pct = batteryPercent();

    if (pct == 255) {
        snprintf(batt, sizeof(batt), "n/a");
    } else {
        snprintf(batt, sizeof(batt), "%u%%", (unsigned)pct);
    }

    Serial.printf(
        "[GELANG] BPM=%u(%s)  RR=%u ms(%s)  SpO2=%u%%(%s)  "
        "gerak=%u mG  posisi=%s  dipakai=%s  sinyal=%u/15  "
        "tahap_alarm=%u  alasan=%u  baterai=%s  HP=%s\n",
        (unsigned)bpm,
        bpmValid ? "sah" : "belum",
        (unsigned)rr,
        currentRRValid ? "sah" : "belum",
        (unsigned)spo2,
        spo2ValueValid ? "sah" : "belum",
        (unsigned)motionMilliG,
        positionLabel(bodyPosition),
        worn ? "YA" : "TIDAK",
        (unsigned)signalQuality,
        (unsigned)ladder.stage(),
        (unsigned)ladder.reason(),
        batt,
        BLE_Connected() ? "tersambung" : "putus"
    );
#endif
}

// =============================================================
// STATUS KONEKSI BLE
// =============================================================

static void linkLoop() {
    bool connected =
        BLE_Connected();

    if (
        connected ==
        wasConnected
    ) {
        return;
    }

    wasConnected =
        connected;

    offline.push(
        nowEpoch(),
        EVT_LINK,
        connected ? 1 : 0
    );

    if (connected) {
        disconnectedSinceMs = 0;

        Serial.printf(
            "[LINK] Tersambung, %u entri menunggu flush\n",
            offline.count()
        );
    } else {
        disconnectedSinceMs =
            millis();
    }

    BLE_UpdateAdvertising(
        ladder.stage(),
        worn
    );
}

// =============================================================
// SETUP
// =============================================================

void setup() {
    Serial.begin(
        SERIAL_BAUD
    );

    for (
        uint32_t startedAt = millis();
        !Serial &&
        millis() - startedAt < 2000;
    ) {
        delay(10);
    }

    Serial.println(
        F("\n=== RePulse Band ESP32-C3 ===")
    );

    pinMode(
        PIN_BUTTON,
        INPUT_PULLUP
    );

    ledcAttach(
        PIN_MOTOR,
        MOTOR_PWM_FREQ,
        MOTOR_PWM_BITS
    );

    ledcWrite(
        PIN_MOTOR,
        0
    );

    if (PIN_ECG_OUT >= 0) {
        pinMode(
            PIN_ECG_LO_P,
            INPUT
        );

        pinMode(
            PIN_ECG_LO_N,
            INPUT
        );

        analogSetPinAttenuation(
            PIN_ECG_OUT,
            ADC_11db
        );
    }

    Wire.begin(
        PIN_I2C_SDA,
        PIN_I2C_SCL
    );

    initMAX30102();
    initMPU6050();

    BandCallbacks callbacks = {
        onConfig,
        onCommand,
        onFlushStart,
        onFlushAck
    };

    BLE_Init(
        callbacks
    );

    Serial.println(
        F("[BOOT] Siap")
    );
}

// =============================================================
// LOOP
// =============================================================

void loop() {
    pumpSensor();

    buttonLoop();
    motorLoop();
    motionLoop();

    anomalyLoop();
    escalationLoop();

    ecgLoop();

    vitalsLoop();
    motionNotifyLoop();
    spo2Loop();
    statusLoop();

    linkLoop();
    offlineSummaryLoop();
    flushLoop();

    logLoop();
}