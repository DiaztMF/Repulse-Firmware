/*
 * ═══════════════════════════════════════════════════════════════
 *  RePulse Band — ESP32-C3
 * ═══════════════════════════════════════════════════════════════
 *
 *  Peran: gelang adalah "RePulse Band" di BLE_GATT_CONTRACT.md.
 *  Ia yang mengiklankan diri, yang tersambung ke HP, dan yang menjalankan
 *  tangga eskalasi — tersambung maupun tidak (§3.5). Layar ESP32-S3 hanya
 *  menerima ringkasan lewat UART; ia tidak memutuskan apa pun.
 *
 *  Sensor : MAX30102 (HR + SpO2), MPU6500 (accel + gyro)
 *  Keluaran: motor getar (PWM), RX TX ke layar S3
 *  Masukan : tombol SOS
 *
 *  Library (Arduino Library Manager):
 *    1. NimBLE-Arduino  >= 2.0   — h2zero
 *    2. ArduinoJson     >= 7.0   — Benoit Blanchon
 *    3. SparkFun MAX3010x Sensor Library — SparkFun
 *    4. MPU6500_WE                       — Wolfgang Ewald
 *
 *  Board: ESP32C3 Dev Module (esp32 core 3.x)
 * ═══════════════════════════════════════════════════════════════
 */

#include <Arduino.h>
#include <Wire.h>
#include <ArduinoJson.h>

#include <MAX30105.h>
#include <heartRate.h>
#include <spo2_algorithm.h>
#include <MPU6500_WE.h>

#include "ble_band.h"
#include "ladder.h"
#include "evtbuf.h"

// ═══════════════════════════════════════════════════════════════
//  PIN — ⚠ COCOKKAN DENGAN SKEMA SEBELUM FLASH
// ═══════════════════════════════════════════════════════════════
// I2C dan tombol/motor diambil dari sketch monitoring_esp32_firebase yang
// sudah terpasang. Sisanya BELUM pernah dikonfirmasi ke perangkat keras.

#define PIN_I2C_SDA        10
#define PIN_I2C_SCL         0
#define PIN_BUTTON          3    // aktif LOW, pull-up internal

// Dikonfirmasi ke wiring 19 Agustus. Sebelumnya 2, yang salah dua kali:
// motornya ada di GPIO1, jadi tidak akan pernah bergetar — dan GPIO2 pada
// ESP32-C3 adalah strapping pin, dibaca saat boot. Gerbang MOSFET yang
// menahannya rendah saat reset bisa membuat papan gagal boot sama sekali,
// yang tampak seperti perangkat mati, bukan seperti pin yang salah.
#define PIN_MOTOR           1    // PWM

// AD8232 tidak ada di rangkaian. Bukan "belum dikonfirmasi" — tidak
// dipasang, dan GPIO1 yang dulu ditulis di sini adalah pin motor, jadi
// kedua fungsi memperebutkan pin yang sama.
//
// -1 mematikannya dengan bersih: characteristic 000A diam dan ecg_start
// ditolak. Aplikasi menyembunyikan layar ECG-nya sendiri; fitur yang tidak
// bisa bekerja tidak boleh tampil sebagai tombol.
#define PIN_ECG_OUT        -1
#define PIN_ECG_LO_P       -1
#define PIN_ECG_LO_N       -1

/* UART ke layar ESP32-S3. Bukan UART0 — Serial dipakai untuk log USB.
 *
 * Dinamai dari sudut pandang pin ini sendiri, bukan dari lawan bicaranya.
 * Nama lama menyebut papan seberang, dan itu bisa dibaca dua cara — "pin
 * tempat kami mengirim ke sana", atau "pin pengirim milik sana tersambung
 * ke sini". Kedua bacaan menghasilkan penyolderan yang berlawanan.
 *
 * Yang menentukan tetap urutan argumen begin(baud, config, rxPin, txPin),
 * bukan namanya. Nama ini hanya berhenti berdebat dengannya:
 *
 *     GPIO7  keluar  ──→  GPIO44 (RX) di S3
 *     GPIO6  masuk   ←──  GPIO43 (TX) di S3, tidak terpakai hari ini
 *
 * Jalur kedua boleh dibiarkan menggantung: Repulse_Link.cpp hanya membaca,
 * tidak pernah menulis. Jam menggambar, ia tidak menjawab. */
#define PIN_UART_OUT        7    // C3 mengirim
#define PIN_UART_IN         6    // C3 menerima

// Pembagi tegangan baterai. -1 = belum ada; status melaporkan 255 —
// "tidak diketahui" menurut §3.6 — dan memperingatkan sekali di log
// daripada mengarang angka yang turun-naik.
#define PIN_BATTERY_ADC    -1

// ═══════════════════════════════════════════════════════════════
//  AMBANG & TEMPO — knob kalibrasi
// ═══════════════════════════════════════════════════════════════

#define SERIAL_BAUD              115200
#define S3_BAUD                  115200

// Cermikan baris RP ke log USB tiap detik. Matikan setelah bring-up kalau
// log-nya mengganggu; selama menelusuri perangkat, diam adalah musuh.
#define SERIAL_HEARTBEAT               1

#define VITALS_INTERVAL_MS         1000   // §3.1 ~1 dtk
#define MOTION_INTERVAL_MS         1000   // §3.3 ~1 dtk
#define STATUS_INTERVAL_MS       300000   // §3.6 tiap 5 menit
#define OFFLINE_SUMMARY_MS        60000   // ringkasan vital ke buffer offline
#define S3_INTERVAL_MS             1000

#define SOS_HOLD_MS                2000   // §3.4 tekan >= 2 detik
#define DISCONNECT_GRACE_MS       30000   // §3.9 putus > 30 dtk = mode offline

// Deteksi terpasang (§7 no. 6): level DC inframerah. Gelang di atas meja
// membaca jauh di bawah ini. Ukur ulang di kulit pengguna sungguhan —
// kulit gelap dan gelang longgar sama-sama menurunkan nilainya.
//
// TERUKUR 19 Agustus 2026 pada arus LED penuh: meja kosong 27-29 rb,
// kulit pergelangan 206-246 rb. 100000 duduk jauh dari keduanya, dan
// histeresis turun di 80000 melewati zona transisi 55-80 rb dengan bersih.
// Ukur ulang di kulit yang lebih gelap sebelum menganggapnya selesai.
//
// Arah salahnya tidak simetris. Terlalu rendah, gelang tengkurap di meja
// membaca worn = 1, tidak menemukan detak, dan §3.5 membaca itu sebagai
// jantung yang berhenti. Ambang yang kerendahan membangunkan satu rumah.
#define IR_WORN_THRESHOLD        100000

// signal_quality 0-15 (§7 no. 7). Diturunkan dari kelebihan IR di atas
// ambang terpasang, dipotong bila tidak ada detak yang terbaca belakangan.
//
// Diskalakan bersama ambang, lalu dipotong: FIFO 18 bit jenuh di 262143,
// jadi 309000 hasil skala murni berarti kualitas 15 tidak pernah tercapai.
#define IR_QUALITY_FULL          220000   // IR di sini ke atas = 15

// Cetak IR mentah tiap 500 ms lewat USB. Nyalakan, pakai gelang, baca
// angkanya, lepas ke meja, baca lagi — IR_WORN_THRESHOLD duduk di
// tengahnya. Lima menit di bangku, bukan satu malam menebak.
#define IR_CALIBRATION                1

// Di bawah ini bukan SpO2 rendah, melainkan pengukuran yang gagal.
#define SPO2_PLAUSIBLE_MIN           70

// Geseran kanan sebelum sampel masuk checkForBeat(). Lihat handleSample().
#define BEAT_INPUT_SHIFT              3
#define BEAT_STALE_MS               4000

#define MOTOR_GATE_MS                200   // §3.5 jeda setelah motor mati
#define MOTOR_SOFT_DUTY              110
#define MOTOR_HARD_DUTY              255
#define MOTOR_PWM_FREQ              5000
#define MOTOR_PWM_BITS                 8

#define SPO2_BUFFER_LEN              100
#define ECG_SAMPLE_HZ                250   // §3.10
#define ECG_SAMPLES_PER_PACKET        90

#define RR_HISTORY                     8   // untuk variabilitas RR

// ═══════════════════════════════════════════════════════════════
//  Konfigurasi yang bisa ditimpa aplikasi (§3.7 `0005`)
// ═══════════════════════════════════════════════════════════════

struct BandConfig {
    uint8_t  baseline_bpm            = 62;
    uint8_t  hr_threshold_delta      = 16;
    float    rr_variability_threshold = 0.18f;
    uint16_t spo2_sample_interval_s  = 20;
};
static BandConfig cfg;

// ═══════════════════════════════════════════════════════════════
//  Keadaan
// ═══════════════════════════════════════════════════════════════

/* Global non-static di dalam heartRate.cpp milik SparkFun. Tidak diumumkan
 * di header-nya, tapi ada di objek yang sudah ter-link, dan ini satu-satunya
 * cara melihat sinyal AC yang benar-benar dipakai detektor. */
extern int16_t IR_AC_Max;
extern int16_t IR_AC_Min;

static MAX30105   maxSensor;
static MPU6500_WE mpu6500(0x68);
static Ladder      ladder;
static EventBuffer offline;
static HardwareSerial S3(1);

static bool max30102Ready = false;
static bool mpu6500Ready  = false;

// Jam gelang. §3.6: 0 = belum pernah disinkronkan, dan itu harus jujur
// dilaporkan — cap epoch nol merusak settle_time_s di aplikasi.
static uint32_t epoch_at_sync   = 0;
static uint32_t millis_at_sync  = 0;

// Vital
static bool     worn            = false;
static uint8_t  signalQuality   = 0;
static uint32_t lastBeatMs      = 0;
static float    avgBPM          = 0;
static uint16_t rrHistory[RR_HISTORY] = {0};
static uint8_t  rrCount         = 0;
static uint8_t  rrIdx           = 0;

// Batch vitals: §3.1 status + maksimum 5 sampel (bpm, rr_ms)
static uint8_t  vitalsBatch[5][3];
static uint8_t  vitalsCount     = 0;

// SpO2
static uint32_t irBuf[SPO2_BUFFER_LEN];
static uint32_t redBuf[SPO2_BUFFER_LEN];
static uint16_t spo2Filled      = 0;
static bool     spo2Collecting  = false;
static uint8_t  spo2Value       = 0;
static uint8_t  bodyPosition    = 255;    // §3.2 255 = tidak diketahui

// Gerakan
static uint16_t motionMilliG    = 0;

// Motor
static uint32_t motorOffAtMs    = 0;
static bool     motorOn         = false;

// Tombol
static volatile uint32_t buttonDownMs = 0;
static volatile bool     buttonHeld   = false;
static bool              sosLatched   = false;

// EKG
static bool     ecgActive       = false;
static uint32_t ecgStopAtMs     = 0;
static uint32_t ecgNextSampleUs = 0;
static int16_t  ecgSamples[ECG_SAMPLES_PER_PACKET];
static uint8_t  ecgFilled       = 0;
static uint8_t  ecgSeq          = 0;
static bool     ecgLeadOn       = false;

// Tempo
static uint32_t lastVitalsMs = 0, lastMotionMs = 0, lastStatusMs = 0;
static uint32_t lastSummaryMs = 0, lastS3Ms = 0, lastSpo2Ms = 0;
static uint32_t disconnectedSinceMs = 0;
static bool     wasConnected = false;

// ═══════════════════════════════════════════════════════════════
//  Waktu
// ═══════════════════════════════════════════════════════════════

static uint32_t nowEpoch() {
    if (epoch_at_sync == 0) return 0;
    return epoch_at_sync + (millis() - millis_at_sync) / 1000u;
}

// ═══════════════════════════════════════════════════════════════
//  Motor getar — §3.5 wajib di-gate terhadap accelerometer
// ═══════════════════════════════════════════════════════════════

static void vibrate(bool hard, uint32_t duration_ms) {
    ledcWrite(PIN_MOTOR, hard ? MOTOR_HARD_DUTY : MOTOR_SOFT_DUTY);
    motorOn      = true;
    motorOffAtMs = millis() + duration_ms;
}

static void motorLoop() {
    if (motorOn && millis() >= motorOffAtMs) {
        ledcWrite(PIN_MOTOR, 0);
        motorOn = false;
    }
}

/* Motor koin ada di pergelangan yang sama dengan IMU, jadi getarannya
 * terbaca sebagai gerakan tubuh dan alarm akan membatalkan dirinya sendiri.
 * Kita pilih gating waktu, bukan ambang amplitudo (jawaban kontrak §7 no. 8):
 * ambang amplitudo perlu diukur ulang tiap kali motor atau tali berganti. */
static bool motionTrusted() {
    return !motorOn && millis() > motorOffAtMs + MOTOR_GATE_MS;
}

// ═══════════════════════════════════════════════════════════════
//  Tombol SOS
// ═══════════════════════════════════════════════════════════════

static void IRAM_ATTR buttonISR() {
    if (digitalRead(PIN_BUTTON) == LOW) {
        buttonDownMs = millis();
        buttonHeld   = true;
    } else {
        buttonHeld   = false;
    }
}

static void buttonLoop() {
    if (!buttonHeld) { sosLatched = false; return; }
    if (sosLatched) return;
    if (millis() - buttonDownMs < SOS_HOLD_MS) return;

    sosLatched = true;
    Serial.println("[SOS] Tombol ditahan >= 2 dtk");
    BLE_IndicateSos();                       // §3.4, harus sampai < 2 dtk
    offline.push(nowEpoch(), EVT_SOS);
    ladder.manualSos(millis());
    vibrate(true, 400);                      // umpan balik bahwa tekanan diterima
}

// ═══════════════════════════════════════════════════════════════
//  MAX30102 — satu aliran sampel untuk HR dan SpO2 sekaligus
// ═══════════════════════════════════════════════════════════════

static void initMAX30102() {
    Serial.print(F("[HR]  MAX30102..."));
    if (!maxSensor.begin(Wire, I2C_SPEED_FAST)) {
        Serial.println(F(" TIDAK DITEMUKAN"));
        return;
    }
    /* Arus LED maksimum (0xFF = 51 mA), atas permintaan firmware senior.
     * Amplitudo lama — IR 0x1F (6,2 mA), merah 0x0A (2,0 mA) — adalah
     * setelan meja: cukup untuk jari yang ditekan, tipis untuk pergelangan
     * berambut, tali longgar, atau kulit gelap.
     *
     * Rentang ADC ikut naik 4096 -> 16384 nA, dan itu bukan pilihan. FIFO
     * MAX30102 lebar 18 bit, jenuh di 262143. IR di kulit terbaca 50-150 rb
     * pada 6,2 mA; 8,2x arus tanpa menaikkan rentang mendorongnya ke
     * 400 rb-1,2 juta — jauh di atas langit-langit. Puncak PPG yang terpotong
     * rata menghancurkan dua hal sekaligus: waktu puncak untuk HR, dan rasio
     * R untuk SpO2. Intensitas maksimum tanpa headroom membuat sinyal lebih
     * buruk, bukan lebih baik. */
    maxSensor.setup(0xFF, 4, 2, 100, 411, 16384);  // brightness, avg, mode, Hz, µs, ADC
    maxSensor.setPulseAmplitudeRed(0xFF);
    maxSensor.setPulseAmplitudeIR(0xFF);
    max30102Ready = true;
    Serial.println(F(" OK"));
}

/* §3.1: worn dan signal_quality adalah dua field yang mencegah alarm palsu.
 * Bagi MAX30102, "tidak ada detak" dan "gelang di atas meja" terbaca
 * identik — dan yang pertama memicu ALERT. */
static void updateWornAndQuality(uint32_t ir) {
#if IR_CALIBRATION
    /* AC adalah selisih yang benar-benar dihitung detektor SparkFun, dan
     * tanpanya setiap penyetelan BEAT_INPUT_SHIFT cuma tebakan. Bacanya:
     *
     *   AC = 0        detektor masih buta, DC belum turun di bawah 32767
     *   AC 1-19       terlalu kecil, ambang bawahnya 20 - kurangi geseran
     *   AC 20-1000    benar, detak harusnya muncul
     *   AC > 1000     ditolak ambang atas - tambah geseran
     *
     * Nilainya dari sampel sebelumnya karena checkForBeat() dipanggil
     * setelah fungsi ini; pada 25 Hz itu 40 ms, tidak ada artinya. */
    static uint32_t lastIrPrintMs = 0;
    if (millis() - lastIrPrintMs >= 500) {
        lastIrPrintMs = millis();
        Serial.printf("[CAL] IR=%lu worn=%d q=%u AC=%d\n",
                      (unsigned long)ir, (int)worn, signalQuality,
                      (int)(IR_AC_Max - IR_AC_Min));
    }
#endif

    /* Histeresis, bukan satu ambang. Gelang yang duduk persis di ambang akan
     * membalik worn tiap detik — dan tiap baliknya mengubah paket advertising
     * dan membuat aplikasi berkedip antara "terpasang" dan "di atas meja".
     * Naik di ambang penuh, turun di 80% darinya. */
    if (worn) worn = ir > (IR_WORN_THRESHOLD * 4) / 5;
    else      worn = ir > IR_WORN_THRESHOLD;

    if (!worn) {
        /* Lepas dari kulit berarti tidak ada lagi yang diukur. Menahan angka
         * terakhir membuat gelang di atas meja melaporkan denyut dan oksigen
         * orang yang sudah melepasnya — pembacaan basi yang di layar terlihat
         * persis seperti pembacaan hidup, dan bertahan sampai pengukuran
         * berikutnya yang mungkin tidak pernah datang. */
        signalQuality = 0;
        avgBPM        = 0.0f;
        spo2Value     = 0;
        rrCount       = 0;
        return;
    }

    uint32_t span  = IR_QUALITY_FULL - IR_WORN_THRESHOLD;
    uint32_t above = ir - IR_WORN_THRESHOLD;
    uint8_t  q     = (uint8_t)(above >= span ? 15 : (above * 15) / span);

    // Sinyal yang bagus tapi tidak menghasilkan detak apa pun bukan sinyal
    // yang bagus. Kalibrasi memakai nilai ini sebagai gerbang.
    if (millis() - lastBeatMs > BEAT_STALE_MS) q = q > 4 ? 4 : q;
    signalQuality = q;
}

static void pushRR(uint16_t rr_ms) {
    rrHistory[rrIdx] = rr_ms;
    rrIdx = (rrIdx + 1) % RR_HISTORY;
    if (rrCount < RR_HISTORY) rrCount++;
}

/* Variabilitas RR sebagai rasio: rata-rata |ΔRR| dibagi rata-rata RR.
 * Skala yang sama dengan rr_variability_threshold di §3.7 (0,18). */
static float rrVariability() {
    if (rrCount < 3) return 0.0f;
    uint32_t sum = 0, diff = 0;
    for (uint8_t i = 0; i < rrCount; i++) sum += rrHistory[i];
    for (uint8_t i = 1; i < rrCount; i++) {
        int32_t d = (int32_t)rrHistory[i] - (int32_t)rrHistory[i - 1];
        diff += (uint32_t)(d < 0 ? -d : d);
    }
    float mean = (float)sum / rrCount;
    if (mean < 1.0f) return 0.0f;
    return ((float)diff / (rrCount - 1)) / mean;
}

static void handleSample(uint32_t ir, uint32_t red) {
    updateWornAndQuality(ir);

    if (spo2Collecting && spo2Filled < SPO2_BUFFER_LEN) {
        irBuf[spo2Filled]  = ir;
        redBuf[spo2Filled] = red;
        spo2Filled++;
    }

    if (!worn) return;
    /* Digeser sebelum masuk ke detektor detak, dan ini bukan selera.
     *
     * checkForBeat() menerima int32_t, tapi seluruh isinya 16 bit. Yang
     * mengikat bukan parameter uint16_t di averageDCEstimator(), melainkan
     * nilai kembaliannya:
     *
     *     int16_t averageDCEstimator(int32_t *p, uint16_t x)
     *     { *p += ((((long) x << 15) - *p) >> 4); return (*p >> 15); }
     *
     * (*p >> 15) memulihkan x apa adanya, jadi begitu DC melewati 32767 ia
     * kembali sebagai bilangan negatif. sample - IR_Average_Estimated lalu
     * melompat ke 65536, diserahkan ke lowPassFIRFilter(int16_t) yang
     * memotongnya jadi nol, dan sinyal AC-nya nol selamanya.
     *
     * Karena itu langit-langitnya 32767, bukan 65535 seperti dugaan pertama —
     * geseran 2 bit membawa DC ke 60 rb, masih di atas batas, dan detektornya
     * tetap buta. Geseran 3 membawanya ke ~30 rb, di bawah langit-langit.
     *
     * Ambang detaknya juga mutlak, bukan relatif: (AC_max - AC_min) harus
     * jatuh antara 20 dan 1000 hitungan mentah. Perfusi pergelangan sekitar
     * 0,1-1% dari DC, jadi IR terukur 206-246 rb digeser 3 bit menjadi DC
     * 26-31 rb dengan AC 26-310 — di dalam jendela di kedua ujungnya, tanpa
     * menyentuh arus LED yang memang berguna untuk menembus pergelangan.
     *
     * SpO2 tidak ikut digeser — buffernya uint32_t dan algoritma Maxim justru
     * butuh resolusi penuh.
     *
     * ponytail: konstanta, bukan penskalaan adaptif. Kalau kulit yang lebih
     * gelap membuat IR turun jauh di bawah 120 rb, turunkan geserannya ke 1. */
    /* Dijepit: IR yang menyentuh langit-langit ADC 18-bit (262143) akan
     * melewati 32767 setelah digeser dan membalik tandanya. Penjepit ini
     * hanya bekerja saat sensornya sudah jenuh — bacaan yang memang tidak
     * berarti — dan mencegahnya berubah menjadi kebutaan senyap. */
    uint32_t beat = ir >> BEAT_INPUT_SHIFT;
    if (!checkForBeat((int32_t)(beat > 32000 ? 32000 : beat))) return;

    uint32_t now   = millis();
    uint32_t delta = now - lastBeatMs;
    lastBeatMs = now;
    if (delta < 240 || delta > 3000) return;       // 20-250 BPM

    float bpm = 60000.0f / (float)delta;
    avgBPM = avgBPM < 1.0f ? bpm : (avgBPM * 0.8f + bpm * 0.2f);
    pushRR((uint16_t)delta);

    if (vitalsCount < 5) {
        vitalsBatch[vitalsCount][0] = (uint8_t)(bpm > 255 ? 255 : bpm);
        vitalsBatch[vitalsCount][1] = (uint8_t)(delta & 0xFF);
        vitalsBatch[vitalsCount][2] = (uint8_t)(delta >> 8);
        vitalsCount++;
    }
}

static void pumpSensor() {
    if (!max30102Ready) return;
    maxSensor.check();
    while (maxSensor.available()) {
        uint32_t ir  = maxSensor.getIR();
        uint32_t red = maxSensor.getRed();
        maxSensor.nextSample();
        handleSample(ir, red);
    }
}

/* §3.2: jangan sampling SpO2 kontinu. Pengumpulan berjalan di aliran sampel
 * yang sama, jadi tidak ada yang memblok tangga eskalasi selama 4 detik —
 * itu perbedaan terpenting dari sketch monitoring lama. */
static void spo2Loop() {
    if (!max30102Ready) return;

    /* Hanya saat terpasang. Mengumpulkan 100 sampel dari sensor yang menatap
     * meja kosong tetap menghasilkan angka — algoritma Maxim mengembalikan
     * spo2Valid = 1 untuk derau — dan angka itu berbohong lebih meyakinkan
     * daripada tidak ada angka sama sekali. */
    if (!spo2Collecting && worn &&
        millis() - lastSpo2Ms >= (uint32_t)cfg.spo2_sample_interval_s * 1000u) {
        spo2Collecting = true;
        spo2Filled     = 0;
        return;
    }
    if (!spo2Collecting || spo2Filled < SPO2_BUFFER_LEN) return;

    int32_t spo2 = 0, hr = 0;
    int8_t  spo2Valid = 0, hrValid = 0;
    maxim_heart_rate_and_oxygen_saturation(irBuf, SPO2_BUFFER_LEN, redBuf,
                                           &spo2, &spo2Valid, &hr, &hrValid);
    /* §3.2: 0 berarti "tidak valid", bukan "nol persen".
     *
     * Lantai fisiologis, bukan sekadar rentang angka. spo2Valid dari algoritma
     * Maxim tidak bisa dipercaya sendirian — bangku uji melaporkan 16% dengan
     * flag valid menyala, konstan, bahkan saat gelang ada di meja. SpO2 16%
     * bukan pembacaan rendah, itu pembacaan gagal; di bawah 70% tidak ada
     * oksimeter yang menampilkan angka, semuanya menampilkan garis. */
    bool plausible = spo2Valid && spo2 >= SPO2_PLAUSIBLE_MIN && spo2 <= 100;
    spo2Value = plausible ? (uint8_t)spo2 : 0;

    BLE_NotifyOxygen(spo2Value, bodyPosition);
    spo2Collecting = false;
    lastSpo2Ms     = millis();
}

// ═══════════════════════════════════════════════════════════════
//  MPU6500 — gerakan dan posisi tubuh
// ═══════════════════════════════════════════════════════════════

static void initMPU6500() {
    Serial.print(F("[IMU] MPU6500..."));
    if (!mpu6500.init()) {
        Serial.println(F(" TIDAK DITEMUKAN"));
        return;
    }
    delay(500);
    /* Rentang dulu, baru kalibrasi. autoOffsets() mengukur nilai diam dan
     * menyimpan koreksinya dalam LSB rentang yang sedang berlaku — mengubah
     * rentang sesudahnya membuat koreksi itu dipakai pada skala yang salah.
     *
     * Urutan terbalik membuat gelang yang tergeletak diam melaporkan ~310 mg,
     * dan itu bukan kesalahan kosmetik: motion_response_mg adalah 150, jadi
     * lantai derau saja sudah dua kali lipat ambang "tubuh merespons". Tangga
     * eskalasi membatalkan dirinya sendiri di tahap 2, setiap kali, dan tahap
     * 3 dan 4 tidak akan pernah tercapai. */
    mpu6500.setAccRange(MPU6500_ACC_RANGE_4G);
    Serial.print(F(" kalibrasi (jangan sentuh)..."));
    mpu6500.autoOffsets();
    mpu6500.enableAccDLPF(true);
    mpu6500.setAccDLPF(MPU6500_DLPF_6);
    mpu6500Ready = true;
    Serial.println(F(" OK"));
}

/* §3.2: kiri dan kanan tidak boleh digabung — ringkasan mingguan harus bisa
 * menyebut di posisi mana desaturasi paling sering muncul.
 * ponytail: dibaca dari sumbu gravitasi pergelangan. Ini perkiraan kasar
 * (tangan bisa di atas kepala); ganti ke fusi dengan giroskop kalau
 * pencatatan posisi ternyata terlalu berisik untuk laporan mingguan. */
static uint8_t positionFrom(float ax, float ay, float az) {
    float absx = fabsf(ax), absy = fabsf(ay), absz = fabsf(az);
    if (absz > absx && absz > absy) return az > 0 ? 0 : 3;   // telentang / tengkurap
    if (absx > absy)                return ax > 0 ? 1 : 2;   // miring kiri / kanan
    return 255;
}

static void motionLoop() {
    if (!mpu6500Ready || !motionTrusted()) return;

    xyzFloat a = mpu6500.getGValues();
    float magnitude = sqrtf(a.x * a.x + a.y * a.y + a.z * a.z);
    float delta_g   = fabsf(magnitude - 1.0f);          // buang gravitasi
    uint32_t mg     = (uint32_t)(delta_g * 1000.0f);
    motionMilliG    = (uint16_t)(mg > 65535 ? 65535 : mg);
    bodyPosition    = positionFrom(a.x, a.y, a.z);

    ladder.motion(millis(), motionMilliG);
}

// ═══════════════════════════════════════════════════════════════
//  Deteksi anomali → tangga eskalasi
// ═══════════════════════════════════════════════════════════════

static void anomalyLoop() {
    // Gelang di atas meja tidak punya denyut, dan itu bukan keadaan darurat.
    if (!worn || avgBPM < 1.0f) { ladder.clear(millis()); return; }

    int16_t deviation = (int16_t)avgBPM - (int16_t)cfg.baseline_bpm;
    bool    outOfBand = abs(deviation) > (int16_t)cfg.hr_threshold_delta;
    bool    irregular = rrVariability() > cfg.rr_variability_threshold;

    if (outOfBand)      ladder.anomaly(millis(), REASON_THRESHOLD);
    else if (irregular) ladder.anomaly(millis(), REASON_IRREGULAR);
    else                ladder.clear(millis());
}

static void escalationLoop() {
    ladder.tick(millis());
    if (!ladder.takeChange()) return;

    uint8_t stage  = ladder.stage();
    uint8_t reason = ladder.reason();
    Serial.printf("[LADDER] stage=%u reason=%u\n", stage, reason);

    BLE_IndicateEscalation(stage, reason);          // §3.5, wajib Indicate
    BLE_UpdateAdvertising(stage, worn);             // §2.1, batas 2 detik
    offline.push(nowEpoch(), EVT_STAGE, stage, reason);

    // §3.5: tahap 2 getar halus, tahap 3 getar keras + minta layar HP menyala
    // (permintaan layar dikirim aplikasi lewat Indicate di atas).
    if (stage == 2) vibrate(false, 600);
    if (stage == 3) vibrate(true, 1200);
    if (stage == 4) vibrate(true, 2000);
}

// ═══════════════════════════════════════════════════════════════
//  Notifikasi berkala
// ═══════════════════════════════════════════════════════════════

/* §3.1: satu byte status, lalu kelompok 3 byte hingga 5 kali, tertua dulu.
 * Paket tetap dikirim saat tidak ada detak — status dengan worn=0 adalah
 * cara aplikasi tahu gelang ada di meja, bukan bahwa gelang mati. */
static void vitalsLoop() {
    if (millis() - lastVitalsMs < VITALS_INTERVAL_MS) return;
    lastVitalsMs = millis();

    /* Bit `worn` ikut disiarkan (§2.1) dan berubah tanpa menunggu tangga
     * eskalasi bergerak. Dipanggil tiap detik; fungsinya keluar lebih awal
     * kalau tidak ada yang berubah, jadi advertising tidak di-restart
     * percuma. Batas kontrak 2 detik terpenuhi dengan longgar. */
    BLE_UpdateAdvertising(ladder.stage(), worn);

    uint8_t packet[16];
    packet[0] = (uint8_t)((worn ? 0x80 : 0x00) | (signalQuality & 0x0F));

    uint8_t n = vitalsCount;
    if (n == 0) {
        packet[1] = 0; packet[2] = 0; packet[3] = 0;
        BLE_NotifyVitals(packet, 4);
        return;
    }
    for (uint8_t i = 0; i < n; i++) memcpy(packet + 1 + i * 3, vitalsBatch[i], 3);
    vitalsCount = 0;
    BLE_NotifyVitals(packet, 1 + n * 3);
}

static void motionNotifyLoop() {
    if (millis() - lastMotionMs < MOTION_INTERVAL_MS) return;
    lastMotionMs = millis();
    BLE_NotifyMotion(motionMilliG);
}

static uint8_t batteryPercent() {
    if (PIN_BATTERY_ADC < 0) {
        static bool warned = false;
        if (!warned) {
            warned = true;
            Serial.println("[BAT] Pembagi tegangan tidak terpasang — lapor 255 (tidak diketahui)");
        }
        // §3.6: 255 berarti tidak diketahui, mengikuti idiom yang sama
        // dengan enum posisi §3.2. Melaporkan 100 adalah angka yang
        // sepenuhnya masuk akal dan sepenuhnya karangan — dan gelang yang
        // bilang penuh sepanjang malam lalu mati adalah persis kegagalan
        // senyap yang produk ini ada untuk mencegahnya.
        return 255;
    }
    // Li-Po 3,3 V kosong → 4,2 V penuh, lewat pembagi 1:2.
    uint32_t mv = analogReadMilliVolts(PIN_BATTERY_ADC) * 2;
    if (mv <= 3300) return 0;
    if (mv >= 4200) return 100;
    return (uint8_t)((mv - 3300) * 100 / 900);
}

static void statusLoop() {
    if (millis() - lastStatusMs < STATUS_INTERVAL_MS) return;
    lastStatusMs = millis();
    BLE_NotifyStatus(batteryPercent(), false, nowEpoch());
}

/* Ringkasan berkala ke buffer offline. Hanya saat HP tidak ada — kalau
 * tersambung, aplikasi sudah menerima semuanya secara langsung. */
static void offlineSummaryLoop() {
    if (BLE_Connected()) return;
    // §3.9: mode offline baru berlaku setelah putus lebih dari 30 detik.
    // Tanpa jeda ini, tiap gangguan koneksi sedetik meninggalkan sampah.
    if (millis() - disconnectedSinceMs < DISCONNECT_GRACE_MS) return;
    if (millis() - lastSummaryMs < OFFLINE_SUMMARY_MS) return;
    lastSummaryMs = millis();
    offline.push(nowEpoch(), EVT_VITALS,
                 (uint8_t)(avgBPM > 255 ? 255 : avgBPM), spo2Value,
                 (uint8_t)(motionMilliG >> 8), (uint8_t)(motionMilliG & 0xFF));
}

// ═══════════════════════════════════════════════════════════════
//  §3.9 Flush buffer offline
// ═══════════════════════════════════════════════════════════════

/* Dua jeda, dan keduanya wajib.
 *
 * flushLoop() dipanggil dari loop() tanpa timer, jadi ia berjalan ribuan
 * kali per detik. Tanpa jeda pertama, 256 entri dilempar sebagai ~22 notify
 * berturut-turut dalam hitungan mikrodetik; kolam mbuf NimBLE habis, notify
 * mulai gagal tanpa suara, dan sebagian malam yang tersimpan hilang justru
 * pada saat ia sedang diselamatkan. Satu paket per interval koneksi.
 *
 * Jeda kedua lebih parah kalau diabaikan. flushing_ baru turun di ack(),
 * jadi begitu paketnya habis, sentinel dikirim ulang di SETIAP putaran loop
 * sampai ACK pulang — ribuan sentinel untuk satu jabat tangan, dan aplikasi
 * membalas satu tulis ACK untuk tiap-tiapnya. Badai dua arah yang cukup
 * untuk menjatuhkan sambungan.
 *
 * Tetap diulang, hanya jarang: sentinel yang hilang tidak boleh membuat
 * gelang menunggu selamanya dengan buffer yang tak pernah dihapus. */
#define FLUSH_PACKET_GAP_MS           30
#define FLUSH_SENTINEL_RETRY_MS     2000

static void flushLoop() {
    static uint32_t lastPacketMs   = 0;
    static uint32_t lastSentinelMs = 0;

    if (!offline.flushing()) { lastSentinelMs = 0; return; }
    if (!BLE_Connected())    { offline.abortFlush(); lastSentinelMs = 0; return; }

    uint8_t  packet[3 + EVTBUF_PER_PACKET * EVTBUF_ENTRY_LEN];
    uint16_t len = 0;

    if (millis() - lastPacketMs < FLUSH_PACKET_GAP_MS) return;

    if (offline.nextPacket(packet, &len)) {
        lastPacketMs = millis();
        BLE_NotifyBuffer(packet, len);
        return;
    }

    // Habis. Tutup dengan sentinel dan tunggu ACK — buffer belum dihapus.
    if (lastSentinelMs != 0 && millis() - lastSentinelMs < FLUSH_SENTINEL_RETRY_MS) return;
    lastSentinelMs = millis();
    len = EventBuffer::sentinel(packet);
    BLE_NotifyBuffer(packet, len);
}

static void onFlushStart() {
    Serial.printf("[BUF] flush_start, %u entri\n", offline.count());
    offline.beginFlush();
}

static void onFlushAck(uint16_t last_seq) {
    Serial.printf("[BUF] flush_ack last_seq=%u\n", last_seq);
    offline.ack(last_seq);
}

// ═══════════════════════════════════════════════════════════════
//  §3.10 EKG
// ═══════════════════════════════════════════════════════════════

static void ecgStart(uint16_t duration_s) {
    if (PIN_ECG_OUT < 0) {
        Serial.println("[ECG] AD8232 tidak terpasang — permintaan ditolak");
        return;
    }
    ecgActive       = true;
    ecgFilled       = 0;
    ecgStopAtMs     = millis() + (uint32_t)duration_s * 1000u;
    ecgNextSampleUs = micros();
    Serial.printf("[ECG] Mulai %u dtk\n", duration_s);
}

static void ecgStop() {
    if (!ecgActive) return;
    ecgActive = false;
    Serial.println("[ECG] Berhenti");
}

static void ecgLoop() {
    if (!ecgActive) return;
    if (millis() >= ecgStopAtMs) { ecgStop(); return; }

    uint32_t now = micros();
    if ((int32_t)(now - ecgNextSampleUs) < 0) return;
    ecgNextSampleUs += 1000000u / ECG_SAMPLE_HZ;

    /* §3.10: lead_on wajib. Gelombang EKG sampah yang ditampilkan sebagai
     * grafik nyata adalah hal paling berbahaya yang bisa dilakukan produk
     * ini — aplikasi menolak menyimpan rekaman dengan lead_on = 0. */
    ecgLeadOn = digitalRead(PIN_ECG_LO_P) == LOW && digitalRead(PIN_ECG_LO_N) == LOW;
    ecgSamples[ecgFilled++] = (int16_t)analogRead(PIN_ECG_OUT);

    if (ecgFilled < ECG_SAMPLES_PER_PACKET) return;

    uint8_t packet[2 + ECG_SAMPLES_PER_PACKET * 2];
    packet[0] = ecgSeq++;
    packet[1] = ecgLeadOn ? 0x80 : 0x00;
    memcpy(packet + 2, ecgSamples, sizeof(ecgSamples));   // little-endian native
    BLE_NotifyEcg(packet, sizeof(packet));
    ecgFilled = 0;
}

// ═══════════════════════════════════════════════════════════════
//  §3.7 & §3.8 — konfigurasi dan perintah dari aplikasi
// ═══════════════════════════════════════════════════════════════

/* §3.7: field yang tidak dikirim berarti tidak diubah — itulah kenapa tiap
 * baris memakai nilai sekarang sebagai default, bukan konstanta pabrik. */
static void onConfig(const char *json) {
    Serial.printf("[CFG] %s\n", json);
    JsonDocument doc;
    if (deserializeJson(doc, json)) { Serial.println("[CFG] JSON tidak valid"); return; }

    cfg.baseline_bpm             = doc["baseline_bpm"]             | cfg.baseline_bpm;
    cfg.hr_threshold_delta       = doc["hr_threshold_delta"]       | cfg.hr_threshold_delta;
    cfg.rr_variability_threshold = doc["rr_variability_threshold"] | cfg.rr_variability_threshold;
    cfg.spo2_sample_interval_s   = doc["spo2_sample_interval_s"]   | cfg.spo2_sample_interval_s;
    ladder.cfg.stage1_s          = doc["stage1_s"]                 | ladder.cfg.stage1_s;
    ladder.cfg.stage2_s          = doc["stage2_s"]                 | ladder.cfg.stage2_s;
    ladder.cfg.stage3_s          = doc["stage3_s"]                 | ladder.cfg.stage3_s;
    ladder.cfg.motion_response_mg =
        doc["motion_response_threshold_mg"] | ladder.cfg.motion_response_mg;
}

static void onCommand(const char *json) {
    Serial.printf("[CMD] %s\n", json);
    JsonDocument doc;
    if (deserializeJson(doc, json)) { Serial.println("[CMD] JSON tidak valid"); return; }

    const char *cmd = doc["cmd"] | "";

    if (!strcmp(cmd, "vibrate")) {
        const char *pattern = doc["pattern"] | "soft";
        vibrate(!strcmp(pattern, "hard"), doc["duration_ms"] | 800u);
        return;
    }
    /* §3.8: sync_time dikirim SETIAP kali tersambung, bukan sekali saat
     * pairing — C3 kehilangan waktu tiap reboot (jawaban §7 no. 4). */
    if (!strcmp(cmd, "sync_time")) {
        epoch_at_sync  = doc["epoch_s"] | 0u;
        millis_at_sync = millis();
        Serial.printf("[TIME] epoch=%lu\n", (unsigned long)epoch_at_sync);
        return;
    }
    if (!strcmp(cmd, "record_baseline")) {
        // Baseline dihitung aplikasi dari aliran vitals; gelang hanya menandai
        // mulainya dengan satu getar supaya pengguna tahu harus diam.
        vibrate(false, 200);
        Serial.printf("[BASE] Mulai %u dtk\n", (unsigned)(doc["duration_s"] | 180u));
        return;
    }
    if (!strcmp(cmd, "ecg_start")) { ecgStart(doc["duration_s"] | 30u); return; }
    if (!strcmp(cmd, "ecg_stop"))  { ecgStop(); }
}

// ═══════════════════════════════════════════════════════════════
//  UART ke layar ESP32-S3
// ═══════════════════════════════════════════════════════════════
// Layar hanya menampilkan; ia tidak memutuskan apa pun. Satu baris teks per
// detik cukup, dan bisa dibaca langsung di serial monitor saat debug.
//
//   RP,<bpm>,<spo2>,<milli_g>,<worn>,<quality>,<stage>,<reason>,<batt>,<link>
//
// ponytail: baris ASCII, bukan protokol biner. Naikkan kalau layar mulai
// perlu mengirim balik sesuatu yang lebih dari perintah tombol.

static void s3Loop() {
    if (millis() - lastS3Ms < S3_INTERVAL_MS) return;
    lastS3Ms = millis();
    /* Satu string format, dua tujuan. Setelah setup selesai gelang tidak
     * mencetak apa pun kecuali ada kejadian — monitor diam berjam-jam dan
     * tidak ada cara membedakan "berjalan tenang" dari "mati".
     *
     * Dicermikan, bukan diformat dua kali: yang kamu baca di monitor adalah
     * byte yang sama persis dengan yang diterima jam. Kalau baris ini muncul
     * di PC tapi jam tetap "--", masalahnya di kabel, bukan di gelang. */
    char line[64];
    snprintf(line, sizeof(line), "RP,%u,%u,%u,%u,%u,%u,%u,%u,%u",
             (unsigned)(avgBPM > 255 ? 255 : avgBPM), spo2Value, motionMilliG,
             worn ? 1 : 0, signalQuality,
             ladder.stage(), ladder.reason(),
             batteryPercent(), BLE_Connected() ? 1 : 0);
    S3.println(line);
#if SERIAL_HEARTBEAT
    Serial.println(line);
#endif
}

// ═══════════════════════════════════════════════════════════════
//  §3.9 Penanda putus / sambung
// ═══════════════════════════════════════════════════════════════

static void linkLoop() {
    bool connected = BLE_Connected();
    if (connected == wasConnected) return;
    wasConnected = connected;

    offline.push(nowEpoch(), EVT_LINK, connected ? 1 : 0);
    if (connected) {
        disconnectedSinceMs = 0;
        Serial.printf("[LINK] Tersambung, %u entri menunggu flush\n", offline.count());
    } else {
        disconnectedSinceMs = millis();
    }
    BLE_UpdateAdvertising(ladder.stage(), worn);
}

// ═══════════════════════════════════════════════════════════════
//  setup / loop
// ═══════════════════════════════════════════════════════════════

void setup() {
    Serial.begin(SERIAL_BAUD);
    /* USB-CDC, bukan UART. Port baru muncul di PC setelah host selesai
     * meng-enumerasi, dan itu perlu satu sampai dua detik — jauh lebih lama
     * dari delay(300) yang dulu ada di sini. Semua yang tercetak sebelum itu
     * hilang tanpa jejak, jadi monitor serial tampak kosong padahal firmware
     * berjalan sempurna.
     *
     * Batas 2 detik, bukan tunggu selamanya: gelang harus tetap boot saat
     * dipakai tidur tanpa kabel. Kalau CDCOnBoot tidak aktif, Serial adalah
     * UART biasa, operator bool()-nya langsung true, dan loop ini lewat. */
    for (uint32_t t0 = millis(); !Serial && millis() - t0 < 2000; ) delay(10);
    Serial.println(F("\n=== RePulse Band (ESP32-C3) ==="));

    S3.begin(S3_BAUD, SERIAL_8N1, PIN_UART_IN, PIN_UART_OUT);

    pinMode(PIN_BUTTON, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_BUTTON), buttonISR, CHANGE);

    ledcAttach(PIN_MOTOR, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
    ledcWrite(PIN_MOTOR, 0);

    if (PIN_ECG_OUT >= 0) {
        pinMode(PIN_ECG_LO_P, INPUT);
        pinMode(PIN_ECG_LO_N, INPUT);
        analogSetPinAttenuation(PIN_ECG_OUT, ADC_11db);
    }

    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    initMAX30102();
    initMPU6500();

    BandCallbacks cb = { onConfig, onCommand, onFlushStart, onFlushAck };
    BLE_Init(cb);

    Serial.println(F("[BOOT] Siap"));
}

void loop() {
    pumpSensor();        // paling sering — checkForBeat butuh aliran rapat
    buttonLoop();
    motorLoop();
    motionLoop();
    anomalyLoop();
    escalationLoop();    // tangga jalan terus, tersambung maupun tidak (§3.5)
    ecgLoop();

    vitalsLoop();
    motionNotifyLoop();
    spo2Loop();
    statusLoop();
    linkLoop();
    offlineSummaryLoop();
    flushLoop();
    s3Loop();
}
