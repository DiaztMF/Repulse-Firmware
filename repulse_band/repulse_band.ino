/*
 * RePulse Band - ESP32-C3
 *
 * Sensor:
 * - MAX30102: BPM, RR, SpO2
 * - MPU6050: gerakan dan posisi tubuh
 *
 * Output:
 * - Motor getar pada GPIO1
 * - UART menuju ESP32-C6 display
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
#include <heartRate.h>
#include <spo2_algorithm.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#include "ble_band.h"
#include "ladder.h"
#include "evtbuf.h"

// =============================================================
// PIN ESP32-C3
// =============================================================

#define PIN_I2C_SDA        10
#define PIN_I2C_SCL         0

#define PIN_BUTTON          3
#define PIN_MOTOR           1

#define PIN_UART_OUT        7
#define PIN_UART_IN         6

#define PIN_ECG_OUT        -1
#define PIN_ECG_LO_P       -1
#define PIN_ECG_LO_N       -1

#define PIN_BATTERY_ADC    -1

// =============================================================
// KOMUNIKASI
// =============================================================

#define SERIAL_BAUD             115200
#define C6_BAUD                 57600

#define SERIAL_HEARTBEAT             1

#define VITALS_INTERVAL_MS        1000
#define MOTION_INTERVAL_MS        1000
#define STATUS_INTERVAL_MS      300000
#define OFFLINE_SUMMARY_MS       60000
#define C6_INTERVAL_MS            1000

#define SOS_HOLD_MS               2000
#define DISCONNECT_GRACE_MS      30000

// =============================================================
// MAX30102
// =============================================================

/* Ambang "dipakai", dan IR yang dianggap kualitas penuh.
 *
 * Angka 100000/220000 yang semula ada di sini adalah angka UJUNG JARI. Di
 * ujung jari IR memang menembus seratus ribu; di pergelangan tangan cahaya
 * menembus kulit yang lebih tebal, lewat tendon dan tulang, dan DC-nya turun
 * satu orde. Sekali kuturunkan jadi setengahnya, dan worn tetap nol terus —
 * yang berarti IR di pergelangan bahkan tidak sampai 50000.
 *
 * Tanpa kulit di depannya, dengan LED menyala, MAX30102 membaca ratusan
 * sampai seribuan. Jadi 5000 masih jauh di atas lantai "tidak dipakai" dan
 * jauh di bawah tebakan mana pun untuk kulit pergelangan.
 *
 * ponytail: 5000/25000 adalah nilai sementara, bukan hasil ukur. [CAL]
 * mencetak IR= dua kali sedetik — baca angkanya di pergelangan DAN di atas
 * meja, lalu taruh ambangnya di tengah kedua angka itu. */
#define IR_WORN_THRESHOLD         5000
#define IR_QUALITY_FULL          25000
#define IR_CALIBRATION               1

/* Titik tengah yang disuapkan ke detektor, dan seberapa lambat garis dasar
 * DC-nya bergerak.
 *
 * checkForBeat menguji amplitudo AC terhadap jendela tetap dalam satuan yang
 * kita suapkan sendiri:
 *
 *     if ((IR_AC_Max - IR_AC_Min) > 20 && (IR_AC_Max - IR_AC_Min) < 1000)
 *
 * Di ujung jari AC sekitar 1-5% dari DC dan jendela itu pas. Di pergelangan
 * AC hanya 0,1-0,5%, dan kode ini dulu menyuapkan `ir >> 3` — membagi AC
 * yang sudah lemah itu delapan kali lagi, jauh di bawah ambang 20. Pembagian
 * itu ada semata untuk menahan DC di bawah batas 16 bit averageDCEstimator,
 * tapi ia menyelesaikan masalah DC dengan menghancurkan sinyal yang dicari.
 *
 * Sekarang DC dibuang dengan PENGURANGAN. Garis dasarnya bergerak lambat
 * (>>8 pada 100 sampel/detik ≈ 2,5 detik, jauh lebih lambat daripada satu
 * denyut, jadi ia tidak ikut memakan pulsanya), dan yang disuapkan adalah
 * titik tengah ditambah simpangannya. DC tetap aman, AC utuh — delapan kali
 * lebih besar daripada sebelumnya di pergelangan yang sama. */
#define BEAT_CENTRE              20000
#define BEAT_BASELINE_SHIFT          8
#define BEAT_STALE_MS             4000

#define SPO2_PLAUSIBLE_MIN          70
#define SPO2_BUFFER_LEN            100

// Sensor menghasilkan 100 sampel/detik.
// Setiap empat sampel dirata-ratakan agar algoritma Maxim
// menerima sekitar 25 sampel/detik.
#define SPO2_AVERAGE_SAMPLES          4

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

extern int16_t IR_AC_Max;
extern int16_t IR_AC_Min;

static MAX30105 maxSensor;
static Adafruit_MPU6050 mpu6050;

static Ladder ladder;
static EventBuffer offline;

static HardwareSerial C6(1);

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
static int32_t  irBaseline = 0;

static uint32_t irBuf[SPO2_BUFFER_LEN];
static uint32_t redBuf[SPO2_BUFFER_LEN];

static uint16_t spo2Filled = 0;
static bool spo2Collecting = false;

static uint32_t spo2IrSum = 0;
static uint32_t spo2RedSum = 0;
static uint8_t spo2AverageCount = 0;

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

static volatile uint32_t buttonDownMs = 0;
static volatile bool buttonHeld = false;

static bool sosLatched = false;

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
static uint32_t lastC6Ms = 0;
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

static void IRAM_ATTR buttonISR() {
    if (digitalRead(PIN_BUTTON) == LOW) {
        buttonDownMs = millis();
        buttonHeld = true;
    } else {
        buttonHeld = false;
    }
}

static void buttonLoop() {
    if (!buttonHeld) {
        sosLatched = false;
        return;
    }

    if (sosLatched) {
        return;
    }

    if (millis() - buttonDownMs < SOS_HOLD_MS) {
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

    /* Nilai-nilai ini TERBUKTI di pergelangan: dengan setelan inilah `worn`
     * pernah bernilai 1 dan kualitas mencapai 4.
     *
     * Sempat kunaikkan arus LED ke 0xFF, kuperkecil rentang ADC ke 4096 nA,
     * dan kupindah laju cuplik ke 400 dengan rata-rata perangkat keras 4 —
     * empat perubahan sekaligus, dan sesudahnya worn serta kualitas jatuh ke
     * nol terus. Empat perubahan sekaligus berarti tidak ada yang tahu mana
     * yang bersalah, jadi semuanya dikembalikan.
     *
     * Kalau sensitivitas optik perlu dinaikkan lagi: ubah SATU angka, uji di
     * pergelangan, baru yang berikutnya. */
    maxSensor.setup(
        0x7F,   // LED brightness
        1,      // tanpa hardware average: 100 sampel/detik
        2,      // RED + IR
        100,    // sample rate
        411,    // pulse width
        16384   // ADC range
    );

    maxSensor.setPulseAmplitudeRed(
        0x7F
    );

    maxSensor.setPulseAmplitudeIR(
        0x7F
    );

    max30102Ready = true;

    Serial.println(
        F(" OK")
    );
}

// =============================================================
// WORN DAN SIGNAL QUALITY
// =============================================================

static void updateWornAndQuality(
    uint32_t ir
) {
    if (worn) {
        worn =
            ir >
            (IR_WORN_THRESHOLD * 4UL) / 5UL;
    } else {
        worn =
            ir >
            IR_WORN_THRESHOLD;
    }

    if (!worn) {
        opticalQuality = 0;
        signalQuality = 0;

        resetVitalData();

        spo2Value = 0;
        spo2ValueValid = false;

        if (spo2Collecting) {
            spo2Collecting = false;
            spo2Filled = 0;

            spo2IrSum = 0;
            spo2RedSum = 0;
            spo2AverageCount = 0;

            lastSpo2Ms = millis();
        }
    } else {
        uint32_t span =
            IR_QUALITY_FULL -
            IR_WORN_THRESHOLD;

        uint32_t above =
            ir -
            IR_WORN_THRESHOLD;

        opticalQuality =
            above >= span
            ? 15
            : (uint8_t)(
                (above * 15UL) / span
            );

        signalQuality =
            opticalQuality;

        bool stale =
            lastBeatMs == 0 ||
            millis() - lastBeatMs >
                BEAT_STALE_MS;

        if (
            stale &&
            signalQuality > 4
        ) {
            signalQuality = 4;
        }
    }

#if IR_CALIBRATION
    static uint32_t lastPrintMs = 0;

    if (millis() - lastPrintMs >= 500) {
        lastPrintMs = millis();

        /* Dicetak SELALU, bukan hanya saat worn. Angka inilah yang diuji
         * checkForBeat terhadap jendela 20..1000, jadi menyembunyikannya
         * justru ketika sensor belum dianggap dipakai berarti menutup satu-
         * satunya petunjuk kenapa ia belum dianggap dipakai. */
        int ac = (int)(IR_AC_Max - IR_AC_Min);

        Serial.printf(
            "[OPTIK] IR_mentah=%lu  dipakai=%s  sinyal=%u/15  "
            "amplitudo_AC=%d (perlu 20..1000)\n",
            (unsigned long)ir,
            worn ? "YA" : "TIDAK",
            signalQuality,
            ac
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

static void collectSpO2Sample(
    uint32_t ir,
    uint32_t red
) {
    if (
        !spo2Collecting ||
        spo2Filled >= SPO2_BUFFER_LEN
    ) {
        return;
    }

    spo2IrSum += ir;
    spo2RedSum += red;
    spo2AverageCount++;

    if (
        spo2AverageCount <
        SPO2_AVERAGE_SAMPLES
    ) {
        return;
    }

    irBuf[spo2Filled] =
        spo2IrSum /
        SPO2_AVERAGE_SAMPLES;

    redBuf[spo2Filled] =
        spo2RedSum /
        SPO2_AVERAGE_SAMPLES;

    spo2Filled++;

    spo2IrSum = 0;
    spo2RedSum = 0;
    spo2AverageCount = 0;
}

// =============================================================
// TAMBAH DATA BLE VITAL
// =============================================================

static void appendVitalSample(
    float bpm,
    uint16_t rr_ms
) {
    if (vitalsCount >= 5) {
        return;
    }

    vitalsBatch[vitalsCount][0] =
        (uint8_t)(
            bpm > 255.0f
            ? 255
            : bpm
        );

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
    updateWornAndQuality(
        ir
    );

    if (worn) {
        collectSpO2Sample(
            ir,
            red
        );
    }

    // Detektor tetap menerima sampel saat sensor tidak dipakai
    // agar filter DC internal tidak berhenti.
    if (irBaseline == 0) {
        irBaseline = ir;
    }

    irBaseline +=
        ((int32_t)ir - (int32_t)irBaseline) >>
        BEAT_BASELINE_SHIFT;

    int32_t beatInput =
        (int32_t)BEAT_CENTRE +
        ((int32_t)ir - (int32_t)irBaseline);

    /* averageDCEstimator menerima uint16_t, jadi apa pun di atas 65535
     * terpotong dan denyut palsu muncul dari luapan. */
    if (beatInput < 0) {
        beatInput = 0;
    }
    if (beatInput > 60000) {
        beatInput = 60000;
    }

    bool beatDetected =
        checkForBeat(
            beatInput
        );

    if (
        !worn ||
        !beatDetected
    ) {
        return;
    }

    uint32_t now =
        millis();

    if (previousPeakMs == 0) {
        previousPeakMs = now;
        return;
    }

    uint32_t rawDelta =
        now -
        previousPeakMs;

    // Puncak terlalu dekat kemungkinan puncak palsu.
    // previousPeakMs tidak diperbarui agar puncak berikutnya
    // tetap dihitung dari puncak sebelumnya.
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

    // Interval terlalu panjang dipakai untuk sinkronisasi ulang,
    // tetapi tidak dimasukkan ke BPM maupun riwayat RR.
    if (rawDelta > cfg.rr_max_ms) {
        resetVitalData();
        previousPeakMs = now;

#if IR_CALIBRATION
        Serial.printf(
            "[DENYUT] jarak=%lu ms  DITOLAK: terlalu renggang\n",
            (unsigned long)rawDelta
        );
#endif
        return;
    }

    previousPeakMs = now;

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

    // Interval lolos pemeriksaan dasar dan boleh dipakai
    // untuk memperbarui BPM.
    lastBeatMs = now;

    if (bpmBeatCount < 255) {
        bpmBeatCount++;
    }

    float bpm =
        60000.0f /
        (float)rawDelta;

    avgBPM =
        avgBPM < 1.0f
        ? bpm
        : (
            avgBPM * 0.8f +
            bpm * 0.2f
        );

    bool intervalValid =
        rrConsistentWithHistory(
            (uint16_t)rawDelta
        );

    if (!intervalValid) {
        rrValid = false;
        lastRRms = 0;

        // BPM tetap dikirim, tetapi RR nol menandakan
        // interval ini tidak boleh dipakai.
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

    if (
        !spo2Collecting &&
        worn &&
        millis() - lastSpo2Ms >=
            (uint32_t)
            cfg.spo2_sample_interval_s *
            1000UL
    ) {
        spo2Collecting = true;
        spo2Filled = 0;

        spo2IrSum = 0;
        spo2RedSum = 0;
        spo2AverageCount = 0;

        return;
    }

    if (
        !spo2Collecting ||
        spo2Filled < SPO2_BUFFER_LEN
    ) {
        return;
    }

    if (!worn) {
        spo2Collecting = false;
        spo2Filled = 0;

        spo2IrSum = 0;
        spo2RedSum = 0;
        spo2AverageCount = 0;

        spo2Value = 0;
        spo2ValueValid = false;
        lastSpo2Ms = millis();

        return;
    }

    int32_t spo2 = 0;
    int32_t hr = 0;

    int8_t spo2Valid = 0;
    int8_t hrValid = 0;

    maxim_heart_rate_and_oxygen_saturation(
        irBuf,
        SPO2_BUFFER_LEN,
        redBuf,
        &spo2,
        &spo2Valid,
        &hr,
        &hrValid
    );

    Serial.printf(
        "[SPO2] saturasi=%ld%%  sah=%s  denyut_versi_spo2=%ld  "
        "sah=%s\n",
        (long)spo2,
        spo2Valid ? "YA" : "TIDAK",
        (long)hr,
        hrValid ? "YA" : "TIDAK"
    );

    bool plausible =
        spo2Valid &&
        spo2 >= SPO2_PLAUSIBLE_MIN &&
        spo2 <= 100;

    spo2ValueValid =
        plausible;

    spo2Value =
        plausible
        ? (uint8_t)spo2
        : 0;

    BLE_NotifyOxygen(
        spo2Value,
        bodyPosition
    );

    spo2Collecting = false;
    spo2Filled = 0;

    spo2IrSum = 0;
    spo2RedSum = 0;
    spo2AverageCount = 0;

    lastSpo2Ms = millis();
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

static void anomalyLoop() {
    if (!bpmIsValid()) {
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

    if (outOfBand) {
        ladder.anomaly(
            millis(),
            REASON_THRESHOLD
        );
    } else if (irregular) {
        ladder.anomaly(
            millis(),
            REASON_IRREGULAR
        );
    } else {
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

    packet[0] =
        (uint8_t)(
            (worn ? 0x80 : 0x00) |
            (signalQuality & 0x0F)
        );

    uint8_t count =
        vitalsCount;

    if (count == 0) {
        packet[1] = 0;
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
    Serial.printf(
        "[CMD] %s\n",
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
            "[CMD] JSON tidak valid"
        );

        return;
    }

    const char *command =
        document["cmd"] | "";

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
// UART ESP32-C3 -> ESP32-C6
// =============================================================

/*
 * Format:
 *
 * RP,
 * bpm,
 * rr_ms,
 * spo2,
 * motion_mg,
 * position,
 * worn,
 * quality,
 * stage,
 * reason,
 * battery,
 * ble_link,
 * bpm_valid,
 * rr_valid,
 * spo2_valid
 *
 * Contoh:
 *
 * RP,81,752,97,92,0,1,15,0,0,255,0,1,1,1
 */

static void c6Loop() {
    if (
        millis() -
        lastC6Ms <
        C6_INTERVAL_MS
    ) {
        return;
    }

    lastC6Ms = millis();

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

    char line[128];

    snprintf(
        line,
        sizeof(line),
        "RP,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u",
        (unsigned)bpm,
        (unsigned)rr,
        (unsigned)spo2,
        (unsigned)motionMilliG,
        (unsigned)bodyPosition,
        worn ? 1U : 0U,
        (unsigned)signalQuality,
        (unsigned)ladder.stage(),
        (unsigned)ladder.reason(),
        (unsigned)batteryPercent(),
        BLE_Connected() ? 1U : 0U,
        bpmValid ? 1U : 0U,
        currentRRValid ? 1U : 0U,
        spo2ValueValid ? 1U : 0U
    );

    C6.println(
        line
    );

#if SERIAL_HEARTBEAT
    /* Baris "RP,..." di atas adalah untuk jam tangan, bukan untuk mata
     * manusia: empat belas angka telanjang tanpa satu pun label. Yang
     * dikirim ke jam tetap apa adanya karena parsernya menghitung koma;
     * yang dicetak ke serial monitor adalah versi berlabelnya. */
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

    C6.begin(
        C6_BAUD,
        SERIAL_8N1,
        PIN_UART_IN,
        PIN_UART_OUT
    );

    pinMode(
        PIN_BUTTON,
        INPUT_PULLUP
    );

    attachInterrupt(
        digitalPinToInterrupt(
            PIN_BUTTON
        ),
        buttonISR,
        CHANGE
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

    c6Loop();
}