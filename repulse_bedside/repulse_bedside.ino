/*
 * ═══════════════════════════════════════════════════════════════
 *  RePulse Bedside — ESP32-C3
 * ═══════════════════════════════════════════════════════════════
 *
 *  Peran: bedside adalah "RePulse Bedside" di BLE_GATT_CONTRACT.md §4.
 *  Ia mengukur kamar, menjalankan aktuator yang diperintahkan aplikasi, dan
 *  melapor selesai. Ia tidak pernah memutuskan apa pun — dengan satu
 *  pengecualian di §2.1: kalau gelang menyiarkan tahap >= 3 dan tidak ada HP
 *  di mana pun, bedside membunyikan sirene atas kemauannya sendiri.
 *
 *  Sensor  : BH1750 (lux), DHT11 (suhu/RH), INMP441 (mikrofon I2S)
 *  Aktuator: WS2812 (lampu), DFPlayer Mini (white noise + sirene),
 *            diffuser aroma lewat modul relay di GPIO20 (lihat PIN_AROMA)
 *
 *  Library (Arduino Library Manager):
 *    1. NimBLE-Arduino  >= 2.0   — h2zero
 *    2. ArduinoJson     >= 7.0   — Benoit Blanchon
 *    3. BH1750                   — Christopher Laws
 *    4. DHT sensor library       — Adafruit
 *    5. Adafruit NeoPixel        — Adafruit
 *    6. DFRobotDFPlayerMini      — DFRobot
 *
 *  Board: ESP32C3 Dev Module — USB CDC On Boot WAJIB Enabled
 * ═══════════════════════════════════════════════════════════════
 */

#include <Arduino.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <BH1750.h>
#include <DHT.h>
#include <Adafruit_NeoPixel.h>
#include <DFRobotDFPlayerMini.h>
#include <ESP_I2S.h>

#include "ble_bedside.h"
#include "snore.h"
#include "aroma.h"
#include "light.h"

// ═══════════════════════════════════════════════════════════════
//  PIN — sesuai tabel wiring bedside
// ═══════════════════════════════════════════════════════════════

/* Mengikuti rangkaian yang sudah terpasang, bukan sebaliknya. Sempat
 * tertukar terhadap firmware — INMP441 menerima aba-aba frame di kaki
 * clock-nya dan detak di kaki frame-nya, jadi ia tidak pernah mengeluarkan
 * satu bit sah pun. Gejalanya nol mutlak di setiap sampel, sama persis
 * dengan kabel putus, dan keempat pemeriksaan kabel lolos karena memang
 * semuanya tersambung — hanya ke lubang yang salah. */
#define PIN_MIC_BCLK       5    // INMP441 SCK
#define PIN_MIC_WS         4    // INMP441 WS / LRCK
#define PIN_MIC_DIN        6    // INMP441 SD / DOUT

#define PIN_I2C_SCL        7    // BH1750
#define PIN_I2C_SDA        9    // BH1750 — strapping pin, lihat catatan bawah

#define PIN_LED            8    // WS2812 DIN — strapping pin, lihat catatan
#define PIN_DHT           10    // DHT11 DATA, pull-up 10k ke 3V3

#define PIN_DFPLAYER_RX    2    // ke TX DFPlayer
#define PIN_DFPLAYER_TX    3    // ke RX DFPlayer, lewat resistor 1k

/* -1 = diffuser tidak punya pin kendali, sesuai keputusan 19 Agustus 2026.
 * Perintah `aroma` dibalas status = 1 (gagal), bukan status = 0 — firmware
 * tidak boleh melapor "selesai" untuk sesuatu yang tidak pernah ia jalankan.
 *
 * ⚠ Konsekuensi keselamatan yang perlu dicatat: diffuser yang dicolok
 * langsung ke 5V menyala sepanjang malam. Justru itu yang dilarang §4.3 —
 * ruangan jadi lembap dan berisiko mengiritasi saluran napas. Selama belum
 * ada MOSFET/relay, cabut diffusernya saat tidur.
 *
 * Sekarang ADA: modul relay di GPIO20.
 *
 * ⚠ GPIO20 adalah U0RXD — kaki RX UART0 bawaan ESP32-C3. Aman dipakai di
 * sini hanya karena CDCOnBoot=cdc memindahkan Serial ke USB, jadi UART0
 * tidak dipakai untuk log. Tapi ROM bootloader tetap menyentuh pasangan
 * 20/21 sesaat setelah reset, dan di jendela itu pin kita belum jadi
 * OUTPUT — ia mengambang. Modul relay aktif-LOW membaca kaki mengambang
 * sebagai LOW dan menyalakan diffuser di setiap boot. Karena itu level
 * mati DITULIS SEBELUM pinMode() di setup(), supaya latch keluarannya
 * sudah benar pada detik pin itu menjadi keluaran.
 *
 * AROMA_ACTIVE_LEVEL — DIUKUR, bukan diasumsikan. Modul relay yang beredar
 * kebanyakan aktif LOW dan itulah yang kutebak semula; modul di meja ini
 * ternyata aktif HIGH. Gejalanya tegas dan tidak bisa disalahartikan:
 * relay menyala saat boot, lalu MATI ketika perintah aroma dikirim — tepat
 * kebalikan dari yang seharusnya.
 *
 * Kalau kelak modulnya diganti dan gejala itu muncul lagi, baliklah define
 * ini dan tidak ada lagi yang perlu disentuh. Salah polaritas di sini bukan
 * ketidaknyamanan: §4.3 melarang diffuser menyala terus-menerus karena
 * risikonya nyata bagi penderita asma, dan polaritas terbalik berarti
 * diffuser menyala sepanjang malam kecuali seseorang memerintahkannya. */
#define PIN_AROMA         20
#define AROMA_ACTIVE_LEVEL HIGH

/* ⚠ CATATAN STRAPPING PIN ESP32-C3: GPIO2, GPIO8, dan GPIO9 ikut menentukan
 * mode boot. GPIO9 aman karena pull-up I2C BH1750 menahannya HIGH. GPIO8
 * (WS2812 DIN) dan GPIO2 (RX dari DFPlayer) mengambang saat boot — pasang
 * resistor pull-up 10k ke 3V3 di keduanya, supaya papan tidak gagal boot
 * tergantung modul mana yang menyala lebih dulu. */

// ═══════════════════════════════════════════════════════════════
//  KNOB KALIBRASI
// ═══════════════════════════════════════════════════════════════

#define LED_COUNT             60
/* 60 LED putih penuh menarik ~3,6 A — adaptor 5V 2A akan drop dan papan
 * ikut reboot. Batas 100 menahannya di sekitar 1,4 A. Naikkan hanya setelah
 * mengukur arus sungguhan, atau setelah ganti adaptor 5V 5A. */
#define LED_MAX_BRIGHTNESS   100

#define MIC_SAMPLE_RATE    16000
#define MIC_BLOCK_SAMPLES    256
/* Offset kalibrasi dB — diturunkan dari lembar data INMP441, bukan ditebak.
 *
 * Sensitivitas INMP441: −26 dBFS pada 94 dB SPL. Jadi:
 *     SPL = dBFS + 120
 *
 * Skala penuh jalur kita: sampel 24 bit rata-kiri di dalam 32 bit, lalu
 * digeser >>14, sehingga puncak skala penuh = 2^31 / 2^14 = 131072. Karena
 * dBFS mengacu pada sinus skala penuh, RMS-nya = 131072 / √2 = 92682.
 *
 *     dBFS(rms) = 20·log10(rms / 92682)
 *     SPL       = 20·log10(rms) − 99,34 + 120
 *               = 20·log10(rms) + 20,66
 *
 * Angka 26,0 yang lama tidak berasal dari mana pun — aku mengarangnya di
 * meja. Yang ini bisa ditelusuri sampai ke lembar data, dan kalau kelak
 * ada yang mengukurnya dengan sound meter, selisihnya akan kecil dan
 * bermakna alih-alih membetulkan tebakan dengan tebakan lain.
 *
 * Sisa biasnya jujur disebut di sini: yang dilaporkan adalah PUNCAK RMS
 * 16 ms selama jendela 100 ms, sedangkan sound meter menampilkan rata-rata
 * lambat. Di ruangan berdenyut bacaan kita akan beberapa dB di atasnya —
 * dan itu memang yang diminta §4.2, yang mencari puncak berulang. */
#define MIC_DB_OFFSET         20.66f

/* §4.1 menyebut "lux tiap 1 menit". Itu batas BAWAH kesegaran untuk satu
 * malam, bukan batas atas laju — dan sepuluh detik bukan biaya bagi
 * perangkat yang dicolok ke listrik.
 *
 * Satu menit membuat sensor mustahil diperiksa. Orang menutup BH1750
 * dengan telapak tangan, menatap layar, tidak terjadi apa-apa, lalu
 * menyimpulkan sensornya mati — padahal ia hanya belum waktunya bicara.
 * Sensor sehat yang tampak rusak selama lima puluh sembilan detik adalah
 * kesalahan yang jauh lebih mahal daripada notify tambahan per menit.
 *
 * Lima detik, bukan sepuluh: pada sepuluh detik, menutup sensor lalu
 * melihat layar masih terasa seperti menunggu jawaban yang tidak datang.
 * Dua belas notify per menit tetap tidak berarti apa-apa bagi perangkat
 * yang dicolok ke listrik. */
#define ROOM_INTERVAL_MS    5000    // §4.1 minimum 1 menit; ini jauh lebih rapat
#define DHT_INTERVAL_MS   300000    // §4.1 suhu/RH tiap 5 menit
#define SNORE_FEED_MS        100    // amplop ~10 Hz

/* Trek di microSD DFPlayer. 0001-0003 white noise, 0004 sirene. */
#define TRACK_WHITE_NOISE_BASE  1
#define TRACK_SIREN             4
#define VOLUME_SIREN           30    // maksimum, §PRD 6.3

// ═══════════════════════════════════════════════════════════════
//  Keadaan
// ═══════════════════════════════════════════════════════════════

static BH1750             lightMeter;
/* DHT11: resolusi cuma derajat bulat, jadi nilai °C×10 di characteristic
 * 0001 selalu berakhiran 0. Kontrak menerimanya, tapi tren suhu semalam
 * jadi bertangga. Ganti ke DHT22 di sini kalau sensornya diupgrade. */
static DHT                dht(PIN_DHT, DHT11);
static Adafruit_NeoPixel  strip(LED_COUNT, PIN_LED, NEO_GRB + NEO_KHZ800);
static DFRobotDFPlayerMini dfplayer;
static HardwareSerial     DfSerial(1);
static I2SClass           i2s;

static SnoreDetector snore;
static AromaLimiter  aromaLimit;

static bool  bh1750Ready = false;
static bool  dfReady     = false;
static bool  micReady    = false;

// Kamar
static float    tempC     = 0.0f;
static float    humidity  = 0.0f;
static uint32_t lux_x100  = 0;
static uint8_t  roomDb    = 0;

// Lampu — §4.3 peredupan eksponensial
static char     lightMode[12] = "off";
static uint16_t lightKelvin   = 2700;
static uint8_t  lightFrom     = 0;
static uint8_t  lightTo       = 0;
static uint32_t lightStartMs  = 0;
static uint32_t lightRampMs   = 0;

// Aroma
static uint32_t aromaOffAtMs = 0;

// Sirene
static bool sirenOn      = false;
static bool sirenIsOurs  = false;   // dinyalakan bedside sendiri, §2.1

// Dengkuran
static bool lastSnoreFlag = false;

// Tempo
static uint32_t lastRoomMs = 0, lastDhtMs = 0, lastSnoreFeedMs = 0;
/* Hanya untuk mendiagnosis mikrofon — dicetak lalu dinolkan tiap [ROOM]. */
static uint32_t micBlocks = 0, micShort = 0;
static int32_t  micRawPeak = 0;
static uint32_t lastBandSeenMs = 0;

// ═══════════════════════════════════════════════════════════════
//  Lampu
// ═══════════════════════════════════════════════════════════════

static void lightApply(uint8_t brightness) {
    Rgb c = light_color(lightMode, lightKelvin, brightness);
    for (uint16_t i = 0; i < LED_COUNT; i++) strip.setPixelColor(i, c.r, c.g, c.b);
    strip.show();
}

static void lightSet(const char *mode, uint16_t kelvin, uint8_t brightness, uint32_t ramp_s) {
    strncpy(lightMode, mode, sizeof(lightMode) - 1);
    lightMode[sizeof(lightMode) - 1] = '\0';
    lightKelvin  = kelvin;
    lightFrom    = lightTo;
    lightTo      = brightness > LED_MAX_BRIGHTNESS ? LED_MAX_BRIGHTNESS : brightness;
    lightStartMs = millis();
    lightRampMs  = ramp_s * 1000u;
    if (lightRampMs == 0) lightApply(lightTo);
}

/* Menulis 60 LED memakan ~1,8 ms dan memblok. Peredupan 25 menit tidak butuh
 * pembaruan seribu kali sedetik — 25 Hz sudah jauh lebih halus daripada yang
 * bisa dilihat mata, dan menyisakan loop untuk mikrofon dan BLE. */
#define LIGHT_REFRESH_MS 40

static void lightLoop() {
    if (lightRampMs == 0) return;
    /* Darurat memiliki strip, tanpa berbagi. Peredupan sunset berjalan 25
     * menit, jadi anomali di tengahnya menemukan ramp yang masih hidup —
     * dan keduanya menulis strip yang sama. lightLoop menyala tiap 40 ms,
     * kedipan darurat tiap 400 ms, jadi yang menang adalah yang meredup:
     * kedipan putih §PRD 6.3 menyusut jadi kedipan 40 ms sekali sedetik,
     * nyaris tak terlihat, tepat pada saat ia paling harus terlihat. */
    if (sirenOn) return;
    static uint32_t lastPaintMs = 0;
    if (millis() - lastPaintMs < LIGHT_REFRESH_MS) return;
    lastPaintMs = millis();

    uint32_t elapsed = millis() - lightStartMs;
    float    progress = (float)elapsed / (float)lightRampMs;
    if (progress >= 1.0f) {
        progress    = 1.0f;
        lightRampMs = 0;
    }
    lightApply(light_ramp(lightFrom, lightTo, progress));
}

/* §PRD 6.3: saat darurat, lampu putih 100% berkedip. */
static void alertFlashLoop() {
    if (!sirenOn) return;
    bool on = (millis() / 400) % 2 == 0;

    // Sama seperti lightLoop: tulis hanya saat keadaannya benar-benar berubah,
    // bukan 250 kali per kedipan.
    static int8_t lastFlash = -1;
    if (on == (lastFlash == 1)) return;
    lastFlash = on ? 1 : 0;

    for (uint16_t i = 0; i < LED_COUNT; i++) {
        strip.setPixelColor(i, on ? LED_MAX_BRIGHTNESS : 0,
                               on ? LED_MAX_BRIGHTNESS : 0,
                               on ? LED_MAX_BRIGHTNESS : 0);
    }
    strip.show();
}

// ═══════════════════════════════════════════════════════════════
//  White noise — §4.4 fade_s
// ═══════════════════════════════════════════════════════════════
/* Aplikasi mengirim fade_s di setiap perintah white noise, dan firmware
 * membuangnya — suara muncul penuh di ketukan pertama. Untuk perangkat yang
 * dinyalakan tepat ketika seseorang mulai terlelap, itu bukan detail kosmetik:
 * bunyi yang datang seketika pada volume 20 justru membangunkan orang yang
 * hendak ditenangkannya.
 *
 * Volume DFPlayer adalah perintah serial 9600 baud, jadi ia tidak boleh
 * ditulis tiap putaran loop. Satu langkah tiap 200 ms sudah lebih halus
 * daripada yang bisa didengar telinga pada tangga 0–30, dan fade 30 detik
 * hanya menjadi tiga puluh penulisan. */

#define NOISE_STEP_MS   200

static bool     noiseOn        = false;
static uint8_t  noiseCurVol    = 0;     // volume yang benar-benar ada di modul
static uint8_t  noiseTargetVol = 0;
static uint8_t  noiseFromVol   = 0;
static uint32_t noiseFadeMs    = 0;     // 0 = tidak ada fade berjalan
static uint32_t noiseFadeStart = 0;
static bool     noiseStopAtEnd = false;

/* DFPlayer bicara meski tidak menjawab ACK.
 *
 * Klon yang mengabaikan reset tetap mengirim pesan tak diminta lewat TX-nya:
 * kartu terpasang, trek selesai, dan — yang paling berharga saat tidak ada
 * bunyi — kode error. Firmware ini tidak pernah membacanya sekali pun, jadi
 * "berkas tidak ditemukan" dan "speaker salah pasang" terlihat persis sama
 * dari luar: sunyi.
 *
 * Dikuras tiap putaran loop dan dicetak apa adanya. Satu baris di serial
 * lebih murah daripada satu jam membongkar kabel yang sudah benar. */
/* Modul ini tidak menghormati perintah loop.
 *
 * `loop(n)` adalah 0x08, "putar berulang", dan sebagian klon memperlakukannya
 * sebagai putar sekali. Gejalanya menipu: white noise berdurasi 15 menit
 * terlihat seperti berhasil berulang, sementara sirene 8 detik berhenti
 * sendiri di tengah keadaan darurat — kegagalan yang cuma kelihatan pada
 * berkas pendek, yaitu justru berkas yang paling penting.
 *
 * Jadi pengulangannya dipegang firmware, bukan modul, lewat dua jalur:
 *
 *   Pesan trek-selesai. Bersih dan tepat waktu, tapi hanya ada kalau kaki TX
 *   modul benar-benar sampai ke GPIO2 — dan sepanjang satu malam ternyata
 *   tidak, karena GND-nya tidak pernah mencapai papan.
 *
 *   Pengawas waktu. Kasar, tapi tetap bekerja pada modul yang sama sekali
 *   bisu. Ia yang menjaga janji §6.3 kalau jalur balik itu putus lagi. */

#define SIREN_TRACK_MS      8000   // panjang berkas sirene; sesuaikan bila diganti
#define REPLAY_GUARD_MS      400   // jangan memicu ulang dua kali untuk satu akhir

static uint8_t  dfTrack     = 0;   // 0 = tidak ada yang harus terus berbunyi
static uint32_t dfStartedMs = 0;

static void dfPlayLooping(uint8_t track) {
    dfTrack     = track;
    dfStartedMs = millis();
    dfplayer.loop(track);
}

static void dfStopLooping() {
    dfTrack = 0;
    dfplayer.stop();
}

/* Hanya sirene yang diawasi jam. White noise berdurasi menit dan tidak punya
 * panjang yang kita ketahui, jadi memicunya ulang berdasarkan tebakan waktu
 * justru memotongnya di tengah — untuk itu pesan trek-selesai sudah cukup,
 * dan jeda seperseratus detik tiap seperempat jam tidak membangunkan siapa
 * pun. Sirene yang diam empat detik membangunkan siapa pun juga tidak. */
static void dfReplayLoop() {
    if (dfTrack != TRACK_SIREN) return;
    if (millis() - dfStartedMs < SIREN_TRACK_MS) return;
    dfStartedMs = millis();
    dfplayer.loop(dfTrack);
}

static void dfLoop() {
    if (!dfplayer.available()) return;

    uint8_t type = dfplayer.readType();
    int     val  = dfplayer.read();

    switch (type) {
        case DFPlayerCardOnline:   Serial.println(F("[DF] kartu SD terbaca")); break;
        case DFPlayerCardInserted: Serial.println(F("[DF] kartu dimasukkan")); break;
        case DFPlayerCardRemoved:  Serial.println(F("[DF] kartu DICABUT")); break;
        case DFPlayerPlayFinished:
            Serial.printf("[DF] trek %d selesai" "\n", val);
            /* Guard: sebagian modul mengirim pesan ini dua kali untuk satu
             * akhir, dan memutar ulang dua kali menghasilkan trek yang
             * terpotong seketika. */
            if (dfTrack != 0 && millis() - dfStartedMs > REPLAY_GUARD_MS) {
                dfStartedMs = millis();
                dfplayer.loop(dfTrack);
            }
            break;
        case DFPlayerFeedBack:     Serial.printf("[DF] balasan %d" "\n", val); break;
        case WrongStack:           Serial.println(F("[DF] paket rusak - periksa kabel TX/RX")); break;
        case TimeOut:              Serial.println(F("[DF] tidak menjawab dalam waktunya")); break;
        case DFPlayerError:
            switch (val) {
                case Busy:             Serial.println(F("[DF] ERROR: sibuk / kartu tidak ditemukan")); break;
                case Sleeping:         Serial.println(F("[DF] ERROR: modul tidur")); break;
                case SerialWrongStack: Serial.println(F("[DF] ERROR: paket serial salah")); break;
                case CheckSumNotMatch: Serial.println(F("[DF] ERROR: checksum tidak cocok")); break;
                case FileIndexOut:     Serial.println(F("[DF] ERROR: NOMOR TREK DI LUAR JANGKAUAN")); break;
                case FileMismatch:     Serial.println(F("[DF] ERROR: BERKAS TIDAK DITEMUKAN")); break;
                case Advertise:        Serial.println(F("[DF] ERROR: sedang memutar iklan")); break;
                default:               Serial.printf("[DF] ERROR kode %d" "\n", val); break;
            }
            break;
        default: Serial.printf("[DF] tipe %u nilai %d" "\n", type, val); break;
    }
}

static void noiseApply(uint8_t vol) {
    if (vol == noiseCurVol) return;
    noiseCurVol = vol;
    dfplayer.volume(vol);
}

/* Sirene merebut DFPlayer tanpa lewat sini, jadi setiap fade yang sedang
 * berjalan harus dibatalkan — kalau tidak, langkah fade berikutnya akan
 * menurunkan volume sirene di tengah keadaan darurat. */
static void noiseCancel(uint8_t volNowOnModule) {
    noiseOn        = false;
    noiseFadeMs    = 0;
    noiseStopAtEnd = false;
    noiseCurVol    = volNowOnModule;
    noiseTargetVol = volNowOnModule;
}

static void noiseSet(bool on, uint8_t level0_3, uint8_t track, uint16_t fade_s) {
    if (!dfReady) return;
    if (level0_3 > 3) level0_3 = 3;
    uint8_t target = on ? (uint8_t)(level0_3 * 10) : 0;

    if (on && !noiseOn) {
        /* Mulai dari senyap, lalu naik. Menyalakan trek dulu pada volume
         * lama berarti satu ketukan keras sebelum fade sempat mulai. */
        noiseApply(0);
        dfPlayLooping(TRACK_WHITE_NOISE_BASE + (track > 0 ? track : 1) - 1);
    }

    noiseOn        = on;
    noiseStopAtEnd = !on;
    noiseFromVol   = noiseCurVol;
    noiseTargetVol = target;
    noiseFadeStart = millis();
    noiseFadeMs    = (uint32_t)fade_s * 1000UL;

    /* fade_s = 0 berarti sekarang juga. Panel uji memakainya untuk
     * mendengar tiap langkah volume tanpa menunggu setengah menit. */
    if (noiseFadeMs == 0) {
        noiseApply(target);
        if (noiseStopAtEnd) dfStopLooping();
    }

    Serial.printf("[NOISE] %s vol %u→%u fade %us\n",
                  on ? "menyala" : "mati", noiseFromVol, target, fade_s);
}

static void noiseLoop() {
    if (!dfReady || noiseFadeMs == 0) return;

    static uint32_t lastStepMs = 0;
    if (millis() - lastStepMs < NOISE_STEP_MS) return;
    lastStepMs = millis();

    uint32_t elapsed = millis() - noiseFadeStart;
    uint8_t  vol;
    if (elapsed >= noiseFadeMs) {
        vol = noiseTargetVol;
    } else {
        int32_t span = (int32_t)noiseTargetVol - (int32_t)noiseFromVol;
        vol = (uint8_t)((int32_t)noiseFromVol +
                        span * (int32_t)elapsed / (int32_t)noiseFadeMs);
    }
    noiseApply(vol);

    if (noiseCurVol == noiseTargetVol) {
        noiseFadeMs = 0;
        /* Baru berhenti setelah senyap. Memanggil stop() saat perintah
         * datang akan memotong fade turun sebelum bunyi pertama meredup. */
        if (noiseStopAtEnd) {
            dfStopLooping();
            noiseStopAtEnd = false;
            Serial.println("[NOISE] senyap, trek berhenti");
        }
    }
}

// ═══════════════════════════════════════════════════════════════
//  Sirene — tidak punya pin sendiri
// ═══════════════════════════════════════════════════════════════
/* Sirene diperankan DFPlayer pada volume maksimum ditambah lampu putih
 * berkedip. ponytail: tidak ada GPIO tersisa di tabel wiring dan speaker
 * 3W pada volume 30 sudah cukup keras untuk membangunkan orang. Kalau
 * belakangan dipasang sirene 12V terpisah, ia butuh satu pin lagi.
 *
 * Ini juga alasan `white_noise` dan `siren` tidak bisa berbunyi bersamaan —
 * dan itu justru benar: §PRD 6.3 memang mematikan white noise saat darurat. */
static void sirenSet(bool on, bool ours) {
    if (on == sirenOn) return;
    sirenOn     = on;
    sirenIsOurs = on ? ours : false;

    if (!dfReady) {
        Serial.println("[SIREN] DFPlayer tidak siap — sirene tidak berbunyi");
        return;
    }
    if (on) {
        noiseCancel(VOLUME_SIREN);
        dfplayer.volume(VOLUME_SIREN);
        dfPlayLooping(TRACK_SIREN);
        Serial.printf("[SIREN] MENYALA (%s)\n", ours ? "mandiri §2.1" : "perintah aplikasi");
    } else {
        dfStopLooping();
        noiseCancel(0);
        lightApply(lightTo);
        Serial.println("[SIREN] mati");
    }
}

// ═══════════════════════════════════════════════════════════════
//  Aroma
// ═══════════════════════════════════════════════════════════════

static void aromaOff() {
    if (PIN_AROMA >= 0) {
        digitalWrite(PIN_AROMA, AROMA_ACTIVE_LEVEL == HIGH ? LOW : HIGH);
    }
    aromaOffAtMs = 0;
}

static void aromaLoop() {
    if (aromaOffAtMs != 0 && millis() >= aromaOffAtMs) {
        aromaOff();
        Serial.println("[AROMA] selesai");
    }
}

// ═══════════════════════════════════════════════════════════════
//  §4.3 Perintah aktuator
// ═══════════════════════════════════════════════════════════════

static void onActuator(const char *json) {
    Serial.printf("[ACT] %s\n", json);

    JsonDocument doc;
    if (deserializeJson(doc, json)) {
        Serial.println("[ACT] JSON tidak valid");
        return;
    }
    uint8_t command_id = doc["command_id"] | 0;
    uint8_t status     = 0;                       // §4.4: 0 selesai

    if (doc["light"].is<JsonObject>()) {
        JsonObject l = doc["light"];
        lightSet(l["mode"] | "off", l["kelvin"] | 2700,
                 l["brightness"] | 0, l["ramp_s"] | 0);
    }

    if (doc["white_noise"].is<JsonObject>()) {
        JsonObject w = doc["white_noise"];
        if (!dfReady) {
            status = 1;                           // §4.4: 1 gagal
        } else {
            noiseSet(w["on"] | false,
                     (uint8_t)(w["volume"] | 2),
                     (uint8_t)(w["track"] | 1),
                     (uint16_t)(w["fade_s"] | 0));
        }
    }

    if (doc["aroma"].is<JsonObject>()) {
        JsonObject a = doc["aroma"];
        uint16_t seconds = a["duration_s"] | 0;

        if (!(a["on"] | false)) {
            aromaOff();
        } else if (seconds == 0 || seconds > AROMA_MAX_SECONDS) {
            /* §4.3: permintaan yang melampaui batas dibalas status = 2, BUKAN
             * diam-diam dijalankan lebih pendek. Uji §6 no. 4 memang dirancang
             * untuk menangkap firmware yang memotong diam-diam.
             *
             * Batas ini diperiksa lebih dulu daripada keberadaan pin: menolak
             * 60 detik adalah keputusan kebijakan, dan jawabannya sama saja
             * apakah diffusernya terpasang atau tidak. */
            status = 2;
            Serial.printf("[AROMA] DITOLAK — %u dtk melampaui batas %u dtk\n",
                          seconds, AROMA_MAX_SECONDS);
        } else if (PIN_AROMA < 0) {
            status = 1;                           // §4.4: 1 gagal
            Serial.println("[AROMA] tidak ada pin kendali — perintah gagal");
        } else {
            uint16_t granted = aromaLimit.request(millis(), seconds);
            if (granted == 0) {
                status = 2;
                Serial.println("[AROMA] DITOLAK — kuota malam ini habis");
            } else {
                digitalWrite(PIN_AROMA, AROMA_ACTIVE_LEVEL);
                aromaOffAtMs = millis() + (uint32_t)granted * 1000u;
                Serial.printf("[AROMA] %u dtk (kejadian ke-%u malam ini)\n",
                              granted, aromaLimit.eventsTonight());
            }
        }
    }

    if (doc["siren"].is<JsonObject>()) {
        sirenSet(doc["siren"]["on"] | false, false);
    }

    BLE_IndicateAck(command_id, status);
}

// ═══════════════════════════════════════════════════════════════
//  §2.1 Siaran gelang → sirene mandiri
// ═══════════════════════════════════════════════════════════════

/* Keempat syarat harus benar semua. Syarat 3 dan 4 terlihat mubazir tapi
 * menjaga kegagalan yang berbeda — yang satu "bedside kehilangan HP", yang
 * lain "gelang kehilangan HP". Bila salah satu masih punya HP, HP yang
 * berkuasa. Ini yang mencegah sirene diperintah dua tuan. */
static void onBandSeen(uint8_t stage, bool phone_connected, bool paired) {
    lastBandSeenMs = millis();

    bool should_sound = stage >= 3            // 1
                     && paired                // 2
                     && !BLE_Connected()      // 3
                     && !phone_connected;     // 4

    if (should_sound && !sirenOn) {
        sirenSet(true, true);
    } else if (!should_sound && sirenOn && sirenIsOurs) {
        // Hanya sirene yang kita nyalakan sendiri yang boleh kita matikan
        // sendiri. Sirene atas perintah aplikasi tetap milik aplikasi.
        sirenSet(false, false);
    }
}

// ═══════════════════════════════════════════════════════════════
//  Mikrofon — §4.2 dengkuran, dan dB untuk §4.1
// ═══════════════════════════════════════════════════════════════

/* SATU blok per putaran loop, dan sengaja TIDAK memakai i2s.available().
 *
 * available() di core ESP32 3.x tidak melaporkan isi buffer sama sekali —
 * ia mengembalikan konstanta I2S_READ_CHUNK_SIZE (1920). Blok kita 1024
 * byte, jadi `while (i2s.available() >= sizeof(block))` berbunyi
 * `while (1920 >= 1024)`: tidak pernah salah, tidak pernah selesai. Dan
 * readBytes memblokir sampai satu blok penuh terkumpul, jadi loop() tidak
 * pernah keluar dari fungsi ini sejak boot. Semua yang berada di bawahnya
 * — roomLoop, lightLoop, aromaLoop, alertFlashLoop, BLE_ScanLoop — belum
 * pernah berjalan satu kali pun. Tidak ada [ROOM], tidak ada [SNORE], lampu
 * tidak pernah meredup, gelang tidak pernah terlihat. Satu baris while
 * mematikan seluruh separuh perangkat, dan sisanya tetap terlihat sehat
 * karena BLE berjalan di task NimBLE sendiri.
 *
 * Satu blok 256 sampel pada 16 kHz adalah 16 ms audio, jadi membaca satu
 * blok per putaran memacu loop() pada laju mikrofon itu sendiri — cukup
 * sering untuk semua yang lain, dan tidak tertinggal selama sisa loop
 * lebih ringan dari 16 ms. Yang dilaporkan ke detektor tetap PUNCAK
 * selama jendela 100 ms, karena §4.2 mencari puncak berulang, bukan
 * rata-rata. */
static void micLoop() {
    if (!micReady) return;

    static int32_t block[MIC_BLOCK_SAMPLES];
    static uint8_t windowPeakDb = 0;

    if (i2s.readBytes((char *)block, sizeof(block)) == sizeof(block)) {
        micBlocks++;
        // INMP441 mengirim 24 bit yang rata kiri di dalam slot 32 bit.
        /* Langkah DUA, bukan satu.
         *
         * slot_mask minta I2S_STD_SLOT_LEFT, tapi driver tetap menyerahkan
         * kedua slot: 1251 blok per 10 detik pada 16 kHz adalah dua kali
         * lipat data yang seharusnya. Diukur dengan memisahkan indeks genap
         * dan ganjil, dan ganjil keluar NOL di setiap pembacaan tanpa
         * kecuali — mikrofon mengisi slot kiri, sisanya bantalan.
         *
         * Merata-ratakan bantalan itu bersama sinyal memotong RMS tepat
         * separuh, yaitu 3,01 dB yang hilang diam-diam. Kalibrasi
         * MIC_DB_OFFSET akan menyerapnya tanpa ada yang sadar, dan setiap
         * ambang di snore.h ikut miring sebesar itu selamanya. */
        /* Buang DC dulu, baru ukur.
         *
         * INMP441 punya offset diam yang tetap, dan RMS yang dihitung tanpa
         * membuangnya sebagian besar mengukur offset itu — bukan suara.
         * Gejalanya: nilai puncak yang berulang-ulang persis sama antar
         * jendela, dan desibel yang, diadu dengan lembar data, menyiratkan
         * kamar tidur seramai mesin pemotong rumput.
         *
         * Rerata per blok sudah cukup sebagai penghalang DC di sini: 128
         * sampel pada 16 kHz adalah 8 ms, jauh lebih pendek daripada amplop
         * dengkuran yang dicari §4.2, jadi ia membuang bias tanpa ikut
         * memakan sinyalnya. */
        double mean = 0;
        for (size_t i = 0; i < MIC_BLOCK_SAMPLES; i += 2) {
            mean += (double)(block[i] >> 14);
        }
        mean /= (MIC_BLOCK_SAMPLES / 2);

        double sum_sq = 0;
        for (size_t i = 0; i < MIC_BLOCK_SAMPLES; i += 2) {
            double s = (double)(block[i] >> 14) - mean;
            sum_sq += s * s;

            /* Puncak, diukur SETELAH DC dibuang. "0 dB" punya beberapa sebab
             * yang di layar terlihat sama — tidak ada blok, blok berisi nol,
             * atau matematikanya salah — dan angka ini memisahkannya. Puncak
             * yang masih berisi bias hanya melaporkan besar offsetnya. */
            int32_t peak = (int32_t)(s < 0 ? -s : s);
            if (peak > micRawPeak) micRawPeak = peak;
        }
        double rms = sqrt(sum_sq / (MIC_BLOCK_SAMPLES / 2));
        double db  = rms > 1.0 ? 20.0 * log10(rms) + MIC_DB_OFFSET : 0.0;
        if (db < 0)   db = 0;
        if (db > 255) db = 255;
        if ((uint8_t)db > windowPeakDb) windowPeakDb = (uint8_t)db;
    } else {
        micShort++;
    }

    if (millis() - lastSnoreFeedMs < SNORE_FEED_MS) return;
    lastSnoreFeedMs = millis();

    roomDb       = windowPeakDb;
    windowPeakDb = 0;

    snore.feed(millis(), roomDb);
    if (snore.flagged() != lastSnoreFlag) {
        lastSnoreFlag = snore.flagged();
        BLE_NotifySnore(lastSnoreFlag, snore.intensity());
        Serial.printf("[SNORE] %s intensitas=%u\n",
                      lastSnoreFlag ? "terdeteksi" : "berhenti", snore.intensity());
    }
}

// ═══════════════════════════════════════════════════════════════
//  §4.1 Sensor kamar
// ═══════════════════════════════════════════════════════════════

static void roomLoop() {
    /* Sebelum ada satu pun bacaan sah, coba tiap 2 detik. DHT11 butuh sekitar
     * sedetik setelah diberi daya, dan menunggu 5 menit penuh berarti demo
     * dimulai dengan 0,0°C di layar. Setelah dapat, baru turun ke 5 menit. */
    bool haveReading = humidity > 0.0f;
    uint32_t dhtInterval = haveReading ? DHT_INTERVAL_MS : 2000;

    if (lastDhtMs == 0 || millis() - lastDhtMs >= dhtInterval) {
        lastDhtMs = millis();
        float t = dht.readTemperature();
        float h = dht.readHumidity();
        if (!isnan(t)) tempC    = t;
        if (!isnan(h)) humidity = h;
    }

    if (millis() - lastRoomMs < ROOM_INTERVAL_MS && lastRoomMs != 0) return;
    lastRoomMs = millis();

    if (bh1750Ready) {
        float lx = lightMeter.readLightLevel();
        /* Dibaca sepanjang malam meski lampu sudah mati — ini dasar penandaan
         * gelap optimal, dan ambangnya < 3 lux. Karena itu dikirim ×100. */
        if (lx >= 0) lux_x100 = (uint32_t)(lx * 100.0f);
    }

    BLE_NotifyRoom((int16_t)(tempC * 10.0f), (uint16_t)(humidity * 10.0f),
                   lux_x100, roomDb);
    Serial.printf("[ROOM] %.1f°C %.1f%% %.2f lux %u dB\n",
                  tempC, humidity, lux_x100 / 100.0f, roomDb);
    Serial.printf("[MIC]  blok=%lu pendek=%lu puncak_mentah=%ld dB=%u" "\n",
                  (unsigned long)micBlocks, (unsigned long)micShort,
                  (long)micRawPeak, roomDb);
    micBlocks = micShort = 0;
    micRawPeak = 0;
}

// ═══════════════════════════════════════════════════════════════
//  setup / loop
// ═══════════════════════════════════════════════════════════════

void setup() {
    Serial.begin(115200);
    /* Sama seperti di gelang: port USB-CDC baru muncul di PC satu sampai
     * dua detik setelah reset, jadi delay(300) membuang seluruh banner boot
     * — termasuk baris TIDAK DITEMUKAN yang justru paling ingin kamu baca
     * saat menelusuri sensor yang belum terpasang. */
    for (uint32_t t0 = millis(); !Serial && millis() - t0 < 2000; ) delay(10);
    Serial.println(F("\n=== RePulse Bedside (ESP32-C3) ==="));

    if (PIN_AROMA >= 0) {
        /* Urutannya penting: latch keluaran diisi dulu, baru pin dijadikan
         * OUTPUT. Terbalik, dan ada satu denyut di mana pin menggerakkan
         * level bawaannya — pada relay aktif-LOW itu adalah diffuser yang
         * menyala sekejap di setiap boot. */
        digitalWrite(PIN_AROMA, AROMA_ACTIVE_LEVEL == HIGH ? LOW : HIGH);
        pinMode(PIN_AROMA, OUTPUT);
        aromaOff();
    } else {
        Serial.println("[AROMA] pin kendali belum ada — perintah aroma akan gagal");
    }

    strip.begin();
    strip.setBrightness(255);          // kecerahan diatur per-warna, bukan global
    strip.clear();
    strip.show();

    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    bh1750Ready = lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE);
    Serial.printf("[BH1750] %s\n", bh1750Ready ? "OK" : "TIDAK DITEMUKAN");

    dht.begin();

    DfSerial.begin(9600, SERIAL_8N1, PIN_DFPLAYER_RX, PIN_DFPLAYER_TX);

    /* Dua percobaan, karena "tidak menjawab" dan "tidak ada" bukan hal yang
     * sama.
     *
     * begin(..., isACK, doReset) mengirim perintah reset lalu menunggu
     * balasannya. Klon DFPlayer yang beredar — MH2024K, GD3200B — sering
     * tidak pernah membalas reset meski memutar berkas dengan sempurna,
     * dan dengan isACK menyala kegagalan itu jadi mutlak: modul sehat
     * dilaporkan hilang, dan sirene tidak akan pernah berbunyi.
     *
     * Percobaan kedua mematikan keduanya. Harganya nyata — tanpa ACK kita
     * tidak lagi tahu apakah sebuah perintah benar-benar diterima — jadi
     * ia hanya dipakai setelah cara yang jujur gagal, dan log mengatakan
     * mana yang berhasil supaya tidak ada yang salah menduga. */
    dfReady = dfplayer.begin(DfSerial, /*isACK=*/true, /*doReset=*/true);
    if (!dfReady) {
        Serial.println(F("[DFPLAYER] tidak membalas reset — coba tanpa ACK"));
        delay(200);
        dfReady = dfplayer.begin(DfSerial, /*isACK=*/false, /*doReset=*/false);
        /* JANGAN tulis "menjawab". begin() dengan isACK=false berakhir
         * `return ... || !isACK`, jadi ia mengembalikan true tanpa bertanya
         * apa pun ke modul — bahkan memalsukan status internal jadi "kartu
         * online". Kalimat lamanya berbunyi seperti konfirmasi, dan sunyi
         * total terbaca sebagai "modul sehat, pasti speakernya". */
        if (dfReady) Serial.println(F("[DFPLAYER] DIANGGAP ada tanpa ACK — modul belum pernah menjawab sepatah pun"));
    }
    Serial.printf("[DFPLAYER] %s\n", dfReady ? "dianggap siap" : "TIDAK DITEMUKAN");
    if (dfReady) {
        dfplayer.volume(20);
        /* Pertanyaan, bukan perintah. Klon yang tidak pernah membalas ACK
         * sering tetap menjawab pertanyaan - dan jawabannya memisahkan dua
         * kegagalan yang dari luar sama-sama sunyi: modul yang tidak membaca
         * kartu sama sekali, dan kartu terbaca tapi treknya salah nomor. */
        int files = dfplayer.readFileCounts();
        if (files > 0) Serial.printf("[DFPLAYER] %d berkas terbaca di kartu" "\n", files);
        else           Serial.println(F("[DFPLAYER] jumlah berkas tidak terjawab - kartu mungkin tidak terbaca"));
    }

    i2s.setPins(PIN_MIC_BCLK, PIN_MIC_WS, -1, PIN_MIC_DIN);
    micReady = i2s.begin(I2S_MODE_STD, MIC_SAMPLE_RATE,
                         I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO);
    Serial.printf("[INMP441] %s\n", micReady ? "OK" : "GAGAL");

    BedsideCallbacks cb = { onActuator, onBandSeen };
    BLE_Init(cb);

    Serial.println(F("[BOOT] Siap"));
}

void loop() {
    micLoop();
    dfLoop();
    dfReplayLoop();
    noiseLoop();
    roomLoop();
    lightLoop();
    aromaLoop();
    alertFlashLoop();
    BLE_ScanLoop();

    /* Gelang menghilang dari udara sama sekali. Sirene mandiri dimatikan —
     * tanpa siaran, syarat 1 dan 2 di §2.1 tidak bisa dibuktikan lagi. */
    if (sirenIsOurs && millis() - lastBandSeenMs > 15000) {
        sirenSet(false, false);
    }
}
