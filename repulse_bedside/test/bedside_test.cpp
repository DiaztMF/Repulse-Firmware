/*
 * Cek yang bisa dijalankan tanpa hardware:
 *
 *   cd test && g++ -std=c++17 -o bedside_test bedside_test.cpp && ./bedside_test
 *
 * Tiga hal yang kontraknya tegas dan gampang dilanggar diam-diam:
 * pola dengkuran bukan rata-rata desibel (§4.2), diffuser menolak bukan
 * memotong (§4.3), dan peredupan eksponensial bukan linear (§4.3).
 */
#include <cassert>
#include <cstdio>
#include "../snore.h"
#include "../aroma.h"
#include "../light.h"

/* Kipas: keras, tapi rata. Kontrak melarang ini dihitung sebagai dengkuran. */
static void test_steady_noise_is_not_snoring() {
    SnoreDetector d;
    for (uint32_t t = 0; t < 60'000; t += 100) d.feed(t, 55);
    assert(!d.flagged());
    assert(d.intensity() == 0);
}

/* Dengkuran: puncak tiap 3 detik = 0,33 Hz, di dalam pita 0,2-0,5 Hz. */
static void test_breathing_rhythm_is_snoring() {
    SnoreDetector d;
    for (uint32_t t = 0; t < 30'000; t += 100) {
        bool peak = (t % 3000) < 600;
        d.feed(t, peak ? 62 : 35);
    }
    assert(d.flagged());
    assert(d.intensity() > 0 && d.intensity() <= 100);
}

/* Terlalu cepat untuk napas — sendok jatuh, pintu, dengkur orang lain
 * yang tumpang tindih. Empat kali sedetik jauh di atas 0,5 Hz. */
static void test_fast_bursts_are_not_snoring() {
    SnoreDetector d;
    for (uint32_t t = 0; t < 30'000; t += 50) {
        bool peak = (t % 250) < 100;
        d.feed(t, peak ? 62 : 35);
    }
    assert(!d.flagged());
}

/* Orangnya berhenti mendengkur: flag harus turun sendiri. */
static void test_snoring_stops() {
    SnoreDetector d;
    for (uint32_t t = 0; t < 30'000; t += 100) {
        bool peak = (t % 3000) < 600;
        d.feed(t, peak ? 62 : 35);
    }
    assert(d.flagged());
    for (uint32_t t = 30'000; t < 60'000; t += 100) d.feed(t, 30);
    assert(!d.flagged());
}

/* §4.3 + uji §6 no. 4: 60 detik DITOLAK, bukan dipotong jadi 30. */
static void test_aroma_refuses_rather_than_clamps() {
    AromaLimiter a;
    assert(a.request(1000, 60) == 0);
    assert(a.eventsTonight() == 0);      // permintaan ditolak tidak terhitung
    assert(a.request(2000, 25) == 25);
    assert(a.eventsTonight() == 1);
}

static void test_aroma_nightly_cap() {
    AromaLimiter a;
    uint32_t t = 0;
    for (int i = 0; i < 4; i++) { t += 60'000; assert(a.request(t, 20) == 20); }
    t += 60'000;
    assert(a.request(t, 20) == 0);       // kejadian kelima ditolak

    t += AROMA_NIGHT_GAP_MS + 1;
    assert(a.request(t, 20) == 20);      // malam berikutnya, hitungan direset
}

/* §4.3: eksponensial, bukan linear. Ujinya sederhana — di tengah waktu,
 * kurva eksponensial sudah jauh lebih redup daripada garis lurus. */
static void test_dimming_is_exponential() {
    uint8_t mid    = light_ramp(255, 0, 0.5f);
    uint8_t linear = 128;
    assert(mid < linear / 2);

    assert(light_ramp(255, 0, 0.0f) == 255);
    assert(light_ramp(255, 0, 1.0f) == 0);      // uji §6 no. 3: benar-benar mati

    // Rasio yang sama untuk langkah waktu yang sama — itu definisi geometris.
    float q1 = (float)light_ramp(255, 0, 0.25f) / 255.0f;
    float q2 = (float)light_ramp(255, 0, 0.50f) / (float)light_ramp(255, 0, 0.25f);
    assert(fabsf(q1 - q2) < 0.02f);

    // Sunrise memakai kurva yang sama, arah sebaliknya.
    assert(light_ramp(0, 255, 0.5f) < 128);
}

/* §4.3: selama sunset, kanal biru jauh di bawah merah. */
static void test_sunset_kills_blue() {
    Rgb sunset = light_color("sunset", 2200, 255);
    assert(sunset.b * 4 < sunset.r);

    Rgb alert = light_color("alert", 6500, 255);
    assert(alert.r == 255 && alert.g == 255 && alert.b == 255);

    Rgb off = light_color("off", 2200, 255);
    assert(off.r == 0 && off.g == 0 && off.b == 0);

    // Sunrise membangunkan — birunya justru dibutuhkan, jangan ditekan.
    Rgb sunrise = light_color("sunrise", 5000, 255);
    assert(sunrise.b > sunset.b);
}

int main() {
    test_steady_noise_is_not_snoring();
    test_breathing_rhythm_is_snoring();
    test_fast_bursts_are_not_snoring();
    test_snoring_stops();
    test_aroma_refuses_rather_than_clamps();
    test_aroma_nightly_cap();
    test_dimming_is_exponential();
    test_sunset_kills_blue();
    printf("snore + aroma + light: semua cek lulus\n");
    return 0;
}
