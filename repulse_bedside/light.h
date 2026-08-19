#pragma once
#include <stdint.h>
#include <math.h>
#include <string.h>

/*
 * Kurva lampu — BLE_GATT_CONTRACT.md §4.3.
 *
 * Dua aturan yang tidak boleh dilanggar:
 *
 * 1. "Peredupan sunset wajib eksponensial, bukan linear — mata mempersepsi
 *    cahaya secara logaritmik, jadi peredupan linear terasa mati mendadak di
 *    ujungnya."
 * 2. "Selama sunset, kanal biru diturunkan jauh di bawah merah."
 *
 * Keduanya murni matematika, jadi keduanya dites di laptop. Uji §6 no. 3
 * mensyaratkan lux akhir < 3, yang berarti kurvanya harus benar-benar sampai
 * nol, bukan berhenti di remang-remang.
 */

/* Interpolasi geometris: tiap langkah waktu yang sama mengalikan keluaran
 * dengan rasio yang sama. Itulah yang dipersepsi mata sebagai peredupan
 * merata. `progress` 0..1. */
inline uint8_t light_ramp(uint8_t from, uint8_t to, float progress) {
    if (progress <= 0.0f) return from;
    if (progress >= 1.0f) return to;

    // Nol tidak punya logaritma, jadi lantainya 1 dan nol dipasang di ujung.
    float a = from < 1 ? 1.0f : (float)from;
    float b = to   < 1 ? 1.0f : (float)to;
    float v = a * powf(b / a, progress);

    if (v < 1.0f) v = 1.0f;
    if (v > 255.0f) v = 255.0f;
    return (uint8_t)(v + 0.5f);
}

struct Rgb { uint8_t r, g, b; };

/* Aproksimasi suhu warna ke RGB (Tanner Helland), 1000-6500 K. Cukup untuk
 * WS2812 — di bawah 2000 K kanal biru memang sudah hampir nol dengan
 * sendirinya, dan itu memang yang kita mau saat sunset. */
inline Rgb kelvin_to_rgb(uint16_t kelvin) {
    if (kelvin < 1000)  kelvin = 1000;
    if (kelvin > 10000) kelvin = 10000;
    float t = kelvin / 100.0f;
    float r, g, b;

    if (t <= 66.0f) {
        r = 255.0f;
        g = 99.4708025861f * logf(t) - 161.1195681661f;
        b = (t <= 19.0f) ? 0.0f
                         : 138.5177312231f * logf(t - 10.0f) - 305.0447927307f;
    } else {
        r = 329.698727446f * powf(t - 60.0f, -0.1332047592f);
        g = 288.1221695283f * powf(t - 60.0f, -0.0755148492f);
        b = 255.0f;
    }

    auto clamp = [](float v) -> uint8_t {
        if (v < 0.0f)   return 0;
        if (v > 255.0f) return 255;
        return (uint8_t)(v + 0.5f);
    };
    return { clamp(r), clamp(g), clamp(b) };
}

/* Penekanan biru tambahan untuk sunset dan amber. Nilai suhu warna saja
 * belum cukup: WS2812 murah punya LED biru yang bocor terang, dan cahaya
 * biru jam sebelas malam persis yang produk ini berusaha hilangkan. */
inline Rgb suppress_blue(Rgb c, uint8_t percent_kept) {
    c.b = (uint8_t)((uint16_t)c.b * percent_kept / 100);
    return c;
}

/* Warna akhir untuk satu mode, sudah dikalikan kecerahan. */
inline Rgb light_color(const char *mode, uint16_t kelvin, uint8_t brightness) {
    Rgb c;
    if (!mode) mode = "off";

    if (!strcmp(mode, "alert")) {
        c = { 255, 255, 255 };            // §PRD 6.3: lampu putih 100%
    } else if (!strcmp(mode, "off")) {
        return { 0, 0, 0 };
    } else {
        c = kelvin_to_rgb(kelvin);
        // sunset dan amber menuju tidur; sunrise membangunkan, biarkan biru.
        if (!strcmp(mode, "sunset") || !strcmp(mode, "amber")) {
            c = suppress_blue(c, 15);
        }
    }

    c.r = (uint8_t)((uint16_t)c.r * brightness / 255);
    c.g = (uint8_t)((uint16_t)c.g * brightness / 255);
    c.b = (uint8_t)((uint16_t)c.b * brightness / 255);
    return c;
}
