#pragma once
#include <stdint.h>

/*
 * Permukaan BLE gelang — BLE_GATT_CONTRACT.md §2 dan §3.
 *
 * Semua penyusunan byte ada di sini; pemanggil bekerja dengan nilai biasa.
 * §1 mengikat seluruh file ini: little-endian, tidak ada float di udara,
 * Indicate untuk apa pun yang menyangkut keselamatan.
 */

struct BandCallbacks {
    void (*onConfig)(const char *json);      // §3.7 `0005`
    void (*onCommand)(const char *json);     // §3.8 `0009`
    void (*onFlushStart)();                  // §3.9 langkah 1
    void (*onFlushAck)(uint16_t last_seq);   // §3.9 langkah 4
};

void BLE_Init(const BandCallbacks &cb);
bool BLE_Connected();

void BLE_NotifyVitals(const uint8_t *packet, uint16_t len);        // §3.1
void BLE_NotifyOxygen(uint8_t spo2_pct, uint8_t position);         // §3.2
void BLE_NotifyMotion(uint16_t milli_g);                           // §3.3
void BLE_IndicateSos();                                            // §3.4
void BLE_IndicateEscalation(uint8_t stage, uint8_t reason);        // §3.5
void BLE_NotifyStatus(uint8_t percent, bool charging, uint32_t epoch_s);  // §3.6
void BLE_NotifyBuffer(const uint8_t *packet, uint16_t len);        // §3.9
void BLE_NotifyEcg(const uint8_t *packet, uint16_t len);           // §3.10

/* §2.1. Tahap eskalasi ikut disiarkan supaya bedside bisa membunyikan
 * sirene sendiri saat HP tidak ada. Kontrak memberi batas 2 detik antara
 * tahap berubah dan siaran ikut berubah. */
void BLE_UpdateAdvertising(uint8_t stage, bool worn);
