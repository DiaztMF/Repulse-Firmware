#pragma once
#include <stdint.h>

/*
 * Permukaan BLE bedside — BLE_GATT_CONTRACT.md §4, ditambah pemindai siaran
 * gelang di §2.1.
 *
 * Bedside punya dua peran sekaligus: GATT server untuk aplikasi, dan observer
 * pasif yang mendengarkan siaran gelang. Peran kedua itu yang menutup lubang
 * keselamatan terbesar — HP mati pukul dua pagi, gelang tetap eskalasi, dan
 * tanpa ini tidak ada yang memerintahkan sirene.
 */

struct BedsideCallbacks {
    /* §4.3. JSON aktuator dari aplikasi. */
    void (*onActuator)(const char *json);
    /* §2.1. Siaran gelang terlihat: tahap eskalasi, bit phone_connected, dan
     * apakah MAC-nya cocok dengan gelang pasangan yang tersimpan. */
    void (*onBandSeen)(uint8_t stage, bool phone_connected, bool paired);
};

void BLE_Init(const BedsideCallbacks &cb);
bool BLE_Connected();
/* §2.1. Tersambung dan menulis sesuatu dalam 30 detik terakhir. Syarat 3
 * sirene mandiri memakai ini, bukan BLE_Connected. */
bool BLE_AppAlive();

void BLE_NotifyRoom(int16_t temp_c_x10, uint16_t rh_pct_x10,
                    uint32_t lux_x100, uint8_t db);              // §4.1
void BLE_NotifySnore(bool flagged, uint8_t intensity);           // §4.2
void BLE_IndicateAck(uint8_t command_id, uint8_t status);        // §4.4

/* Pemindai gelang. Dipanggil berkala dari loop; NimBLE mengerjakan
 * pemindaiannya sendiri di latar belakang. */
void BLE_ScanLoop();

/* MAC gelang pasangan, disimpan di NVS supaya bertahan setelah listrik mati.
 * Kosong berarti belum pernah dipasangkan. */
bool BLE_HasPairedBand();
void BLE_ForgetPairedBand();
