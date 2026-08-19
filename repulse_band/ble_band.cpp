#include <Arduino.h>
#include <NimBLEDevice.h>
#include <ArduinoJson.h>
#include "ble_band.h"

/* §2. Aplikasi memindai dengan filter service UUID, bukan nama. Kalau UUID
 * ini tidak ada di paket advertising, gelang tidak akan pernah ditemukan. */
#define SVC_UUID  "4FA10000-8C3A-4B8F-A292-3E83D02F1A00"
#define CH(n)     "4FA1000" n "-8C3A-4B8F-A292-3E83D02F1A00"

static BandCallbacks       g_cb        = {};
static NimBLEServer       *g_server    = nullptr;
static NimBLEAdvertising  *g_adv       = nullptr;
static bool                g_connected = false;

static NimBLECharacteristic *c_vitals, *c_oxygen, *c_motion, *c_sos,
                            *c_config, *c_buffer, *c_escalation,
                            *c_status, *c_command, *c_ecg;

/* Nilai siaran terakhir, supaya BLE_UpdateAdvertising boleh dipanggil
 * sesering apa pun tanpa me-restart advertising kalau tidak ada yang
 * berubah — restart advertising memutus jadwal iklan yang sedang jalan. */
static uint8_t g_adv_stage = 0;
static uint8_t g_adv_flags = 0;

// ─── Callbacks ───────────────────────────────────────────────

class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer *server, NimBLEConnInfo &info) override {
        (void)server;
        g_connected = true;
        Serial.printf("[BLE] Connected: %s\n", info.getAddress().toString().c_str());
        BLE_UpdateAdvertising(g_adv_stage, (g_adv_flags & 0x80) != 0);
    }
    void onDisconnect(NimBLEServer *server, NimBLEConnInfo &info, int reason) override {
        (void)server; (void)info;
        g_connected = false;
        Serial.printf("[BLE] Disconnected (reason %d)\n", reason);
        /* Bit phone_connected berubah, jadi siaran wajib ikut berubah — itulah
         * yang memberi izin bedside membunyikan sirene sendiri (§2.1). */
        BLE_UpdateAdvertising(g_adv_stage, (g_adv_flags & 0x80) != 0);
        NimBLEDevice::startAdvertising();
    }
};

class ConfigCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *c, NimBLEConnInfo &info) override {
        (void)info;
        if (g_cb.onConfig) g_cb.onConfig(c->getValue().c_str());
    }
};

class CommandCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *c, NimBLEConnInfo &info) override {
        (void)info;
        if (g_cb.onCommand) g_cb.onCommand(c->getValue().c_str());
    }
};

/* §3.9. Dua perintah saja: flush_start dan flush_ack. */
class BufferCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *c, NimBLEConnInfo &info) override {
        (void)info;
        JsonDocument doc;
        if (deserializeJson(doc, c->getValue().c_str())) return;
        const char *cmd = doc["cmd"] | "";

        if (!strcmp(cmd, "flush_start") && g_cb.onFlushStart) {
            g_cb.onFlushStart();
        } else if (!strcmp(cmd, "flush_ack") && g_cb.onFlushAck) {
            g_cb.onFlushAck(doc["last_seq"] | 0u);
        }
    }
};

// ─── Init ────────────────────────────────────────────────────

void BLE_Init(const BandCallbacks &cb) {
    g_cb = cb;

    NimBLEDevice::init("RePulse Band");
    NimBLEDevice::setMTU(185);                       // §1
    /* §1: bonding aktif, supaya reconnect tengah malam tidak meminta pairing
     * ulang. Tanpa MITM — gelang tidak punya keypad maupun layar. */
    NimBLEDevice::setSecurityAuth(true, false, true);

    g_server = NimBLEDevice::createServer();
    g_server->setCallbacks(new ServerCallbacks());
    /* Kalau HP menghilang, gelang harus kembali mengiklankan diri sendiri —
     * ini yang menjaga tahap eskalasi tetap terlihat oleh bedside. */
    g_server->advertiseOnDisconnect(true);

    NimBLEService *svc = g_server->createService(SVC_UUID);

    c_vitals     = svc->createCharacteristic(CH("1"), NIMBLE_PROPERTY::NOTIFY);
    c_oxygen     = svc->createCharacteristic(CH("2"), NIMBLE_PROPERTY::NOTIFY);
    c_motion     = svc->createCharacteristic(CH("3"), NIMBLE_PROPERTY::NOTIFY);
    c_sos        = svc->createCharacteristic(CH("4"), NIMBLE_PROPERTY::INDICATE);
    c_config     = svc->createCharacteristic(CH("5"), NIMBLE_PROPERTY::WRITE);
    c_buffer     = svc->createCharacteristic(CH("6"), NIMBLE_PROPERTY::NOTIFY |
                                                      NIMBLE_PROPERTY::WRITE);
    c_escalation = svc->createCharacteristic(CH("7"), NIMBLE_PROPERTY::INDICATE);
    c_status     = svc->createCharacteristic(CH("8"), NIMBLE_PROPERTY::READ |
                                                      NIMBLE_PROPERTY::NOTIFY);
    c_command    = svc->createCharacteristic(CH("9"), NIMBLE_PROPERTY::WRITE);
    c_ecg        = svc->createCharacteristic(CH("A"), NIMBLE_PROPERTY::NOTIFY);

    c_config->setCallbacks(new ConfigCallbacks());
    c_command->setCallbacks(new CommandCallbacks());
    c_buffer->setCallbacks(new BufferCallbacks());

    /* Tidak ada svc->start(): sejak NimBLE 2.x service dimulai bersama
     * server, dan memanggilnya sendiri hanya memicu peringatan usang. */

    g_adv = NimBLEDevice::getAdvertising();
    g_adv_stage = 0xFF;                              // paksa isi pertama
    BLE_UpdateAdvertising(0, false);
    NimBLEDevice::startAdvertising();
    Serial.println("[BLE] Advertising as RePulse Band");
}

bool BLE_Connected() { return g_connected; }

// ─── §2.1 Siaran tahap eskalasi ──────────────────────────────

void BLE_UpdateAdvertising(uint8_t stage, bool worn) {
    uint8_t flags = (uint8_t)((worn ? 0x80 : 0x00) | (g_connected ? 0x01 : 0x00));
    if (stage == g_adv_stage && flags == g_adv_flags) return;
    g_adv_stage = stage;
    g_adv_flags = flags;
    if (!g_adv) return;

    /* Anggaran 31 byte: flags 3 + service UUID 128-bit 18 + manufacturer data
     * 7 = 28. Nama tidak ikut muat, jadi ia pindah ke scan response — bedside
     * tersambung listrik, active scan tidak jadi soal (§2.1). */
    uint8_t mfr[5] = { 0xFF, 0xFF, 0x01, stage, flags };

    NimBLEAdvertisementData data;
    data.setFlags(BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP);
    data.setCompleteServices(NimBLEUUID(SVC_UUID));
    data.setManufacturerData(mfr, sizeof(mfr));

    NimBLEAdvertisementData scan_response;
    scan_response.setName("RePulse Band");

    /* Mengganti isi paket saat advertising sedang jalan tidak dijamin
     * langsung terpakai di semua versi controller. §2.1 memberi batas 2 detik
     * antara tahap berubah dan siarannya ikut berubah, dan itu satu-satunya
     * cara bedside tahu ada keadaan darurat saat HP mati — jadi di sini kita
     * hentikan lalu mulai lagi, bukan berharap. Fungsi ini sudah keluar lebih
     * awal kalau tidak ada yang berubah, jadi ini tidak terjadi tiap detik. */
    bool was_advertising = g_adv->isAdvertising();
    if (was_advertising) g_adv->stop();

    g_adv->setAdvertisementData(data);
    g_adv->setScanResponseData(scan_response);

    if (was_advertising) g_adv->start();
}

// ─── Notify / Indicate ───────────────────────────────────────

static void push(NimBLECharacteristic *c, const uint8_t *b, uint16_t n, bool ind) {
    if (!c || !g_connected) return;
    c->setValue(b, n);
    if (ind) c->indicate();
    else     c->notify();
}

void BLE_NotifyVitals(const uint8_t *packet, uint16_t len) {
    push(c_vitals, packet, len, false);
}

void BLE_NotifyOxygen(uint8_t spo2_pct, uint8_t position) {
    uint8_t b[2] = { spo2_pct, position };
    push(c_oxygen, b, sizeof(b), false);
}

void BLE_NotifyMotion(uint16_t milli_g) {
    uint8_t b[2] = { (uint8_t)(milli_g & 0xFF), (uint8_t)(milli_g >> 8) };
    push(c_motion, b, sizeof(b), false);
}

/* §3.4: wajib Indicate, dan harus sampai di aplikasi di bawah 2 detik. */
void BLE_IndicateSos() {
    uint8_t b = 1;
    push(c_sos, &b, 1, true);
}

void BLE_IndicateEscalation(uint8_t stage, uint8_t reason) {
    uint8_t b[2] = { stage, reason };
    push(c_escalation, b, sizeof(b), true);
}

void BLE_NotifyStatus(uint8_t percent, bool charging, uint32_t epoch_s) {
    uint8_t b[6] = { percent, (uint8_t)(charging ? 1 : 0),
                     (uint8_t)( epoch_s        & 0xFF),
                     (uint8_t)((epoch_s >> 8)  & 0xFF),
                     (uint8_t)((epoch_s >> 16) & 0xFF),
                     (uint8_t)((epoch_s >> 24) & 0xFF) };
    if (c_status) c_status->setValue(b, sizeof(b));   // juga terbaca lewat Read
    push(c_status, b, sizeof(b), false);
}

void BLE_NotifyBuffer(const uint8_t *packet, uint16_t len) {
    push(c_buffer, packet, len, false);
}

void BLE_NotifyEcg(const uint8_t *packet, uint16_t len) {
    push(c_ecg, packet, len, false);
}
