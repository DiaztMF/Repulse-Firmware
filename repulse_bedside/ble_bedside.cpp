#include <Arduino.h>
#include <NimBLEDevice.h>
#include <Preferences.h>
#include "ble_bedside.h"

/* §2. Aplikasi memindai dengan filter service UUID, bukan nama. */
#define SVC_UUID   "4FA20000-8C3A-4B8F-A292-3E83D02F1A00"
#define CH(n)      "4FA2000" n "-8C3A-4B8F-A292-3E83D02F1A00"
/* §2.1. Yang kita cari di udara: service gelang, bukan service kita sendiri. */
#define BAND_SVC   "4FA10000-8C3A-4B8F-A292-3E83D02F1A00"

static BedsideCallbacks g_cb        = {};
static NimBLEServer    *g_server    = nullptr;
static bool             g_connected = false;
static Preferences      g_prefs;
static String           g_paired_mac;

static NimBLECharacteristic *c_room, *c_snore, *c_actuator, *c_ack;

// ─── GATT server ─────────────────────────────────────────────

class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer *server, NimBLEConnInfo &info) override {
        (void)server;
        g_connected = true;
        Serial.printf("[BLE] Aplikasi tersambung: %s\n",
                      info.getAddress().toString().c_str());
    }
    void onDisconnect(NimBLEServer *server, NimBLEConnInfo &info, int reason) override {
        (void)server; (void)info;
        g_connected = false;
        Serial.printf("[BLE] Aplikasi putus (reason %d)\n", reason);
        NimBLEDevice::startAdvertising();
    }
};

class ActuatorCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *c, NimBLEConnInfo &info) override {
        (void)info;
        if (g_cb.onActuator) g_cb.onActuator(c->getValue().c_str());
    }
};

// ─── §2.1 Pemindai siaran gelang ─────────────────────────────

class ScanCallbacks : public NimBLEScanCallbacks {
    void onResult(const NimBLEAdvertisedDevice *dev) override {
        if (!dev->isAdvertisingService(NimBLEUUID(BAND_SVC))) return;

        std::string mfr = dev->getManufacturerData();
        // §2.1: company_id(2) + protocol_version(1) + stage(1) + flags(1)
        if (mfr.size() < 5) return;
        if ((uint8_t)mfr[2] != 0x01) return;             // versi protokol lain

        uint8_t stage = (uint8_t)mfr[3];
        uint8_t flags = (uint8_t)mfr[4];
        bool phone_connected = (flags & 0x01) != 0;

        /* Identitas gelang tidak ikut di payload — kita pakai alamat MAC yang
         * sudah ada di header tiap paket, dan menyimpannya saat pertama kali
         * terlihat. ponytail: gelang pertama yang terlihat jadi pasangan;
         * kalau kelak ada lebih dari satu set perangkat keras, pemasangan
         * harus lewat perintah dari aplikasi, bukan siapa-cepat-dia-dapat. */
        String mac = String(dev->getAddress().toString().c_str());
        if (g_paired_mac.isEmpty()) {
            g_paired_mac = mac;
            g_prefs.putString("band_mac", mac);
            Serial.printf("[SCAN] Gelang dipasangkan: %s\n", mac.c_str());
        }

        if (g_cb.onBandSeen) {
            g_cb.onBandSeen(stage, phone_connected, mac == g_paired_mac);
        }
    }
};

static NimBLEScan *g_scan = nullptr;

void BLE_ScanLoop() {
    if (g_scan && !g_scan->isScanning()) {
        g_scan->start(0, false, true);       // 0 = terus-menerus
    }
}

bool BLE_HasPairedBand() { return !g_paired_mac.isEmpty(); }

void BLE_ForgetPairedBand() {
    g_paired_mac = "";
    g_prefs.remove("band_mac");
    Serial.println("[SCAN] Pasangan gelang dilupakan");
}

// ─── Init ────────────────────────────────────────────────────

void BLE_Init(const BedsideCallbacks &cb) {
    g_cb = cb;

    g_prefs.begin("repulse", false);
    g_paired_mac = g_prefs.getString("band_mac", "");
    if (!g_paired_mac.isEmpty()) {
        Serial.printf("[SCAN] Gelang pasangan tersimpan: %s\n", g_paired_mac.c_str());
    }

    NimBLEDevice::init("RePulse Bedside");
    NimBLEDevice::setMTU(185);                       // §1
    NimBLEDevice::setSecurityAuth(true, false, true);

    g_server = NimBLEDevice::createServer();
    g_server->setCallbacks(new ServerCallbacks());
    g_server->advertiseOnDisconnect(true);

    NimBLEService *svc = g_server->createService(SVC_UUID);
    c_room     = svc->createCharacteristic(CH("1"), NIMBLE_PROPERTY::NOTIFY);
    c_snore    = svc->createCharacteristic(CH("2"), NIMBLE_PROPERTY::NOTIFY);
    c_actuator = svc->createCharacteristic(CH("3"), NIMBLE_PROPERTY::WRITE);
    c_ack      = svc->createCharacteristic(CH("4"), NIMBLE_PROPERTY::INDICATE);
    c_actuator->setCallbacks(new ActuatorCallbacks());
    /* Tidak ada svc->start(): sejak NimBLE 2.x service dimulai bersama
     * server, dan memanggilnya sendiri hanya memicu peringatan usang. */

    NimBLEAdvertisementData data;
    data.setFlags(BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP);
    data.setCompleteServices(NimBLEUUID(SVC_UUID));
    NimBLEAdvertisementData scan_response;
    scan_response.setName("RePulse Bedside");

    NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
    adv->setAdvertisementData(data);
    adv->setScanResponseData(scan_response);
    NimBLEDevice::startAdvertising();

    /* Active scan boleh — bedside tersambung listrik, jadi biaya dayanya
     * tidak jadi soal (§2.1).
     *
     * Tapi jendelanya TIDAK boleh penuh, dan itu pelajaran mahal. Window 99
     * dari interval 100 berarti radio mendengarkan 99% waktu — dan satu radio
     * BLE tidak bisa memindai sambil mengiklan, keduanya memakai pemancar
     * yang sama. Bedside jadi mengumumkan dirinya hanya di 1% sisa waktu,
     * dan HP tidak pernah kebetulan menyimak di celah sekecil itu. Perangkat
     * yang sehat, mengiklan sesuai log, dan tak terlihat oleh siapa pun.
     *
     * Setengah jendela menyisakan separuh waktu untuk iklan. Deteksi tahap
     * eskalasi tetap jauh di bawah batas 2 detik §2.1: gelang menyiarkan
     * berkali-kali per detik, jadi kehilangan separuhnya hanya menggandakan
     * waktu tunggu rata-rata dari milidetik ke milidetik. */
    g_scan = NimBLEDevice::getScan();
    g_scan->setScanCallbacks(new ScanCallbacks(), false);
    g_scan->setActiveScan(true);
    g_scan->setInterval(160);            // 100 ms
    g_scan->setWindow(80);               // 50 ms mendengar, 50 ms bebas mengiklan
    g_scan->start(0, false, true);

    Serial.println("[BLE] Advertising as RePulse Bedside, memindai gelang");
}

bool BLE_Connected() { return g_connected; }

// ─── Notify / Indicate ───────────────────────────────────────

/* §4.1. Semua pecahan dikirim sebagai integer berskala — §1 melarang float
 * di udara. Lux dikali 100 karena ambang gelap optimal adalah < 3 lux dan
 * pembacaan nyata bisa 0,4 lux; integer polos membulatkannya jadi 0 dan
 * membuat seluruh verifikasi kegelapan tidak berarti. */
void BLE_NotifyRoom(int16_t temp_c_x10, uint16_t rh_pct_x10,
                    uint32_t lux_x100, uint8_t db) {
    if (!c_room || !g_connected) return;
    uint8_t b[9] = {
        (uint8_t)( (uint16_t)temp_c_x10       & 0xFF),
        (uint8_t)(((uint16_t)temp_c_x10 >> 8) & 0xFF),
        (uint8_t)( rh_pct_x10       & 0xFF),
        (uint8_t)((rh_pct_x10 >> 8) & 0xFF),
        (uint8_t)( lux_x100        & 0xFF),
        (uint8_t)((lux_x100 >> 8)  & 0xFF),
        (uint8_t)((lux_x100 >> 16) & 0xFF),
        (uint8_t)((lux_x100 >> 24) & 0xFF),
        db,
    };
    c_room->setValue(b, sizeof(b));
    c_room->notify();
}

void BLE_NotifySnore(bool flagged, uint8_t intensity) {
    if (!c_snore || !g_connected) return;
    uint8_t b[2] = { (uint8_t)(flagged ? 1 : 0), intensity };
    c_snore->setValue(b, sizeof(b));
    c_snore->notify();
}

/* §4.4. Tanpa command_id, konfirmasi tidak bisa dipasangkan ke perintahnya.
 * Indicate, bukan Notify — ini menyangkut apakah aktuator keselamatan benar
 * berjalan atau tidak. */
void BLE_IndicateAck(uint8_t command_id, uint8_t status) {
    if (!c_ack || !g_connected) return;
    uint8_t b[2] = { command_id, status };
    c_ack->setValue(b, sizeof(b));
    c_ack->indicate();
}
