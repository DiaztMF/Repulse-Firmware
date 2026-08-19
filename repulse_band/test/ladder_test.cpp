/*
 * Satu cek yang bisa dijalankan tanpa hardware:
 *
 *   cd test && g++ -std=c++17 -o ladder_test ladder_test.cpp && ./ladder_test
 *
 * Menguji dua hal yang salahnya baru ketahuan jam 3 pagi: urutan tangga
 * eskalasi (§3.5) dan aturan "hapus hanya setelah ACK" di buffer offline
 * (§3.9 langkah 5).
 */
#include <cassert>
#include <cstdio>
#include "../ladder.h"
#include "../evtbuf.h"

static void test_ladder_full_climb() {
    Ladder l;
    l.anomaly(0, REASON_THRESHOLD);
    assert(l.stage() == 1 && l.reason() == REASON_THRESHOLD);
    assert(l.takeChange() && !l.takeChange());   // sekali lapor, sekali saja

    l.tick(19'000);  assert(l.stage() == 1);     // 20 dtk belum lewat
    l.tick(20'000);  assert(l.stage() == 2);
    l.tick(35'000);  assert(l.stage() == 3);
    l.tick(65'000);  assert(l.stage() == 4);
    l.tick(90'000);  assert(l.stage() == 4);     // tahap 4 tidak jalan sendiri
}

static void test_motion_cancels_only_from_stage_2() {
    Ladder l;
    l.anomaly(0, REASON_IRREGULAR);
    l.motion(1'000, 900);
    assert(l.stage() == 1);                      // tahap 1 diam, gerakan bukan respons

    l.tick(20'000);  assert(l.stage() == 2);
    l.motion(21'000, 149);
    assert(l.stage() == 2);                      // di bawah ambang, bukan respons
    l.motion(21'500, 151);
    assert(l.stage() == 0 && l.reason() == REASON_NONE);
    assert(l.takeChange());                      // turun ke 0 WAJIB dilaporkan
}

static void test_motion_never_cancels_sos() {
    Ladder l;
    l.manualSos(0);
    assert(l.stage() == 4 && l.reason() == REASON_MANUAL);
    l.motion(1'000, 5'000);
    l.clear(2'000);
    assert(l.stage() == 4);                      // hanya aplikasi yang boleh menutup
    l.reset(3'000);
    assert(l.stage() == 0 && l.takeChange());
}

static void test_anomaly_does_not_restart_timer() {
    Ladder l;
    l.anomaly(0, REASON_THRESHOLD);
    l.anomaly(10'000, REASON_IRREGULAR);         // anomali kedua saat tangga jalan
    l.tick(20'000);
    assert(l.stage() == 2 && l.reason() == REASON_THRESHOLD);
}

static void test_buffer_keeps_data_until_ack() {
    EventBuffer b;
    for (int i = 0; i < 25; i++) b.push(1'786'512'000u + i, EVT_VITALS, 60, 97, 0, 42);
    assert(b.count() == 25);

    uint8_t  pkt[192];
    uint16_t len = 0;

    b.beginFlush();
    assert(b.nextPacket(pkt, &len));
    assert(pkt[0] == 0 && pkt[1] == 0 && pkt[2] == EVTBUF_PER_PACKET);
    assert(len == 3 + 19 * EVTBUF_ENTRY_LEN);
    assert(pkt[3] == (uint8_t)(1'786'512'000u & 0xFF));   // little-endian, entri tertua dulu

    assert(b.nextPacket(pkt, &len));
    assert(pkt[0] == 1 && pkt[2] == 6);
    assert(!b.nextPacket(pkt, &len));

    assert(b.count() == 25);                     // belum di-ACK, belum hilang
    b.ack(1);
    assert(b.count() == 0);
}

static void test_buffer_survives_dropped_flush() {
    EventBuffer b;
    for (int i = 0; i < 25; i++) b.push(100 + i, EVT_STAGE, 3, REASON_THRESHOLD);
    uint8_t pkt[192]; uint16_t len = 0;
    b.beginFlush();
    b.nextPacket(pkt, &len);
    b.abortFlush();                              // koneksi putus di tengah
    assert(b.count() == 25);
}

static void test_buffer_overwrites_oldest_when_full() {
    EventBuffer b;
    for (int i = 0; i < EVTBUF_CAPACITY + 5; i++) b.push(1000 + i, EVT_SOS);
    assert(b.count() == EVTBUF_CAPACITY);
    uint8_t pkt[192]; uint16_t len = 0;
    b.beginFlush();
    b.nextPacket(pkt, &len);
    uint32_t oldest = (uint32_t)pkt[3] | (uint32_t)pkt[4] << 8 |
                      (uint32_t)pkt[5] << 16 | (uint32_t)pkt[6] << 24;
    assert(oldest == 1005);                      // lima tertua sudah tergusur
}

int main() {
    test_ladder_full_climb();
    test_motion_cancels_only_from_stage_2();
    test_motion_never_cancels_sos();
    test_anomaly_does_not_restart_timer();
    test_buffer_keeps_data_until_ack();
    test_buffer_survives_dropped_flush();
    test_buffer_overwrites_oldest_when_full();
    printf("ladder + evtbuf: semua cek lulus\n");
    return 0;
}
