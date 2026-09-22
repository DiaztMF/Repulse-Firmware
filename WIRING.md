# RePulse — Wiring Final

Dokumen ini adalah satu-satunya sumber kebenaran untuk perakitan. Nomor pin
di sini sudah cocok dengan firmware yang berjalan — kalau ada perbedaan
dengan gambar atau catatan lama, **dokumen ini yang benar**.

Dua papan, keduanya **ESP32-C3**, dan keduanya berdiri sendiri. Tidak ada
kabel apa pun di antara gelang dan bedside — semuanya lewat Bluetooth.

> **Tidak ada layar.** Rancangan lama punya smartwatch ESP32-S3 yang
> tersambung ke gelang lewat UART. Layar itu dihapus. Kabel GPIO7 gelang ke
> layar **tidak dipasang**, dan kode pengirimnya sudah dibuang dari firmware.

---

## Aturan yang berlaku untuk kedua papan

**GPIO2, GPIO8, dan GPIO9 adalah strapping pin.** Ketiganya ikut menentukan
mode boot ESP32-C3. GPIO9 rendah saat reset melempar chip ke *download mode*:
firmware tidak jalan, serial monitor kosong, LED biru mati, dan papannya
tampak seperti rusak padahal sehat.

Ini bukan teori — unit bedside sempat mati total karena SDA BH1750 dipasang
di GPIO9. Jangan pindahkan apa pun ke tiga pin itu tanpa membaca catatan di
bagian bedside.

**GND harus menyatu.** Setiap modul yang punya GND wajib tersambung ke GND
ESP32-C3 yang sama. UART, I²C, dan WS2812 sama-sama diam total tanpa ground
bersama, dan gejalanya mirip modul rusak.

---

## Papan 1 — Gelang (ESP32-C3)

Mengukur pergelangan tangan, menjalankan tangga eskalasi sendiri, dan
melapor ke HP lewat BLE.

### Bus I²C — dua sensor berbagi dua kabel

| ESP32-C3 | Sinyal |
|---|---|
| GPIO10 | SDA |
| GPIO0 | SCL |
| 3V3 | VCC |
| GND | GND |

Kedua sensor di bawah ini disambung **paralel** ke empat kabel yang sama.

| Modul | Alamat I²C | Catatan |
|---|---|---|
| **MAX30102** | `0x57` | Detak jantung, SpO₂, deteksi dipakai |
| **MPU6050** | `0x68` | Gerakan dan posisi tubuh. Pin `AD0` ke **GND** |

Kalau `AD0` pada MPU6050 dibiarkan mengambang atau ditarik ke 3V3,
alamatnya berubah jadi `0x69` dan firmware akan melaporkan
`[IMU] MPU6050... GAGAL`.

Modul breakout keduanya sudah membawa resistor pull-up sendiri. Jangan
tambah pull-up lagi.

### Tombol SOS

| ESP32-C3 | ke |
|---|---|
| GPIO3 | satu kaki tombol |
| GND | kaki tombol satunya |

Aktif LOW dengan pull-up internal. **Tidak perlu resistor eksternal.**
Tekan dan tahan 2 detik untuk memicu SOS.

### Motor getar

| ESP32-C3 | ke |
|---|---|
| GPIO1 | gate/base driver |

⚠ **Jangan sambungkan motor langsung ke GPIO1.** GPIO ESP32-C3 aman sampai
sekitar 20 mA, sedangkan motor getar koin menarik 70–100 mA saat mulai
berputar. Sambungkan lewat MOSFET logic-level (2N7002, AO3400) atau
transistor (S8050, 2N2222) dengan **dioda flyback** (1N4148) paralel dengan
motor.

Firmware menggerakkannya dengan PWM 5 kHz dan memberi sentakan tegangan
penuh 60 ms di awal setiap getaran, karena motor koin butuh ±2,3 V untuk
lepas dari gesekan statis sedangkan duty halus hanya menghasilkan ±1,4 V.

### Belum dipasang

| Fitur | Status |
|---|---|
| **ECG AD8232** | Dibatalkan dari desain. Denyut diambil dari MAX30102 saja |
| **Divider baterai** | Belum ada. Gelang melapor `255` = "tidak terukur", dan aplikasi menampilkannya sebagai kosong, bukan 0% |

Kalau divider baterai mau dipasang, gunakan **GPIO4**. Jangan GPIO2 — itu
strapping pin, dan tegangan divider (~2,1 V dari baterai penuh) bisa terbaca
LOW saat boot lalu melempar chip ke download mode. GPIO5 sebaiknya juga
dihindari: itu ADC2, yang pada ESP32-C3 tidak bisa diandalkan saat radio
menyala. GPIO6 dan GPIO7 tidak bisa sama sekali karena bukan pin ADC.
Setelah terpasang, ubah `PIN_BATTERY_ADC` di firmware dari `-1` ke `4`.

### Peta pin gelang — lengkap

| GPIO | Dipakai untuk |
|---|---|
| 0 | I²C SCL |
| 1 | Motor getar (lewat driver) |
| 2 | — *strapping, biarkan kosong* |
| 3 | Tombol SOS |
| 4 | bebas — ADC1, pilihan untuk divider baterai |
| 5 | bebas — ADC2, hindari untuk pengukuran |
| 6 | bebas |
| 7 | bebas — dulu UART ke layar |
| 8 | — *strapping, biarkan kosong* |
| 9 | — *strapping, biarkan kosong* |
| 10 | I²C SDA |

---

## Papan 2 — Bedside (ESP32-C3)

Mengukur ruangan, menjalankan aktuator, dan bisa membunyikan sirene sendiri
saat tidak ada HP yang hidup.

### Mikrofon INMP441 (I²S)

| ESP32-C3 | INMP441 |
|---|---|
| GPIO5 | SCK |
| GPIO4 | WS / LRCK |
| GPIO6 | SD / DOUT |
| 3V3 | VDD |
| GND | GND, dan **L/R ke GND** |

`L/R` ke GND memilih kanal kiri, yang dibaca firmware.

### BH1750 (cahaya, I²C)

| ESP32-C3 | BH1750 |
|---|---|
| GPIO7 | SCL |
| **GPIO1** | SDA |
| 3V3 | VCC |
| GND | GND, dan **ADDR ke GND** |

⚠ **SDA di GPIO1, bukan GPIO9.** Sebelumnya SDA memang di GPIO9 dengan
asumsi pull-up modul menahannya tetap tinggi. Saat modulnya melemah, GPIO9
tertarik rendah, dan papannya mati total tanpa satu pun pesan di serial.
Jangan dikembalikan.

### DHT11 (suhu & kelembapan)

| ESP32-C3 | DHT11 |
|---|---|
| GPIO10 | DATA, **dengan pull-up 10 kΩ ke 3V3** |
| 3V3 | VCC |
| GND | GND |

Resistor pull-up wajib kalau modulnya versi telanjang 4 kaki. Modul 3 kaki
di atas PCB biru biasanya sudah membawanya.

### WS2812 (lampu)

| ESP32-C3 | Strip |
|---|---|
| GPIO8 | DIN, **dengan pull-up 10 kΩ ke 3V3** |
| — | 5 V dari catu daya, **bukan dari ESP32** |
| GND | GND bersama |

⚠ **GPIO8 adalah strapping pin.** Pull-up 10 kΩ itu bukan opsional — tanpa
itu, pin mengambang saat boot dan papan bisa boot berbeda tergantung modul
mana yang menyala duluan.

Dua hal lain soal daya:
- **Data 3,3 V ke strip 5 V itu marginal.** Beri strip 4,5 V, atau pasang
  level shifter 74AHCT125. Kalau tidak, strip bisa jalan di unit ini dan
  tidak di unit berikutnya.
- **60 piksel putih penuh butuh ±3,6 A.** Firmware membatasi kecerahan
  hingga ±1,4 A. Jangan naikkan batas itu tanpa catu daya 5 V 5 A.

### DFPlayer Mini (white noise & sirene)

| DFPlayer | ke ESP32-C3 |
|---|---|
| **TX** (kaki 3) | **GPIO2** |
| **RX** (kaki 2) | **GPIO3**, lewat **resistor 1 kΩ** seri |
| VCC (kaki 1) | **5 V**, bukan 3,3 V |
| GND (kaki 7 atau 10) | GND bersama ESP32 |
| SPK_1 / SPK_2 | speaker 3 W 4 Ω |

⚠ **UART selalu menyilang.** TX satu sisi ke RX sisi lain. Memasangnya lurus
(RX ke RX, TX ke TX) membuat dua RX saling mendengar dan dua TX saling
mendorong — tidak ada satu byte pun lewat, dan serial akan mencetak
`DFPLAYER tidak membalas reset`.

Resistor 1 kΩ ada untuk menurunkan level 3,3 V ESP32 ke input DFPlayer.
Letaknya di jalur menuju **RX DFPlayer**, bukan sebaliknya.

⚠ **GPIO2 juga strapping pin** — pasang pull-up 10 kΩ ke 3V3, alasan sama
dengan GPIO8.

**SPK_1 dan SPK_2 adalah keluaran terjembatani.** Tidak ada plus dan minus,
dan **tidak satu pun boleh menyentuh GND** — itu merusak penguatnya.

**microSD:** FAT32, maksimal 32 GB, berkas di root:

| Berkas | Isi |
|---|---|
| `0001.mp3` | White noise 1 |
| `0002.mp3` | White noise 2 |
| `0003.mp3` | White noise 3 |
| `0004.mp3` | Sirene |

Urutan penting. Firmware memutar nomor trek, bukan nama berkas, jadi trek 4
yang salah tempat berarti sirene berbunyi saat pengguna minta white noise.

### Relay aroma diffuser

| ESP32-C3 | Relay |
|---|---|
| GPIO20 | IN |
| 3V3 atau 5 V | VCC, sesuai modul |
| GND | GND |

Modul di meja ini **aktif HIGH**. Kalau modulnya diganti dan gejalanya
muncul — relay menyala saat boot lalu **mati** ketika perintah aroma dikirim
— itu berarti modul barunya aktif LOW; balik `AROMA_ACTIVE_LEVEL` di
firmware dan tidak ada lagi yang perlu disentuh.

Polaritas terbalik di sini bukan sekadar merepotkan: diffuser akan menyala
sepanjang malam kecuali diperintahkan mati, dan itu risiko nyata bagi
penderita asma.

GPIO20 hanya bebas karena firmware dikompilasi dengan `CDCOnBoot=cdc`, yang
mengalihkan log ke USB. Tanpa flag itu GPIO20 menjadi UART0 RX dan bertabrakan
dengan relay ini.

### Peta pin bedside — lengkap

| GPIO | Dipakai untuk |
|---|---|
| 1 | BH1750 SDA |
| 2 | DFPlayer TX → ESP RX · *strapping, pull-up 10 kΩ* |
| 3 | DFPlayer RX ← ESP TX, lewat 1 kΩ |
| 4 | Mikrofon WS |
| 5 | Mikrofon SCK |
| 6 | Mikrofon SD |
| 7 | BH1750 SCL |
| 8 | WS2812 DIN · *strapping, pull-up 10 kΩ* |
| 9 | — *strapping, biarkan kosong* |
| 10 | DHT11 DATA, pull-up 10 kΩ |
| 20 | Relay aroma — bebas hanya karena log lewat USB, lihat catatan |

---

## Daftar periksa sebelum menyalakan

1. Semua GND menyatu — modul, catu daya, dan ESP32.
2. GPIO9 kosong di kedua papan.
3. Pull-up 10 kΩ terpasang di GPIO2 dan GPIO8 bedside.
4. DFPlayer menyilang: TX-nya ke GPIO2, RX-nya ke GPIO3.
5. DFPlayer dapat 5 V, bukan 3,3 V.
6. `AD0` MPU6050 ke GND.
7. `L/R` INMP441 ke GND, `ADDR` BH1750 ke GND.
8. Strip WS2812 tidak mengambil daya dari ESP32.
9. Motor gelang lewat driver, bukan langsung ke GPIO1.

## Kalau tidak jalan

**Serial monitor kosong dan LED biru mati** — hampir pasti strapping pin.
Tutup serial monitor, lalu:

```bash
esptool --chip esp32c3 -p COM12 --before no-reset --after no-reset read-mem 0x60004038
```

`0xc` berarti boot normal. `0x5` berarti download mode — bit 3 adalah GPIO9,
bit 2 adalah GPIO8. Cari apa yang menarik pin itu rendah.

**Serial kosong tapi LED biru menyala** — bukan strapping. Papan dikompilasi
tanpa `CDCOnBoot=cdc`, jadi log-nya keluar ke GPIO21/20 dan bukan ke USB.

**BH1750 tidak ditemukan** — firmware memindai seluruh bus dan mencetak
alamat mana pun yang menjawab. Kalau tidak ada yang menjawab sama sekali,
masalahnya daya atau modulnya mati, bukan alamatnya.

**DFPlayer tidak membalas** — cek silang TX/RX lebih dulu, lalu 5 V di kaki
modul, lalu kartu SD. Untuk memastikan modulnya hidup tanpa melibatkan
ESP32: hubungkan singkat kaki `IO_2` ke GND selama sedetik. Kalau speaker
berbunyi, modul dan SD sehat dan masalahnya murni di UART.
