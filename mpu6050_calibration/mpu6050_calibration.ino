/*
 * Kalibrasi MPU6050 standalone — RePulse Band (ESP32-C3)
 *
 * TANPA BLE, TANPA MAX30102, TANPA ladder/evtbuf.
 * Cuma: Wire + Adafruit_MPU6050 + Serial.
 *
 * Setting ikut firmware asli (repulse_band.ino):
 * - SDA = GPIO10, SCL = GPIO0
 * - accel 4G, gyro 500 deg/s, filter 5 Hz
 *
 * Board:
 * - ESP32C3 Dev Module
 * - USB CDC On Boot: Enabled
 *
 * Cara pakai:
 * 1. Flash sketch ini.
 * 2. Buka Serial Monitor 115200. Sketch otomatis SCAN bus I2C dulu,
 *    lalu coba address 0x68 dan 0x69.
 * 3. Kalau sensor ketemu: taruh device DIAM di meja, kirim karakter
 *    apa pun untuk mulai kalibrasi, tunggu ~1000 sampel, catat offset.
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// Test pin alternatif (asli firmware: SDA=10 SCL=0).
// 4/5 dipilih karena bebas di C3 SuperMini (8=LED onboard, 9=BOOT).
#define PIN_I2C_SDA 4
#define PIN_I2C_SCL 5

#define SERIAL_BAUD 115200
#define CALIB_SAMPLES 1000
#define SAMPLE_GAP_MS 5

static Adafruit_MPU6050 mpu;
static uint8_t mpuAddr = 0;

// Hasil kalibrasi (satuan SI, sama kayak getEvent)
static float accOffX = 0, accOffY = 0, accOffZ = 0;
static float gyrOffX = 0, gyrOffY = 0, gyrOffZ = 0;
static bool calibrated = false;

// Scan bus, return jumlah device yang jawab. Print semua address yang ACK.
static int i2cScan() {
  Serial.printf("[I2C] Scan SDA=%d SCL=%d ...\n", PIN_I2C_SDA, PIN_I2C_SCL);
  int found = 0;
  for (uint8_t addr = 0x08; addr < 0x78; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  -> device jawab di 0x%02X\n", addr);
      found++;
    }
  }
  if (found == 0) {
    Serial.println(F("  -> TIDAK ADA device yang jawab."));
  }
  return found;
}

static void waitSerialStart() {
  Serial.println(F("\n=== Kalibrasi MPU6050 ==="));
  Serial.println(F("Taruh device DIAM di meja, jangan dipegang."));
  Serial.println(F("Kirim karakter apa pun di Serial untuk mulai..."));
  while (!Serial.available()) {
    delay(50);
  }
  while (Serial.available()) Serial.read();
}

static bool readRaw(float &ax, float &ay, float &az,
                    float &gx, float &gy, float &gz) {
  sensors_event_t a, g, temp;
  if (!mpu.getEvent(&a, &g, &temp)) return false;
  ax = a.acceleration.x;
  ay = a.acceleration.y;
  az = a.acceleration.z;
  gx = g.gyro.x;
  gy = g.gyro.y;
  gz = g.gyro.z;
  return true;
}

static void doCalibration() {
  double sAx = 0, sAy = 0, sAz = 0;
  double sGx = 0, sGy = 0, sGz = 0;
  uint16_t n = 0;

  Serial.printf("Ambil %u sampel...\n", (unsigned)CALIB_SAMPLES);

  float ax, ay, az, gx, gy, gz;
  while (n < CALIB_SAMPLES) {
    if (!readRaw(ax, ay, az, gx, gy, gz)) {
      delay(SAMPLE_GAP_MS);
      continue;
    }
    sAx += ax;
    sAy += ay;
    sAz += az;
    sGx += gx;
    sGy += gy;
    sGz += gz;
    n++;
    if (n % 100 == 0) {
      Serial.printf("  %u/%u\n", n, (unsigned)CALIB_SAMPLES);
    }
    delay(SAMPLE_GAP_MS);
  }

  float mAx = (float)(sAx / n);
  float mAy = (float)(sAy / n);
  float mAz = (float)(sAz / n);

  // Gyro saat diam harusnya 0 -> offset = rata-rata
  gyrOffX = (float)(sGx / n);
  gyrOffY = (float)(sGy / n);
  gyrOffZ = (float)(sGz / n);

  // Accel saat diam telentang: X~0, Y~0, Z~+9.80665
  accOffX = mAx - 0.0f;
  accOffY = mAy - 0.0f;
  accOffZ = mAz - SENSORS_GRAVITY_STANDARD;

  calibrated = true;

  Serial.println(F("\n--- HASIL KALIBRASI (copy ke firmware) ---"));
  Serial.printf("accOffX = %.5f  // m/s^2\n", accOffX);
  Serial.printf("accOffY = %.5f  // m/s^2\n", accOffY);
  Serial.printf("accOffZ = %.5f  // m/s^2\n", accOffZ);
  Serial.printf("gyrOffX = %.5f  // rad/s\n", gyrOffX);
  Serial.printf("gyrOffY = %.5f  // rad/s\n", gyrOffY);
  Serial.printf("gyrOffZ = %.5f  // rad/s\n", gyrOffZ);
  Serial.println(F("---"));
  Serial.println(F("Cara pakai di repulse_band:"));
  Serial.println(F("  ax_corr = accel.x - accOffX; dst."));
  Serial.println(F("--- MONITOR mulai di bawah ---"));
}

static uint8_t positionFrom(float ax, float ay, float az) {
  float bx = fabsf(ax), by = fabsf(ay), bz = fabsf(az);
  if (bz > bx && bz > by) return az > 0 ? 0 : 3;
  if (bx > by) return ax > 0 ? 1 : 2;
  return 255;
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 2000) delay(10);

  Serial.println(F("\n=== MPU6050 Calibrator ==="));

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);

  // Bukti #1: ada device apa aja di bus?
  i2cScan();

  // Coba 0x68 dulu (AD0=GND), fallback 0x69 (AD0=VCC)
  Serial.print(F("[IMU] coba 0x68..."));
  if (mpu.begin(0x68, &Wire)) {
    mpuAddr = 0x68;
    Serial.println(F(" OK"));
  } else {
    Serial.println(F(" gagal."));
    Serial.print(F("[IMU] coba 0x69..."));
    if (mpu.begin(0x69, &Wire)) {
      mpuAddr = 0x69;
      Serial.println(F(" OK (AD0 modulmu ke VCC)"));
    } else {
      Serial.println(F(" gagal."));
    }
  }

  if (mpuAddr == 0) {
    Serial.println(F("\n[IMU] TIDAK DITEMUKAN di 0x68 maupun 0x69."));
    Serial.println(F("Cek (urut paling sering jadi biang):"));
    Serial.println(F(" 1. VCC modul -> pin 5V/VUSB board (GY-521 butuh 5V)."));
    Serial.println(F(" 2. GND modul -> GND board (wajib common)."));
    Serial.printf(" 3. SDA modul -> GPIO%d, SCL modul -> GPIO%d (jangan ketuker).\n", PIN_I2C_SDA, PIN_I2C_SCL);
    Serial.println(F(" 4. AD0 modul -> GND (kalau ngambang, address bisa 0x69)."));
    Serial.println(F(" 5. Kabel jumper: coba ganti, kabel breadboard sering putus dalam."));
    Serial.println(F("Kalau scan di atas NOL device, masalahnya di 1/2/3/5."));
    Serial.println(F("Kalau scan nemu address selain 0x68/0x69, modulnya bukan MPU6050."));
    while (true) delay(1000);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  waitSerialStart();
  doCalibration();
}

void loop() {
  float ax, ay, az, gx, gy, gz;
  if (!readRaw(ax, ay, az, gx, gy, gz)) {
    delay(10);
    return;
  }

  float cx = ax - (calibrated ? accOffX : 0);
  float cy = ay - (calibrated ? accOffY : 0);
  float cz = az - (calibrated ? accOffZ : 0);
  float wx = gx - (calibrated ? gyrOffX : 0);
  float wy = gy - (calibrated ? gyrOffY : 0);
  float wz = gz - (calibrated ? gyrOffZ : 0);

  // Rumus milli-g sama kayak motionLoop() firmware
  float gx_ = cx / SENSORS_GRAVITY_STANDARD;
  float gy_ = cy / SENSORS_GRAVITY_STANDARD;
  float gz_ = cz / SENSORS_GRAVITY_STANDARD;
  float mag = sqrtf(gx_ * gx_ + gy_ * gy_ + gz_ * gz_);
  uint16_t milliG = (uint16_t)(fabsf(mag - 1.0f) * 1000.0f);

  static uint32_t lastPrint = 0;
  if (millis() - lastPrint >= 500) {
    lastPrint = millis();
    Serial.printf(
      "acc=[%+.3f %+.3f %+.3f] gyr=[%+.4f %+.4f %+.4f] milliG=%u pos=%u\n",
      cx, cy, cz, wx, wy, wz,
      (unsigned)milliG, (unsigned)positionFrom(gx_, gy_, gz_));
  }

  delay(20);
}
