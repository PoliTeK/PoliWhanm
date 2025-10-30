#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include <esp_mac.h>

// ---------- Config ----------
#define ESPNOW_WIFI_CHANNEL 6
#define SDA_PIN 6
#define SCL_PIN 7
#define MPU_ADDR 0x68

// MPU6050 registers
#define PWR_MGMT_1    0x6B
#define SMPLRT_DIV    0x19
#define CONFIG_REG    0x1A
#define GYRO_CONFIG   0x1B
#define ACCEL_CONFIG  0x1C
#define ACCEL_XOUT_H  0x3B
#define GYRO_XOUT_H   0x43

// ---- Data packet ----
typedef struct __attribute__((packed)) {
  uint32_t seq;
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
} imu_packet_t;

// ---- Globals ----
uint32_t seq = 0;
uint8_t broadcastAddress[] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

// ---- I2C helpers ----
int16_t read16(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((int)MPU_ADDR, 2);
  int16_t hi = Wire.read();
  int16_t lo = Wire.read();
  return (hi << 8) | lo;
}

void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void mpuInit() {
  delay(100);
  mpuWrite(PWR_MGMT_1, 0x01); // wake, PLL X gyro
  delay(50);
  mpuWrite(CONFIG_REG, 0x04); // DLPF ~20 Hz
  mpuWrite(GYRO_CONFIG, 0x00); // ±250 dps
  mpuWrite(ACCEL_CONFIG, 0x00); // ±2 g
  mpuWrite(SMPLRT_DIV, 9);      // 100 Hz
  delay(50);
}

// ---- Send status callback ----
void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  if (info) {
    Serial.printf("Send to " MACSTR " %s\n",
                  MAC2STR(info->des_addr),
                  status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
  } else {
    Serial.printf("Send complete, status: %s (no tx_info)\n",
                  status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // I2C + MPU
  Wire.begin(SDA_PIN, SCL_PIN, 100000);
  mpuInit();

  // Wi-Fi / ESP-NOW
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  WiFi.setChannel(ESPNOW_WIFI_CHANNEL);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed, rebooting...");
    delay(2000);
    ESP.restart();
  }

  esp_now_register_send_cb(onDataSent);

  // Add broadcast peer (everyone)
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = ESPNOW_WIFI_CHANNEL;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add broadcast peer!");
  }

  Serial.println("MASTER ready.");
  Serial.print("MAC: "); Serial.println(WiFi.macAddress());
  Serial.printf("Channel: %d\n", ESPNOW_WIFI_CHANNEL);
}

void loop() {
  imu_packet_t pkt;
  pkt.seq = seq++;
  pkt.ax = read16(ACCEL_XOUT_H);
  pkt.ay = read16(ACCEL_XOUT_H + 2);
  pkt.az = read16(ACCEL_XOUT_H + 4);
  pkt.gx = read16(GYRO_XOUT_H);
  pkt.gy = read16(GYRO_XOUT_H + 2);
  pkt.gz = read16(GYRO_XOUT_H + 4);

  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&pkt, sizeof(pkt));
  if (result != ESP_OK) {
    Serial.printf("Send failed! (%d)\n", result);
  } else {
    Serial.printf("Sent seq=%lu | ax=%d ay=%d az=%d | gx=%d gy=%d gz=%d\n",
                  (unsigned long)pkt.seq,
                  pkt.ax, pkt.ay, pkt.az,
                  pkt.gx, pkt.gy, pkt.gz);
  }
  delay(20); // ~50 Hz
}