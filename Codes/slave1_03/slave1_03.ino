#include <WiFi.h>
#include <esp_now.h>
#include <esp_mac.h>
#include <SPI.h>

#define ESPNOW_WIFI_CHANNEL 6

//---- MCP4100 SPI pins (adapt if needed)
#define MCP_CS_PIN   10   // CS
#define MCP_MOSI_PIN 7    // SI
#define MCP_SCK_PIN  6    // SCK

// ---- Incoming packet structure (MUST match master)
typedef struct __attribute__((packed)) {
  uint32_t seq;
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
} imu_packet_t;

static uint32_t last_seq = 0;
static bool first_pkt = true;

// ---------------------------------------------------
// ---- MCP4100 Single-Pot Write ----
// ---------------------------------------------------

void mcpWrite(uint8_t value) {
  digitalWrite(MCP_CS_PIN, LOW);
  SPI.transfer(0x11);      // Write command
  SPI.transfer(value);     // Wiper: 0–255
  digitalWrite(MCP_CS_PIN, HIGH);
}

// ---------------------------------------------------
// ---- Complementary Filter IMU Processing ----
// ---------------------------------------------------

static float previousAngle = 0.0f;
static bool firstAngle = true;

void processIMU(float ax, float ay, float az, float gx, float gy, float gz) {

  // ---- 1) Compute tilt angle from accelerometer ----
  float angleAcc = atan2(ay, az) * 180.0f / PI;    // degrees

  // ---- 2) Gyro rate (gx is already in deg/sec) ----
  float gyroRate = gx;

  // ---- 3) Time step (approx 100Hz ESP-NOW packets) ----
  float dt = 0.01f;   // 10 ms

  // ---- 4) First run: initialize angle ----
  if (firstAngle) {
    previousAngle = angleAcc;
    firstAngle = false;
  }

  // ---- 5) Integrate gyro ----
  float angleGyro = previousAngle + gyroRate * dt;

  // ---- 6) Complementary filter ----
  float alpha = 0.98f;   // 98% gyro, 2% accel
  float angle = alpha * angleGyro + (1.0f - alpha) * angleAcc;

  previousAngle = angle;

  // ---- 7) Normalize angle e.g., -60..60 deg → 0..1 ----
  float normalized = (angle + 60.0f) / 120.0f;
  normalized = constrain(normalized, 0.0f, 1.0f);

  // ---- 8) Convert to 0–255 ----
  uint8_t potValue = (uint8_t)(normalized * 255.0f);

  // ---- 9) Write to the MCP4100 ----
  mcpWrite(potValue);

  // Debug print
  Serial.printf("Angle=%.2f  | pot=%d\n", angle, potValue);
}

// ---------------------------------------------------
// ---- ESP-NOW Receive Callback ----
// ---------------------------------------------------

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {

  if (len != sizeof(imu_packet_t)) {
    Serial.printf("From " MACSTR " | unexpected len=%d\n",
                   MAC2STR(info->src_addr), len);
    return;
  }

  const imu_packet_t *p = (const imu_packet_t*)data;

  // packet loss detection
  if (!first_pkt && p->seq != last_seq + 1) {
    Serial.printf("⚠ Seq jump: got %lu expected %lu\n",
      (unsigned long)p->seq,
      (unsigned long)(last_seq + 1));
  }
  first_pkt = false;
  last_seq = p->seq;

  // convert IMU to physical units
  float ax = p->ax / 16384.0f;
  float ay = p->ay / 16384.0f;
  float az = p->az / 16384.0f;

  float gx = p->gx / 131.0f;
  float gy = p->gy / 131.0f;
  float gz = p->gz / 131.0f;

  // ---- Use complementary filter to get stable angle ----
  processIMU(ax, ay, az, gx, gy, gz);

  // Optional: full IMU debug

  Serial.printf("Seq %lu | ax=% .2f ay=% .2f az=% .2f | gx=%.2f gy=%.2f gz=%.2f\n",
                (unsigned long)p->seq,
                ax, ay, az,
                gx, gy, gz);
}

// ---------------------------------------------------
// -------------------- Setup ------------------------
// ---------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(500);

  // init SPI
  SPI.begin(MCP_SCK_PIN, -1, MCP_MOSI_PIN, MCP_CS_PIN);
  pinMode(MCP_CS_PIN, OUTPUT);
  digitalWrite(MCP_CS_PIN, HIGH);

  // start the pot at midpoint
  mcpWrite(128);

  // WiFi + ESP-NOW
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  WiFi.setChannel(ESPNOW_WIFI_CHANNEL);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed!");
    delay(1000);
    ESP.restart();
  }

  esp_now_register_recv_cb(onDataRecv);

  Serial.println("Slave ready.");
  Serial.println(WiFi.macAddress());
}

// ---------------------------------------------------
// --------------------- Loop ------------------------
// ---------------------------------------------------

void loop() {
  delay(10);
}