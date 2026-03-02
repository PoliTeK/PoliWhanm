#include <WiFi.h>
#include <esp_now.h>
#include <esp_mac.h>
#include <SPI.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>

#define ESPNOW_WIFI_CHANNEL 6

//---- MCP4100 SPI pins
#define MCP_CS_PIN   10   // CS
#define MCP_MOSI_PIN 7    // SI
#define MCP_SCK_PIN  6    // SCK

// ---------- I2C LCD pins
#define I2C_SDA_PIN  9
#define I2C_SCL_PIN  8

// LCD I2C address:
#define LCD_ADDR 0x27

LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);

// ---------- Button ----------
#define BUTTON_PIN  3
static uint32_t lastDebounceMs = 0;
static bool lastRawBtn = HIGH;
static bool stableBtn = HIGH;
static const uint32_t debounceDelayMs = 30;

// ---- Incoming packet structure
typedef struct __attribute__((packed)) {
  uint32_t seq;
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
} imu_packet_t;

static uint32_t last_seq = 0;
static bool first_pkt = true;

// --------- Shared data between ESP-NOW callback and loop ----------
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
static imu_packet_t latestPkt;
static volatile bool havePkt = false;

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
// ---- Presets (mapping normalized 0..1 -> 0..255) ----
// ---------------------------------------------------
enum Preset : uint8_t {
  PRESET_LINEAR = 0,
  PRESET_INVERT,
  PRESET_SOFT_CENTER,
  PRESET_DEADZONE,
  PRESET_STEPPED_8,
  PRESET_COUNT
};

static Preset presetIndex = PRESET_LINEAR;

const char* presetName(Preset p) {
  switch (p) {
    case PRESET_LINEAR:      return "LINEAR";
    case PRESET_INVERT:      return "INVERT";
    case PRESET_SOFT_CENTER: return "SOFT-CENTER";
    case PRESET_DEADZONE:    return "DEADZONE";
    case PRESET_STEPPED_8:   return "STEPPED-8";
    default:                 return "UNKNOWN";
  }
}

// Apply mapping. Input: normalized 0..1. Output: 0..255.
uint8_t applyPreset(Preset p, float x01) {
  x01 = constrain(x01, 0.0f, 1.0f);

  switch (p) {
    case PRESET_LINEAR: {
      return (uint8_t)lroundf(x01 * 255.0f);
    }

    case PRESET_INVERT: {
      float y = 1.0f - x01;
      return (uint8_t)lroundf(y * 255.0f);
    }

    case PRESET_SOFT_CENTER: {
      // More resolution near the center (S-curve-ish).
      // Simple smoothstep then remap.
      float y = x01 * x01 * (3.0f - 2.0f * x01); // smoothstep
      return (uint8_t)lroundf(y * 255.0f);
    }

    case PRESET_DEADZONE: {
      // Deadzone around center (e.g. +/- 7%)
      const float dz = 0.07f;
      float centered = x01 - 0.5f;
      if (fabsf(centered) < dz) {
        return 128; // lock in center
      }
      // Rescale outside deadzone back to 0..1
      float sign = (centered >= 0) ? 1.0f : -1.0f;
      float mag = (fabsf(centered) - dz) / (0.5f - dz); // 0..1
      float y01 = 0.5f + sign * (mag * 0.5f);
      return (uint8_t)lroundf(constrain(y01, 0.0f, 1.0f) * 255.0f);
    }

    case PRESET_STEPPED_8: {
      // Quantize to 8 steps (0..7)
      int step = (int)lroundf(x01 * 7.0f);
      step = constrain(step, 0, 7);
      float y = step / 7.0f;
      return (uint8_t)lroundf(y * 255.0f);
    }

    default:
      return (uint8_t)lroundf(x01 * 255.0f);
  }
}

// ---------------------------------------------------
// ---- Complementary Filter IMU Processing ----
// ---------------------------------------------------

static float previousAngle = 0.0f;
static bool firstAngle = true;

// Keep some state to display
static float lastAngleDeg = 0.0f;
static uint8_t lastPotValue = 128;

// dt: use measured time between packets instead of hardcoding 0.01
static uint32_t lastProcessMicros = 0;

void processIMU(float ax, float ay, float az, float gx, float gy, float gz) {

  // ---- 1) Compute tilt angle from accelerometer ----
  float angleAcc = atan2(ay, az) * 180.0f / PI;    // degrees

  // ---- 2) Gyro rate (gx is already in deg/sec) ----
  float gyroRate = gx;

  // ---- 3) dt from micros ----
  uint32_t nowUs = micros();
  float dt = 0.01f;
  if (lastProcessMicros != 0) {
    dt = (nowUs - lastProcessMicros) / 1e6f;
    // clamp dt so weird WiFi stalls don't explode the integrator
    dt = constrain(dt, 0.002f, 0.05f);
  }
  lastProcessMicros = nowUs;

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
  lastAngleDeg = angle;

  // ---- 7) Normalize angle e.g., -60..60 deg → 0..1 ----
  float normalized = (angle + 60.0f) / 120.0f;
  normalized = constrain(normalized, 0.0f, 1.0f);

  // ---- 8) Apply preset mapping -> pot ----
  uint8_t potValue = applyPreset(presetIndex, normalized);
  lastPotValue = potValue;

  // ---- 9) Write to the MCP4100 ----
  mcpWrite(potValue);

  // Debug print
  Serial.printf("Angle=%.2f  | pot=%d\n", angle, potValue);
}

// ---------------------------------------------------
// ---- LCD UI ----
// ---------------------------------------------------
static uint32_t lastLcdMs = 0;

void lcdShowPresetAndValue(bool force = false) {
  uint32_t now = millis();
  if (!force && (now - lastLcdMs) < 100) return; // limit refresh
  lastLcdMs = now;

  lcd.setCursor(0, 0);
  // Line 1: "P:2/5 INVERT"
  char line1[17];
  snprintf(line1, sizeof(line1), "P:%u/%u %-9s",
           (unsigned)(presetIndex + 1),
           (unsigned)PRESET_COUNT,
           presetName(presetIndex));
  lcd.print(line1);

  lcd.setCursor(0, 1);
  // Line 2: "pot:128 ang:+12"
  char line2[17];
  snprintf(line2, sizeof(line2), "pot:%3u ang:%+3d",
           (unsigned)lastPotValue,
           (int)lroundf(lastAngleDeg));
  lcd.print(line2);
}

// ---------------------------------------------------
// ---- Button handling ----
// ---------------------------------------------------
void handleButton() {
  bool raw = digitalRead(BUTTON_PIN); // HIGH idle, LOW pressed (pullup)

  if (raw != lastRawBtn) {
    lastDebounceMs = millis();
    lastRawBtn = raw;
  }

  if ((millis() - lastDebounceMs) > debounceDelayMs) {
    if (raw != stableBtn) {
      stableBtn = raw;

      // detect press edge: HIGH -> LOW
      if (stableBtn == LOW) {
        presetIndex = (Preset)((presetIndex + 1) % PRESET_COUNT);
        lcdShowPresetAndValue(true); // immediate feedback
      }
    }
  }
}

// ---------------------------------------------------
// ---- ESP-NOW Receive Callback ----
// ---------------------------------------------------
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != (int)sizeof(imu_packet_t)) return;

  // Copy packet quickly, do NOT do heavy work here
  portENTER_CRITICAL_ISR(&mux);
  memcpy((void*)&latestPkt, data, sizeof(imu_packet_t));
  havePkt = true;
  portEXIT_CRITICAL_ISR(&mux);

  // Optional: lightweight seq loss detection (still cheap)
  const imu_packet_t *p = (const imu_packet_t*)data;
  if (!first_pkt && p->seq != last_seq + 1) {
    // Keep prints minimal in callback; comment out if it causes jitter
    // Serial.printf("⚠ Seq jump: got %lu expected %lu\n",
    //   (unsigned long)p->seq, (unsigned long)(last_seq + 1));
  }
  first_pkt = false;
  last_seq = p->seq;
}

// ---------------------------------------------------
// -------------------- Setup ------------------------
// ---------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(200);

  // Button
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // SPI for MCP4100
  SPI.begin(MCP_SCK_PIN, -1, MCP_MOSI_PIN, MCP_CS_PIN);
  pinMode(MCP_CS_PIN, OUTPUT);
  digitalWrite(MCP_CS_PIN, HIGH);
  mcpWrite(128);

  // I2C + LCD
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Slave starting");
  lcd.setCursor(0, 1);
  lcd.print("Preset UI...");

  // WiFi + ESP-NOW
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);
  WiFi.setChannel(ESPNOW_WIFI_CHANNEL);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed!");
    delay(300);
    ESP.restart();
  }
  esp_now_register_recv_cb(onDataRecv);

  delay(300);
  lcd.clear();
  lcdShowPresetAndValue(true);

  Serial.println("Slave ready.");
  Serial.println(WiFi.macAddress());
}

// ---------------------------------------------------
// --------------------- Loop ------------------------
// ---------------------------------------------------
void loop() {
  handleButton();

  // If a new packet arrived, process it here (not in callback)
  if (havePkt) {
    imu_packet_t pktCopy;
    portENTER_CRITICAL(&mux);
    pktCopy = latestPkt;
    havePkt = false;
    portEXIT_CRITICAL(&mux);

    // Convert IMU to physical units
    float ax = pktCopy.ax / 16384.0f;
    float ay = pktCopy.ay / 16384.0f;
    float az = pktCopy.az / 16384.0f;

    float gx = pktCopy.gx / 131.0f;
    float gy = pktCopy.gy / 131.0f;
    float gz = pktCopy.gz / 131.0f;

    processIMU(ax, ay, az, gx, gy, gz);

    // Optional debug (this is now safe in loop)
    // Serial.printf("Seq %lu | ax=% .2f ay=% .2f az=% .2f | gx=%.2f gy=%.2f gz=%.2f\n",
    //               (unsigned long)pktCopy.seq, ax, ay, az, gx, gy, gz);
  }

  // Refresh LCD periodically
  lcdShowPresetAndValue(false);

  delay(1);
}