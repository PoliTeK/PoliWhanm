#include <WiFi.h>
#include <esp_now.h>
#include <esp_mac.h>
#include <SPI.h>

#define ESPNOW_WIFI_CHANNEL 6

//---- MCP4200  SPI pins (Might need to adapt to the wiring)
#define MCP_CS_PIN 10 // CS (Chip select)
#define MCP_MOSI_PIN 7 // SI
#define MCP_SCK_PIN 6 // SCK



// Must match master's struct exactly
typedef struct __attribute__((packed)) {
  uint32_t seq;
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  uint8_t pot0; //MCP 4200 Wiper 0 
  uint8_t pot1; //MCP 4200 Wiper 1
} imu_packet_t;

static uint32_t last_seq = 0;
static bool first_pkt = true;

// ---- MCP4200 helper functions ----
void mcpWriteByte(uint8_t cmd, uint8_t value) {
  digitalWrite(MCP_CS_PIN, LOW);
  SPI.transfer(cmd);
  SPI.transfer(value);
  digitalWrite(MCP_CS_PIN, HIGH);
}

// write pot 0 (0–255)
void mcpSetPot0(uint8_t value) {
  // 0x11 = write data to pot 0 (MCP4200 command)
  mcpWriteByte(0x11, value);
}

// write pot 1 (0–255)
void mcpSetPot1(uint8_t value) {
  // 0x12 = write data to pot 1
  mcpWriteByte(0x12, value);
}

// Callback: called automatically when a packet is received
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  // malformed  packet
  if (len != sizeof(imu_packet_t)) {
    Serial.printf("From " MACSTR " | unexpected len=%d\n", MAC2STR(info->src_addr), len);
    return;
  }
  // packet is ok

  const imu_packet_t *p = (const imu_packet_t*)data;
  
    // controls digipot
    mcpSetPot0(p->pot0);
    mcpSetPot1(p->pot1);
    
  

  const imu_packet_t *p = (const imu_packet_t*)data;

  // Detect packet loss
  if (!first_pkt) {
    if (p->seq != last_seq + 1) {
      Serial.printf("⚠️ Seq jump: got %lu expected %lu\n",
                    (unsigned long)p->seq, (unsigned long)(last_seq + 1));
    }
  }
  first_pkt = false;
  last_seq = p->seq;

  // Convert to physical units
  float ax_g = p->ax / 16384.0f;
  float ay_g = p->ay / 16384.0f;
  float az_g = p->az / 16384.0f;
  float gx_dps = p->gx / 131.0f;
  float gy_dps = p->gy / 131.0f;
  float gz_dps = p->gz / 131.0f;

  Serial.printf("From " MACSTR " | seq=%lu | "
                "A[g]: x=% .3f y=% .3f z=% .3f | "
                "G[dps]: x=% .2f y=% .2f z=% .2f\n",
                MAC2STR(info->src_addr),
                (unsigned long)p->seq,
                ax_g, ay_g, az_g,
                gx_dps, gy_dps, gz_dps);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // ---- init SPI & MCP4200 ----
  SPI.begin(MCK_SCK_PIN, -1, MCP_MOSI_PIN, MCP_CS_PIN);
  pinMode(MCP_CS_PIN, OUTPUT);
  digitalWrite(MCP_CS_PIN, HIGH); // Deselect pin
  
  // initial pot values (We have to alter them after connecting to the circuit)
  mcpSetPot0(128);  // mid-scale
  mcpSetPot1(128);
  
  // Wi-Fi STA mode on fixed channel
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  WiFi.setChannel(ESPNOW_WIFI_CHANNEL);

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed, rebooting...");
    delay(2000);
    ESP.restart();
  }

  // Register the receive callback
  esp_now_register_recv_cb(onDataRecv);

  Serial.println("SLAVE ready (native esp_now).");
  Serial.print("MAC: "); Serial.println(WiFi.macAddress());
  Serial.printf("Channel: %d\n", ESPNOW_WIFI_CHANNEL);
}

void loop() {

  delay(1000);
}