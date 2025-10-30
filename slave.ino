#include <WiFi.h>
#include <esp_now.h>
#include <esp_mac.h>

#define ESPNOW_WIFI_CHANNEL 6

// Must match master's struct exactly
typedef struct __attribute__((packed)) {
  uint32_t seq;
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
} imu_packet_t;

static uint32_t last_seq = 0;
static bool first_pkt = true;

// Callback: called automatically when a packet is received
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != sizeof(imu_packet_t)) {
    Serial.printf("From " MACSTR " | unexpected len=%d\n", MAC2STR(info->src_addr), len);
    return;
  }

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