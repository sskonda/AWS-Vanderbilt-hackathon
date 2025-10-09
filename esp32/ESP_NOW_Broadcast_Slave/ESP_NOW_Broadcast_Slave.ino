/*
    ESP-NOW Broadcast Slave (updated to print RSSI)
    Adapted from Lucas Saavedra Vaz - 2024
    Added: receive callback that prints RSSI (dBm) and auto-registers new masters on broadcast
*/

#include "ESP32_NOW.h"
#include "WiFi.h"
#include <esp_mac.h>   // For MAC2STR / MACSTR macros
#include <esp_now.h>   // For esp_now_recv_info_t and esp_now_register_recv_cb
#include <vector>

/* Definitions */
#define ESPNOW_WIFI_CHANNEL 6

/* Classes */

// Creating a new class that inherits from the ESP_NOW_Peer class is required.
class ESP_NOW_Peer_Class : public ESP_NOW_Peer {
public:
  // Constructor of the class
  ESP_NOW_Peer_Class(const uint8_t *mac_addr, uint8_t channel, wifi_interface_t iface, const uint8_t *lmk)
    : ESP_NOW_Peer(mac_addr, channel, iface, lmk) {}

  // Destructor of the class
  ~ESP_NOW_Peer_Class() {}

  // Function to register the master peer
  bool add_peer() {
    if (!add()) {
      log_e("Failed to register the broadcast peer");
      return false;
    }
    return true;
  }

  // Function to print the received messages from the master (kept for compatibility)
  void onReceive(const uint8_t *data, size_t len, bool broadcast) {
    Serial.printf("Received a message from master " MACSTR " (%s)\n", MAC2STR(addr()), broadcast ? "broadcast" : "unicast");
    Serial.printf("  Message: %.*s\n", (int)len, (char *)data);
  }
};

/* Global Variables */

// List of all the masters. It will be populated when a new master is registered
// Note: Using pointers instead of objects to prevent dangling pointers when the vector reallocates
std::vector<ESP_NOW_Peer_Class *> masters;

/* Helpers */

bool mac_in_masters(const uint8_t *mac) {
  for (size_t i = 0; i < masters.size(); ++i) {
    if (masters[i] && memcmp(mac, masters[i]->addr(), 6) == 0) return true;
  }
  return false;
}

/* Receive callback (modern esp_now_recv_info_t signature)
   This callback is registered with esp_now_register_recv_cb and will be called for every ESP-NOW packet received.
*/
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (!info) return;

  // Extract RSSI (signed 8-bit, in dBm)
  int8_t rssi = info->rx_ctrl->rssi;

  // Print basic packet info
  Serial.printf("Recv: src=" MACSTR "  dst=" MACSTR "  RSSI=%d dBm  len=%d\n",
                MAC2STR(info->src_addr), MAC2STR(info->des_addr), rssi, len);
  Serial.printf("  Payload: %.*s\n", len, (char *)data);

  // If this is a broadcast packet and the sender is unknown, register it as a master
  if (memcmp(info->des_addr, ESP_NOW.BROADCAST_ADDR, 6) == 0) {
    if (!mac_in_masters(info->src_addr)) {
      Serial.printf("Unknown broadcast sender " MACSTR "  registering as new master (RSSI=%d dBm)\n",
                    MAC2STR(info->src_addr), rssi);

      ESP_NOW_Peer_Class *new_master =
        new ESP_NOW_Peer_Class(info->src_addr, ESPNOW_WIFI_CHANNEL, WIFI_IF_STA, nullptr);

      if (!new_master->add_peer()) {
        Serial.println("Failed to register the new master");
        delete new_master;
      } else {
        masters.push_back(new_master);
        Serial.printf("Successfully registered master " MACSTR " (total masters: %zu)\n",
                      MAC2STR(new_master->addr()), masters.size());
      }
    } else {
      // Known master broadcast — optional: find that master and call its onReceive()
      for (size_t i = 0; i < masters.size(); ++i) {
        if (masters[i] && memcmp(info->src_addr, masters[i]->addr(), 6) == 0) {
          masters[i]->onReceive(data, len, true);
          break;
        }
      }
    }
  } else {
    // Non-broadcast (unicast) packet; print or ignore depending on your logic
    Serial.println("Received a unicast packet (ignored for registration).");
  }
}

/* Main */

void setup() {
  Serial.begin(115200);
  delay(50);

  // Initialize the Wi-Fi module
  WiFi.mode(WIFI_STA);
  WiFi.setChannel(ESPNOW_WIFI_CHANNEL);
  // Wait briefly for STA to start — this check mirrors your earlier code
  unsigned long start = millis();
  while (!WiFi.STA.started() && (millis() - start) < 5000) {
    delay(50);
  }

  Serial.println("ESP-NOW Example - Broadcast Slave (RSSI enabled)");
  Serial.println("Wi-Fi parameters:");
  Serial.println("  Mode: STA");
  Serial.println("  MAC Address: " + WiFi.macAddress());
  Serial.printf("  Channel: %d\n", ESPNOW_WIFI_CHANNEL);

  // Initialize the ESP-NOW protocol (using your wrapper)
  if (!ESP_NOW.begin()) {
    Serial.println("Failed to initialize ESP-NOW");
    Serial.println("Reeboting in 5 seconds...");
    delay(5000);
    ESP.restart();
  }

  Serial.printf("ESP-NOW version: %d, max data length: %d\n", ESP_NOW.getVersion(), ESP_NOW.getMaxDataLen());

  // Register a receive callback that provides esp_now_recv_info_t (RSSI is available here)
  // Use the underlying esp_now_register_recv_cb so we get the esp_now_recv_info_t-based callback.
  // Note: this function expects a callback whose first arg is const esp_now_recv_info_t* (as used above).
  esp_err_t rc = esp_now_register_recv_cb(onDataRecv);
  if (rc != ESP_OK) {
    Serial.printf("Warning: esp_now_register_recv_cb returned %d\n", (int)rc);
    // Some wrapper versions might offer a different register func; if this fails, you can try
    // using ESP_NOW.onNewPeer(...) as a fallback (but that callback will not fire for every packet).
  }

  Serial.println("Setup complete. Waiting for a master to broadcast a message...");
}

void loop() {
  // Print debug information every 10 seconds
  static unsigned long last_debug = 0;
  if (millis() - last_debug > 10000) {
    last_debug = millis();
    Serial.printf("Registered masters: %zu\n", masters.size());
    for (size_t i = 0; i < masters.size(); i++) {
      if (masters[i]) {
        Serial.printf("  Master %zu: " MACSTR "\n", i, MAC2STR(masters[i]->addr()));
      }
    }
  }

  delay(100);
}
