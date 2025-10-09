#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

// ---------- CONFIG ----------
const char* OWN_NAME = "ESP0";   // Change per board: "ESP0", "ESP1", etc.
const int SCAN_SECONDS = 1;      // BLE scan duration
const int RSSI_THRESHOLD = -60;  // connect if RSSI >= -60 dBm
const unsigned long LOST_TIMEOUT_MS = 3000; // remove stale peers
// ----------------------------

struct Peer {
  String name;
  int rssi;
  unsigned long lastSeen;
  bool connected;   // true = within range (RSSI >= threshold)
};

std::vector<Peer> peers;
BLEScan* pBLEScan;

// ---------------------------------------------------------------------------
// Find peer by name
int findPeerIndex(const String &name) {
  for (size_t i = 0; i < peers.size(); ++i) {
    if (peers[i].name == name) return (int)i;
  }
  return -1;
}

// ---------------------------------------------------------------------------
// Update peer info from a BLE scan
void updatePeerFromScan(const String &name, int rssi) {
  if (name == String(OWN_NAME)) return; // ignore self

  int idx = findPeerIndex(name);
  unsigned long now = millis();

  if (idx < 0) {
    // New peer found
    Peer p;
    p.name = name;
    p.rssi = rssi;
    p.lastSeen = now;
    p.connected = (rssi >= RSSI_THRESHOLD);
    peers.push_back(p);

    if (p.connected) {
      Serial.printf("Connected to %s (RSSI=%d dBm)\n", p.name.c_str(), rssi);
    } else {
      Serial.println("searching...");
    }
  } else {
    // Existing peer update
    Peer &p = peers[idx];
    bool wasConnected = p.connected;
    p.rssi = rssi;
    p.lastSeen = now;

    // Check connection logic
    if (!wasConnected && rssi >= RSSI_THRESHOLD) {
      p.connected = true;
      Serial.printf("Connected to %s (RSSI=%d dBm)\n", p.name.c_str(), rssi);
    } 
    else if (wasConnected && rssi < RSSI_THRESHOLD) {
      p.connected = false;
      Serial.printf("ESP %s disconnected (RSSI=%d dBm)\n", p.name.c_str(), rssi);
    }
  }
}

// ---------------------------------------------------------------------------
// Remove peers that have gone missing
void checkForLeftPeers() {
  unsigned long now = millis();
  for (size_t i = 0; i < peers.size(); ++i) {
    Peer &p = peers[i];
    if ((now - p.lastSeen) > LOST_TIMEOUT_MS) {
      if (p.connected) {
        Serial.printf("ESP %s disconnected (timeout)\n", p.name.c_str());
      }
      peers.erase(peers.begin() + i);
      --i;
    }
  }
}

// ---------------------------------------------------------------------------
// BLE Advertising setup
void setupAdvertising() {
  BLEDevice::init(OWN_NAME);
  BLEServer* pServer = BLEDevice::createServer();
  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();

  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);

  BLEAdvertisementData advData;
  advData.setName(String(OWN_NAME));

  String msg = String("MSG:Hello from ") + String(OWN_NAME);
  advData.setManufacturerData(msg);
  pAdvertising->setAdvertisementData(advData);
  pAdvertising->start();
}

// ---------------------------------------------------------------------------
// Arduino setup
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.printf("Starting BLE range limiter (%s)\n", OWN_NAME);

  setupAdvertising();

  pBLEScan = BLEDevice::getScan();
  pBLEScan->setActiveScan(true);
}

// ---------------------------------------------------------------------------
// Arduino loop
void loop() {
  BLEScanResults* found = pBLEScan->start(SCAN_SECONDS, false);
  if (found == nullptr) {
    Serial.println("searching...");
    delay(200);
    return;
  }

  int count = found->getCount();
  bool anyConnected = false;

  for (int i = 0; i < count; ++i) {
    BLEAdvertisedDevice adv = found->getDevice(i);
    String name = adv.getName();
    if (name.length() == 0) continue;
    int rssi = adv.getRSSI();

    if (name.startsWith("ESP")) {
      updatePeerFromScan(name, rssi);

      // Only print messages if connected
      if (rssi >= RSSI_THRESHOLD && adv.haveManufacturerData()) {
        String mdata = adv.getManufacturerData();
        if (mdata.startsWith("MSG:")) {
          String payload = mdata.substring(4);
          Serial.printf("Message from %s (RSSI=%d): %s\n", name.c_str(), rssi, payload.c_str());
          anyConnected = true;
        }
      }
    }
  }

  if (!anyConnected) {
    Serial.println("searching...");
  }

  pBLEScan->clearResults();
  checkForLeftPeers();
  delay(50);
}
