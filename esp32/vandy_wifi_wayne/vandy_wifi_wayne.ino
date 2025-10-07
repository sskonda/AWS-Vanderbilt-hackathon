// /* ESP32 BLE proximity demo
//    - Set OWN_NAME to "ESP0" / "ESP1" / "ESP2" on each board
//    - Each board advertises its name and scans for others.
//    - When another ESP enters estimated ~1m range (RSSI >= RSSI_THRESHOLD),
//      this board prints "Hello from <OWN_NAME>" (once on entry for that peer).
//    - When a peer leaves range (timeout or RSSI drops), prints "ESP <peer> has left the range".
   
//    Notes: RSSI thresholds are environment-dependent. See comments below for tuning.
// */

// #include <Arduino.h>
// #include <BLEDevice.h>
// #include <BLEUtils.h>
// #include <BLEScan.h>
// #include <BLEAdvertisedDevice.h>

// // ---------- CONFIG ----------
// const char* OWN_NAME = "ESP1";   // <-- Change this for each board: "ESP0", "ESP1", "ESP2"
// const int SCAN_SECONDS = 1;      // scan time (seconds)
// const int RSSI_THRESHOLD = -65;  // RSSI threshold for "within ~1 meter" (tune for your env)
// const unsigned long LOST_TIMEOUT_MS = 3000; // treat as left if not seen for this many ms
// // ----------------------------

// struct Peer {
//   String name;
//   int rssi;
//   unsigned long lastSeen;
//   bool inRange;
// };

// std::vector<Peer> peers;
// BLEScan* pBLEScan;

// int findPeerIndex(const String &name) {
//   for (size_t i = 0; i < peers.size(); ++i) {
//     if (peers[i].name == name) return (int)i;
//   }
//   return -1;
// }

// void updatePeerFromScan(const String &name, int rssi) {
//   if (name == String(OWN_NAME)) return; // ignore ourself

//   int idx = findPeerIndex(name);
//   unsigned long now = millis();
//   if (idx < 0) {
//     // new peer
//     Peer p;
//     p.name = name;
//     p.rssi = rssi;
//     p.lastSeen = now;
//     p.inRange = (rssi >= RSSI_THRESHOLD);
//     peers.push_back(p);
//     if (p.inRange) {
//       // entry event
//       Serial.printf("Hello from %s\n", OWN_NAME);
//       // (We could also print that we saw the peer, but user asked the devices to send their Hello.)
//     }
//   } else {
//     // update existing
//     Peer &p = peers[idx];
//     bool wasInRange = p.inRange;
//     p.rssi = rssi;
//     p.lastSeen = now;
//     p.inRange = (rssi >= RSSI_THRESHOLD);
//     if (!wasInRange && p.inRange) {
//       // peer entered range
//       Serial.printf("Hello from %s\n", OWN_NAME);
//     }
//     // No immediate action when still in range; we only announce on entry.
//   }
// }

// void checkForLeftPeers() {
//   unsigned long now = millis();
//   for (size_t i = 0; i < peers.size(); ++i) {
//     Peer &p = peers[i];
//     bool consideredLeft = false;
//     if ((now - p.lastSeen) > LOST_TIMEOUT_MS) {
//       consideredLeft = true;
//     } else if (p.rssi < RSSI_THRESHOLD && p.inRange) {
//       // immediate drop below threshold -- treat as left (optional)
//       consideredLeft = true;
//     }

//     if (consideredLeft && p.inRange) {
//       // If we previously thought they were inRange, announce left
//       Serial.printf("ESP %s has left the range\n", p.name.c_str());
//       p.inRange = false;
//     }

//     // if long gone, we may purge to save memory (optional)
//     if ((now - p.lastSeen) > (LOST_TIMEOUT_MS * 10)) {
//       // remove stale entries
//       peers.erase(peers.begin() + i);
//       --i;
//     }
//   }
// }

// void setupAdvertising() {
//   // Initialize BLE as peripheral advertiser so others see this device
//   BLEDevice::init(OWN_NAME);
//   BLEServer* pServer = BLEDevice::createServer(); // required by library to create advertising
//   BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
//   // advertise device name
//   pAdvertising->setAdvertisementType(ADV_TYPE_NONCONN_IND); // non-connectable (we only need advertisement)
//   pAdvertising->setScanResponse(false);
//   pAdvertising->setMinPreferred(0x06);  // recommended to avoid connection issues
//   pAdvertising->setMinPreferred(0x12);
//   // attach device name in advertisement
//   BLEAdvertisementData advData;
//   advData.setName(String(OWN_NAME));
//   pAdvertising->setAdvertisementData(advData);
//   pAdvertising->start();
// }

// void setup() {
//   Serial.begin(115200);
//   delay(1000);
//   Serial.printf("Starting BLE proximity (%s)\n", OWN_NAME);

//   // Start advertising our name
//   setupAdvertising();

//   // Set up scanner
//   pBLEScan = BLEDevice::getScan();
//   pBLEScan->setActiveScan(true); // active scan to solicit scan response and get RSSI
// }

// void loop() {
//   // Run a blocking scan for SCAN_SECONDS seconds
//   BLEScanResults found = *(pBLEScan->start(SCAN_SECONDS, false));
//   int count = found.getCount();
//   for (int i = 0; i < count; ++i) {
//     BLEAdvertisedDevice adv = found.getDevice(i);
// String name = adv.getName();
// if (name.length() == 0) continue;
//     int rssi = adv.getRSSI();
//     // Only consider devices with names starting with "ESP" (helps ignore phones, etc.)
//     if (name.startsWith("ESP")) {
//       updatePeerFromScan(name, rssi);
//       printf("RSSI: %d\n", rssi);
//     }
//   }
//   // after processing scan results, check for peers that have left
//   checkForLeftPeers();

//   // small delay to avoid hogging CPU; scanning already took SCAN_SECONDS
//   delay(50);
// }

/* ESP32 BLE proximity demo (message-in-advertisement)
   - Set OWN_NAME to "ESP0" / "ESP1" / "ESP2" on each board
   - Each board advertises its name and a short message in manufacturer-data:
       "MSG:Hello from ESP0"
   - Scanners print the peer name, RSSI, and the message (if present).
   - This variant matches Arduino-ESP32 core 3.3.x style APIs (BLEScan::start returns a pointer;
     BLEAdvertisementData uses Arduino String for manufacturer data).
*/

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

// ---------- CONFIG ----------
const char* OWN_NAME = "ESP1";   // <-- Change this for each board: "ESP0", "ESP1", "ESP2"
const int SCAN_SECONDS = 1;      // scan time (seconds)
const int RSSI_THRESHOLD = -65;  // RSSI threshold for "within ~1 meter" (tune for your env)
const unsigned long LOST_TIMEOUT_MS = 3000; // treat as left if not seen for this many ms
// ----------------------------

struct Peer {
  String name;
  int rssi;
  unsigned long lastSeen;
  bool inRange;
};

std::vector<Peer> peers;
BLEScan* pBLEScan;

int findPeerIndex(const String &name) {
  for (size_t i = 0; i < peers.size(); ++i) {
    if (peers[i].name == name) return (int)i;
  }
  return -1;
}

void updatePeerFromScan(const String &name, int rssi) {
  if (name == String(OWN_NAME)) return; // ignore ourself

  int idx = findPeerIndex(name);
  unsigned long now = millis();
  if (idx < 0) {
    // new peer
    Peer p;
    p.name = name;
    p.rssi = rssi;
    p.lastSeen = now;
    p.inRange = (rssi >= RSSI_THRESHOLD);
    peers.push_back(p);
    if (p.inRange) {
      // entry event
      Serial.printf("Hello from %s\n", OWN_NAME);
    }
  } else {
    // update existing
    Peer &p = peers[idx];
    bool wasInRange = p.inRange;
    p.rssi = rssi;
    p.lastSeen = now;
    p.inRange = (rssi >= RSSI_THRESHOLD);
    if (!wasInRange && p.inRange) {
      // peer entered range
      Serial.printf("Hello from %s\n", OWN_NAME);
    }
  }
}

void checkForLeftPeers() {
  unsigned long now = millis();
  for (size_t i = 0; i < peers.size(); ++i) {
    Peer &p = peers[i];
    bool consideredLeft = false;
    if ((now - p.lastSeen) > LOST_TIMEOUT_MS) {
      consideredLeft = true;
    } else if (p.rssi < RSSI_THRESHOLD && p.inRange) {
      // immediate drop below threshold -- treat as left (optional)
      consideredLeft = true;
    }

    if (consideredLeft && p.inRange) {
      // If we previously thought they were inRange, announce left
      Serial.printf("ESP %s has left the range\n", p.name.c_str());
      p.inRange = false;
    }

    // if long gone, we may purge to save memory (optional)
    if ((now - p.lastSeen) > (LOST_TIMEOUT_MS * 10)) {
      // remove stale entries
      peers.erase(peers.begin() + i);
      --i;
    }
  }
}

void setupAdvertising() {
  // Initialize BLE as peripheral advertiser so others see this device
  BLEDevice::init(OWN_NAME);
  BLEServer* pServer = BLEDevice::createServer(); // required by library to create advertising
  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  // advertise device name
  pAdvertising->setAdvertisementType(ADV_TYPE_NONCONN_IND); // non-connectable (we only need advertisement)
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x06);  // recommended to avoid connection issues
  pAdvertising->setMinPreferred(0x12);

  // attach device name in advertisement and a short manufacturer-data payload containing the message
  BLEAdvertisementData advData;
  advData.setName(String(OWN_NAME));

  // Build a simple message payload and set as manufacturer data.
  // We'll use the prefix "MSG:" so scanners can detect user message content easily.
  String msg = String("MSG:Hello from ") + String(OWN_NAME);
  advData.setManufacturerData(msg); // use Arduino String on this core

  pAdvertising->setAdvertisementData(advData);
  pAdvertising->start();
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.printf("Starting BLE proximity (%s)\n", OWN_NAME);

  // Start advertising our name + message
  setupAdvertising();

  // Set up scanner
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setActiveScan(true); // active scan to solicit scan response and get RSSI
}

void loop() {
  // Run a blocking scan for SCAN_SECONDS seconds
  BLEScanResults* found = pBLEScan->start(SCAN_SECONDS, false); // note: returns pointer on this core
  if (found == nullptr) {
    // Defensive: nothing found or scan failed
    delay(50);
    return;
  }

  int count = found->getCount();
  for (int i = 0; i < count; ++i) {
    BLEAdvertisedDevice adv = found->getDevice(i);

    // Get the advertiser name (Arduino String type on many ESP32 builds)
    String name = adv.getName();
    if (name.length() == 0) continue;
    int rssi = adv.getRSSI();

    // Only consider devices with names starting with "ESP" (helps ignore phones, etc.)
    if (name.startsWith("ESP")) {
      updatePeerFromScan(name, rssi);

      // Check for manufacturer data (our message)
      if (adv.haveManufacturerData()) {
        // On this core, getManufacturerData() returns an Arduino String.
        String mdata = adv.getManufacturerData();
        // If payload uses our "MSG:" prefix, print the payload after it
        if (mdata.startsWith("MSG:")) {
          String payload = mdata.substring(4); // drop "MSG:"
          Serial.printf("Message from %s (RSSI=%d dBm): %s\n", name.c_str(), rssi, payload.c_str());
        } else {
          // Print raw manufacturer data if it doesn't match our prefix
          Serial.printf("Manufacturer data from %s (RSSI=%d dBm): %s\n", name.c_str(), rssi, mdata.c_str());
        }
      }
    }
  }

  // free scan results memory (good practice)
  pBLEScan->clearResults(); // note: clearResults() frees internal buffers used by start()

  // after processing scan results, check for peers that have left
  checkForLeftPeers();

  // small delay to avoid hogging CPU; scanning already took SCAN_SECONDS
  delay(50);
}

