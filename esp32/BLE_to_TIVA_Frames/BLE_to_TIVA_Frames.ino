#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

// ---------- CONFIG ----------
const char* OWN_NAME = "ESP0";   // Change for each board
const int RSSI_THRESHOLD = -60;
const unsigned long LOST_TIMEOUT_MS = 3000;
// ----------------------------

// ---------- UART CONFIG ----------
const int UART_TX = 17;  // ESP32 TX → Tiva RX
const int UART_RX = 16;  // ESP32 RX (not used)
const int UART_BAUD = 115200;
// ----------------------------

BLEScan* pBLEScan;

// Persistent connection state
bool prevESP0 = false;
bool prevESP1 = false;
bool prevESP2 = false;

void setupAdvertising() {
  BLEDevice::init(OWN_NAME);
  BLEServer* pServer = BLEDevice::createServer();
  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();

  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);

  BLEAdvertisementData advData;
  advData.setName(String(OWN_NAME));
  advData.setManufacturerData("MSG:Hello");
  pAdvertising->setAdvertisementData(advData);
  pAdvertising->start();
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(UART_BAUD, SERIAL_8N1, UART_RX, UART_TX);
  delay(500);

  Serial.printf("Starting BLE multi-peer scanner (%s)\n", OWN_NAME);
  Serial2.println("Ready");

  setupAdvertising();

  pBLEScan = BLEDevice::getScan();
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(160);
  pBLEScan->setWindow(120);
}

void checkConnections() {
  bool currESP0 = false, currESP1 = false, currESP2 = false;

  BLEScanResults* found = pBLEScan->start(1, false);  // integer seconds only
  int count = found->getCount();

  for (int i = 0; i < count; ++i) {
    BLEAdvertisedDevice adv = found->getDevice(i);
    String name = adv.getName();
    if (name.length() == 0) continue;
    int rssi = adv.getRSSI();
    if (rssi < RSSI_THRESHOLD) continue;

    if (name == "ESP0") currESP0 = true;
    else if (name == "ESP1") currESP1 = true;
    else if (name == "ESP2") currESP2 = true;
  }

  // --- Connection events ---
  if (!prevESP0 && currESP0) { Serial.println("ESP0 connected"); Serial2.print('0'); }
  if (!prevESP1 && currESP1) { Serial.println("ESP1 connected"); Serial2.print('1'); }
  if (!prevESP2 && currESP2) { Serial.println("ESP2 connected"); Serial2.print('2'); }

  // --- Disconnection events ---
  if (prevESP0 && !currESP0) { Serial.println("ESP0 lost"); Serial2.print('A'); }
  if (prevESP1 && !currESP1) { Serial.println("ESP1 lost"); Serial2.print('B'); }
  if (prevESP2 && !currESP2) { Serial.println("ESP2 lost"); Serial2.print('C'); }

  prevESP0 = currESP0;
  prevESP1 = currESP1;
  prevESP2 = currESP2;

  pBLEScan->stop();          // Explicitly stop scanning
  pBLEScan->clearResults();  // Free memory
}

void loop() {
  checkConnections();
  delay(400);  // ~0.4s pause between scans
}
