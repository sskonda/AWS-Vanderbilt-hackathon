# README — BLE → Tiva Simulation (Submarine Swarm Branch)

**Branch purpose:** This branch implements a BLE-based *simulation* of the optical transceivers used on the submarine swarm.
Because we do not have the physical optical transceivers yet, **ESP32 boards act as stand-ins** that advertise a short BLE packet and scan for each other. When an ESP32 detects (or loses) another ESP32 nearby, it sends a single ASCII character over UART to the Tiva to indicate that connection state.

This README explains how the code works, what characters are sent, hardware wiring, runtime behavior, and a few notes about the environment (G8RTOS, build/runtime tips).

---

# Table of contents

* Overview
* High-level behaviour
* Important design decisions & limitations
* Files in this branch
* How to run (ESP32)
* How the Tiva receives data
* UART / wiring
* Expected UART messages (protocol)
* Troubleshooting
* Notes about G8RTOS
* License & acknowledgements

---

# Overview

Each ESP32 in the swarm runs identical code with a small configuration change (`OWN_NAME`) so each board identifies itself (`"ESP0"`, `"ESP1"`, `"ESP2"`, ...). Each ESP32:

* Advertises a short BLE advertisement (so other ESPs can find it).
* Performs BLE scans periodically.
* Tracks which peers are present (within RSSI threshold).
* On state changes (peer appears or disappears) sends a **single ASCII character** to the Tiva via UART2 to indicate the event.

The Tiva simply reads bytes from UART and can act on them (display, store, etc.). That Tiva-side code was designed to be modular and receive the single-character frames.

This branch is explicitly for **simulating the physical submarines** — BLE simulates the optical transceiver behavior (range/LOS). When we later get real optical transceivers, the same message semantics can be used (appearance/disappearance events).

---

# High-level behaviour

* Each ESP runs BLE advertising and periodic scanning **simultaneously**.
* The scanner is run in short bursts (1 second scan call) with a loop delay of **0.4 s** between checks (so we check every ~1.4 seconds effectively). This approach avoids a BLE controller deadlock while allowing advertising and scanning to both operate.
* Each scan run gathers all advertisements, so multiple peers can be detected in the same scan (i.e., multiple simultaneous detections).
* For each peer (ESP0, ESP1, ESP2):

  * When the peer **appears** (was previously absent and now present) the ESP sends the *connect* character via UART (`'0'`, `'1'`, `'2'` respectively).
  * When the peer **disappears** (was previously present and now missing) the ESP sends the *disconnect* character via UART (`'A'`, `'B'`, `'C'` respectively).

---

# Important design decisions & limitations

* BLE acts as a **simulation** of optical transceivers. BLE has different physical properties but suffices to test detection, state changes, and UART integration.
* The ESP32 BLE API expects integer seconds for `start()`; passing fractional seconds like `0.4` to `start()` can hang. We therefore use `start(1, false)` and control the loop frequency with `delay(400)`.
* The scan interval/window are tuned to improve likelihood of catching several advertisers (e.g., `setInterval(160)` / `setWindow(120)`), but BLE is best-effort — environmental conditions and advertising intervals affect detection.
* This system uses simple ASCII one-byte frames for event signaling (low overhead, easy to parse). If you later need more complex messages, extend the protocol with framing, checksums, or length prefixes.

---

# Files in this branch

* `BLE_to_TIVA_Frames.ino` (ESP32 sketch)

  * The working ESP32 BLE + UART code (set `OWN_NAME` per board).
* `README.md` (this file)
* (Other Tiva files live in the multimod / G8RTOS repository — Tiva side expects single-byte inputs and should be listening on the configured UART.)

---

# How to run (ESP32)

1. Install the Arduino ESP32 core ([https://github.com/espressif/arduino-esp32](https://github.com/espressif/arduino-esp32)) and ensure the `ESP32 Dev Module` board is available in the Arduino IDE.
2. Open `BLE_to_TIVA_Frames.ino`.
3. For each physical ESP32 board:

   * Set `OWN_NAME` near the top of the file to the board's identity (`"ESP0"`, `"ESP1"`, `"ESP2"`, …).
   * Upload the sketch to the board.
4. Connect the ESP32 TX pin (default in the code: GPIO 17) to the Tiva RX pin of whichever UART you chose on the Tiva. Ground must be common between ESP32 and Tiva.

---

# How the code works (ESP32 sketch)

Important snippets & behavior:

* **Configuration**

  ```cpp
  const char* OWN_NAME = "ESP1";
  const int RSSI_THRESHOLD = -60;  // dBm threshold for "in range"
  const int UART_TX = 17;          // ESP32 pin to send UART out
  const int UART_BAUD = 115200;    // UART rate for Tiva
  ```

* **Advertising**
  The ESP advertises a short piece of manufacturer data so other ESPs see it:

  ```cpp
  BLEAdvertisementData advData;
  advData.setName(String(OWN_NAME));
  advData.setManufacturerData("MSG:Hello");
  pAdvertising->setAdvertisementData(advData);
  ```

* **Scanning & detection**
  Scans using:

  ```cpp
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(160);
  pBLEScan->setWindow(120);
  BLEScanResults* found = pBLEScan->start(1, false);
  ```

  Then it iterates all found devices and sets flags `currESP0`, `currESP1`, `currESP2` based on `adv.getName()` and RSSI.

* **Event detection & UART**
  On state changes:

  ```cpp
  if (!prevESP0 && currESP0) Serial2.print('0');  // ESP0 connected
  if (prevESP0 && !currESP0) Serial2.print('A');  // ESP0 lost
  // likewise for ESP1/ESP2 with '1'/'B' and '2'/'C'
  ```

  The code sends single ASCII bytes to the Tiva; the Tiva should handle them.

* **Scan loop timing**
  After processing the `start(1, false)` call and stopping/clearing results, the loop waits `delay(400)` (0.4 s) before scanning again. This balances BLE controller demands with the requested 0.4 s periodic check.

---

# UART / Wiring

* **ESP32 → Tiva**:

  * ESP32 TX (GPIO 17 by default in the sketch) → Tiva RX (your chosen Tiva UART RX pin)
  * GND (ESP32) ↔ GND (Tiva)
* **Serial settings**:

  * Baud: **115200**
  * Data: 8-bit
  * Parity: None
  * Stop bits: 1

Make sure the Tiva-side UART is configured for the same serial parameters and that the Tiva reads single characters as they arrive. The Tiva can then map `'0'`/`'A'` etc. to UI or state changes.

---

# Expected UART messages (protocol)

These are single ASCII characters sent from each ESP whenever that ESP detects connect/disconnect events for a peer:

* Connect messages:

  * `0` — ESP0 became visible (connected)
  * `1` — ESP1 became visible
  * `2` — ESP2 became visible
* Disconnect messages:

  * `A` — ESP0 lost
  * `B` — ESP1 lost
  * `C` — ESP2 lost

Example:

* If ESP1 sees both ESP0 and ESP2 appear, it may send:

  ```
  0
  2
  ```

  (each as a single character; depending on implementation the Tiva may get them back-to-back)

* If ESP1 later loses ESP0:

  ```
  A
  ```

---

# Troubleshooting

* If the ESP32 seems to “hang” on boot with weird output: verify the sketch uses `pBLEScan->start(1, false)` (integer seconds). Passing in fractional seconds into `start()` can lock the BLE controller.
* If multiple devices are not detected simultaneously:

  * Ensure each ESP is advertising (advertising is started once in `setup()` and not restarted frequently).
  * The environment affects BLE detections (antenna orientation, interference). You may increase `setWindow()` or decrease `setInterval()` to widen scan coverage, but watch power/stack constraints.
* UART issues:

  * Ensure common ground between boards.
  * Confirm correct TX→RX wiring (TX of ESP to RX of Tiva).
  * Match baud/format on both ends.
* If you need faster refresh than the scan+delay method allows, consider:

  * Using one ESP as a continuous scanner (but that prevents it from advertising at the same time) or
  * Using continuous scanning with careful coordination — both approaches have tradeoffs.

---

# Notes about G8RTOS

This project (Tiva firmware) runs on top of **G8RTOS**, a custom RTOS built from the ground up by the team. G8RTOS contains:

* A scheduler
* Thread primitives
* Interrupt handling
* Timing/sleep primitives

The Tiva-side code that receives UART characters is written to be modular so it can be called by threads created under G8RTOS (for example, a UART listener thread that pushes state updates to a display or to logging threads). The RTOS details and internals will be documented separately — you mentioned you will provide more explanation later, and we can add detailed diagrams and API docs in a follow-up.

---

# Next steps / recommended improvements

* Add simple framing or timestamps to UART messages if ordering/deduplication becomes important.
* Add a small debounce or rate-limiter on the ESP side if spurious connect/disconnect events happen frequently.
* Add logic on the Tiva side to coalesce or display the state of the full swarm, not just raw bytes.
* When you have optical transceivers, switch advertising/detection to the optical link and reuse UART framing semantics.

---

# License & Acknowledgements

This project is a team project for the Vanderbilt hackathon. Use and modification permitted for the team and collaborators. When reusing code, please preserve attribution in the source files. We used the Arduino ESP32 BLE libraries and TI TivaWare (for the Tiva code).

---
