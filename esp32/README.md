# High-level behaviour

* Each ESP runs BLE advertising and periodic scanning **simultaneously**.
* The scanner is run in short bursts (1 second scan call) with a loop delay of **0.4 s** between checks (so we check every ~1.4 seconds effectively). This avoids BLE controller deadlocks while allowing continuous advertising.
* Each scan run gathers all advertisements, so multiple peers can be detected in the same scan (i.e., multiple simultaneous detections).
* For each peer (ESP0, ESP1, ESP2):

  * When the peer **appears** (was previously absent and now present) the ESP sends the *connect* character via UART (`'0'`, `'1'`, `'2'` respectively).
  * When the peer **disappears** (was previously present and now missing) the ESP sends the *disconnect* character via UART (`'A'`, `'B'`, `'C'` respectively).

This makes the ESP32s act like **optical link detectors** — reporting to the Tiva when another submarine moves in or out of range.

---

# Important design decisions & limitations

* BLE acts as a **simulation** of optical transceivers. BLE’s propagation characteristics differ from optical systems, but it is sufficient to emulate detection, state changes, and UART message flow.
* The ESP32 BLE API expects integer seconds for `start()`; passing fractional seconds (like `0.4`) can hang the BLE stack. We therefore use `start(1, false)` and add `delay(400)` between scans for the target refresh rate.
* The scan interval/window (`setInterval(160)` / `setWindow(120)`) are tuned to increase multi-peer detection probability, but BLE discovery is still best-effort and environment-dependent.
* This system uses **one-byte ASCII messages** for signaling. They are lightweight, fast, and easy for the Tiva to parse. Future iterations can extend this with structured frames or timestamps if needed.

---

# Files in this branch

* `BLE_to_TIVA_Frames.ino` — ESP32 BLE scanning and UART communication firmware.

---

# How to run (ESP32)

1. Install the Arduino ESP32 core ([https://github.com/espressif/arduino-esp32](https://github.com/espressif/arduino-esp32)).
2. Open `BLE_to_TIVA_Frames.ino` in the Arduino IDE.
3. For each ESP32 board:

   * Set `OWN_NAME` to `"ESP0"`, `"ESP1"`, or `"ESP2"`.
   * Upload the code.
4. Connect ESP32 TX (GPIO 17 by default) to the Tiva UART RX.
5. Ensure all grounds (ESP32 + Tiva + LCD) share a common ground.

---

# UART / Wiring

* **ESP32 → Tiva:**

  * ESP32 TX (GPIO 17) → Tiva RX (your chosen UART RX pin)
  * Common GND between ESP32 and Tiva

* **Serial configuration:**

  * Baud: 115200
  * Data: 8 bits
  * Parity: None
  * Stop bits: 1

The UART is unidirectional (ESP → Tiva). The Tiva listens continuously for the event bytes sent by the ESP.

---

# Troubleshooting

* **ESP32 hangs on startup:**
  Check that the scan call uses `pBLEScan->start(1, false)` (integer seconds).
  Fractional values like `0.4` will hang the BLE stack.

* **Only one ESP detected at a time:**
  Confirm that each ESP is continuously advertising (don’t restart advertising each loop).
  Ensure RSSI threshold isn’t too strict (`-60` is typical).

* **LCD flickers or freezes:**
  Make sure all drawing loops are wrapped in semaphore protection:

  ```c
  G8RTOS_WaitSemaphore(&sem_SPI);
  ... ST7789_DrawPixel() ...
  G8RTOS_SignalSemaphore(&sem_SPI);
  ```

* **UART garbage:**

  * Verify 115200 baud on both sides.
  * Tie ESP32 GND to Tiva GND.
  * Use short, shielded wires to avoid noise.

* **Visuals out of sync:**
  Ensure `Read_ESP32()` runs with higher priority than `Draw_Subs()` so UART updates are processed promptly.

---