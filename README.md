<!-- Powered by AWS (separate line) -->
<div align="center"style="text-align:center;margin-top:12px;">
  <img src="docs/assets/aws/PoweredByAWS.png" alt="Powered by AWS" style="height:36px;object-fit:contain;">
</div>
<!-- AWS Service Logos -->
<div align="center" style="display:flex;align-items:center;justify-content:center;flex-wrap:wrap;gap:16px;">
  <img src="docs/assets/aws/DynamoDB.png" alt="DynamoDB" style="height:60px;object-fit:contain;">
  <img src="docs/assets/aws/App Runner.png" alt="App Runner" style="height:60px;object-fit:contain;">
  <img src="docs/assets/aws/Elastic Container Registry.png" alt="Elastic Container Registry" style="height:60px;object-fit:contain;">
  <img src="docs/assets/aws/IoT Core.png" alt="IoT Core" style="height:60px;object-fit:contain;">
  <img src="docs/assets/aws/IoT Greengrass.png" alt="IoT Greengrass" style="height:60px;object-fit:contain;">
  <img src="docs/assets/aws/S3 on Outposts.png" alt="S3 on Outposts" style="height:60px;object-fit:contain;">
</div>



 # Operation Duloc

## Branches
### BLE → Tiva Simulation (Submarine Swarm Branch)**
This branch implements a BLE-based *simulation* of the optical transceivers used on the submarine swarm.
Because we do not have the physical optical transceivers yet, **ESP32 boards act as stand-ins** that advertise a short BLE packet and scan for each other. When an ESP32 detects (or loses) another ESP32 nearby, it sends a single ASCII character over UART to the Tiva to indicate that connection state.

This README explains how the code works, what characters are sent, hardware wiring, runtime behavior, and how the Tiva-side visualization (running G8RTOS) displays submarine connection status on the LCD screen.

---

# Table of contents

* Overview
* High-level behaviour
* Important design decisions & limitations
* Files in this branch
* How to run (ESP32)
* How the Tiva receives and visualizes data
* UART / wiring
* Expected UART messages (protocol)
* Troubleshooting
* Notes about G8RTOS
* RTOS architecture overview
* Next steps / recommended improvements
* License & acknowledgements

---

# Overview

Each ESP32 in the swarm runs identical code with a small configuration change (`OWN_NAME`) so each board identifies itself (`"ESP0"`, `"ESP1"`, `"ESP2"`, ...). Each ESP32:

* Advertises a short BLE advertisement (so other ESPs can find it).
* Performs BLE scans periodically.
* Tracks which peers are present (within RSSI threshold).
* On state changes (peer appears or disappears) sends a **single ASCII character** to the Tiva via UART2 to indicate the event.

The Tiva simply reads bytes from UART and uses those codes to **update an LCD visualization** of which submarines are connected or disconnected.

This branch is explicitly for **simulating the physical submarines** — BLE simulates the optical transceiver behavior (range/line of sight). When we later get real optical transceivers, the same message semantics can be reused for the physical optical communication layer.

---

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
* `general_drivers.h` — Tiva LCD graphics logic for submarine rendering and overlay effects.
* `vanderbilt_threads.c` / `final_threads.c` — G8RTOS-based threads that drive display updates, UART reception, and button handling.
* `README.md` — this file.

(Other shared drivers, such as `multimod_uart.c` and `G8RTOS/`, are pulled from the main repo.)

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

# How the Tiva receives and visualizes data

The Tiva-side firmware, running under **G8RTOS**, receives single-character UART frames from one ESP32 and updates a dynamic LCD visualization.

## UART Reception (`Read_ESP32()` thread)

* Continuously polls for UART characters using `UART_ESP32_ReadChar()`.
* Maps received ASCII bytes to submarine state variables:

  ```c
  if (c == '1') current_state_sub_1 = 0; // Sub 1 connected
  else if (c == '2') current_state_sub_2 = 0; // Sub 2 connected
  else if (c == 'B') current_state_sub_1 = 1; // Sub 1 lost
  else if (c == 'C') current_state_sub_2 = 1; // Sub 2 lost
  ```
* These state variables are shared with the drawing thread (`Draw_Subs()`).

## Display Rendering (`Draw_Subs()` thread)

* Continuously checks submarine connection states (`current_state_sub_x` vs. `last_state_sub_x`).
* Uses two pixel generators defined in `general_drivers.h`:

  * `get_submarine_pixel()` — draws normal submarine.
  * `get_submarine_pixel_with_x()` — draws the same submarine with a **red “X” overlay** to indicate loss of link.
* Each submarine is rendered into its section of the 160×120 **ST7789 LCD**.
* The drawing operations are protected by the **SPI semaphore (`sem_SPI`)** to ensure thread-safe drawing under G8RTOS.

When a submarine goes out of range (BLE disconnect), the LCD immediately updates to show that submarine crossed out with a red X. When it reconnects, the red X disappears.

## Depth and Buttons

* `Draw_Depth()` simulates a live variable (depth) and updates a display box with current depth reading every second.
* `Read_Buttons()` reads from the Multimod button interface and prints button states over UART for debugging. (This can later be extended to manual control.)

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

# Expected UART messages (protocol)

Each ASCII character represents a connect or disconnect event for one of the simulated submarines.

| Message | Meaning        |
| ------- | -------------- |
| `'0'`   | ESP0 connected |
| `'1'`   | ESP1 connected |
| `'2'`   | ESP2 connected |
| `'A'`   | ESP0 lost      |
| `'B'`   | ESP1 lost      |
| `'C'`   | ESP2 lost      |

Example UART sequence:

```
0
2
A
```

→ ESP0 and ESP2 connected; later ESP0 went out of range.
The Tiva uses these codes to show or hide the red “X” overlay per submarine.

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

# Notes about G8RTOS

This project (Tiva firmware) runs on **G8RTOS**, a fully custom RTOS developed from the ground up.
It implements:

* A pre-emptive scheduler with priority-based thread control.
* Lightweight semaphores for peripheral protection (SPI, GPIO, UART).
* Sleep/timing primitives for task scheduling.
* Interrupt handling and FIFO-based inter-thread communication.

The Tiva’s main threads (UART read, drawing, depth update, buttons) each execute under the G8RTOS scheduler, allowing concurrent and deterministic updates on the LCD while handling asynchronous UART events from the ESP32.

Further documentation on the scheduler and threading model will be added later as part of the G8RTOS internals section.

---

# RTOS Architecture Overview (G8RTOS Core)

**G8RTOS** is a lightweight real-time operating system designed for deterministic embedded concurrency.
It provides thread scheduling, inter-thread communication, semaphores, and critical-section control — all running directly on the TM4C123 MCU.

## Scheduler

* Threads are represented as `tcb_t` structs arranged in a circular linked list.
* `G8RTOS_Scheduler()` executes at each `SysTick` interrupt and selects the next runnable thread based on priority and sleep/block flags.
* Context switching is handled by **`PendSV_Handler`** in assembly.

## Semaphores

Semaphores protect shared resources (like the ST7789 SPI bus):

```c
G8RTOS_WaitSemaphore(&sem_SPI);
ST7789_DrawPixel(...);
G8RTOS_SignalSemaphore(&sem_SPI);
```

Threads that call `WaitSemaphore` on a locked resource are blocked until another thread signals it.

## Sleep and Timing

Threads yield with:

```c
sleep(ms);
```

The `SysTick_Handler` decrements sleep counters and wakes threads automatically.

## IPC (FIFO)

Used for ISR-to-thread communication:

```c
G8RTOS_WriteFIFO(0, data);
uint32_t val = G8RTOS_ReadFIFO(0);
```

Ideal for passing UART or sensor data.

## Critical Sections

Implemented in assembly:

* `StartCriticalSection()` disables interrupts (saves PRIMASK).
* `EndCriticalSection()` restores PRIMASK.

---

## G8RTOS Context Switch & Timing Diagram

```text
┌──────────────────────────────────────────────────────────────────────────────┐
│                             SysTick Interrupt                                │
│  Every 1 ms → increments system time and wakes sleeping threads              │
└──────────────────────────────────────────────────────────────────────────────┘
              │
              ▼
┌──────────────────────────────────────────────────────────────────────────────┐
│                          G8RTOS_Scheduler()                                 │
│  - Chooses next ready thread (by priority)                                   │
│  - Skips sleeping or blocked threads                                         │
└──────────────────────────────────────────────────────────────────────────────┘
              │
              ▼
┌──────────────────────────────────────────────────────────────────────────────┐
│                           PendSV_Handler (ASM)                              │
│  - Saves current thread registers (R4–R11)                                   │
│  - Loads next thread’s stack pointer                                         │
│  - Restores R4–R11 and returns to thread context                             │
└──────────────────────────────────────────────────────────────────────────────┘
              │
              ▼
┌──────────────────────────────────────────────────────────────────────────────┐
│                           Active Thread Runs                                │
│  e.g., Draw_Subs(), Read_ESP32(), or Idle_Thread()                           │
│  Threads run cooperatively until sleep(), block, or interrupt occurs         │
└──────────────────────────────────────────────────────────────────────────────┘
```

This cycle repeats continuously, ensuring smooth LCD updates and responsive UART event handling in real time.

---

# Next steps / recommended improvements

* Add a **heartbeat message** from each ESP when at least one connection is active, so the Tiva can detect total silence or timeouts.
* Add timestamps or sequence numbers to the UART messages for debug traceability.
* Expand the LCD UI to three submarines (currently two visible).
* Replace BLE simulation with the **real optical transceiver hardware** once available, reusing the same UART framing.
* Integrate depth, heading, and other telemetry into the same RTOS display pipeline.
* Optionally, log connection history to SD card or host PC for testing swarm formation logic.

---

# License & acknowledgements

This project was developed as part of the **Vanderbilt Hackathon submarine swarm project**.
Use and modification are permitted for team development and research purposes.
When reusing or publishing derivative work, please preserve authorship and attribution.

**Technologies used:**

* Arduino ESP32 BLE library
* TI TivaWare peripheral drivers
* Custom **G8RTOS** kernel (scheduling, semaphores, threads, UART/SPI drivers)

---

> “The BLE packets are just placeholders — when the optical transceivers arrive, the code will already know how to see.”

---
