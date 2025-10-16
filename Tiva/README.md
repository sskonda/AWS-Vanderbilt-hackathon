# Overview

Each ESP32 in the swarm runs identical code with a small configuration change (`OWN_NAME`) so each board identifies itself (`"ESP0"`, `"ESP1"`, `"ESP2"`, ...). Each ESP32:

* Advertises a short BLE advertisement (so other ESPs can find it).
* Performs BLE scans periodically.
* Tracks which peers are present (within RSSI threshold).
* On state changes (peer appears or disappears) sends a **single ASCII character** to the Tiva via UART2 to indicate the event.

The Tiva simply reads bytes from UART and uses those codes to **update an LCD visualization** of which submarines are connected or disconnected.

This branch is explicitly for **simulating the physical submarines** — BLE simulates the optical transceiver behavior (range/line of sight). When we later get real optical transceivers, the same message semantics can be reused for the physical optical communication layer.

---

# Files in this branch

* `general_drivers.h` — Tiva LCD graphics logic for submarine rendering and overlay effects.
* `vanderbilt_threads.c` / `final_threads.c` — G8RTOS-based threads that drive display updates, UART reception, and button handling.

(Other shared drivers, such as `multimod_uart.c` and `G8RTOS/`, are pulled from the main repo.)

---

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

# Threads.c additions: IMU processing, position tracking, and ROS telemetry

This section documents the new logic added in `final_threads.c` that fuses BMI160 IMU data with joystick input to estimate orientation and position, renders those results on the ST7789 LCD, and streams compact telemetry frames to the BeagleBone Black for ROS ingestion.

## Threads overview and responsibilities

The following threads and ISRs run under G8RTOS on the Tiva:

* `Get_Data()`

  * Reads BMI160 over I2C
  * Computes tilt and a normalized direction vector `(fx, fy, fz)`
  * Reads joystick Y from FIFO 1, computes thrust, updates `ship_state` position `px, py, pz`
  * Uses `sem_I2C` while accessing BMI160 and `sem_UART` for debug prints
* `Draw_Data()`

  * Renders 3D axes and a thick direction vector on the ST7789 LCD
  * Uses `sem_SPI` to guard LCD SPI access
  * Refreshes the direction vector periodically (about 5 Hz based on sleeps)
* `Draw_Position()`

  * Renders `px, py, pz` as text boxes on the LCD
  * Uses `sem_SPI` to guard LCD SPI access
* `Draw_Subs()`

  * Draws two submarine glyphs and overlays a red X when a sub is lost
  * Uses `sem_SPI`
  * Reacts to `current_state_sub_1` and `current_state_sub_2`
* `Read_ESP32()`

  * Consumes single ASCII bytes from the ESP32 BLE bridge
  * Updates submarine states
  * Forwards concise proximity events to the BeagleBone over UART4 as ASCII frames
  * Uses `sem_UART4` for UART4 output and `sem_UART` for console prints
* `Send_PO_Data()`

  * Periodically sends Position and Orientation frames to the BeagleBone over UART4
  * Uses `sem_UART4`
* `BeagleBone_Do()`

  * Consumes 8 byte records placed into FIFO 0 by the UART4 ISR
  * Parses and prints the record and can trigger state changes or processing
  * Uses `sem_UART` for console prints
* `Get_Joystick()` (periodic at 10 Hz)

  * Samples joystick via `JOYSTICK_GetXY()`
  * Writes the raw 12 bit packed XY sample to FIFO 1
* `Read_Buttons()` (aperiodic)

  * Debounces and reports button presses
  * Uses `sem_GPIOE` to synchronize with the GPIOE ISR
* `Idle_Thread()`

  * Background no op
* `UART4_Handler()` (aperiodic ISR)

  * Reads all pending bytes from UART4
  * Packs first 8 bytes into two 32 bit words and writes them to FIFO 0
* `GPIOE_Handler()` (aperiodic ISR)

  * Signals `sem_GPIOE` for `Read_Buttons()`

## IMU acquisition and tilt computation

BMI160 acceleration is read over I2C in `Get_Data()`:

* I2C access is protected by `sem_I2C`.
* Raw accelerations are normalized by gravity and converted into two tilt angles.
* Helper function:

  ```c
  void computeTilt(int16_t ax, int16_t ay, int16_t az, Tilt_t *t);
  ```

  This computes

  * `t->forward = atan2f(y, sqrtf(x*x + z*z))` normalized to about [-1, 1]
  * `t->side = atan2f(x, sqrtf(y*y + z*z))` normalized to about [-1, 1]

From these tilts, the code maintains a unit direction vector `(fx, fy, fz)` in `ship_state`. That vector is used by rendering and by the telemetry path.

## Position update with joystick thrust

* `Get_Joystick()` runs as a periodic event at 10 Hz and writes the current joystick reading to FIFO 1.
* `Get_Data()` reads FIFO 1, extracts the Y channel, maps it to [-1, 1], applies a deadband, and computes thrust magnitude.
* `updateShipPosition(&ship_state, tilt_data)` advances `px, py, pz` using the current thrust and direction vector.

## LCD visualization under SPI semaphore

All LCD drawing uses the ST7789 driver under `sem_SPI`:

* `Draw_Data()` shows a 3D axes widget plus a thick line representing the unit direction vector. It erases the previous vector and draws the new one. The sequence uses two short sleeps to manage refresh cadence and reduce flicker.
* `Draw_Position()` prints `X`, `Y`, and `Z` in three text boxes and updates them once per second.
* `Draw_Subs()` renders two submarine glyphs and overlays a red X when a peer is lost. The drawing of each pixel is wrapped by `G8RTOS_WaitSemaphore(&sem_SPI)` and `G8RTOS_SignalSemaphore(&sem_SPI)` blocks.

## Proximity event relay to BeagleBone over UART4

`Read_ESP32()` converts incoming single byte proximity events from the ESP32 into short ASCII frames on UART4 for the BeagleBone. The exact mapping depends on which submarine the firmware is compiled for using one of the compile time macros `SUB_0`, `SUB_1`, or `SUB_2`.

Frames:

* `E0F\n` means ESP0 found
* `E1F\n` means ESP1 found
* `E2F\n` means ESP2 found
* `E0L\n` means ESP0 lost
* `E1L\n` means ESP1 lost
* `E2L\n` means ESP2 lost

Examples of mapping inside `Read_ESP32()`:

* If compiled with `SUB_0`

  * Incoming `'1'` sets Sub 1 connected and sends `E1F\n`
  * Incoming `'2'` sets Sub 2 connected and sends `E2F\n`
  * Incoming `'B'` sets Sub 1 lost and sends `E1L\n`
  * Incoming `'C'` sets Sub 2 lost and sends `E2L\n`

* If compiled with `SUB_1`

  * Incoming `'0'` sets Sub 1 connected and sends `E0F\n`
  * Incoming `'2'` sets Sub 2 connected and sends `E2F\n`
  * Incoming `'A'` sets Sub 1 lost and sends `E0L\n`
  * Incoming `'C'` sets Sub 2 lost and sends `E2L\n`

* If compiled with `SUB_2`

  * Incoming `'0'` sets Sub 1 connected and sends `E0F\n`
  * Incoming `'1'` sets Sub 2 connected and sends `E1F\n`
  * Incoming `'A'` sets Sub 1 lost and sends `E0L\n`
  * Incoming `'B'` sets Sub 2 lost and sends `E1L\n`

All UART4 writes in this thread are protected by `sem_UART4`.

## Pose telemetry frames for ROS bridge

`Send_PO_Data()` sends compact ASCII frames to the BeagleBone over UART4, which are then parsed and published to ROS:

* Position frame

  ```
  P,<px>,<py>,<pz>\n
  ```

  Example

  ```
  P,12,5,3
  ```

* Orientation frame

  ```
  O,<fx>,<fy>,<fz>\n
  ```

  Example

  ```
  O,0.23,-0.45,0.81
  ```

The BeagleBone parses these lines and maps them to ROS topics such as `/sub/position` and `/sub/orientation`.

## FIFO usage

Two FIFOs are used for decoupled, ISR safe data transport:

* FIFO 0

  * Producer: `UART4_Handler()` ISR packs up to 8 received bytes into two 32 bit words and writes both words to FIFO 0
  * Consumer: `BeagleBone_Do()` reads both words, reconstructs the 8 byte record, and handles it

* FIFO 1

  * Producer: `Get_Joystick()` periodic thread writes joystick reading
  * Consumer: `Get_Data()` reads the most recent joystick value when updating thrust

## Semaphores used

* `sem_SPI` protects ST7789 LCD SPI access for `Draw_Data`, `Draw_Position`, `Draw_Subs`
* `sem_I2C` protects BMI160 I2C access in `Get_Data`
* `sem_UART` protects console UART prints in multiple threads
* `sem_UART4` serializes telemetry writes to BeagleBone in `Read_ESP32` and `Send_PO_Data`
* `sem_GPIOE` synchronizes the button ISR and the `Read_Buttons` thread

## Aperiodic and periodic events

* Aperiodic events

  * `UART4_Handler()` processes inbound UART4 bytes and fills FIFO 0
  * `GPIOE_Handler()` signals button events
* Periodic events

  * `Get_Joystick()` runs at 10 Hz to provide consistent operator input sampling

## Display cadence and timing notes

* `Draw_Data()` alternates vector erase and redraw with two 100 ms sleeps, for an effective update around 5 Hz
* `Draw_Position()` updates once per second
* `Draw_Subs()` updates immediately when state variables change, then sleeps briefly to yield

These timings balance visual responsiveness with CPU and SPI bandwidth while leaving headroom for UART and ISR work.

---