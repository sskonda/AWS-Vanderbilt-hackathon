<table>
  <tr>
    <!-- Team logo (left) -->
    <td>
      <img src="docs/assets/team_branding/TeamLogoNoBackground.png" alt="Team Logo" height="100">
    </td>
    <!-- AWS logos (right) -->
    <td>
      <img src="docs/assets/aws/DynamoDB.png" alt="DynamoDB" height="60">
      <img src="docs/assets/aws/App Runner.png" alt="App Runner" height="60">
      <img src="docs/assets/aws/Elastic Container Registry.png" alt="Elastic Container Registry" height="60">
      <img src="docs/assets/aws/IoT Core.png" alt="IoT Core" height="60">
      <img src="docs/assets/aws/IoT Greengrass.png" alt="IoT Greengrass" height="60">
      <img src="docs/assets/aws/S3 on Outposts.png" alt="S3 on Outposts" height="60">
      <br>
      <img src="docs/assets/aws/PoweredByAWS.png" alt="Powered by AWS" height="40">
    </td>
  </tr>
</table>

# Operation Duloc
Built by University of Florida team "What Are You Doing In My Swamp?"

## Project Overview

### [Web App UI Github + README](https://github.com/emma-coronado/Swamp-Portal?tab=readme-ov-file#quick-start)


### Built with:

### Technologies used:
* Arduino ESP32 BLE library
* TI TivaWare peripheral drivers
* Custom **G8RTOS** kernel (scheduling, semaphores, threads, UART/SPI drivers)
* ROS2 Humble for UUV Control and interfacing with MQTT
* AWS Cloud Services not limited to IoT Core, EC2, etc.
* Spring and AngularJS for frontend

### Architecture

#### AWS DynamoDB
* Scalable and relatively easy-to-use database system
* Integrates easily with other AWS services
* Free-form NoSQL format for on-the-fly data format changes

#### AWS IoT Core
* Come standard with MQTT publish/subscribe model
* Allow for a centralized staging area for devices

#### AWS IoT Rules
* Straightforward data redirection framework
* SQL-based digestion engine for data parsing before redirection
* Forwards to DynamoDB instance & to Webserver via HTTP packet for display
* Solid error fallback to Cloudwatch Event Logs for diagnostics

#### AWS EC2
* Quick-deploy for AWS server instances
* Houses a Mission Control Core logic that publishes downstream to ROS UUVs
* Subscribes to ROS UUV-published snapshot

#### AWS GreenGrass
* Used for component deployment and device partitioning
* Interfaces well with IPC and MQTT5

#### AWs Elastic Container Registtry
* Deployment of a 32-bit ROS Image in Docker (for ARM)
* Integrates seamlessly with EC2 and Greengrass for smooth deployments
* Allows for a scalable featureset to our solution to add more UUVs to (a) swarm
* Also utilized for automatic frontend deployment and build lifetime

#### AWS App Runner
* Fully managed service that deploys app containers on update
* Automatic scaling for larger and larger webserver feature additions
* Pulls containers from Elastic-Container Registry to build for deployment
* Uses Githuub action to pull from main repo to source docker containers  

## Getting Started

#### Web App UI
See [Swamp Portal README](https://github.com/emma-coronado/Swamp-Portal?tab=readme-ov-file#quick-start)

### Installation Instructions

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

## ⚙️ Control Signals, Topics, and Code Deep-Dive

This section documents the ROS 2 interfaces and the inner workings of the `SubNode` shown below, including role/state rotation, plan exchange, event flow, and the JSON snapshots published for your IoT bridge.

---

### 🛰 ROS 2 Topic Map (by role namespace)

> Each node publishes/consumes *role-scoped* topics to keep a stable interface even as roles rotate among `team_ids`.

| Topic | Type | Pub | Sub | Purpose / Notes |
|---|---|---|---|---|
| `/<sub_id>/target_pose` | `geometry_msgs/PoseStamped` | Self | — | Per-tick target waypoint based on current role/state. Can double as pose if `publish_current_pose=true`. |
| `/<sub_id>/pose` | `geometry_msgs/PoseStamped` | Self | — | Optional mirror of `target_pose` as “current pose” for quick testing/sim. |
| `/plans/sub1` | `custom_msg/Plan` | Any | All | Role topic: the plan for whoever currently holds role **SUB1**. |
| `/plans/sub2` | `custom_msg/Plan` | Any | All | Role topic: plan for **SUB2**. |
| `/plans/mid`  | `custom_msg/Plan` | Any | All | Role topic: plan for **MID_TIER**. |
| `/echo/sub1`  | `custom_msg/Plan` | Any | All | “Previous plan” echo channel for SUB1 holder (diagnostics/redundancy). |
| `/echo/sub2`  | `custom_msg/Plan` | Any | All | Same for SUB2. |
| `/echo/mid`   | `custom_msg/Plan` | Any | All | Same for MID. |
| `/relay/sub1` | `custom_msg/Plan` | MID | All | MID publishes merged knowledge for SUB1 at **SURFACE_RELAY**. |
| `/relay/sub2` | `custom_msg/Plan` | MID | All | Same for SUB2. |
| `/relay/mid`  | `custom_msg/Plan` | MID | All | Same for MID. |
| `/relay/snapshot_json` | `std_msgs/String` | MID | Bridge | JSON snapshot of **plans by role** and **by identity** keyed on numeric `Identity`. |
| `/relay/events_json` | `std_msgs/String` | MID | Bridge | JSON uplink of most recent events per role and “new since surface” counters. |
| `/events/sub1` | `custom_msg/Event` | SUB1 | All | Event stream produced by current SUB1 holder at **SURFACE_INIT**. |
| `/events/sub2` | `custom_msg/Event` | SUB2 | All | Same for SUB2. |
| `/events/mid`  | `custom_msg/Event` | MID  | All | MID events (optional, used in counters/uplink). |
| `/expected/sub1/pose` | `geometry_msgs/PoseStamped` | All | — | Locally estimated SUB1 position at **now** via plan interpolation. |
| `/expected/sub2/pose` | `geometry_msgs/PoseStamped` | All | — | Same for SUB2. |
| `/expected/mid/pose`  | `geometry_msgs/PoseStamped` | All | — | Same for MID. |

**QoS**: Default (history depth 10). For field tests, set `reliable` on `/plans/*`, `/relay/*`, `/events/*`, keep `best_effort` on `/expected/*` if needed.


## ROS Behavior Logic: How the Node Thinks and Acts

This section explains the runtime behavior of `SubNode`—how roles rotate, how topics are used, what happens each slot, and how plans/events are merged and relayed. It is written to mirror the code structure so you can map each behavior to a function.

---

### 1) Node Lifecycle & Startup Sequence

**Boot flow**
1. `main()` creates a temporary `bootstrap` node to read `sub_id`.
2. `SubNode(sub_id)` is constructed:
   - Declares/reads parameters (`team_ids`, `leader_id`, `Identity`, `team_identity_numbers`, `state_dwell_s`, `start_epoch`, `publish_current_pose`).
   - Validates that `leader_id ∈ team_ids`, dedupes `team_ids`, aligns `team_identity_numbers`.
   - Computes `leader0_idx_` and **auto-assigns** `start_epoch_ = ceil(now)+2` if unset.
   - Creates all publishers & subscribers (role-scoped topics).
   - Starts a **0.25 s** wall timer → `tick()`.

**Initialization guarantees**
- Role topics `/plans/{sub1,sub2,mid}` are always present.
- Per-identity map is pre-seeded on first `SURFACE_INIT` via `ensureSeedPlans()`.
- If publishing pose for testing, `/<sub_id>/pose` mirrors the target for quick visualization.

---

### 2) Time, Rotation, and Roles

The **timekeeper** is `compute(t)`:
- **Cycle**: `T = 5 * dwell_s_`
- **Slot**: `slot5 = floor(((t - start_epoch_) mod T) / dwell_s_) ∈ {0..4}`
- **Role holders** for this cycle:
  - `id_mid  = team_ids_[(leader0_idx_ + cycles) % N]`
  - `id_s1   = team_ids_[(leader0_idx_ + cycles + 1) % N]`
  - `id_s2   = team_ids_[(leader0_idx_ + cycles + 2) % N]`
- Your **current role** is derived by comparing `sub_id_` with role holders.

**Why role topics are stable:**  
Even though role holders rotate, the **topic names stay fixed** (`/plans/sub1`, `/plans/sub2`, `/plans/mid`). This makes monitor/bridging logic simpler and avoids re-wiring subscriptions every rotation.

---

### 3) Per-Tick Behavior (`tick()`)

Executed every **0.25 s**:
- Recompute schedule `sch = compute(now)`.
- Detect slot changes and call `onSlotEnter(sch)` once per slot.
- Publish per-tick **target**:
  - MID → `midTarget(sch.mid_state)`
  - SUB1 → `sub1Target(sch.sub_state)`
  - SUB2 → `sub2Target(sch.sub_state)`
  - Topic: `/<sub_id>/target_pose`
  - Optional: also publish as `/<sub_id>/pose` when `publish_current_pose=true`
- Publish **expected poses** (interpolations from merged plans):
  - `/expected/sub1/pose`, `/expected/sub2/pose`, `/expected/mid/pose`
- Throttled log with role, slot, dwell, “new-since-surface” counters.

**Design note:**  
All node-to-node sync happens via topics; **no services** and **no shared memory** are used.

---

### 4) Slot Entry Behavior (`onSlotEnter(sch)`)

Triggered **once** when `slot5` or `id_mid` changes.

**If you are SUB1/SUB2:**
- At `SURFACE_INIT` (slot 0), publish a **local event** with your current target:
  - `publishMyEventIfNeeded(sch)` → `/events/sub1` or `/events/sub2`
  - `event_type` comes from `last_local_event_type_` (if set by your higher-level logic)

**If you are MID:**
- **Slot 0 – SURFACE_INIT**
  - `ensureSeedPlans()` → ensures role plans are non-empty
  - `resetAddedSinceSurface()` → zero “new points” counters
  - `uplinkEventsJSON(sch)` → summarize latest events + counters to `/relay/events_json`
- **Slot 1 – HANDOFF_1**
  - `midHandoffOnce("sub1", sch.id_s1)` → publish current `/plans/sub1` and `/plans/mid` **once**
  - Echo previous MID plan on `/echo/mid` (for diagnostics/backfill)
- **Slot 2 – HANDOFF_2**
  - Same as slot 1, but for `"sub2"`
- **Slot 3 – SURFACE_RELAY**
  - `surfaceRelayAggregate()`:
    - Publish merged role knowledge on `/relay/{sub1,sub2,mid}`
    - Produce **by-identity** JSON snapshot to `/relay/snapshot_json`
    - Reset “new-since-surface” counters
- **Slot 4 – RETURN_END**
  - Navigation only; no messaging beyond status logs

**One-shot protection:**  
Handoff publishers are gated by `published_this_slot_` to prevent duplicate bursts.

## Reporting, Snapshots, and IoT Bridge Integration

The ROS 2 side of the system continuously builds a **structured data layer** describing the swarm’s real-time state.  
At key synchronization points, this data is published as **JSON snapshots** for the IoT bridge to forward to AWS IoT Core via MQTT5.

This ensures a robust **ROS → IoT → Cloud feedback loop**, where local multi-robot coordination is preserved even when the cloud connection is intermittent.

---

### Snapshot Overview

Each cycle (5 slots, total duration = `5 * dwell_s` seconds) has **two primary reporting phases**:

| Phase | Slot | Trigger | Publisher Role | Topic | Description |
|-------|------|----------|----------------|--------|--------------|
| **Events Report** | 0 – `SURFACE_INIT` | Beginning of each cycle | MID_TIER | `/relay/events_json` | Summarizes recent events reported by all roles (sub1, sub2, mid). |
| **Plans Snapshot** | 3 – `SURFACE_RELAY` | Midway through the cycle | MID_TIER | `/relay/snapshot_json` | Full snapshot of merged plans, per-role and per-identity, for archival and cloud sync. |

Both snapshots are text-based JSON strings published on `std_msgs/String` topics and subscribed by the **IoT bridge**.

---

### 1. Events Report (`/relay/events_json`)

Triggered at **SURFACE_INIT** (slot 0) by the MID node.

**Purpose:**  
- Provides a compact view of mission events from the previous cycle (e.g., detection, faults, telemetry flags).
- Resets counters after every uplink.
- Guarantees the bridge gets one event packet per cycle, even under rotation.

**JSON structure example:**
```json
{
  "kind": "surface_relay_snapshot",
  "sender": "subMID_node",
  "Identity": 0,
  "role": "MID_TIER",
  "cycle": {
    "epoch": 1760070000,
    "dwell_s": 20.0,
    "slot": 0
  },
  "events": {
    "mid":  {"event_type": "nothing_new", "x": 0, "y": 0, "z": 0},
    "sub1": {"event_type": "foreign_uuv", "x": 35, "y": 20, "z": -20},
    "sub2": {"event_type": "low_battery", "x": 20, "y": 40, "z": -20}
  },
  "new_counts": {"mid": 0, "sub1": 2, "sub2": 1},
  "sent_at": 1760070020.42
}
```
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

## License

This project was developed as part of the **Vanderbilt Hackathon submarine swarm project**.

See `LICENSE` file for license information.

## ⭐ Acknowledgments 
**Special thanks to the following teams for joining our collaborative dashboard vision:**
* AutoGators
* Aardvark
* Goblin Gang
* Jet2Holiday
* Clankers
* The Hivemind
* Prober
* One
