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