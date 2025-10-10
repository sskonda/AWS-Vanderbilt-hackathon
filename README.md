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

### Built with:

### Technologies used:
* Arduino ESP32 BLE library
* TI TivaWare peripheral drivers
* Custom **G8RTOS** kernel (scheduling, semaphores, threads, UART/SPI drivers)

### Architecture

## AWS DynamoDB
* Scalable and relatively easy-to-use database system
* Integrates easily with other AWS services
* Free-form NoSQL format for on-the-fly data format changes

## AWS IoT Core
* Come standard with MQTT publish/subscribe model
* Allow for a centralized staging area for devices

## AWS IoT Rules
* Straightforward data redirection framework
* SQL-based digestion engine for data parsing before redirection
* Forwards to DynamoDB instance
* Forwards to Webserver via HTTP packet for display
* Solid error fallback for diagnostics

## Getting Started

### Installation Instructions

#### ESP32
1. Install the Arduino ESP32 core ([https://github.com/espressif/arduino-esp32](https://github.com/espressif/arduino-esp32)).
2. Open `BLE_to_TIVA_Frames.ino` in the Arduino IDE.
3. For each ESP32 board:

   * Set `OWN_NAME` to `"ESP0"`, `"ESP1"`, or `"ESP2"`.
   * Upload the code.
4. Connect ESP32 TX (GPIO 17 by default) to the Tiva UART RX.
5. Ensure all grounds (ESP32 + Tiva + LCD) share a common ground.

#### Web App UI
See [Swamp Portal README](https://github.com/emma-coronado/Swamp-Portal?tab=readme-ov-file#quick-start)

## License

This project was developed as part of the **Vanderbilt Hackathon submarine swarm project**.

---
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
