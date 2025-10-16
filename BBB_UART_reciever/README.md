# ROS UART Publisher

This ROS2 node reads data from a UART interface and publishes it for other ROS nodes.  
It handles **two types of UART data**:

1. **Pose Frames** — position and orientation vectors `(x, y, z)` and `(x̂, ŷ, ẑ)` at **2 Hz**.  
2. **Event Messages** — asynchronous messages describing specific events (e.g. button press, alert, or state change).

---

## 📘 Overview

The node:
- Opens a UART port (e.g., `/dev/ttyS4`)
- Continuously listens for data
- Parses incoming frames into **Pose** and **Event** messages
- Publishes these to ROS2 topics so other nodes can subscribe and use the data

---

## 🧾 UART Message Format

The device sends ASCII-formatted lines over UART.

### 1. Pose Frames (2 Hz)

Position:    P,x,y,z\n
Orientation: O,x,y,z\n

### 2. Aperiodic Events

| Message | Meaning        |
| ------- | -------------- |
| `'0'`   | ESP0 connected |
| `'1'`   | ESP1 connected |
| `'2'`   | ESP2 connected |
| `'A'`   | ESP0 lost      |
| `'B'`   | ESP1 lost      |
| `'C'`   | ESP2 lost      |
