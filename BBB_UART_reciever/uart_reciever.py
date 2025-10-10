#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
BeagleBone Black UART4 Receiver
-------------------------------
Reads frames from ESP32 → Tiva → BBB via UART4 (P9_11 TX, P9_13 RX).

Frame formats:
--------------
P,<x>,<y>,<z>\n    → Position frame (signed ints)
O,<x>,<y>,<z>\n    → Orientation frame (floats, -1.0 to 1.0)
E<num><char>\n     → Event frame (num = 0–2, char = F/L)

Example inputs:
---------------
P,1234,5678,910
O,1.0,-0.2,0.5
E1F

Hardware Notes:
---------------
UART4 Pins → TX = P9_11, RX = P9_13
Enable overlay in /boot/uEnv.txt:
    enable_uboot_overlays=1
    uboot_overlay_addr4=/lib/firmware/BB-UART4-00A0.dtbo

Pin config (if needed):
    sudo config-pin P9_11 uart
    sudo config-pin P9_13 uart
"""

import os
import serial
import time
import sys


def parse_frame(frame: str):
    """Parse a single frame and return a structured dictionary."""
    frame = frame.strip()
    if not frame:
        return None

    frame_type = frame[0]

    # --- Position Frame ---
    if frame_type == 'P':
        try:
            _, x, y, z = frame.split(',')
            return {"type": "position", "x": int(x), "y": int(y), "z": int(z)}
        except ValueError:
            return None

    # --- Orientation Frame ---
    elif frame_type == 'O':
        try:
            _, x, y, z = frame.split(',')
            return {"type": "orientation", "x": float(x), "y": float(y), "z": float(z)}
        except ValueError:
            return None

    # --- Event Frame ---
    elif frame_type == 'E':
        try:
            sub_id = int(frame[1])
            status_char = frame[2].upper()
            if status_char not in ['F', 'L']:
                raise ValueError
            status = "found" if status_char == 'F' else "lost"
            return {"type": "event", "sub_id": sub_id, "status": status}
        except (IndexError, ValueError):
            return None

    else:
        return None


def uart_receiver(port="/dev/ttyS4", baudrate=115200):
    """
    Continuously read and parse UART4 data.
    Automatically checks that UART4 is enabled and accessible.
    """
    # --- Check that the UART4 device exists ---
    if not os.path.exists(port):
        print(f"[ERROR] UART device '{port}' not found!")
        print("--------------------------------------------------")
        print("👉 To enable UART4 on BeagleBone Black:")
        print("   1. Edit /boot/uEnv.txt and ensure these lines exist:")
        print("        enable_uboot_overlays=1")
        print("        uboot_overlay_addr4=/lib/firmware/BB-UART4-00A0.dtbo")
        print("   2. Save and reboot:")
        print("        sudo reboot")
        print("--------------------------------------------------")
        sys.exit(1)

    try:
        ser = serial.Serial(port, baudrate, timeout=1)
    except serial.SerialException as e:
        print(f"[ERROR] Could not open {port}: {e}")
        sys.exit(1)

    buffer = ""
    try:
        while True:
            data = ser.read().decode(errors='ignore')
            if data:
                if data == '\n':
                    frame = buffer.strip()
                    buffer = ""
                    parsed = parse_frame(frame)
                    if parsed:
                        return parsed
                else:
                    buffer += data
    except KeyboardInterrupt:
        return "interrupted"
    finally:
        ser.close()


if __name__ == "__main__":
    while(1):
        val = uart_receiver(port="/dev/ttyS1", baudrate=115200)
        print(val)
