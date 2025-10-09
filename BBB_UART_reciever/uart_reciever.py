import serial
import time

def parse_frame(frame: str):
    """
    Parse a single frame and return a dictionary describing its contents.
    """
    frame = frame.strip()
    if not frame:
        return None

    frame_type = frame[0]

    # --- Position Frame ---
    if frame_type == 'P':
        try:
            _, x, y, z = frame.split(',')
            return {
                "type": "position",
                "x": int(x),
                "y": int(y),
                "z": int(z)
            }
        except ValueError:
            print(f"[WARN] Invalid Position Frame: {frame}")
            return None

    # --- Orientation Frame ---
    elif frame_type == 'O':
        try:
            _, x, y, z = frame.split(',')
            return {
                "type": "orientation",
                "x": float(x),
                "y": float(y),
                "z": float(z)
            }
        except ValueError:
            print(f"[WARN] Invalid Orientation Frame: {frame}")
            return None

    # --- Event Frame ---
    elif frame_type == 'E':
        try:
            sub_id = int(frame[1])
            status_char = frame[2].upper()
            if status_char not in ['F', 'L']:
                raise ValueError
            status = "found" if status_char == 'F' else "lost"
            return {
                "type": "event",
                "sub_id": sub_id,
                "status": status
            }
        except (IndexError, ValueError):
            print(f"[WARN] Invalid Event Frame: {frame}")
            return None

    else:
        print(f"[WARN] Unknown Frame Type: {frame}")
        return None


def uart_receiver(port="/dev/ttyS0", baudrate=115200):
    """
    Continuously read and parse UART data from the specified serial port.
    """
    print(f"[INFO] Listening on {port} at {baudrate} baud...")
    try:
        with serial.Serial(port, baudrate, timeout=1) as ser:
            buffer = ""
            while True:
                data = ser.read().decode(errors='ignore')
                if data:
                    if data == '\n':
                        frame = buffer.strip()
                        buffer = ""
                        parsed = parse_frame(frame)
                        if parsed:
                            print("[RECV]", parsed)
                    else:
                        buffer += data
    except serial.SerialException as e:
        print(f"[ERROR] Serial error: {e}")
    except KeyboardInterrupt:
        print("\n[INFO] Exiting UART receiver.")


if __name__ == "__main__":
    # Adjust the serial port to your BeagleBone's UART device (e.g., /dev/ttyS1 or /dev/ttyUSB0)
    uart_receiver(port="/dev/ttyS4", baudrate=115200)

