import serial

def read_uart9port="/dev/tty50", baudrate=115200):
	with serial.Serial(port, baudrate, timeout=1) as ser:
		while True:
			data = ser.readline().decode().strip()
			if data:
				print("Recieved: ", data)


if _name_=="__main__":
	read_uart()
