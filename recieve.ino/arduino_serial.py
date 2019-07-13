import serial
import json
 
portPath = "COM1"	   		# Must match value shown on Arduino IDE
baud = 115200				# Must match Arduino baud rate
timeout = 999				# Seconds
j_filename = "data.json"	# File to output data into

class GSMSerialHandler:
	def __init__(self, port, baud):
		self.ser = serial.Serial(port=port, baudrate=baud) # Maybe add a timeout

	def get_line_from_serial(self):
		return serial.readline().decode("utf-8")

def upload_to_firebase():
	pass

def parse_csv():
	pass

if __name__ == "__main__":
	serial_handler = GSMSerialHandler(portPath, baud)
	print(serial_handler.get_line_from_serial())