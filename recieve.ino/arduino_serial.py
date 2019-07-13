import serial
import json
import random
import pyrebase
from time import time as t
 
portPath = "COM1"	   		# Must match value shown on Arduino IDE
baud = 115200				# Must match Arduino baud rate

CREDENTIALS = {
  "apiKey": "AIzaSyBI6chLsGI6XX67LGTXXH_gXLfngCS3aNs",
  "authDomain": "sustaingineering-horus.firebaseapp.com",
  "databaseURL": "https://sustaingineering-horus.firebaseio.com",
  "storageBucket": "sustaingineering-horus.appspot.com"
}

JSON = {
	"power": random.randrange(200, 600),
	"surface-temperature": random.randrange(10, 30),
	"op-temp": random.randrange(10, 30),
	"current": random.randrange(0, 5),
	"water-breaker": random.randrange(0, 3),
	"time-stamp": int(t()),
	"voltage": random.randrange(0, 10)
}

ORDER = [
	"voltage",
	"current",
	"power",
	"op-temp",
	"surface-temperature",
	"water-breaker"
]

class GSMSerialHandler:
	def __init__(self, port, baud):
		self.ser = serial.Serial(port=port, baudrate=baud) # Maybe add a timeout

	def get_line_from_serial(self):
		return self.ser.readline().decode("utf-8")

class FirebaseHandler:
	def __init__(self, creds):
		self.fb = pyrebase.initialize_app(creds)
		self.db = self.fb.database()
	
	def send_data(self, dest, data):
		self.db.child(dest).push(data)

# Expects a valid CSV string, does not do any validation
# Also expects the json to contain 
def parse_csv(json, string, order):
	j = json.copy()
	for s, i in zip(string.split(","), range(0, len(json) + 1)):
		j[order[i]] = float(s)
	j["timestamp"] = int(t())
	return j

# Validates that the string is a CSV in a super lazy way
def validate_valid_csv(string, item_count):
	c = string.count(",")
	return c == item_count - 1

if __name__ == "__main__":
	serial_handler = GSMSerialHandler(portPath, baud)
	firebase_handler = FirebaseHandler(CREDENTIALS)

	while True:
		s = serial_handler.get_line_from_serial()
		print(s)
		if validate_valid_csv(s, 6):
			j = parse_csv(JSON, s, ORDER)
			firebase_handler.send_data("69420", j)
