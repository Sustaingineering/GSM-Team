import serial
import re
import unicodedata
import Tkinter	

portPath = "COM1"	   # Must match value shown on Arduino IDE
baud = 115200					 # Must match Arduino baud rate
timeout = 999					  # Seconds
max_num_readings = 1
num_signals = 1

sms=" "
 
Fields = ["Temperature"]
 
def create_serial_obj(portPath, baud_rate, tout):
	"""
	Given the port path, baud rate, and timeout value, creates
	and returns a pyserial object.
	"""
	return serial.Serial(portPath, baud_rate, timeout = tout)

def initialization(serial):
	serial.flushInput()
	
	print "Please wait..."
	
	serial_line=serial.readline()
	while serial_line.find("Number of messages:")==-1:
		print serial_line
		serial_line=serial.readline()
		
	print "Ready to begin"
	
def read_serial_data(serial):
	"""
	Given a pyserial object (serial). Outputs a list of lines read in
	from the serial port
	"""
	
	serial.flushInput()
	
	serial_data = []
	readings_left = True
	timeout_reached = False
	
	while readings_left and not timeout_reached:
		serial_line = serial.readline()
		sms=""
		if serial_line.find("Temperature: ")==0:
			sms=serial_line.partition(": ")[2]
		if len(serial_data) == max_num_readings:
			readings_left = False
	print serial_data
	return serial_data
		   
def clean_serial_data(data):
	"""
	Given a list of serial lines (data). Removes all characters.
	Returns the cleaned list of lists of digits.
	Given something like: ['0.5000,33\r\n', '1.0000,283\r\n']
	Returns: [[0.5,33.0], [1.0,283.0]]
	"""
	clean_data=[]
	for ascii in data:
		s=unicode(ascii, "utf-8")
		clean_data.append(("".join(ch for ch in s if unicodedata.category(ch)[0]!="C")).encode("utf-8"))
 
	return clean_data		   
 
class app(Tkinter.Tk):	
	
	def __init__(self,parent):
		Tkinter.Tk.__init__(self,parent)
		self.parent=parent
		self.x = 0
		self.after(1000, self.initialize)
		
	def initialize(self):
		self.x+=1
		print "Creating serial object..."
		self.serial_obj = create_serial_obj(portPath, baud, timeout)

		initialization(self.serial_obj)
		self.after(1000, self.callback)
		
	def callback(self):
		self.x += 1
		print "Reading serial data..."
		serial_data=read_serial_data(self.serial_obj)
		print "Length: "+str(len(serial_data))

		print "Cleaning data..."
		clean_data =  clean_serial_data(serial_data)
		print("Temperature: "+clean_data)
		self.after(1000, self.callback)  
	
if __name__=="__main__":
	app=app(None)
	app.title('Display SMS')
	app.mainloop()
	
	