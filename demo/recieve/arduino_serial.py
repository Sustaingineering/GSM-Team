import serial
import re
import unicodedata
import Tkinter	
from Tkinter import *

portPath = "COM1"	   # Must match value shown on Arduino IDE
baud = 115200					 # Must match Arduino baud rate
timeout_const = 10					  # Seconds
timeout=timeout_const
max_num_readings = 1
num_signals = 1
 
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
	
	readings_left = True
	timeout_reached = False
	
	sms=""
	timeout=timeout_const
	
	while readings_left and not timeout_reached:
		serial_line = serial.readline()
		if serial_line.find("Temperature: ")==0:
			sms=serial_line.partition(": ")[2]
		timeout-=1
		if(timeout==0):
			timeout_reached=True
	print sms
	return sms   
 
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
		if(serial_data):
			print("Temperature: "+serial_data)
			text="Temperature: "+serial_data
			canvas.itemconfigure(Label1,text=text)
		self.after(100, self.callback)  
	
if __name__=="__main__":
	app=app(None)
	app.title('Display SMS')
	screen_width=app.winfo_screenwidth()
	screen_height=app.winfo_screenheight()
	canvas = Canvas(app, height=screen_height, width=screen_width, relief=RAISED, bg='black')
	canvas.grid()
	canvas.pack()
	Display_box1 = canvas.create_rectangle(screen_width, screen_height, 0, 0,  fill = 'black')
	Label1 = canvas.create_text(screen_width/2, screen_height/2, text="Temperature: ", font=('Impact', -100,), fill="white")

	app.mainloop()
	
	