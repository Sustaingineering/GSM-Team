import serial
# import csv
import json
import re
import matplotlib.pyplot as plt
import unicodedata
 
portPath = "COM3"	   # Must match value shown on Arduino IDE
baud = 115200					 # Must match Arduino baud rate
timeout = 999					  # Seconds
# c_filename = "data.csv"
j_filename = "data.json"	# File to output data into
 
# Updating this list of fields will automatically cause the new field to be handled correctly
# The field must also be added in the microcontroller code

# Fields = ["Load Voltage: ", "Load Current: ", "Power: ", "Atmospheric Temperature: ", "Solar Panel Temperature: ", "Water Breaker Flag: "]
Fields = ["Number", "Date", "Time", "ID", "Load Voltage", "Load Current", "Power", "Atmospheric Temperature", "Solar Panel Temperature", "Water Breaker Flag"]
 
# There are three values collected which are not a field: these are sender's number, date, and time
max_num_readings = len(Fields)

def create_serial_obj(portPath, baud_rate, tout):
	"""
	Given the port path, baud rate, and timeout value, creates
	and returns a pyserial object.
	"""
	return serial.Serial(portPath, baud_rate, timeout = tout)

def initialization(serial):
	"""
	Connecting to microcontroller via serial port
	"""
	serial.flushInput()
	
	print "Please wait..." # Waiting for microcontroller to connect to GSM network
	
	serial_line=serial.readline()
	while serial_line.find("Number of messages:")==-1:
		print serial_line
		serial_line=serial.readline()
		
	# Serial sometimes gets stuck here. If so, press enter
	print "Ready to begin"
	
def read_serial_data(serial):
	"""
	Given a pyserial object (serial). Outputs a list of lines read in
	from the serial port
	"""
	
	# Failsafe
	for i in Fields:
		i="Error"
	
	# Request serial input
	serial.flushInput()
	
	serial_data = []
	# Check if there are more readings. If there are no more readings, exit the loop
	readings_left = max_num_readings
	timeout_reached = False
	read_header=False
	
	# Read serial until all values have been found
	while readings_left and not timeout_reached:
		# Read serial input
		serial_line = serial.readline()		
		print(serial_line) # For debugging
		
		# First, read the header values
		# These values appear twice in microcontroller output. Make sure only to read values once
		# Parse serial output for header values line (begins with +CMGR: )
		if(not read_header and serial_line.find("+CMGR: ")==0):
			# Split header line into values
			temp_array=serial_line.split(",")
			# Select the correct values
			serial_data.append(temp_array[1].replace("\"","")) #Append sender's phone number
			readings_left-=1
			serial_data.append(temp_array[3].replace("\"","")) #Append date SMS was recieved
			readings_left-=1
			serial_data.append(temp_array[4].replace("\"","")) #Append time SMS was recieved
			readings_left-=1
			read_header=True
			
		# Read values of fields
		for s in Fields:
			# Parse serial input for field name
			if serial_line.find(s+": ")==0:
				# Extract value for field
				serial_data.append(serial_line.partition(": ")[2])
				readings_left-=1
	return serial_data
	
	
 
def is_number(string):
	"""
	Given a string returns True if the string represents a number.
	Returns False otherwise.
	"""
	try:
		float(string)
		return True
	except ValueError:
		return False
		
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
	
	# clean_data = []
	
	# for line in data:
		# line_data = re.findall("\d*\.\d*|\d*",line) # Find all digits
		# line_data = [float(element) for element in line_data if is_number(element)] # Convert strings to float
		# if len(line_data) >= 2:
			# clean_data.append(line_data)
 
	return clean_data		   
 
# Not used
def save_to_csv(data, c_filename):
	"""
	Saves a list of lists (data) to filename.csv
	"""
	# Broken
	with open(c_filename, 'wb') as csvfile:
		csvread = csv.reader(csvfile)
		for row in csvread:
			row_array.append(row)
		
		csvwrite = csv.writer(csvfile)
		
		csvwrite.writerow(data)
 
def save_to_json(data, j_filename):
	"""
	Saves a list of lists (data) to filename.json
	"""
	
	"""
	Writing to a file overwrites the data.
	We must read the current contents of the JSON into memory, and then write it back to the file again.
	"""
	fr=open(j_filename,"r")
	jsoncontent=fr.read()
	fr.close()
	
	# Open file to write
	fw=open(j_filename, "w")
	
	# Format output
	jsondata="{\n"
	# jsondata+="\t\"Number\": \""
	# jsondata+=str(data[0])
	# jsondata+="\",\n"
	# jsondata+="\t\"Date\": \""
	# jsondata+=str(data[1])
	# jsondata+="\",\n"
	# jsondata+="\t\"Time\": \""
	# jsondata+=str(data[2])
	# jsondata+="\",\n"
	for i in range (0,3):
		jsondata+="\""+Fields[i]+"\": "
		jsondata+="\""+str(data[i+3])+"\""
		jsondata+=",\n"
	jsondata+="\t\"Data\":[\n"
	for i in range(len(Fields)):
		jsondata+="\t\t"
		jsondata+="\""+Fields[i]+"\": "
		jsondata+="\""+str(data[i+3])+"\""
		jsondata+=",\n"
	jsondata+="\t]\n"
	jsondata+="}\n"
	
	# Write output
	fw.write(jsoncontent+jsondata)
	
	#IMPORTANT: File must be closed so that GUI can also read JSON file
	fw.close()
	

# Main function

# Initialize serial port
print "Creating serial object..."
serial_obj = create_serial_obj(portPath, baud, timeout)

initialization(serial_obj)

# Read SMS
while 1:

	# Reading serial data...
	serial_data=read_serial_data(serial_obj)

	# Cleaning data
	clean_data =  clean_serial_data(serial_data)

	# Saving to json
	save_to_json(clean_data, j_filename)