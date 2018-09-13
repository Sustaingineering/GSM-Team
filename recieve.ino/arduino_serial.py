import serial
import csv
import json
import re
import matplotlib.pyplot as plt
import unicodedata
# import pandas as pd
 
portPath = "COM1"	   # Must match value shown on Arduino IDE
baud = 115200					 # Must match Arduino baud rate
timeout = 999					  # Seconds
c_filename = "data.csv"
j_filename = "data.json"
max_num_readings = 6
num_signals = 1

sms=" "
 
# Fields = ["Load Voltage: ", "Load Current: ", "Power: ", "Atmospheric Temperature: ", "Solar Panel Temperature: ", "Water Breaker Flag: "]
Fields = ["Load Voltage", "Load Current", "Power", "Atmospheric Temperature", "Solar Panel Temperature", "Water Breaker Flag"]
 
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
		# print serial_line
		sms=""
		for s in Fields:
			print "Serial line: "+str(serial_line)
			print "Finding: "+str(s+": ")
			print "Found="+str(serial_line.find(s+": "))
			if serial_line.find(s+": ")==0:
				serial_data.append(serial_line.partition(": ")[2])
				print "Creating Temp: "
				temp=""+s+": "+serial_data[-1]
				print temp
				sms=sms+temp+","
				
				# serial_data.append(serial_line)
				print "Length serial_data: "+str(len(serial_data))
				print "serial_data: "+str(serial_data) 
				if len(serial_data) == max_num_readings:
					readings_left = False
	print "SMS: \n"+str(sms)
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
	print "Clean data: "+str(clean_data)
 
	return clean_data		   
 
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
	fr=open(j_filename,"r")
	jsoncontent=fr.read()
	fr.close()
	fw=open(j_filename, "w")
	jsondata="{\n"
	for i in range(len(Fields)):
		jsondata+="\t"
		jsondata+="\""+Fields[i]+"\": "
		jsondata+="\""+str(data[i])+"\""
		jsondata+=",\n"
	jsondata+="}\n"
	fw.write(jsoncontent+jsondata)
	fw.close()
	
def gen_col_list(num_signals):
	"""
	Given the number of signals returns
	a list of columns for the data.
	E.g. 3 signals returns the list: ['Time','Signal1','Signal2','Signal3']
	"""
	col_list = ['Time']
	for i in range(1,num_signals+1):
		col = 'Signal'+str(i)
		col_list.append(col)
		
	return col_list
	
def map_value(x, in_min, in_max, out_min, out_max):
	return (((x - in_min) * (out_max - out_min))/(in_max - in_min)) + out_min
 
	
def simple_plot(csv_file, columns, headers):
	plt.clf()
	plt.close()
	plt.plotfile(csv_file, columns, names=headers, newfig=True)
	plt.show()
 
# def plot_csv(csv_file, cols):
	# # Create Pandas DataFrame from csv data
	# data_frame = pd.read_csv(csv_file)
	# # Set the names of the columns
	# data_frame.columns = cols
	# # Set the first column (Time) as the index 
	# data_frame = data_frame.set_index(cols[0])
	# # Map the voltage values from 0-1023 to 0-5
	# data_frame = data_frame.apply(lambda x: map_value(x,0.,1023,0,5))
	# # Bring back the Time column
	# data_frame = data_frame.reset_index()
	# plt.clf()
	# plt.close()
	# # Plot the data
	# data_frame.plot(x=cols[0],y=cols[1:])
	# plt.show()
	

print "Creating serial object..."
serial_obj = create_serial_obj(portPath, baud, timeout)

initialization(serial_obj)

while 1:

	print "Reading serial data..."
	serial_data=read_serial_data(serial_obj)
	print "Length: "+str(len(serial_data))

	print "Cleaning data..."
	clean_data =  clean_serial_data(serial_data)

	# clean_data=[1,2,3,4,5,6]
	# print "Saving to csv..."
	# save_to_csv(clean_data, c_filename)

	print "Saving to json..."
	save_to_json(clean_data, j_filename)
 
# print "Plotting data..."
# #simple_plot(filename, (0,1,2), ['time (s)', 'voltage1', 'voltage2'])
# #simple_plot(filename, (0,1), ['time (s)', 'voltage1'])
# plot_csv(filename, gen_col_list(num_signals))