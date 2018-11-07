# Collecting Arduino Serial via Python
Source: [ttmarek](https://gist.github.com/ttmarek/f3312eaf18a2e59398a2#file-arduino_serial-py)
1. Installations
This script was tested with Python v2.7
Install any missing libraries using `pip install *library*`
This probably includes pySerial (`pip install pySerial`)
2. From the command line, in the correct directory, run `python arduino_serial.py`
3. To stop the program, you will have to unplug the microcontroller from the computer momentarily.
NOTE: The serial sometimes gets stuck. You may need to press enter occasionally.
Depending on your version of Python, you may be required to use brackets () around your print statements
You may need to convert string to a bytes-like object. Do this by adding `.encode()` after the string 
`print("SampleString".encode())`