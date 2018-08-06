/*
 * EEPROM_R_W.h
 *
 * Reference: https://www.arduino.cc/en/Reference/EEPROM
 * Arduino Uno contains 1kb EEPROM storage
 * 
 * How to use this file:
 * 
 * Include the EEPROM_R_W folder in your ardunio libraries folder.
 * Include the following lines in your code:
 *		#include <EEPROM_R_W.h>
 * 		EEPROM_R_W eeprom = EEPROM_R_W();
 * You can call a function by accessing the eeprom object.
 * E.g.:
 * 		eeprom.writeStr("This will be written to the EEPROM");
 * 
 * 
 * This is a collection of functions to enable easy use of the EEPROM onboard memory of the Arduino Uno
 * These functions enable reading and writing to the EEPROM. Data can be read or written as either a string, or as any data type.
 * The addresses to read from and write to are updated automatically upon read or write. 
 * However, it is possible to retrieve these values, or to set them, if desired.
 * 
 * Functions:
 *
 * bool writeStr(char*)
 * 		Takes a string as an argument, and writes the string to the EEPROM
 * 		Returns true if successful.
 * bool writeData(Type)
 *		Takes any data type as an argument, and writes data to the EEPROM
 *		Returns true if successful.
 * bool writeFromAnalog(int)
 * 		Takes an integer representing an analog input pin number as an argument. 
 * 		Writes the current value at the pin (divided by 4) to the EEPROM.
 * 		To get an input stream, this must be called repeatedly. However, the frequency of calls cannot exceed 0.3mHz
 * bool readStr(char*, int)
 *		Takes a pointer to a string as an argument, and the maximum length of the string.
 *		Reads data from the EEPROM, and writes data at the pointer location until the end of the string, or maximum length is reached.
 * 		If the end of the string is successfully found, returns true.
 * 		If the maximum length of the string is reached, returns false.
 * Type readData()
 * 		Reads data of any type. 
 * 		When calling this function, data type must be specified. 
 * 		Data will be returned as specified type.
 * 		E.g.: int data=readData<int>();
 *		Returns true if successful.
 * int getReadAddr()
 *		Returns the next address to be read from as an integer.
 * int getWriteAddr()
 * 		Returns the previous address written to as an integer.
 * bool setReadAddr(int)
 *		Takes an integer between 0 and 1024. Sets the next address to read from as this value.
 *		If the number is out of range, the value will not be set, and the function will return false.
 *		Returns true if successful.
 * bool setWriteAddr(int)
 * 		Takes an integer between 0 and 1024. Sets the next address to write to as this value.
 *		NOTE: This works differently to getWriteAddr. If setWriteAddr(10) is called, then getWriteAddr() will return 9.
 * 		If the number is out of range, the value will not be set, and the function will return false.
 *		Returns true if successful.
 */

#ifndef EEPROM_R_W_H
#define EEPROM_R_W_H

#include <EEPROM.h>

class EEPROM_R_W 
{
	private:
		int writeAddr=-1;
		int readAddr=0;
		/*
		 * To increase lifespan of EEPROM, starting address should be chosen at random.
		 * Value of writeAddr after write operation will be the location of the just written character
		 */
		int getAddr(void)
		{
			writeAddr++;
			if(writeAddr == EEPROM.length()) writeAddr=0;
			return writeAddr;
		}
	public:
		EEPROM_R_W()
		{
		}
		EEPROM_R_W(int startAddr)
		{
			readAddr=startAddr;
			writeAddr=startAddr-1;
		}
		/*
		 * Used to read data from an analog input pin, and store in EEPROM
		 * NOTE: Write time of EEPROM is 3.3ms. Do not exceed this period.
		 */
		bool writeFromAnalog(int pin)
		{
		  /***
			Need to divide by 4 because analog inputs range from
			0 to 1023 and each byte of the EEPROM can only hold a
			value from 0 to 255.
		  ***/
		  int val=analogRead(pin)/4;

		  /***
			Write the value to the appropriate byte of the EEPROM.
			these values will remain there when the board is
			turned off.
		  ***/
		  writeData(val);
		  
		  return true;

		}
		
		/*
		 * Write string to EEPROM
		 * String must be null-terminated
		 */
		bool writeStr(char* message)
		{
		  for(int index=0;message[index]!='\0';index++)
		  {
		  /* 
		   *  EEPROM.update will write data only if data is different to current value at address.
		   *  This increases life span of EEPROM (100,000 write cycles) and speeds up write operations where data does not need to be written.
		   *  Use EEPROM.write to unconditionally write to an address.
		   */
			EEPROM.update(getAddr(),message[index]);
		  }
		  EEPROM.update(getAddr(),'\0');
		  
		  return true;
		}
		
		/*
		 * Write data of any type to the EEPROM
		 */
		template <typename T>
		bool writeData(T data)
		{
		  EEPROM.put(getAddr(), data);
		  /*
		   * Set value of writeAddr to the last bit of the float.
		   */
		  writeAddr+=sizeof(T)-1;
		  if(writeAddr>=EEPROM.length())writeAddr-=EEPROM.length();
		  
		  return true;
		}
		
		/*
		 * Read data of any type
		 * Type must be specified in function call
		 * E.g.: int data=readData<int>();
		 */
		template <typename T>
		T readData(void)
		{
		  T data=0;
		  EEPROM.get(readAddr, data);
		  readAddr+=sizeof(T);
		  if(readAddr>=EEPROM.length())readAddr-=EEPROM.length();
		  
		  return data;
		}
		
		/*
		 * Reads a string from the EEPROM
		 */
		bool readStr(char* message, int maxLen)
		{
		  int i=0;
		  Serial.println(F("In readStr"));
		  message[i]=EEPROM.read(readAddr+i);
		  Serial.print(F("First character is: "));
		  Serial.print(message[i]);
		  for(i=1;message[i-1]!='\0'&&i<maxLen;i++)
		  {
			if(readAddr+i>=EEPROM.length()) readAddr=-i;
			message[i]=EEPROM.read(readAddr+i);
		  }
		  if(message[i]!='\0')
		  {
			  i++;
			  message[i]='\0';
			  readAddr+=i;
			  if(readAddr>=EEPROM.length()) readAddr-=EEPROM.length();
			  return false;
		  }
		  Serial.print(F("\n"));
		  readAddr+=i;
		  if(readAddr>=EEPROM.length()) readAddr-=EEPROM.length();
		
		  return true;
		}
		
		/*
		 * Reads a float from the EEPROM
		 */
		/* float readFloat(void)
		 {
			 float num=EEPROM.read(readAddr);
			 readAddr+=sizeof(float);
			 return num;
		 }*/
		 
		 /*
		 * Reads an integer from the EEPROM
		 */
		/* int readInt(void)
		 {
			 int num=EEPROM.read(readAddr);
			 readAddr+=sizeof(int);
			 return num;
		 }*/
		
		/*
		 * Sets the address to read from
		 * By default, the address will be initialized to 0
		 * If specified address is invalid, address will not be set, and false will be returned
		 * Else, true will be returned
		 */
		bool setReadAddr(int addr)
		{
			if(addr<0||addr>=EEPROM.length())
				return false;
			readAddr=addr;
			return true;
		}
		
		/*
		 * Sets the address to write to
		 * By default, the address will be initialized to 0
		 * If specified address is invalid, address will not be set, and false will be returned
		 * Else, true will be returned
		 * NOTE: 	The value of writeAddr is the most recent address that has been written to. 
		 * 			This function allows specification of the next address to be written to. 
		 * 			Therefore, 1 will be subtracted from the entered value.
		 */
		bool setWriteAddr(int addr)
		{
			if(addr<0||addr>=EEPROM.length())
				return false;
			writeAddr=addr-1;
			return true;
		}
		
		/*
		 * Returns the current address to read from as an integer
		 */
		int getReadAddr(void)
		{
			return readAddr;
		}
		
		/*
		 * Returns the most recent address written from as an integer
		 * If no address has been written to, returns -1
		 */
		int getWriteAddr(void)
		{
			return writeAddr;
		}
};

#endif