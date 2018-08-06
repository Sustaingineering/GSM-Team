/*
 * Reference: https://www.arduino.cc/en/Reference/EEPROM
 * Arduino Uno contains 1kb EEPROM storage
 */
#include <EEPROM.h>


template <typename T>
void writeData(T data);
template <typename T>
T readData(int readAddr);

int writeAddr=EEPROM.length()-3; //Don't use directly. Call int getAddr();
int readAddr=EEPROM.length()-2;
int getAddr(void);
void writeFromAnalog(int pin);
void writeFromCode(char* message);
void writeFloat(float fVal);
void readStr(char* message, int readAddr);


void setup() {
  Serial.begin(115200); //Baud rate
  Serial.println(EEPROM.length());
  // Read data from code
  /*
   * If entering numbers instead of characters, be sure to end with NULL ('\0')
   * NOTE: EEPROM can only hold values from 0 to 255
   */
 
  /*char message[(int)(EEPROM.length())]="How well does this work? \tCan it handle\nmultiline output?";
  writeFromCode(message);
*/
  /*
   * Reads a string from EEPROM, terminated by NULL ('\0')
   * Prints the message to the console
   */
   /*char message[(int)(EEPROM.length())];
   Serial.println(F("Printing stored data: "));
   readStr(message, readAddr);
   for(int i=0; message[i]!='\0'; i++)
   {
    Serial.print(message[i]);
   }
   Serial.print(F("\n"));*/

   int testInt=12;
   float testFloat=9.8f;
   double testDouble=3.141590;

   //writeData(testInt*2);
   writeData(testFloat*2);
   writeData(testDouble*2);
   writeData(testInt+2);

   /*Serial.print(F("Read address: "));
   Serial.println(readAddr);
   Serial.print(F("Integer: "));
   testInt=readData<int>(readAddr);
   Serial.println(testInt);
   readAddr+=sizeof(int);
   if(readAddr>=EEPROM.length())readAddr-=EEPROM.length();
   Serial.print(F("Read address: "));
   Serial.println(readAddr);*/
   Serial.print(F("Float: "));
   testFloat=readData<float>(readAddr);
   Serial.println(testFloat);
   readAddr+=sizeof(float);
   if(readAddr>=EEPROM.length())readAddr-=EEPROM.length();
    Serial.print(F("Read address: "));
   Serial.println(readAddr);
   Serial.print(F("Double: "));
   testDouble=readData<double>(readAddr);
   Serial.println(testDouble);
   readAddr+=sizeof(double);
   if(readAddr>=EEPROM.length())readAddr-=EEPROM.length();
    Serial.print(F("Read address: "));
   Serial.println(readAddr);
   Serial.print(F("Integer: "));
   testInt=readData<int>(readAddr);
   Serial.println(testInt);
   readAddr+=sizeof(int);
   if(readAddr>=EEPROM.length())readAddr-=EEPROM.length();

   Serial.print(F("Read address: "));
   Serial.println(readAddr);
  
}

void loop() {
 /* // Read data from pin:
  int pin=0; //Select pin to write from
  void writeFromAnalog(pin);
  // Currently, this runs continuously, 10 times per second, forever
  // Add end conditions and change frequency of data read here
  // NOTE: Write time of EEPROM is 3.3ms. Do not exceed this requency
  delay(100);
  */
}
/*
 * To increase lifespan of EEPROM, starting address should be chosen at random.
 * Value of writeAddr after write operation will be the location of the just written character
 */
int getAddr()// WARNING: If data length excedes EEPROM size, previous data will be overwritten
{
  writeAddr++;
  if(writeAddr >= EEPROM.length()) writeAddr=0;
  return writeAddr;
}

/*
 * Used to read data from an analog input pin, and store in EEPROM
 */
void writeFromAnalog(int pin)
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
  /* 
   *  EEPROM.update will write data only if data is different to current value at address.
   *  This increases life span of EEPROM (100,000 write cycles) and speeds up write operations where data does not need to be written.
   *  Use EEPROM.write to undconditionally write to an address.
   */
  EEPROM.update(getAddr(), val); 

}

void writeFromCode(char* message)
{
  for(int index=0;message[index]!='\0';index++)
  {
    EEPROM.update(getAddr(),message[index]);
  }
  EEPROM.update(getAddr(),'\0');
}

void writeFloat(float fVal)
{
  EEPROM.put(getAddr(), fVal);
  /*
   * Set value of writeAddr to the last bit of the float.
   */
  writeAddr+=sizeof(float)-1;
  /*
   * Unsure of how EEPROM.put handles overflow. Assuming it wraps to beginning.
   */
  if(writeAddr>=EEPROM.length())writeAddr-=EEPROM.length();
  
}

void readStr(char* message, int readAddr)
{
  for(int i=0;message[i]!='\0';i++)
  {
    if(readAddr+i>=EEPROM.length()) readAddr=-i;
    message[i]=EEPROM.read(readAddr+i);
  }
}

template <typename T>
void writeData(T data)
{
  EEPROM.put(getAddr(), data);
  writeAddr+=sizeof(T)-1;
  if(writeAddr>=EEPROM.length())writeAddr-=EEPROM.length();
}

template <typename T>
T readData(int readAddr)
{
  T data=0;
  EEPROM.get(readAddr, data);
  return data;
}


