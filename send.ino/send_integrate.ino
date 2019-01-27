/*
   Integrate master_code.ino and send.ino so that sensor values can be sent out
*/

// ===================================================================================

// Header files and global variables from send.ino


//By Rico Jia, May 16, 2018

//SG_FONA folder must be included in \Documents\Arduino\libraries
//Arduino must be set to port 1. Go to device manager -> Ports(COM & LPT) -> Arduino Uno -> Properties -> Port Settings -> Advanced -> COM Port Number. Then re-plug the USB.
//Serial may need to be set to 4800bps manually first. On my device, this was the default.
//Serial Monitor must be set to Both NL & CR, as well as 115200 Baud
//Many cell carriers do not support GSM. Only supported network may be Rogers (this will be shutting down soon)

//For more on interrupts, see https://www.robotshop.com/letsmakerobots/arduino-101-timers-and-interrupts
#include "SG_FONA.h"
volatile char ISR_Count = 0;

//Optional header file to enable I/O to the EEPROM memory
/*#include <EEPROM_R_W.h>

  EEPROM_R_W eeprom = EEPROM_R_W();*/

#define FONA_TX 4 //Soft serial port
#define FONA_RX 5 //Soft serial port
#define FONA_RI 3    //let's test it!
#define FONA_RST 9

#define FONA_POWER 8
#define FONA_POWER_ON_TIME  180  /* 180ms*/
#define FONA_POWER_OFF_TIME 1000 /* 1000ms*/

char sendto[21] = "7783173724";   // IMPORTANT: Enter destination number here

// We default to using software serial. If you want to use hardware serial
// (because softserial isnt supported) comment out the following three lines
// and uncomment the HardwareSerial line
#include <SoftwareSerial.h>
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;

// Hardware serial is also possible!
// HardwareSerial *fonaSerial = &Serial1;

// Use this for FONA 800 and 808s
//Adafruit_FONA fona = Adafruit_FONA(FONA_RST);
// Use this one for FONA 3G
Adafruit_FONA_3G fona = Adafruit_FONA_3G(FONA_RST);

uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);    // Is it for reading from serial port?

uint8_t type;

volatile int8_t numsms;

// -----------------------------------------------------------------------------

// Header files and global variables from master_code.ino

/* Defining PIN assignment
  // Voltage Sensor circuit
      A1 - Voltage Divider

  // Current Sensor circuit
      A0 - Hall Effect sensor

  // Temperature Sensor circuit
      A2 - Temperature sensor

  // SD Card SPI bus circuit
      pin 11 - MOSI
      pin 12 - MISO
      pin 13 - CLK
      pin 4 - CS (depends on your SD card shield or module, 4 used here)

  // Relay circuit
      pin 6 - Relay test signal

  // LED Test Interface
      pin 2 - Arduino ON/OFF
      pin 3 - Voltage reading
      pin 5 - Current reading
      pin 6 - Realy Open/Close
*/


// Header files
#include <SPI.h>
#include <SD.h>

// Declare all global variables
// Volatge Sensor
double DivVoltage = 0;       // Voltage divider reading
double SourceVoltage = 0;        // Final voltage reading result

// Current Sensor
double HallValue1 = 0;     // Variables for current running average
double HallValue2 = 0;
double HallValue3 = 0;
double HallVoltage = 0;   // Voltage reading of Hall Effect
double HallAmps = 0;      // Current result from Hall Effect Sensor

// Temperature Sensor
double TempVolt = 0; //previously volt
double Temp = 0;      // previously temp

// Data Logging
File myFile;
unsigned long Time;

// Relay Circuit
int RelayTest;

// Declare any global constants
double RH = 969000;   // Voltage Divider High Resistance
double RL = 24783;    // Voltage Divider Low Resistance


// Function prototypes
void send_sms(float LoadVoltage, float LoadCurrent, float Power, float AtmTemp, float SolTemp, bool WaterBreakerFlag);


// ===================================================================================

void setup() {

  Serial.begin(2400); //send.ino requires a 2400 baud rate

  // Serial.begin(9600);             // Setup Baud rate

  //from master_code.ino
  Serial.print("Initializing SD card...");

  if (!SD.begin(4)) {
    Serial.println("initialization failed!");
    while (1);
  }

  Serial.println("initialization done.");

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.

  //---------------------
  /*
      Note: pin numbers from master_code.ino and send.ino might overlap.
            For now, there is no change in any pin number setup
  */
  //---------------------

  // Defining LED Interface pins
  pinMode(2, OUTPUT);   // Arduino On LED
  pinMode(3, OUTPUT);   // Voltage LED
  pinMode(5, OUTPUT);   // Current LED
  pinMode(6, OUTPUT);   // Relay Open/Close

  //-------------------------------------------

  //from send.ino

  pinMode(FONA_POWER, OUTPUT);
  pinMode(FONA_RI, INPUT);
  digitalWrite(FONA_POWER, HIGH);
  delay(FONA_POWER_ON_TIME);
  digitalWrite(FONA_POWER, LOW);
  delay(3000);

  while (!Serial);    // wait till serial gets initialized

  //Serial.begin(2400); //Baud rate
  Serial.println(F("Sustaingineering 3G TxRx!!"));
  Serial.println(F("Initializing....(May take 3 seconds)"));

  fonaSerial->begin(4800);
  while (! fona.begin(*fonaSerial)) {
    Serial.println(F("Couldn't find FONA"));     //reboot arduino and fona if this shows up! (Should probably do this automatically for robustness)
    delay(1000);
  }
  type = fona.type();
  Serial.println(F("FONA is OK"));

  Serial.println(F("Searching for network\n"));
  bool SIMFound = false;
  for (int countdown = 600; countdown >= 0 && SIMFound == false; countdown--) {
    //Serial.print(F("Countdown: "));
    //Serial.println(countdown);
    uint8_t n = fona.getNetworkStatus();      // constantly check until network is connected to home    sendCheckReply(F("AT+CLVL="), i, ok_reply);
    if (n == 1)
    {
      SIMFound = true;
      Serial.print(F("Found ")); //If program hangs here, SIM card cannot be read/connect to network
      Serial.println(F("Network Connected"));
    }
    // uint8_t m = fona.setSMSInterrupt(1);    // this is for setting up the Ring Indicator Pin
  }
  if (!SIMFound)
  {
    Serial.println(F("SIM card could not be found. Please ensure that your SIM card is compatible with dual-band UMTS/HSDPA850/1900MHz WCDMA + HSDPA."));
    while (1) {}
  }
  //-------------------------------------------
}

// ===================================================================================

void loop() {

  // value for sensing SolTemp

  float sensorValue = analogRead(A2) ;
  // Convert the analog reading ( which goes from 0 - 1023) to a voltage (0 - 5V):
  float voltage = voltage / 1023;
  voltage = sensorValue * 5000; //(in mv)

  float volt = voltage - 1670; //(while cold conjuction is 22 degree
  //   the voltage cross thermalcouple is 1.67v)
  volt =  volt / 150; //(opamp apmplified 150 times)
  float temp = (volt) / 0.041 + 22  ; //( 1 degree = 0.0404 mv in K type )

  Serial.println(temp);

  //-------------------------------------

  //Sensor values to be sent

  // Smth ...
  Time = millis();

  //for now, use the have the functions from master_code.ino return void

  //voltage
  VoltageDivider();
  float LoadVoltage = SourceVoltage;

  //current
  HallEffect();
  float LoadCurrent = HallAmps;

  //power
  float Power = LoadCurrent * LoadVoltage; //not 100% sure if thats what its meant to do; delete comment if correct

  //atmospheric temperature
  Thermolcouple();
  float AtmTemp = Temp;

  //solar temperature
  float SolTemp = temp;

  //water breaker flag
  bool WaterBreakerFlag = false;

  //-------------------------------------

  //other program functions from master_code.ino

  /* LED Interface Control Testing */
  LEDInterface();

  /* Displaying Sensor Results */
  DisplayResults();

  /* Data Logging */
  SDLog();

  // 1 second delay
  delay(1000);

  //-------------------------------------

  //Send data into mobile device - send.ino
  send_sms(LoadVoltage, LoadCurrent, Power, AtmTemp, SolTemp, WaterBreakerFlag);
  //-------------------------------------

  while (1)
  {
  }
}

// ------------------------------------------------------------------------------

//Functions for sensor values (for now, let them return void)
//from master_code.ino

// Voltage Divider Sensor
void VoltageDivider() {
  // Read Voltage at divider and convert to decimal
  DivVoltage = ((analogRead(A1)) / 1023.0) * 5;

  // Final Source Voltage reading
  SourceVoltage = (DivVoltage * (RH + RL)) / RL;
}

// Hall Effect sensor
void HallEffect() {
  HallValue3 = HallValue2;         // Take reading at time index (n-2)
  HallValue2 = HallValue1;         // Take reading at time index (n-1)
  HallValue1 = analogRead(A0);     // Take reading at time index (n)

  // Take running average
  HallVoltage = (HallValue1 + HallValue2 + HallValue3) / 3;

  // Convert to decimal
  HallVoltage = (HallVoltage / 1023.0) * 5;

  // Compute the current from the voltage reading
  // Equation: ...
  HallAmps = (HallVoltage / 0.13720695) - (2.51 / 0.13720695);
}

// Thermolcouple sensor
void Thermolcouple() {
  // Read sensor value and convert to mV
  TempVolt = (analogRead(A2) * 5000) / 1023 + 25;

  // Check if upper voltage bound
  if (TempVolt > 2500) {
    double volt = TempVolt - 2500;    // Assuming cold junction at 22 degree
    volt = volt / 123;                // OpAmp amplified 150 times
    Temp = (volt / 0.041) + 25;       // 0.0404 mv/degree in K type
  }

  // Else if lower bound
  else {
    double volt = 2500 - TempVolt;    // Assuming cold junction at 22 degree
    // The voltage cross thermolcouple is 1.67v)
    volt = volt / 123;                // OpAmp apmplified 150 times
    Temp = 25 - (volt / 0.0404);      // 0.0404 mv/degree in K type
  }
}

// Display Results
void DisplayResults() {
  // Voltage divider voltage
  Serial.print("Divider Node Voltage = ");
  Serial.print(DivVoltage, 3);

  // Panel voltage Result
  Serial.print("\t Panel Voltage = ");
  Serial.print(SourceVoltage, 3);

  // Hall Effect voltage reading
  Serial.print("\t Hall Effect Voltage = ");
  Serial.print(HallVoltage, 3);

  // Current reading
  Serial.print("\t Panel Current = ");
  Serial.print(HallAmps, 3);

  // Panel temperature
  Serial.print("\t Panel Temperature = ");
  Serial.println(Temp);
}

// LED Interface
void LEDInterface() {
  RelayTest = digitalRead(6); // Checks if relay is closed
  digitalWrite(2, HIGH);      // Arduino on light
  digitalWrite(3, LOW);       // Voltage light
  digitalWrite(5, LOW);       // Current light
  digitalWrite(6, LOW);       // Relay light

  if (SourceVoltage > 0)
    digitalWrite(3, HIGH);

  if (HallAmps > 0)
    digitalWrite(5, HIGH);

  if (RelayTest == 1)
    digitalWrite(6, HIGH);
}

// Log Results on SD
void SDLog() {
  // Open test file
  // The file name testNUM is the text file we write to
  myFile = SD.open("test155.txt", FILE_WRITE);

  // if the file opened okay, write to it
  if (myFile) {
    // Record time
    myFile.print("Time (s) = ");
    myFile.print(Time / 1000);

    // Record divider voltage
    myFile.print("\t Divider Node Voltage = ");
    myFile.print(DivVoltage);

    // Record panel voltage
    myFile.print("\t Panels Voltage = ");
    myFile.print(SourceVoltage);

    // Record hall effect voltage signal
    myFile.print("\t Hall Effect Voltage = ");
    myFile.print(HallVoltage);

    // Record panel current
    myFile.print("\t Panel Current = ");
    myFile.print(HallAmps);

    // Record temperature
    myFile.print("\t Panel Temperature = ");
    myFile.println(Temp);

    // close the file:
    myFile.close();
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }


  // this code will read the data just stored on the SD Card, ensure file names match
  myFile = SD.open("test151.txt");
  if (myFile) {
    Serial.println("test151.txt:");

    // read from the file until there's nothing else in it:
    while (myFile.available()) {
      Serial.write(myFile.read());
    }
    // close the file:
    myFile.close();
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }
}


// ------------------------------------------------------------------------------

//Function for sending data
//from send.ino

void send_sms(float LoadVoltage, float LoadCurrent, float Power, float AtmTemp, float SolTemp, bool WaterBreakerFlag) { // send an SMS!

  char message[141];
  bool loop = true;
  while (loop)
  {
    flushSerial();    // THIS IS IMPORTANT! OTHERWISE what you typed in might be missing in what you send
    loop = false;

    String str;
    str = (String)(LoadVoltage) + "," + (String)(LoadCurrent) + "," + (String)(Power) + "," + (String)(AtmTemp) + "," + (String)(SolTemp) + "," + (String)(WaterBreakerFlag);
    str.toCharArray(message, 141);
  }
  Serial.print(F("Your message is: "));
  Serial.println(message);
  Serial.print(F("Value of message sent status: "));
  if (fona.sendSMS(sendto, message) == 0)
  {
    Serial.println(F("SMS sending failed."));
  }
  else
  {
    Serial.println(F("SMS sending succeeded."));
  }
}

void flushSerial() {
  while (Serial.available())
    Serial.read();
}

uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout) {
  uint16_t buffidx = 0;
  boolean timeoutvalid = true;
  if (timeout == 0) timeoutvalid = false;

  while (true) {
    if (buffidx > maxbuff) {
      //Serial.println(F("SPACE"));
      break;
    }

    while (Serial.available()) {
      char c =  Serial.read();

      //Serial.print(c, HEX); Serial.print("#"); Serial.println(c);

      if (c == '\r') continue;
      if (c == 0xA) {
        if (buffidx == 0)   // the first 0x0A is ignored.
          continue;

        timeout = 0;         // the second 0x0A is the end of the line
        timeoutvalid = true;
        break;
      }
      buff[buffidx] = c;
      buffidx++;
    }

    if (timeoutvalid && timeout == 0) {
      //Serial.println(F("TIMEOUT"));
      break;
    }
    delay(1);
  }
  buff[buffidx] = 0;  // null term
  return buffidx;
}













