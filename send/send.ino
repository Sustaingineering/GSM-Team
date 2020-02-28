//list of libraries

#include "SG_FONA.h"
#include <SPI.h>
#include "RTClib.h"
#include <Wire.h>
#include <OneWire.h>
#include <SoftwareSerial.h>

//constants

//use either pins 2 & 3 or 4 & 5 for the TX and RX respectively for Software Serial, based on the connector pins on the GSM
#define FONA_TX 4 //Soft serial port
#define FONA_RX 5 //Soft serial port

#define FONA_RST 9               //just need a digital pin......
#define FONA_POWER 8            //this has to be the clock 0 pin or pin 8
#define FONA_POWER_ON_TIME 180   /* 180ms*/
#define FONA_POWER_OFF_TIME 1000 /* 1000ms*/
#define FONA_SINGLE_MESSAGE_DELAY_TIME 1000

// -------------------------------------------

/*
List of Phone Numbers: 
7789525137 - Forbes
6047283793 - Abdul
7789391063 - GSM1
7789391268 - GSM2
*/

char sendto[21] = "7789525137";
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;
Adafruit_FONA_3G fona = Adafruit_FONA_3G(FONA_RST);
uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0); // Is it for reading from serial port?
uint8_t type;
volatile int8_t numsms;

// -------------------------------------------

// Declare all global variables

// Voltage Sensor
double DivVoltage = 0;    // Voltage divider reading (for debugging purposes)
double SourceVoltage = 0; // Final voltage reading result
double RH = 1000000;      //981300; // Voltage Divider High Resistance
double RL = 25000;        //24743; // Voltage Divider Low Resistance

// Current Sensor
double HallVoltage = 0; // Voltage reading of Hall Effect
double HallAmps = 0;    // Current result from Hall Effect Sensor
float Power = 0;

// Temperature Sensor
double Temp = 0; // intermediate temperature calculation
float SolTemp = 0; //solar panel temperature

int DS18S20_Pin = 2; //DS18S20 Signal pin can be on digital 4 as well - don't overlap with the TX of the GSM

OneWire ds(DS18S20_Pin); // temperature chip i/o

//Water Pump Sensor
bool WaterBreakerFlag = false;

// Data Logging
// File myFile;
unsigned long Time;

// RTC time
//RTC_DS3231 rtc;

// -------------------------------------------

void setup()
{
  // -------------------------------------------

  //Adafruit FONA setup

  //very important for the GSM to automatically send messages when powered on after powering off
  pinMode(FONA_POWER, OUTPUT);
  digitalWrite(FONA_POWER, HIGH);
  delay(FONA_POWER_ON_TIME);
  digitalWrite(FONA_POWER, LOW);
  delay(3000);
  while (!Serial)
    ;                 // wait till serial gets initialized
  Serial.begin(115200); //baud rate

  Wire.begin(); // Initiate the Wire library and join the I2C bus as a master or slave

  // -------------------------------------------

  // Initializing DS3231 RTC Module

  /*
    Pin Setup (from Arduino UNO pinout):
    SDA A4
    SCL A5
    VCC is 5V
  */

  Serial.println("Initializing RTC Timer ...");
  if (!rtc.begin())
  {
    Serial.println(F("Couldn't find RTC. Please sure DS3231 RTC module is setup correctly"));
    //don't do anything anymore
    while (1)
      ;
  }
  Serial.println(F("RTC Timer initialized"));
  delay(1000);
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); //adjusts RTC time to current time
  Serial.println(F("RTC Timer sync with real time"));

  // -------------------------------------------

  // Initializing FONA

  Serial.println(F("Welcome to Sustaingineering 3G TxRx."));
  Serial.println(F("Launching...."));
  fonaSerial->begin(4800);
  while (!fona.begin(*fonaSerial))
  {
    Serial.println(F("Cannot find FONA. Please try rebooting.")); //reboot arduino and fona if this shows up! (Should probably do this automatically for robustness)
    delay(1000);
    while (1)
      ;
  }
  type = fona.type();
  Serial.println(F("FONA is OK!"));
  Serial.println(F("Searching for network...\n"));
  bool SIMFound = false;
  for (int countdown = 600; countdown >= 0 && SIMFound == false; countdown--)
  {
    uint8_t n = fona.getNetworkStatus(); // constantly check until network is connected to home    sendCheckReply(F("AT+CLVL="), i, ok_reply);
    if (n == 1)
    {
      SIMFound = true;
      Serial.print(F("Found!")); // If program hangs here, SIM card cannot be read/connect to network
      Serial.println(F("Network Connected"));
    }
  }
  if (!SIMFound)
  {
    Serial.println(F("SIM card could not be found. Please ensure that your SIM card is compatible with dual-band UMTS/HSDPA850/1900MHz WCDMA + HSDPA."));
    while (1)
      ;
  }

  // -------------------------------------------
}
void loop()
{
  // Construct Real Time string to be sent

  // -------------------------------------------

  // Sensing Data
  /*
   Analog Pin Assignments for Measurement Variables
   SourceVoltage: A1
   HallAmps: A0
   Power: calculation from code
   SolTemp: D2
   WaterBreakerFlag: assigned from code
   Real Time: SCL-A5 & SDA-A4
  */

  VoltageDivider(); // voltage sensing <- updates LoadVoltage
  HallEffect();     // current sensing <- updates HallAmps
  Thermolcouple();  // temperature sensing <- updates Temp
  Power = SourceVoltage * HallAmps;
  WaterBreakerFlag = false;

  delay(100); //not sure if this is necessary

  // -------------------------------------------
  
  //Sending SMS

  delay(10000);

  send_sms(SourceVoltage, HallAmps, Power, SolTemp, WaterBreakerFlag);
}

void send_sms(float LoadVoltage, float LoadCurrent, float Power, float SolTemp, bool WaterBreakerFlag)
{

  //rtc time

  DateTime now = rtc.now(); //have to do this inside send function for continous successful rtc time data sending

  //fix hour, minute, and seconds format since they are only integers in the RTC library
  // String rtc_hour = ((now.hour() >= 10) ? (String)(now.hour()) : "0" + (String)(now.hour()));
  // String rtc_minute = ((now.minute() >= 10) ? (String)(now.minute()) : "0" + (String)(now.minute()));
  // String rtc_second = ((now.second() >= 10) ? (String)(now.second()) : "0" + (String)(now.second()));

  // --------------

  // send an SMS!
  char message[0];

  String str;
  str = (String)(now.unixtime()) + "," + (String)(LoadVoltage) + "," + (String)(LoadCurrent) + "," + (String)(Power) + "," + (String)(SolTemp) + "," + (String)(WaterBreakerFlag);
//  str = (String)(LoadVoltage) + "," + (String)(LoadCurrent) + "," + (String)(Power) + "," + (String)(SolTemp) + "," + (String)(WaterBreakerFlag);

  Serial.print("str content: ");
  Serial.println(str);
  str.toCharArray(message, 141);

  Serial.print(F("Your message is: "));
  Serial.println(message);

  while (fona.sendSMS(sendto, message) == 0)
  {
    Serial.println(F("SMS sending failed."));
  }
  Serial.println(F("SMS sending succeeded."));
  Serial.println();
}

// -------------------------------------------

//Sensor Functions

// -------------------------------------------

// Voltage Divider Sensor
void VoltageDivider()
{
  // Read Voltage at divider and convert to decimal
  int x = analogRead(A1);
  DivVoltage = x * (5.0 / 1023.0);
  // Final Source Voltage reading
  SourceVoltage = (DivVoltage * (RH + RL)) / RL;
}

// -------------------------------------------

// Hall Effect sensor
void HallEffect()
{
  int x = analogRead(A0); // Take reading
  // Convert to decimal
  HallVoltage = x * (5.0 / 1023.0);
  // Compute the current from the voltage reading equation
  HallAmps = (HallVoltage * 22.0) / 3.0 - (55.0 / 3.0);
}

// -------------------------------------------

// Thermolcouple sensor
void Thermolcouple()
{
  float temperature = getTemp();
  Temp = (temperature - 32) * 5 / 9;
  SolTemp = ((Temp > 1000) ? 1000 : Temp); //this condition necessary to send for some reason (checks to makes sure thermocouple reading is within range to send)
}

float getTemp()
{
  //returns the temperature from one DS18S20 in DEG Celsius

  byte data[12];
  byte addr[8];

  if (!ds.search(addr))
  {
    //no more sensors on chain, reset search
    ds.reset_search();
    return -1000;
  }

  if (OneWire::crc8(addr, 7) != addr[7])
  {
    Serial.println("CRC is not valid!");
    return -1000;
  }

  if (addr[0] != 0x10 && addr[0] != 0x28)
  {
    Serial.print("Device is not recognized");
    return -1000;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1); // start conversion, with parasite power on at the end

  byte present = ds.reset();
  ds.select(addr);
  ds.write(0xBE); // Read Scratchpad

  for (int i = 0; i < 9; i++)
  { // we need 9 bytes
    data[i] = ds.read();
  }

  ds.reset_search();

  byte MSB = data[1];
  byte LSB = data[0];

  float tempRead = ((MSB << 8) | LSB); //using two's compliment
  float TemperatureSum = tempRead / 16;

  return (TemperatureSum * 18 + 5) / 10 + 32;
}
