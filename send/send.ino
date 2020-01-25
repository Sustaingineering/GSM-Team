//SG_FONA folder must be included in \Documents\Arduino\libraries
//Arduino must be set to port 1. Go to device manager -> Ports(COM & LPT) -> Arduino Uno -> Properties -> Port Settings -> Advanced -> COM Port Number. Then re-plug the USB.
//Serial may need to be set to 4800bps manually first. On my device, this was the default.
//Serial Monitor must be set to Both NL & CR, as well as 115200 Baud
//Many cell carriers do not support GSM. Only supported network may be Rogers (this will be shutting down soon)
//For more on interrupts, see https://www.robotshop.com/letsmakerobots/arduino-101-timers-and-interrupts
#include "SG_FONA.h"
volatile char ISR_Count = 0;
//Optional header file to enable I/O to the EEPROM memory
/*  #include <EEPROM_R_W.h>
    EEPROM_R_W eeprom = EEPROM_R_W(); */
// Other Header files
#include <SPI.h>
#include "RTClib.h"
#include <Wire.h>

//IMPORTANT: either use 2 & 3 or 4 & 5 for the TX and RX respectively for Software Serial, based on the connector pins on the GSM
#define FONA_TX 4 //Soft serial port
#define FONA_RX 5 //Soft serial port

#define FONA_RST 9//9 //just need a digital pin......
#define FONA_POWER 10//8 //this has to be the clock 0 pin
#define FONA_POWER_ON_TIME 180   /* 180ms*/
#define FONA_POWER_OFF_TIME 1000 /* 1000ms*/
#define FONA_SINGLE_MESSAGE_DELAY_TIME 1000

/*
List of Phone Numbers: 
7789525137 - Forbes
6047283793 - Abdul
7789391063 - GSM1
7789391268 - GSM2
*/
char sendto[21] = "7789525137";
// We default to using software serial. If you want to use hardware serial
// (because softserial isnt supported) comment out the following three lines
// and uncomment the HardwareSerial line
#include <SoftwareSerial.h>
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;
// Use this one for FONA 3G
Adafruit_FONA_3G fona = Adafruit_FONA_3G(FONA_RST);
uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0); // Is it for reading from serial port?
uint8_t type;
volatile int8_t numsms;

// Declare all global variables

// Voltage Sensor
double DivVoltage = 0;    // Voltage divider reading
double SourceVoltage = 0; // Final voltage reading result
// Current Sensor
double HallVoltage = 0; // Voltage reading of Hall Effect
double HallAmps = 0;    // Current result from Hall Effect Sensor
float Power = 0;

// Temperature Sensor
double TempVolt = 0; //previously volt
double Temp = 0;     // previously temp
float AtmTemp = 0;
float SolTemp = 0;

//Water Pump Sensor
bool WaterBreakerFlag = false;

// Data Logging
// File myFile;
unsigned long Time;

// RTC time
// String timestamp_GSM = "";
RTC_DS3231 rtc;

// Declare any global constants
double RH = 1000; //test on a different prototype board    //981300;   // Voltage Divider High Resistance
double RL = 1000; //test on a diffferent prototype board   //24743;    // Voltage Divider Low Resistance

void setup()
{
  //helps setting up the fona library
  pinMode(FONA_POWER, OUTPUT);
  digitalWrite(FONA_POWER, HIGH);
  delay(FONA_POWER_ON_TIME);
  digitalWrite(FONA_POWER, LOW);
  delay(3000);
  while (!Serial)
    ;                 // wait till serial gets initialized
  Serial.begin(4800); //baud rate

  Wire.begin(); // Initiate the Wire library and join the I2C bus as a master or slave

  // -------------------------------------------
  // Initializing DS3231 RTC
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
}
void loop()
{
  // Construct Real Time string

  // -------------------------------------------
  // Sensing Data
  /*
   Analog Pin Assignments for Measurement Variables
   SourceVoltage: A1
   HallAmps: A0
   Power: calculation from code
   AtmTemp: A2
   SolTemp: A2
   WaterBreakerFlag: assigned from code
  */

  //intermediate data value collected by arduino (will be used to calculated our final measured data values)

  float sensorValue = analogRead(A2);

  // Convert the analog reading ( which goes from 0 - 1023) to a voltage (0 - 5V):
  float voltage = sensorValue / 1023;
  voltage = sensorValue * 5000; //(in mv)
  float volt = voltage - 1670;  //(while cold conjuction is 22 degree

  // the voltage cross thermalcouple is 1.67v)
  volt = volt / 150; //(opamp apmplified 150 times)

  // ------------

  //assigning all measured data

  float temp = (volt) / 0.041 + 22; //( 1 degree = 0.0404 mv in K type )
  VoltageDivider();                 // voltage sensing <- updates LoadVoltage
  HallEffect();                     // current sensing <- updates HallAmps
  Thermolcouple();                  // temperature sensing <- updates Temp
  Power = SourceVoltage * HallAmps;
  SolTemp = ((temp > 1000) ? 1000 : temp); //this condition necessary to send for some reason (checks to makes sure thermocouple reading is within range to send)
  AtmTemp = Temp;
  WaterBreakerFlag = false;

  delay(100); //not sure if this is necessary

  // -------------------------------------------
  //Sending SMS

  delay(1000);

  send_sms(SourceVoltage, HallAmps, Power, AtmTemp, SolTemp, WaterBreakerFlag);
}

void send_sms(float LoadVoltage, float LoadCurrent, float Power, float AtmTemp, float SolTemp, bool WaterBreakerFlag)
{

  //rtc time

  DateTime now = rtc.now(); //have to do this inside send function for continous successful rtc time data sending

  //fix hour, minute, and seconds format since they are only integers in the RTC library
  String rtc_hour = ((now.hour() >= 10) ? (String)(now.hour()) : "0" + (String)(now.hour()));
  String rtc_minute = ((now.minute() >= 10) ? (String)(now.minute()) : "0" + (String)(now.minute()));
  String rtc_second = ((now.second() >= 10) ? (String)(now.second()) : "0" + (String)(now.second()));

  // --------------

  // send an SMS!
  char message[0];

  String str;
  str = (String)(now.year()) + "/" + (String)(now.month()) + "/" + (String)(now.day()) + "-" + rtc_hour + ":" + rtc_minute + ":" + rtc_second + "," + (String)(LoadVoltage) + "," + (String)(LoadCurrent) + "," + (String)(Power) + "," + (String)(AtmTemp) + "," + (String)(SolTemp) + "," + (String)(WaterBreakerFlag);

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

// Voltage Divider Sensor
void VoltageDivider()
{
  // Read Voltage at divider and convert to decimal
  int x = analogRead(A1);
  //  DivVoltage = x * (5.0 / 1023.0);
  //  // Final Source Voltage reading
  //  SourceVoltage = (DivVoltage * (RH + RL)) / RL;
}

// Hall Effect sensor
void HallEffect()
{
  int x = analogRead(A0); // Take reading
  // Convert to decimal
  HallVoltage = x * (5.0 / 1023.0);
  // Compute the current from the voltage reading equation
  HallAmps = (HallVoltage * 22.0) / 3.0 - (55.0 / 3.0);
}

// Thermolcouple sensor
void Thermolcouple()
{
  // Read sensor value and convert to mV
  int x = analogRead(A2);
  TempVolt = x * (5000.0 / 1023.0) + 25;
  // Check if upper voltage bound
  if (TempVolt > 2500)
  {
    double volt = TempVolt - 2500; // Assuming cold junction at 22 degree
    volt = volt / 123;             // OpAmp amplified 150 times
    Temp = (volt / 0.041) + 25;    // 0.0404 mv/degree in K type
  }
  // Else if lower bound
  else
  {
    double volt = 2500 - TempVolt; // Assuming cold junction at 22 degree
    // The voltage cross thermolcouple is 1.67v)
    volt = volt / 123;           // OpAmp apmplified 150 times
    Temp = 25 - (volt / 0.0404); // 0.0404 mv/degree in K type
  }
}
