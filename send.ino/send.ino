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
// #include <SD.h>
// #include "SdFat.h"
#include "RTClib.h"
// #include <Wire.h>
#include <DHT.h>

// SdFat SD;

#define FONA_TX 4 //Soft serial port
#define FONA_RX 5 //Soft serial port
#define FONA_RI 3 //let's test it!
#define FONA_RST 9

#define FONA_POWER 8
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

char sendto[11] = "7789525137";

// We default to using software serial. If you want to use hardware serial
// (because softserial isnt supported) comment out the following three lines
// and uncomment the HardwareSerial line
#include <SoftwareSerial.h>
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
// SoftwareSerial sdSS = SoftwareSerial(6, 7);

// SoftwareSerial *sdSerial = &sdSS;
// SoftwareSerial *fonaSerial = &fonaSS;

// Hardware serial is also possible!
// HardwareSerial *fonaSerial = &Serial;

// Use this for FONA 800 and 808s
// Adafruit_FONA fona = Adafruit_FONA(FONA_RST);

// Use this one for FONA 3G
Adafruit_FONA_3G fona = Adafruit_FONA_3G(FONA_RST);

// uint8_t type;

// volatile int8_t numsms;

// // Declare all global variables
// // Voltage Sensor
// double DivVoltage = 0;    // Voltage divider reading
// double SourceVoltage = 0; // Final voltage reading result

// // Current Sensor
// double HallVoltage = 0; // Voltage reading of Hall Effect
// double HallAmps = 0;    // Current result from Hall Effect Sensor
// float Power = 0;

// // Temperature Sensor
// double TempVolt = 0; //previously volt
// double Temp = 0;     // previously temp
// float AtmTemp = 0;
// float SolTemp = 0;

// // Humidity Sensor
// double Humidity = 0; //humidity sensor value

// //Water Pump Sensor
// bool WaterBreakerFlag = false;

// Data Logging
// unsigned long Time;

// RTC time
// String timestamp_GSM = "";
RTC_DS3231 rtc;

// Humidity Sensor
/*
    Pin Setup (from Arduino UNO pinout):
    OUT D7
    VCC is 5V
*/
DHT dht(7, DHT11);

// Declare any global constants
double RH = 1000; //test on a different prototype board    //981300;   // Voltage Divider High Resistance
double RL = 1000; //test on a diffferent prototype board   //24743;    // Voltage Divider Low Resistance

void setup()
{

  pinMode(FONA_POWER, OUTPUT);
  digitalWrite(FONA_POWER, HIGH);
  delay(FONA_POWER_ON_TIME);
  digitalWrite(FONA_POWER, LOW);
  delay(3000);

  Serial.begin(4800); //baud rate

  // Wire.begin(); // Initiate the Wire library and join the I2C bus as a master or slave
  while (!Serial)
    ; // wait till serial gets initialized

  // //SD Initialization

  // /*
  //   Pin Setup (from Arduino UNO pinout):
  //   MISO D12
  //   MOSI D11
  //   SCK D13
  //   SS D10 (slave select or chip select)
  //   VCC is 5V
  // */
  // const int slaveSelect = 10;

  // Serial.println(F("Initializing SD card ..."));
  // if (!SD.begin(slaveSelect))
  // {
  //   Serial.println(F("SD card fail to initialize. Please ensure SD card is set up properly"));
  //   //don't do anything anymore
  //   while (1)
  //     ;
  // }
  // Serial.println(F("SD card initialized"));

  // -------------------------------------------

  // Initializing Humidity Sensor

  Serial.println(F("Initializing Humidity Sensor"));

  dht.begin();

  Serial.println(F("Humidity Sensor initialized"));

  // delay(1000);

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

  // delay(1000);

  rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); //adjusts RTC time to current time

  // Serial.println(F("RTC Timer sync with real time"));

  // -------------------------------------------

  // Initializing FONA

  Serial.println(F("Welcome to Sustaingineering 3G TxRx."));
  Serial.println(F("Launching...."));

  fonaSS.begin(4800);
  while (!fona.begin(fonaSS))
  {
    Serial.println(F("Cannot find FONA. Please try rebooting.")); //reboot arduino and fona if this shows up! (Should probably do this automatically for robustness)
    delay(1000);
  }
  // type = fona.type();
  Serial.println(F("FONA is OK!"));

  Serial.println(F("Searching for network...\n"));
  bool SIMFound = false;
  for (int countdown = 600; countdown >= 0 && SIMFound == false; countdown--)
  {
    //Serial.print(F("Countdown: "));
    //Serial.println(countdown);
    uint8_t n = fona.getNetworkStatus(); // constantly check until network is connected to home    sendCheckReply(F("AT+CLVL="), i, ok_reply);
    if (n == 1)
    {
      SIMFound = true;
      Serial.print(F("Found!")); // If program hangs here, SIM card cannot be read/connect to network
      Serial.println(F("Network Connected"));
      break;
    }
    // uint8_t m = fona.setSMSInterrupt(1);    // this is for setting up the Ring Indicator Pin
  }
  if (!SIMFound)
  {
    Serial.println(F("SIM card could not be found. Please ensure that your SIM card is compatible with dual-band UMTS/HSDPA850/1900MHz WCDMA + HSDPA."));
    while (1)
    {
    }
  }

  // rtc.begin(); // Initialize the rtc object
}

void loop()
{
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

  // Humidity Sensor
  double Humidity = 0; //humidity sensor value

  //Water Pump Sensor
  bool WaterBreakerFlag = false;
  // DateTime now = rtc.now();

  // Serial.print(now.year(), DEC);
  // Serial.print('/');
  // Serial.print(now.month(), DEC);
  // Serial.print('/');
  // Serial.print(now.day(), DEC);
  // Serial.print(",");
  // Serial.print(now.hour(), DEC);
  // Serial.print(':');
  // Serial.print(now.minute(), DEC);
  // Serial.print(':');
  // Serial.print(now.second(), DEC);
  // Serial.println();

  // for SolTemp
  float sensorValue = analogRead(A2);
  // Convert the analog reading ( which goes from 0 - 1023) to a voltage (0 - 5V):
  float voltage = (sensorValue / 1023) * 5000;

  float volt = voltage - 1670; //(while cold conjuction is 22 degree
  //   the voltage cross thermalcouple is 1.67v)
  volt = volt / 150;                //(opamp apmplified 150 times)
  float temp = (volt) / 0.041 + 22; //( 1 degree = 0.0404 mv in K type )

  // //Serial.println(temp);

  SourceVoltage = VoltageDivider(); // voltage sensing <- updates LoadVoltage
  HallAmps = HallEffect();          // current sensing <- updates HallAmps
  AtmTemp = Thermolcouple();        // temperature sensing <- updates Temp
  Humidity = HumiditySense();       // humidity sensing <- updates humidity editor

  //  float LoadVoltage=-1;
  //  float LoadCurrent=-1;
  //  float Power=-1;

  Power = SourceVoltage * HallAmps;

  // float AtmTemp = 100;
  SolTemp = temp; //I have to set SolTemp to some variable (haven't tried 100.0 though)

  // AtmTemp = Temp;
  // SolTemp = temp; //when i print this, it does not work
  WaterBreakerFlag = false;

  /*
     Analog Pin Assignments for Measurement Variables
     SourceVoltage: A1
     HallAmps: A0
     Power: calculation from code
     AtmTemp: A2MM
     SolTemp: A2
     WaterBreakerFlag: assigned from code
  */
  //  delay(100); //not sure if this is necessary

  // this condition necessary to send for some reason (checks to makes sure thermocouple reading is within range to send)
  if (SolTemp > 1000)
  {
    SolTemp = 1000;
  }
  //Storing data into SD card

  // SDLog(SourceVoltage, HallAmps, Power, AtmTemp, SolTemp, Humidity, WaterBreakerFlag);

  Serial.println(F("sd card data saved"));

  // SDRead();

  //Sending SMS

  // 5000ms eliminates initial sending failures for RTC timer and Fona SMS; also works for opening RTC timer and hopefully writing date and time in file
  delay(2000);
  // delay(1000); //this makes the initial send message to have no failures (a delay of 1800 works before as well) ****apparently a Defined constant on top does not work as the way it should

  //when plug and unplug the USB blaster from the Arduino to the laptop,
  //Serial monitor will stop printing while the GSM sheild will take some time to continue sending messages periodically
  send_sms(SourceVoltage, HallAmps, Power, AtmTemp, SolTemp, Humidity, WaterBreakerFlag);
}

void send_sms(float LoadVoltage, float LoadCurrent, float Power, float AtmTemp, float SolTemp, float Humidity, bool WaterBreakerFlag)
{ // send an SMS!

  Serial.println(F("Start send"));
  // char *message;
  // flushSerial(); // THIS IS IMPORTANT! OTHERWISE what you typed in might be missing in what you send
  // Serial.println("Serial Flushed");

  // fona.flush();
  // fona.flushInput();
  //          Serial.print("LoadVoltage: ");
  //          Serial.println((String)LoadVoltage);

  // String concat
  String str = (String)(LoadVoltage) + "," + (String)(LoadCurrent) + "," + (String)(Power) + "," + (String)(AtmTemp) + "," + (String)(SolTemp) + "," + (String)(Humidity) + "," + (String)(WaterBreakerFlag);
  //          message[str.length()+1];
  //          Serial.println("str made");
  // message = (char*)malloc(str.length());
  Serial.print(F("str content: "));
  Serial.println(str);

  // str.toCharArray(message, str.length());

  //          Serial.println("Message Made");

  //        Serial.println("While(loop) out");

  // delay(100000);

  // Serial.print(F("Your message is: "));
  // Serial.println(str);
  // Serial.print(F("Value of message sent status: "));

  //        Serial.println("status printed");
  if (!fona.sendSMS(sendto, (char *)str.c_str()))
  {
    Serial.println(F("SMS sending failed."));
  }
  else
  {
    Serial.println(F("SMS sending succeeded."));
  }
}

void flushSerial()
{
  while (Serial.available())
    Serial.read();
}

// Humidity Sensor

double HumiditySense()
{
  return dht.readHumidity();
  // delay(2000);
}

// Voltage Divider Sensor
double VoltageDivider()
{
  // Read Voltage at divider and convert to decimal
  int x = analogRead(A1);
  double DivVoltage = x * (5.0 / 1023.0);

  // Final Source Voltage reading
  return (DivVoltage * (RH + RL)) / RL;
}

// Hall Effect sensor
double HallEffect()
{
  int x = analogRead(A0); // Take reading

  // Convert to decimal
  double HallVoltage = x * (5.0 / 1023.0);

  // Compute the current from the voltage reading
  // Equation: ...
  double HallAmps = (HallVoltage * 22.0) / 3.0 - (55.0 / 3.0);

  return HallAmps;
}

// Thermolcouple sensor
double Thermolcouple()
{
  // Read sensor value and convert to mV
  int x = analogRead(A2);
  double TempVolt = x * (5000.0 / 1023.0) + 25;

  // Check if upper voltage bound
  if (TempVolt > 2500)
  {
    double volt = TempVolt - 2500; // Assuming cold junction at 22 degree
    volt = volt / 123;             // OpAmp amplified 150 times
    return (volt / 0.041) + 25;    // 0.0404 mv/degree in K type
  }

  // Else if lower bound
  else
  {
    double volt = 2500 - TempVolt; // Assuming cold junction at 22 degree
    // The voltage cross thermolcouple is 1.67v)
    volt = volt / 123;           // OpAmp apmplified 150 times
    return 25 - (volt / 0.0404); // 0.0404 mv/degree in K type
  }
}

// //For writing to SD (note file will be called "test155.txt") //limited to 7 characters before ".csv"
// void SDLog(float LoadVoltage, float LoadCurrent, float Power, float AtmTemp, float SolTemp, float Humidity, bool WaterBreakerFlag)
// {
//   // Open test file
//   // The file name testNUM is the text file we write to
//   //  String filename = "test155";
//   //  String file_format = ".txt";
//   //  filename = filename + file_format;

//   File myFile = SD.open("t2.csv", FILE_WRITE);

//   // if the file opened okay, write to it
//   if (myFile)
//   {
//     Serial.print(F("Writing to "));
//     //    Serial.print(filename);
//     Serial.print(F(" ... "));
//     //-------------------------------------------------------
//     //-------------------------------------------------------

//     // Record Date and Time --> YYYY/MM/DD,hh:mm:ss,
//     DateTime now = rtc.now();

//     myFile.print(now.year(), DEC);
//     myFile.print('/');
//     myFile.print(now.month(), DEC);
//     myFile.print('/');
//     myFile.print(now.day(), DEC);
//     myFile.print(",");
//     myFile.print(now.hour(), DEC);
//     myFile.print(':');
//     myFile.print(now.minute(), DEC);
//     myFile.print(':');
//     myFile.print(now.second(), DEC);
//     myFile.print(",");

//     Serial.println(F("RTC done."));

//     // Serial.println();
//     // Serial.println(now.year(), DEC);
//     // Serial.println(now.month(), DEC);
//     // Serial.println(now.day(), DEC);
//     // Serial.println(now.hour(), DEC);
//     // Serial.println(now.minute(), DEC);
//     // Serial.println(now.second(), DEC);

//     // Record panel voltage
//     myFile.print(LoadVoltage);
//     myFile.print(",");

//     // Record panel current
//     myFile.print(LoadCurrent);
//     myFile.print(",");

//     // Record panel power
//     myFile.print(Power);
//     myFile.print(",");

//     // Record atmospheric temperature
//     myFile.print(AtmTemp);
//     myFile.print(",");

//     // Record solar panel temperature
//     myFile.print(SolTemp);
//     myFile.print(",");

//     // Record Humidity
//     myFile.print(Humidity);
//     myFile.print(",");

//     // Record water breaker flag
//     myFile.println(WaterBreakerFlag);

//     Serial.println(F("Data done."));

//     // // close the file:
//     // myFile.close();
//     // Serial.println("done");

//     // //Serial print data values for checking

//     // Serial.println(SourceVoltage);
//     // Serial.println(HallAmps);
//     // Serial.println(Power);
//     // Serial.println(AtmTemp);
//     // Serial.println(SolTemp);
//     // Serial.println(Humidity);
//     // Serial.println(WaterBreakerFlag);
//     // Serial.println();

//     // String str;
//     // str = String(SourceVoltage, 2) + "," + (String)(HallAmps) + "," + (String)(Power) + "," + (String)(AtmTemp) + "," + (String)(SolTemp) + "," + (String)(WaterBreakerFlag);
//     // myFile.println(str);

//     delay(1000);
//     myFile.close();
//     Serial.println(F("done"));
//   }
//   else
//   {
//     // if the file didn't open, print an error:
//     Serial.println(F("error opening test file"));
//     //    Serial.println(filename);
//   }
// }

// void SDRead()
// {
//   // Open test file
//   // The file name testNUM is the text file we write to
//   //  String filename = "test155";
//   //  String file_format = ".txt";
//   //  filename = filename + file_format;

//   File myFile = SD.open("t2.csv");

//   // if the file opened okay, write to it
//   if (myFile)
//   {
//     Serial.println("t2.csv:");

//     // read from the file until there's nothing else in it:
//     while (myFile.available())
//     {
//       Serial.write(myFile.read());
//     }
//     // close the file:
//     myFile.close();
//   }
//   else
//   {
//     // if the file didn't open, print an error:
//     Serial.println(F("error opening test file"));
//     //    Serial.println(filename);
//   }
// }