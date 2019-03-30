/*#include <BlockDriver.h>
  #include <FreeStack.h>
  #include <MinimumSerial.h>
  #include <SdFat.h>
  #include <SdFatConfig.h>
  #include <sdios.h>
  #include <SysCall.h>
*/

//#include <SD.h>

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
/*  #include <EEPROM_R_W.h>
    EEPROM_R_W eeprom = EEPROM_R_W(); */

// Header files
#include <SPI.h>
#include "SdFat.h"
#include <Wire.h>
#include "DS3231.h"
//Do not need this
//#include <SD.h>

RTClib RTC;
SdFat SD;

#define FONA_TX 4 //Soft serial port
#define FONA_RX 5 //Soft serial port
#define FONA_RI 3 //let's test it!
#define FONA_RST 9

#define FONA_POWER 8
#define FONA_POWER_ON_TIME  180  /* 180ms*/
#define FONA_POWER_OFF_TIME 1000 /* 1000ms*/

char sendto[21] = "6043674089";//"6047283793"; //"7787722839";   // (604)367-4089 (number of receiving arduino); IMPORTANT: Enter destination number here

// We default to using software serial. If you want to use hardware serial
// (because softserial isnt supported) comment out the following three lines
// and uncomment the HardwareSerial line
#include <SoftwareSerial.h>
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial sdSS = SoftwareSerial(6, 7);

SoftwareSerial *sdSerial = &sdSS;
SoftwareSerial *fonaSerial = &fonaSS;

// Hardware serial is also possible!
// HardwareSerial *fonaSerial = &Serial;

// Use this for FONA 800 and 808s
// Adafruit_FONA fona = Adafruit_FONA(FONA_RST);
// Use this one for FONA 3G
Adafruit_FONA_3G fona = Adafruit_FONA_3G(FONA_RST);

uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);    // Is it for reading from serial port?

uint8_t type;

volatile int8_t numsms;

// Declare all global variables
// Voltage Sensor
double DivVoltage = 0;       // Voltage divider reading
double SourceVoltage = 0;        // Final voltage reading result

// Current Sensor
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
double RH = 1000;//test on a different prototype board    //981300;   // Voltage Divider High Resistance
double RL = 1000;//test on a diffferent prototype board   //24743;    // Voltage Divider Low Resistance

void setup() {

  // Defining LED Interface pins
  pinMode(2, OUTPUT);   // Arduino On LED
  pinMode(3, OUTPUT);

  // Voltage LED
  pinMode(5, OUTPUT);   // Current LED
  pinMode(7, OUTPUT);   // Relay Open/Close LED
  pinMode(9, OUTPUT);   // Relay Control Pin

  pinMode(FONA_POWER, OUTPUT);
  pinMode(FONA_RI, INPUT);
  digitalWrite(FONA_POWER, HIGH);
  delay(FONA_POWER_ON_TIME);
  digitalWrite(FONA_POWER, LOW);
  delay(3000);

  Serial.begin(4800); //baud rate

  Wire.begin();       // Initiate the Wire library and join the I2C bus as a master or slave
  while (!Serial);    // wait till serial gets initialized

  //SD Initialization
  //  Serial.print("Initializing SD card...");

  /*  sdSerial->begin(4800);
    if (!SD.begin(10)) {
      Serial.println("initialization SD failed!");
      while (1);
    }
    Serial.println("initialization done.");
  */
  Serial.println(F("Welcome to Sustaingineering 3G TxRx."));
  Serial.println(F("Launching...."));

  fonaSerial->begin(4800);
  while (!fona.begin(*fonaSerial)) {
    Serial.println(F("Cannot find FONA. Please try rebooting."));     //reboot arduino and fona if this shows up! (Should probably do this automatically for robustness)
    delay(1000);
  }
  type = fona.type();
  Serial.println(F("FONA is OK!"));

  Serial.println(F("Searching for network...\n"));
  bool SIMFound = false;
  for (int countdown = 600; countdown >= 0 && SIMFound == false; countdown--) {
    //Serial.print(F("Countdown: "));
    //Serial.println(countdown);
    uint8_t n = fona.getNetworkStatus();      // constantly check until network is connected to home    sendCheckReply(F("AT+CLVL="), i, ok_reply);
    if (n == 1)
    {
      SIMFound = true;
      Serial.print(F("Found!")); // If program hangs here, SIM card cannot be read/connect to network
      Serial.println(F("Network Connected"));
    }
    // uint8_t m = fona.setSMSInterrupt(1);    // this is for setting up the Ring Indicator Pin
  }
  if (!SIMFound)
  {
    Serial.println(F("SIM card could not be found. Please ensure that your SIM card is compatible with dual-band UMTS/HSDPA850/1900MHz WCDMA + HSDPA."));
    while (1) {}
  }
}


void loop()
{
  //for SolTemp
  float sensorValue = analogRead(A2);
  // Convert the analog reading ( which goes from 0 - 1023) to a voltage (0 - 5V):
  float voltage = voltage / 1023;
  voltage = sensorValue * 5000; //(in mv)

  float volt = voltage - 1670; //(while cold conjuction is 22 degree
  //   the voltage cross thermalcouple is 1.67v)
  volt =  volt / 150; //(opamp apmplified 150 times)
  float temp = (volt) / 0.041 + 22; //( 1 degree = 0.0404 mv in K type )

  //Serial.println(temp);

  VoltageDivider(); //voltage sensing <- updates LoadVoltage
  HallEffect(); //current sensing <- updates HallAmps
  Thermolcouple(); //temperature sensing <- updates Temp

  //  float LoadVoltage=-1;
  //  float LoadCurrent=-1;
  //  float Power=-1;

  float Power = SourceVoltage * HallAmps;

  //  float AtmTemp = 100;
  float SolTemp = temp; //I have to set SolTemp to some variable (haven't tried 100.0 though)

  float AtmTemp = Temp;
  //  float SolTemp=temp; //when i print this, it does not work
  bool WaterBreakerFlag = false;

  /*
     Analog Pin Assignments for Measurement Variables
     SourceVoltage: A1
     HallAmps: A0
     Power: calculation from code
     AtmTemp: A2
     SolTemp: A2
     WaterBreakerFlag: assigned from code
  */
  //  delay(100); //not sure if this is necessary

  //this condition necessary to send for some reason (checks to makes sure thermocouple reading is within range to send)
  if (SolTemp > 1000) {
    SolTemp = 1000;
  }
  send_sms(SourceVoltage, HallAmps, Power, AtmTemp, SolTemp, WaterBreakerFlag);

  //SDLog();

  //  send_sms(LoadVoltage, LoadCurrent,Power,AtmTemp,SolTemp,WaterBreakerFlag);

}

void send_sms(float LoadVoltage, float LoadCurrent, float Power, float AtmTemp, float SolTemp, bool WaterBreakerFlag)
{ // send an SMS!

  //        Serial.println("Start send");
  char message[0];
  bool loop = true;
  while (loop)
  {
    flushSerial();    // THIS IS IMPORTANT! OTHERWISE what you typed in might be missing in what you send
    //          Serial.println("Serial Flushed");
    loop = false;

    //          Serial.print("LoadVoltage: ");
    //          Serial.println((String)LoadVoltage);

    String str;
    str = (String)(LoadVoltage) + "," + (String)(LoadCurrent) + "," + (String)(Power) + "," + (String)(AtmTemp) + "," + (String)(SolTemp) + "," + (String)(WaterBreakerFlag);

    //          message[str.length()+1];
    //          Serial.println("str made");
    //          Serial.print("str content: ");
    //          Serial.println(str);

    str.toCharArray(message, 141);

    //          Serial.println("Message Made");
  }

  //        Serial.println("While(loop) out");

  Serial.print(F("Your message is: "));
  Serial.println(message);
  Serial.print(F("Value of message sent status: "));

  //        Serial.println("status printed");
  while (fona.sendSMS(sendto, message) == 0)
  {
    Serial.println(F("SMS sending failed."));
  }

  Serial.println(F("SMS sending succeeded."));


  Serial.println();
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



// Voltage Divider Sensor
void VoltageDivider() {
  // Read Voltage at divider and convert to decimal
  int x = analogRead(A1);
  DivVoltage = x * (5.0 / 1023.0);

  // Final Source Voltage reading
  SourceVoltage = (DivVoltage * (RH + RL)) / RL;
}

// Hall Effect sensor
void HallEffect() {
  int x = analogRead(A0);     // Take reading

  // Convert to decimal
  HallVoltage = x * (5.0 / 1023.0);

  // Compute the current from the voltage reading
  // Equation: ...
  HallAmps = (HallVoltage * 22.0) / 3.0 - (55.0 / 3.0);
}

// Thermolcouple sensor
void Thermolcouple() {
  // Read sensor value and convert to mV
  int x = analogRead(A2);
  TempVolt = x * (5000.0 / 1023.0) + 25;

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


//For writing to SD (note file will be called "test155.txt")
void SDLog() {
  // Open test file
  // The file name testNUM is the text file we write to
  myFile = SD.open("timertestreal15.txt", FILE_WRITE);

  // if the file opened okay, write to it
  if (myFile) {
    // Record time
    //DateTime now = RTC.now();
    myFile.print("Time (s) = ");
    //   myFile.print(now.year(), DEC);
    myFile.print('/');
    // myFile.print(now.month(), DEC);
    myFile.print('/');
    //myFile.print(now.day(), DEC);
    myFile.print(' ');
    //myFile.print(now.hour(), DEC);
    myFile.print(':');
    //if(now.minute() < 10){
    //myFile.print('0');
    //}
    //myFile.print(now.minute(), DEC);
    myFile.print(':');
    //if(now.second() < 10){
    //myFile.print('0');
    //}
    //myFile.print(now.second(), DEC);

    // Record divider voltage
    myFile.print("\t Divider Voltage = ");
    myFile.print(DivVoltage);

    // Record panel voltage
    myFile.print("\t Source Voltage = ");
    myFile.print(SourceVoltage);

    // Record hall effect voltage signal
    myFile.print("\t Hall Voltage = ");
    myFile.print(HallVoltage);

    // Record panel current
    myFile.print("\t Panel Current = ");
    myFile.print(HallAmps);

    // Record temperature (may add another)
    myFile.print("\t Panel Temperature = ");
    myFile.println(Temp);

    // close the file:
    myFile.close();
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening timertest10.txt");
  }
}
