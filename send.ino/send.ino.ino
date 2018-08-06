//By Rico Jia, May 16, 2018

//SG_FONA folder must be included in \Documents\Arduino\libraries
//Arduino must be set to port 1. Go to device manager -> Ports(COM & LPT) -> Arduino Uno -> Properties -> Port Settings -> Advanced -> COM Port Number. Then re-plug the USB.
//Serial may need to be set to 4800bps manually first. On my device, this was the default.
//Serial Monitor must be set to Both NL & CR, as well as 115200 Baud
//Many cell carriers do not support GSM. Only supported network may be Rogers (this will be shutting down soon)

//For more on interrupts, see https://www.robotshop.com/letsmakerobots/arduino-101-timers-and-interrupts
#include "SG_FONA.h"
volatile char ISR_Count=0;

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

char sendto[21] = "6472333143";   // IMPORTANT: Enter destination number here

// We default to using software serial. If you want to use hardware serial
// (because softserial isnt supported) comment out the following three lines 
// and uncomment the HardwareSerial line
#include <SoftwareSerial.h>
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX,FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;

// Hardware serial is also possible!
//  HardwareSerial *fonaSerial = &Serial1;

// Use this for FONA 800 and 808s
//Adafruit_FONA fona = Adafruit_FONA(FONA_RST);
// Use this one for FONA 3G
Adafruit_FONA_3G fona = Adafruit_FONA_3G(FONA_RST);

uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);    // Is it for reading from serial port?

uint8_t type;

volatile int8_t numsms;

void setup() {

  pinMode(FONA_POWER, OUTPUT);
  pinMode(FONA_RI, INPUT);
  digitalWrite(FONA_POWER, HIGH);
  delay(FONA_POWER_ON_TIME);
  digitalWrite(FONA_POWER, LOW);
  delay(3000);      

  while (!Serial);    // wait till serial gets initialized

  Serial.begin(115200); //Baud rate
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
  bool SIMFound=false;
  for (int countdown=600;countdown>=0&&SIMFound==false;countdown--){
    //Serial.print(F("Countdown: "));
    //Serial.println(countdown);
    uint8_t n = fona.getNetworkStatus();      // constantly check until network is connected to home    sendCheckReply(F("AT+CLVL="), i, ok_reply);
    if (n == 1) 
    {
      SIMFound=true;
      Serial.print(F("Found ")); //If program hangs here, SIM card cannot be read/connect to network
      Serial.println(F("Network Connected")); 
    }
  // uint8_t m = fona.setSMSInterrupt(1);    // this is for setting up the Ring Indicator Pin
  }
  if(!SIMFound)
  {
    Serial.println(F("SIM card could not be found. Please ensure that your SIM card is compatible with dual-band UMTS/HSDPA850/1900MHz WCDMA + HSDPA."));
    while(1){}
  }
}
void loop()
{
   Serial.println(F("Hit enter to wake up Fona"));
   while (! Serial.available() ) { 
       //Do other stuff
   }
  send_sms();   
} 

void send_sms()
   { // send an SMS!
        char message[141];
        bool loop=true;
        while(loop)
        {
          flushSerial();    // THIS IS IMPORTANT! OTHERWISE what you typed in might be missing in what you send
          Serial.println(sendto);
          //Temporary solution to demonstrate ability to text floats in .csv format
          Serial.print(F("Press 1 to enter a message, or 2 to enter data:\t"));
          readline(message, 140);
          Serial.print(F("\n\n"));
          if(message[0]=='1')
          {
            loop=false;
            Serial.print(F("Type out one-line message (140 char): "));
            readline(message, 140); //Gets message from console
          }
          else if(message[0]=='2')
          {
            loop=false;
            float LoadVoltage=0;
            float LoadCurrent=0;
            float Power=0;
            float AtmTemp=0;
            float SolTemp=0;
            bool WaterBreakerFlag=false;
            Serial.print(F("Enter Load Voltage:\t"));
            readline(message,140);
            LoadVoltage=atof(message);
            Serial.println(LoadVoltage);
            Serial.print(F("Enter Load Current:\t"));
            readline(message,140);
            LoadCurrent=atof(message);
            Serial.println(LoadCurrent);
            Serial.print(F("Enter Power:\t"));
            readline(message,140);
            Power=atof(message);
            Serial.println(Power);
            Serial.print(F("Enter Atmospheric Temperature:\t"));
            readline(message,140);
            AtmTemp=atof(message);
            Serial.println(AtmTemp);
            Serial.print(F("Enter Solar Panel Temperature:\t"));
            readline(message,140);
            SolTemp=atof(message);
            Serial.println(SolTemp);
            Serial.print(F("Enter Water Braker Flag (0/1):\t"));
            readline(message,140);
            WaterBreakerFlag=(int)(message[0]-'0');
            Serial.println(WaterBreakerFlag);
            Serial.print(F("\n"));
            
            String str;
            str=(String)(LoadVoltage)+","+(String)(LoadCurrent)+","+(String)(Power)+","+(String)(AtmTemp)+","+(String)(SolTemp)+","+(String)(WaterBreakerFlag);
            str.toCharArray(message, 141);
          }
        }
        Serial.print(F("Your message is: "));
        Serial.println(message);
        if (!fona.sendSMS(sendto, message)) {
          Serial.println(F("Failed"));
        } else {
          Serial.println(F("Sent!"));
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
