  //By Rico Jia, May 16, 2018

//SG_FONA folder must be included in \Documents\Arduino\libraries
//Arduino must be set to port 1. Go to device manager -> Ports(COM & LPT) -> Arduino Uno -> Properties -> Port Settings -> Advanced -> COM Port Number. Then re-plug the USB.
//Serial may need to be set to 4800bps manually first. On my device, this was the default.
//Serial Monitor must be set to Both NL & CR, as well as 115200 Baud
//Many cell carriers do not support GSM. Only supported network may be Rogers (this will be shutting down soon)

//For more on interrupts, see https://www.robotshop.com/letsmakerobots/arduino-101-timers-and-interrupts
#include "SG_FONA.h"

//The SD library uses a lot of memory (30 percentage points). Usure how to resolve this
#include <SD.h> //Because baud rate is set very high, a high quality SD card (less than 100ms write latency)
#include <SPI.h>

File data;

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
char replybuffer[255];   // this is a large buffer for replies
int timecounts = 0;
int last_timecounts = 0;

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
      numsms = fona.getNumSMS();
      if (numsms != -1)  
      {
        Serial.print(F("Number of messages: "));
        Serial.println(numsms);
        break;
      }      
    }
  // uint8_t m = fona.setSMSInterrupt(1);    // this is for setting up the Ring Indicator Pin
  }
  if(!SIMFound)
  {
    Serial.println(F("SIM card could not be found. Please ensure that your SIM card is compatible with dual-band UMTS/HSDPA850/1900MHz WCDMA + HSDPA."));
    while(1){}
  }

  /*Serial.print("Initializing SD card...");

  if (!SD.begin(4)) { //This sets the cspin  to pin 4. This will have to be changed when the module is attatched
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");*/
}

void loop()
{
   //Serial.println(F("Waiting for SMS"));
   while (! Serial.available() ) { 
    if(time_sms())
    {
      check_get_sms();
    }
   }  
   //Do other stuff
} 
  

boolean time_sms(){  // use millis to check the number of sms every second. 
  timecounts = millis();
  if (timecounts > last_timecounts + 1000) {
    last_timecounts = timecounts;
     return 1;
    }
  else return 0;
 }

void check_get_sms()
{
    if(numsms != fona.getNumSMS())
    {
      flushSerial();
      numsms = fona.getNumSMS();
      uint8_t smsn = numsms - 1;  // the sms# starts from 0
      if (! fona.getSMSSender(smsn, replybuffer, 250)) {
          Serial.println("Failed!");
          return;
        }
        Serial.print(F("FROM: ")); Serial.println(replybuffer);

        // Retrieve SMS value.
        uint16_t smslen;
        if (! fona.readSMS(smsn, replybuffer, 250, &smslen)) { // pass in buffer and max len!
          Serial.println("Failed!");
          return;
        }

        /*
         * Data is sent in csv format. The data will appear as follows:
         *    float LoadVoltage, float LoadCurrent, float Power, float AtmTemp, float SolTemp, bool WaterBreakerFlag
         */
        
        //Retrieve delimited values for use
        int data_length=10;
        char* vars[data_length];
        char* labels[data_length]={"Number", "Date", "Time", "ID", "Load Voltage", "Load Current", "Power", "Atmospheric Temperature", "Solar Panel Temperature", "Water Breaker Flag"};
        vars[0]=strtok(replybuffer,",");
        for(int i=1;i<data_length;i++)
        {
          vars[i]=strtok(NULL,",");
        }
        for(int i=0;i<data_length;i++)
        {
          Serial.print(labels[i]);
          Serial.print(F(": "));
          Serial.println(vars[i]);
        }
        
/*
        //Read file from memory
        //File is overwritten when writing to SD, so we must first retrieve the current contents of the file
        char* data_inmem="";
        data=SD.open("data.txt");
        if(data)
        {
          while(data.available())
          {
            strcat(data_inmem,data.read());
          }
          data.close();
        }
        else {
          // if the file didn't open, print an error:
          Serial.println("error opening data.txt");
        }


        //Append the new values to the old values, terminated with a new line
        strcat(data_inmem,replybuffer);
        strcat(data_inmem,"\n");

        //Write the full data back to the SD card
        data = SD.open("data.txt", FILE_WRITE);

        // if the file opened okay, write to it:
        if (data) {
          data.println(data_inmem);
          // close the file:
          data.close();
        } else {
          // if the file didn't open, print an error:
          Serial.println("error opening data.txt");
        }
        */

        
        
        Serial.print(" ("); Serial.print(smslen); 
        Serial.println(F("*****"));
        Serial.println(F("Waiting for next SMS"));   // this is to promp user to send data
        return;
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
