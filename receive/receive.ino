// 22nd June,2019

//SG_FONA folder must be included in \Documents\Arduino\libraries
//Arduino must be set to port 1. Go to device manager -> Ports(COM & LPT) -> Arduino Uno -> Properties -> Port Settings -> Advanced -> COM Port Number. Then re-plug the USB.
//Serial may need to be set to 4800bps manually first. On my device, this was the default.
//Serial Monitor must be set to Both NL & CR, as well as 115200 Baud
//Many cell carriers do not support GSM. Only supported network may be Rogers (this will be shutting down soon)

//For more on interrupts, see https://www.robotshop.com/letsmakerobots/arduino-101-timers-and-interrupts
#include "SG_FONA.h"
#include <SoftwareSerial.h>

//The SD library uses a lot of memory (30 percentage points). Usure how to resolve this
#include <SD.h> //Because baud rate is set very high, a high quality SD card (less than 100ms write latency)
#include <SPI.h>

File data;

//IMPORTANT: either use 2 & 3 or 4 & 5 for the TX and RX respectively for Software Serial!
#define FONA_TX 4 //Soft serial port 
#define FONA_RX 5 //Soft serial port
#define FONA_RST 9

#define FONA_POWER 8
#define FONA_POWER_ON_TIME 180   /* 180ms*/
#define FONA_POWER_OFF_TIME 1000 /* 1000ms*/

char replybuffer[255]; // this is a large buffer for replies
int timecounts = 0;
int last_timecounts = 0;

SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;
Adafruit_FONA_3G fona = Adafruit_FONA_3G(FONA_RST);

uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0); // Is it for reading from serial port?

uint8_t type;

volatile int8_t numsms;

//we can try to delete all the previously stored messages in the GSM sim card during the set up before
//we run the code/program. Then depending on the need, we can just keep deleting the recent message
//that was just sent (either Delete#0 or Delete#1)

void setup()
{
  //for initializing the GSM
  pinMode(FONA_POWER, OUTPUT);
  digitalWrite(FONA_POWER, HIGH);
  delay(FONA_POWER_ON_TIME);
  digitalWrite(FONA_POWER, LOW);
  delay(3000);

  while (!Serial); // wait till serial gets initialized

  Serial.begin(115200); // baud rate
  Serial.println(F("Sustaingineering 3G TxRx!!"));
  Serial.println(F("Initializing....(May take 3 seconds)"));

  fonaSerial->begin(4800);
  while (!fona.begin(*fonaSerial))
  {
    Serial.println(F("Couldn't find FONA")); // reboot arduino and fona if this shows up! (Should probably do this automatically for robustness)
    delay(1000);
  }
  type = fona.type();
  Serial.println(F("FONA is OK"));

  Serial.println(F("Searching for network\n"));
  bool SIMFound = false;
  for (int countdown = 600; countdown >= 0 && SIMFound == false; countdown--)
  {
    uint8_t n = fona.getNetworkStatus(); // constantly check until network is connected to home    sendCheckReply(F("AT+CLVL="), i, ok_reply);
    if (n == 1)
    {
      SIMFound = true;
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
  }

  if (!SIMFound)
  {
    Serial.println(F("SIM card could not be found. Please ensure that your SIM card is compatible with dual-band UMTS/HSDPA850/1900MHz WCDMA + HSDPA."));
    while (1);
  }

  //----------------------------------

  //initializing SD card

  /*Serial.print("Initializing SD card...");
    if (!SD.begin(4)) { //This sets the cspin  to pin 4. This will have to be changed when the module is attatched
    Serial.println("initialization failed!");
    while (1);
    }
    Serial.println("initialization done.");*/
}

void loop()
{
  if(numsms != 0)
  {
    Serial.print(F("Number of Messages: ")); Serial.println(numsms);
    Serial.println(F("Delete all SMS"));
    delete_SMS_all();
  }

  while (!Serial.available())
  {
    if (time_sms())
      check_get_sms();
  }
}

boolean time_sms()
{ // use millis to check the number of sms every second.
  timecounts = millis();
  if (timecounts > last_timecounts + 1000)
  {
    last_timecounts = timecounts;
    return 1;
  }
  else
    return 0;
}

void check_get_sms()
{
  Serial.println(F("Start check_get_sms"));
  if (numsms != fona.getNumSMS()) //we might get an "AT+CMGR=255" fail code, but this is nothing of deep concern
  {
    numsms = fona.getNumSMS();
    uint8_t smsn = numsms - 1; // the sms# (aka SMS index) starts from 0
    
    if (!fona.getSMSSender(smsn, replybuffer, 250))
    {
      Serial.println(F("Failed!")); 
      return;
    }
    Serial.print(F("FROM: "));
    Serial.println(replybuffer);
    
    //retrieve SMS value.
    uint16_t smslen;
    if (!fona.readSMS(smsn, replybuffer, 250, &smslen))
    { //pass in buffer and max len!
      Serial.println(F("Failed!"));
      return;
    }
    /*
       Data is sent in csv format. The data will appear as follows:
       RTC Time, LoadVoltage, LoadCurrent, Power, AtmTemp, SolTemp, WaterBreakerFlag
    */

    //retrieve delimited values for use
    char *vars[5];
    vars[0] = strtok(replybuffer, ",");
    for (int i = 1; i < 6; i++)
      vars[i] = strtok(NULL, ",");

    //Serial print results
  
    Serial.print(F("RTC Time [YYYY/MM/DD-hh:mm:ss]: "));
    Serial.println(vars[0]);
    Serial.print(F("Load Voltage [V]: "));
    Serial.println(vars[1]);
    Serial.print(F("Load Current [A]: "));
    Serial.println(vars[2]);
    Serial.print(F("Power [W]: "));
    Serial.println(vars[3]);
    // Serial.print(F("Atmospheric Temperature [^oC]: "));
    // Serial.println(vars[4]);
    Serial.print(F("Solar Panel Temperature [^oC]: "));
    Serial.println(vars[4]);
    Serial.print(F("Water Breaker Flag: "));
    Serial.println(vars[5]);

    //Since all messages are deleted in the beginning, we just need to delete the current message at index 0

    if (fona.deleteSMS(0))
      Serial.println(F("Deleted SMS index #0"));
    else
      Serial.println(F("Couldn't delete"));

    /*
            // Read file from memory
            // File is overwritten when writing to SD, so we must first retrieve the current contents of the file
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
            // Append the new values to the old values, terminated with a new line
            strcat(data_inmem,replybuffer);
            strcat(data_inmem,"\n");
            // Write the full data back to the SD card
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

   
  }
}

void delete_SMS_all()
{
  //delete all SMS
  int max_SMS_num = 50;
  
  for (int i = 0; i < max_SMS_num; i++)
  {
    if (fona.deleteSMS(i))
      Serial.println(F("Deleted SMS index # "));
    else
      Serial.println(F("Couldn't delete"));
  }
  Serial.println(F("All messages are cleared"));
}
