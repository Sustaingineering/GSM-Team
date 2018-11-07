//By Rico Jia, May 16, 2018

//SG_FONA folder must be included in \Documents\Arduino\libraries
//Arduino must be set to port 1. Go to device manager -> Ports(COM & LPT) -> Arduino Uno -> Properties -> Port Settings -> Advanced -> COM Port Number. Then re-plug the USB.
//Serial may need to be set to 4800bps manually first. On my device, this was the default.
//Serial Monitor must be set to Both NL & CR, as well as 2400 Baud
//Many cell carriers do not support GSM. Only supported network may be Rogers (this will be shutting down soon)
//Ensure that in the IDE, under Tools -> Board, the value is set to Arduino/Genuino Uno.

#define FONA_TX 4 //Soft serial port
#define FONA_RX 5 //Soft serial port
#define FONA_RI 3 
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
#include "SG_FONA.h"
Adafruit_FONA_3G fona = Adafruit_FONA_3G(FONA_RST);

uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);    // For reading from serial port

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

  Serial.begin(9600); //Baud rate
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
  randomSeed(analogRead(0));
}
void loop()
{
 
  /*float sensorValue = analogRead(A2) ;
  float volt;
  float temp;
  // Convert the analog reading ( which goes from 0 - 1023) to a voltage (0 - 5V):
  float voltage = sensorValue *5000/1023+25;

        if(voltage > 2500)
        {
          volt = voltage-2500;//(while cold conjuction is 22 degree 
          volt=  volt / 123; //(opamp apmplified 150 times)
          temp = (volt)/0.041 +25  ;//( 1 degree = 0.0404 mv in K type )
        }
       
        else{
          volt = 2500-voltage;//(while cold conjuction is 22 degree 
                             //   the voltage cross thermalcouple is 1.67v) 
                volt=  volt / 123; //(opamp apmplified 150 times)
        
          temp = 25-(volt)/0.0404  ;//( 1 degree = 0.0404 mv in K type )
  
        }
                      
       
  Serial.println(temp);
  delay(1000);*/

  
  double temp=random(0,400)/100.+23.;

  /*uint8_t year=0;
  uint8_t month=0;
  uint8_t date=0;
  uint8_t hr=0;
  uint8_t min=0;
  uint8_t sec=0;
  
  fona.readRTC(&year,&month,&date,&hr,&min,&sec);

  Serial.print(F("Year: "));
  Serial.println(year);
  Serial.print(F("month: "));
  Serial.println(month);
  Serial.print(F("date: "));
  Serial.println(date);
  Serial.print(F("hr: "));
  Serial.println(hr);
  Serial.print(F("min: "));
  Serial.println(min);
  Serial.print(F("sec: "));
  Serial.println(sec);*/
  
  
  send_sms(temp);

  //delay(10000); //Wait 10 seconds before repeating
} 

void send_sms(float value)
   { // send an SMS!
        char message[141];
        bool loop=true;
        while(loop)
        {
          flushSerial();    // THIS IS IMPORTANT! OTHERWISE what you typed in might be missing in what you send
          loop=false;
           
          String str;
          str=(String)(String)(value);
          str.toCharArray(message, 141);
        }
        Serial.print(F("Your message is "));
        if(fona.sendSMS(sendto, message)==0)
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
