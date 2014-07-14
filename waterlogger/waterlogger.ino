/*
 Water level datalogger
 Arduino Pro Mini clone (3V3 @ 16MHz)
 HC-SR04 ultrasonic sensor
 DS3231 RTC
 LCSoft SD card holder
 5V SMD regulator with shutdown pin control
 
 Menu on restart to set RTC, board ID number
 Take measurements every 1 hour, store to SDcard
 */


//#include <Fat16.h>	                         // the SD Card library
//#include <Fat16util.h>
#include <SD.h>                                  // the SD Card library
#include <Wire.h>                                // I2C functions
#include <Narcoleptic.h>                         // low-power sleep routine
#include <EEPROM.h>                              // built-in EEPROM routines


// ------- declare variables ----------------------------------------------------
int   IDnum;                                     // Board/Site identifier
char  name[] = "SNC-xxx.txt";                    // SDcard file
int   sleepint = 7;                              // sleep interval (sleepint * 8 sec)
char  a[4];                                      // buffer array for itoa function 

byte  i;                                         // for-loop counter
byte  j;                                         // kPa array index
int   mV;                                        // ADC value for LM35
int   ADV;                                       // ADC value
int   ADval;
long  ADCmax = 1023;
long  Vcc;                                       // supply/Aref voltage read with secret voltmeter

int   Tair10;                                    // air temp * 10 for Vsound calculation
long  TLM35whole;                                // air temp, whole part
long  TLM35dec;                                  //           decimal part

long  duration;                                  // length of ultrasonic echo time
long  distance;                                  // uncorrected distance
long  dist_whole;                                // uncorrected distance, whole part
long  dist_dec;                                  //                       decimal part
long  pulsetotal;                                // time for echo return in microsec
long  distTcorr;                                 // temperature-corrected distance
long  distTcorr_whole;                           // correct distance, whole part
long  distTcorr_dec;                             //                   decimal part
long  Vsound;                                    // temperature-corrected speed of sound

int   DS1337ctrl = B1101000;                     // DS1337 i2c identifier
int   rtc_bcd[8];                                // time, date values
int   rtc[8];
int   secs;
int   mins;
int   hrs;
int   dow;
int   days;
int   mnths;
int   yrs;

int  lastmin;  // One-minute logging for testing

int   menuinput;                                 // user input for menu 
long  timeout;                                   // seconds before menu times out
int   indata;                                    // user-input data
int   input;                                    
int   addr;                                      // EEPROM address
byte  EEPROMbyte;                                // EEPROM data to store/read


// ------- declare pins ------------------------------------------------------------
int   extPower = 8;                              // power to HC-SR04 sensor and SD card
int   trigPin = 7;                               // Output to trigger pin
int   echoPin = 6;                               // Input from echo pin

int   LM35power = 9;                             // power to LM35 temp sensor
int   LM35out = A6;                               // ADC channel for LM35 temperature sensor

// On the Ethernet Shield, CS is pin 4. Note that even if it's not
// used as the CS pin, the hardware CS pin (10 on most Arduino boards,
// 53 on the Mega) must be left as an output or the SD library
// functions will not work.
const int chipSelect = 10;

//SdCard card;                                     // initialize SDcard
//Fat16 file;                                      // initialize FAT

File file;

/*
#define error(s) error_P(PSTR(s))                // store error strings in flash to save RAM
void error_P(const char* str)                    // print out error messages
{
  PgmPrint("error: ");
  SerialPrintln_P(str);
  if (card.errorCode) {
    PgmPrint("SD error: ");
    Serial.println(card.errorCode, HEX);
  }
  //  while(1);
}
*/

// --------------------- setup ---------------------------------------------------
void setup()
{
  analogReference(DEFAULT);                      // ADC voltage reference

  pinMode(extPower, OUTPUT);                      // power for SDcard
  digitalWrite(extPower, HIGH);                   // Turn on SD power so we can read the card

  pinMode(LM35power,OUTPUT);                     // power for LM35
  digitalWrite(LM35power, LOW);                   // Turn it off for now

  // Setup HC-SR04 module
  pinMode(echoPin, INPUT);
  pinMode(trigPin,OUTPUT);
  digitalWrite(trigPin, LOW);

  Serial.begin(9600);                            // enable screen output
  Wire.begin();                                  // enable i2c bus

  Serial.println("Initializing SD card...");
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(10, OUTPUT);
 
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present.");
  }
  else
  {
    Serial.println("Card initialized.");
  }
  
  menu();                                        // goto menu
}


// -------------------------------- main loop -----------------------------------------
void loop()                                      // main program
{  
  readRTC();                                     // read RTC

//    if(mins == 0)                                // 1 hr measurement interval
  if (mins != lastmin)                           // 1 minute logging interval for testing
  {
    readLM35();                                  // read LM35 air temperature
    ping();                                      // read Ping ultrasonic sensor

    printData();                                 // output data to screen
    storeData();                                 // store data to SD card
    lastmin=mins;  // For testing
  }
  sleepytime();                                  // go to low-power sleep routine
}

// =============================== end void loop ==================================
//==================================================================================



// ****************************** subroutines ****************************************
// ------------------------ routine to read LM35 temp sensor ----------------------
void readLM35()
{  
  digitalWrite(LM35power,HIGH);                  // turn on power to LM35 sensor
  
  readVcc();                                     // read ADC supply/Aref voltage

  ADV = analogRead(LM35out);                     // dummy reading to settle ADC channel
  delay(10);      

  ADval = 0;                                     // initialize total
  for(i=1; i<=10; i++)                           // take 10 samples
  {
    ADV = analogRead(LM35out);                   // read ADC channel
    delay(10);      
    ADval += ADV;                                // totalize ADval readings
  }
  
   digitalWrite(LM35power,LOW);                  // turn off sensor
  
  mV = ADval * Vcc / ADCmax;                     // calculate mV

  Tair10 = mV; // / 10;                              // air temp * 10 (deg C) for Vsound calculation

  //long TdegF = (long(mV) * 18) / 10 + 3200;      // convert to deg F

  //TLM35whole = TdegF / 100;                      // temp in deg F, whole part
  //TLM35dec = TdegF % 100;                        //                decimal part

  TLM35whole = Tair10 / 100;                      // temp in deg C, whole part
  TLM35dec = Tair10 % 100;                        //                decimal part

}
//====================================================================================



// ------------------------ routine to read HC-SR04 sensor --------------------------
void ping()
{
  digitalWrite(extPower,HIGH);                  // turn on power to sensor
  delay(500);

  // take dummy reading
  digitalWrite(trigPin, HIGH);                   // Generate trigger pulse of 10us width
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);             // measure time of echo pulse
  delay(200);

  pulsetotal = 0;                                // initialize totalizer
  for(i=1; i<=10; i++)                           // take 10 samples
  {
    digitalWrite(trigPin, HIGH);                 // Generate trigger pulse of 10us width
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    duration = pulseIn(echoPin, HIGH);              // measure time of echo pulse

    pulsetotal = pulsetotal + duration;          // totalize

    delay(200);
  }  
  digitalWrite(extPower,LOW);                   // turn off ultrasonic sensor

  distance = pulsetotal * 10 / 148;              // calculate uncorrected distance from time of pulse, inches*100
  dist_whole = distance / 100;                   // distance, whole inches
  dist_dec = distance % 100;                     //           decimal part  
  
  Vsound = 331 + 6 * (Tair10/10) / 100;               // calculate Vsound = 331m/s + 0.6m/sC*Tair

  distTcorr = Vsound * 3936 / 100;               // corrected distance, converting m to in
  distTcorr = distTcorr * pulsetotal / 200000;   // calculate distance, in * 100
  distTcorr_whole = distTcorr / 100;             // corrected distance, whole part
  distTcorr_dec = distTcorr % 100;               //                     decimal part
}
// ============================================================================



//---------- read DS1337 RTC --------------------------------
void readRTC()
{
  Wire.beginTransmission(DS1337ctrl);            // send i2c control identifier
  Wire.write(0x00);                               // set register to 0
  Wire.endTransmission();

  Wire.requestFrom(DS1337ctrl, 7);               // read 7 bytes: secs,min,hr,dow,day,mth,yr
  rtc_bcd[1] = Wire.read();
  rtc_bcd[2] = Wire.read();
  rtc_bcd[3] = Wire.read();
  rtc_bcd[4] = Wire.read();
  rtc_bcd[5] = Wire.read();
  rtc_bcd[6] = Wire.read();
  rtc_bcd[7] = Wire.read();
  Wire.endTransmission();

  for(int i=1; i<=7; i++)                        // convert BCD to decimal
  {
    rtc[i] = (rtc_bcd[i]/16)*10 + rtc_bcd[i]%16;
  }

  secs = rtc[1];
  mins = rtc[2];
  hrs = rtc[3];
  days = rtc[5];
  mnths = rtc[6];
  yrs = rtc[7];
  yrs = yrs + 2000;                              // change year to 20xx format
}
//===================================================================



//---------- low-power sleep -----------------------------------------
void sleepytime()
{
  ADCSRA = 0;                                    // disable ADC

  for(i=1; i<=sleepint; i++)
  {
    Narcoleptic.delay(8000);                     // power consumption is minimised
  }
    ADCSRA |= (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADEN);     // set prescaler to 128 and enable ADC 
}
//****************************************************************  



//--------- secret voltmeter ------------------------------------------
void readVcc()
{                                                // read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(20);                                     // Wait for Vref to settle
  ADCSRA |= _BV(ADSC);                           // Convert
  while (bit_is_set(ADCSRA,ADSC));
  Vcc = ADCL;
  Vcc |= ADCH<<8;
  Vcc = 1126400L / Vcc;                          // Back-calculate AVcc in mV
}
//**********************************************************************


//-------- print data to screen ---------------------------------
void printData()
{
  Serial.print(IDnum);                           // Site identifier
  Serial.print(' ');
  Serial.print(mnths);                           // date
  Serial.print('-');
  Serial.print(days);
  Serial.print('-');
  Serial.print(yrs);
  Serial.print(' ');

  Serial.print(hrs);                             // time
  Serial.print(':');
  if(mins < 10) Serial.print('0');
  Serial.print(mins);
  Serial.print(':');
  if(secs < 10) Serial.print('0');
  Serial.print(secs);
  Serial.print(' ');

  Serial.print(TLM35whole);                      // air temperature
  Serial.print('.');
  if(TLM35dec < 10) Serial.print('0');
  Serial.print(TLM35dec);
  Serial.print(' ');

  Serial.print(dist_whole);                      // distance, uncorrected
  Serial.print('.');
  if(dist_dec < 10) Serial.print('0');
  Serial.print(dist_dec);  
  Serial.print(' ');

  Serial.print(distTcorr_whole);                 // distance, temperature corrected
  Serial.print('.');
  if(distTcorr_dec < 10) Serial.print('0');
  Serial.print(distTcorr_dec);

  Serial.println();
  delay(10);
}
//===============================================================



// ------- store data to SDcard ---------------------------------------
void storeData()
{
  digitalWrite(extPower,HIGH);                    // turn on power to SDcard
  delay(500);

/*
  if (!card.init()) error("card.init");          // initialize the SD card
  if (!Fat16::init(&card)) error("Fat16::init");    // initialize a FAT16 volume
  // O_CREAT - create the file if it does not exist
  // O_EXCL - fail if the file exists
  // O_WRITE - open for write only
  if (file.open(name, O_CREAT | O_APPEND | O_WRITE));
  file.writeError = false;                       // clear write error
*/

  file = SD.open(name, FILE_WRITE);

  if (file) {
  
  
  file.print(IDnum);                             // Site identifier
  file.print(',');
  file.print(mnths);                             // date
  file.print('-');
  file.print(days);
  file.print('-');
  file.print(yrs);
  file.print(',');

  file.print(hrs);                               // hour
  file.print(',');

  file.print(TLM35whole);                        // air temperature
  file.print('.');
  if(TLM35dec < 10) file.print('0');
  file.print(TLM35dec);
  file.print(' ');

  file.print(dist_whole);                        // distance, uncorrected
  file.print('.');
  if(dist_dec < 10) file.print('0');
  file.print(dist_dec);
  file.print(' ');

  file.print(distTcorr_whole);                   // distance, temp corrected
  file.print('.');
  if(distTcorr_dec < 10) file.print('0');
  file.print(distTcorr_dec);

  file.println();

  file.close();
  
/*
  if (file.writeError) error("write data");
  if (!file.sync()) error("sync");               // don't sync too often - requires 2048 bytes of I/O to SD card
*/

  }
  else {
    Serial.println("error opening log file");
  }
  
  delay(100);

  digitalWrite(extPower,LOW);                     // turn off SDcard
}
//==========================================================================



//--------------------------------------------------------
void menu()
{
  addr = 1;                                      // read ID number from EEPROM
  EEPROMbyte = EEPROM.read(addr);                // high byte
  IDnum = EEPROMbyte * 256;
  addr ++;                                       // next address
  EEPROMbyte = EEPROM.read(addr);                // low byte
  IDnum += EEPROMbyte;                           // combine

    itoa(IDnum,a,10);                            // convert ID number to character array
  name[4] = a[0];                                // put into file name[] array
  name[5] = a[1];
  name[6] = a[2];

  Serial.println();                              // print out board info
  Serial.print("Board number:  ");               // IDnum
  Serial.println(IDnum);

  Serial.print("Data filename: ");               // SD filename
  Serial.println(name);

  readRTC();
  Serial.print("Current date:  ");
  Serial.print(mnths);                           // date
  Serial.print('-');
  Serial.print(days);
  Serial.print('-');
  Serial.println(yrs);

  Serial.print("Current time:  ");
  Serial.print(hrs);                             // time
  Serial.print(':');
  if(mins < 10) Serial.print('0');
  Serial.print(mins);
  Serial.print(':');
  if(secs < 10) Serial.print('0');
  Serial.println(secs);

  lastmin = mins;

  Serial.println();                              // menu options
  Serial.println("1. Set clock");
  Serial.println("2. Set ID number");
  Serial.println();

  timeout = millis() + 5000;                     // wait for input before timing out
  while(millis() < timeout)
  {
    if(Serial.available())                       // if something typed, go to menu
    {
      menuinput = Serial.read();
      break;
    }
  }

  menuinput -= 48;                               // convert ASCII to decimal
  switch(menuinput)
  {
  case 1:          //-------------------------------------------------------
    Serial.println("Set clock:");                // set RTC

    Serial.print("  input month:  ");
    getinput();
    mnths = indata;
    Serial.print("  input day:    ");
    getinput();
    days = indata;
    Serial.print("  input year:   ");
    getinput();
    yrs = indata;
    Serial.print("  input hour:   ");
    getinput();
    hrs = indata;
    Serial.print("  input minute: ");
    getinput();
    mins = indata;

    mins = (mins/10)*16 + mins%10;               // convert decimal to BCD
    hrs = (hrs/10)*16 + hrs%10;
    days = (days/10)*16 + days%10;
    mnths = (mnths/10)*16 + mnths%10;
    yrs = (yrs/10)*16 + yrs%10;

    Wire.beginTransmission(DS1337ctrl);          // set time, date
    Wire.write(0);

    Wire.write(secs);
    Wire.write(mins);
    Wire.write(hrs);
    Wire.write(dow);
    Wire.write(days);
    Wire.write(mnths);
    Wire.write(yrs);

    Wire.endTransmission();

    break;

  case 2:          // ----------------------------------------------------
    Serial.println("Set board ID number:");      // set board IB number

    Serial.print("  input ID number: ");
    getinput();
    IDnum = indata;

    addr = 1;                                    // store IDnum in EEPROM
    EEPROMbyte = IDnum / 256;                    // high byte
    EEPROM.write(addr,EEPROMbyte);
    delay(20);
    addr ++;
    EEPROMbyte = IDnum % 256;                    // low byte
    EEPROM.write(addr,EEPROMbyte);
    delay(20);

    break;
  }

  int k;
  for(k=1; k<=5; k++)                            // print out 5 readings
  {
    readRTC();
    readLM35();
    ping();
    printData();
    delay(2000);
  }
  Serial.println();
  Serial.println("ok ...");
  Serial.println();
  delay(5);
}
//================================================================


//----------------- read user input -----------------------------------------------
void getinput()
{
  timeout = millis() + 5000;                     // period to wait before timing out       

  indata = 0;
  while(millis() < timeout)
  {
    i = 0;
    if(Serial.available())                       // if something availabe from serial port
    {
      input = Serial.read();                     // read input
      if(input == 13) break;                     // if carraige return, done
      if(input > 47 and input < 58)              // look for number between 0 and 9
      {
        input -= 48;                             // convert ASCII input to decimal value
        Serial.print(input);
        delay(5);
        indata = indata*10 + input;              // concatenate multiple inputs
      }
      else
      {
        name[i] = input;
        Serial.print(input);
        i++;
      }
    }
  }
  Serial.println();
  delay(5);
}
//===================================================================

