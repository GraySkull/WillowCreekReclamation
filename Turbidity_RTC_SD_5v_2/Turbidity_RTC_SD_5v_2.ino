/* Raw_RTC_SD_5v.ino
  Data logger with real time clock (RTC) board and 5V microSD card board.
  Logs temperature data from the sensor on the RTC to the microSD card.
  Displays these data on the serial monitor in real time.
  Interval between data write events defaults to 1 min
  Sleeps some things between read/write events.
  Modified from Ed Mallon's Cave Pearl Logger sketch.
  C. Fastie, September 2017
  D. Robinson, April 2018
*/
#include <SdFat.h>                           // https://github.com/greiman/SdFat/
#include <SPI.h>
#include <Wire.h>
#include "LowPower.h"                        // https://github.com/rocketscream/Low-Power
#include <RTClib.h>                          // https://github.com/MrAlvin/RTClib  
RTC_DS3231 RTC;                              // RTC will be the RTC_DS3231 object
#define DS3231_I2C_ADDRESS 0x68              // RTC address
SdFat SD;                                    // SD will be the SdFat object
const int chipSelect = 8;                   // for SD card
const int cardDetect = 9;
#define MOSIpin 11                           // for SD card
#define MISOpin 12                           // for SD card

#define TurbidityPin A0

char TmeStrng[] = "0000/00/00,00:00:00";     // string template for RTC time stamp
int RTC_INTERRUPT_PIN = 2;
byte Alarmhour;
byte Alarmminute;
byte Alarmday;
char CycleTimeStamp[ ] = "0000/00/00,00:00:00"; // template for a data/time string
#define SampleIntervalMinutes 15                 // change the logging interval here
volatile boolean clockInterrupt = true;         //this flag is set to true when the RTC interrupt handler is executed
float temp3231;                                 //variables for reading the DS3231 RTC temperature register
byte tMSB = 0;
byte tLSB = 0;


char fileName[] = "Turbidity.txt"; // SD library only supports up to 8.3 names
bool alreadyBegan = false;

void setup() {
  // Setting the SPI pins high helps some sd cards go into sleep mode
  // the following pullup resistors only need to be enabled for the stand alone logger builds - not the UNO loggers
  pinMode(chipSelect, OUTPUT); digitalWrite(chipSelect, HIGH);    //Always pullup the CS pin with the SD library
  //and you may need to pullup MOSI/MISO
  pinMode(MOSIpin, OUTPUT); digitalWrite(MOSIpin, HIGH);          //pullup the MOSI pin
  pinMode(MISOpin, INPUT); digitalWrite(MISOpin, HIGH);           //pullup the MISO pin
  delay(100);

  Serial.begin(9600);

  initializeCard();

  Wire.begin();                          // initialize the I2C interface
  RTC.begin();                           // initialize the RTC

  // Uncomment the line below and load the sketch to the Pro Mini to set the RTC time. Then the line must
  // be commented out and the sketch loaded again or the time will be wrong.
  //RTC.adjust(DateTime((__DATE__), (__TIME__)));        // sets the RTC to the time the sketch was compiled.
  // print a header to the data file:
  File dataFile = SD.open(fileName, FILE_WRITE);

  if (dataFile) {                        // if the file is available, write a header to it:
    dataFile.println("Date, Time, UTC time, RTC temp C, voltage V, Turbidity NTU");
    dataFile.close();
  }
  else {
    Serial.println("Cannot save to SD");        // if the file is not open, display an error:
  }

}                                        // end of setup

void loop() {
  DateTime now = RTC.now();            // read the time from the RTC, then construct two data strings:
  sprintf(TmeStrng, "%04d/%02d/%02d,%02d:%02d:%02d", now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
  sprintf(CycleTimeStamp, "%04d/%02d/%02d,%02d:%02d:%02d", now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());

  Serial.println();
  Serial.print("RTC time: ");
  Serial.println(TmeStrng);
  Serial.print("RTC UTC time: ");
  Serial.println(now.unixtime());

  if (clockInterrupt)
  {
    if (RTC.checkIfAlarm(1))
    { // Is the RTC alarm still on?
      RTC.turnOffAlarm(1);                // then turn it off.
    }
    // print (optional) debugging message to the serial window if you wish
    Serial.print("Alarm triggered at ");
    Serial.println(CycleTimeStamp);
    clockInterrupt = false;             // reset the interrupt flag to false
  }
  // read the RTC temp register and print that out
  // Note: the DS3231 temp registers (11h-12h) are only updated every 64seconds
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.write(0x11);                    // the register where the temp data is stored
  Wire.endTransmission();
  Wire.requestFrom(DS3231_I2C_ADDRESS, 2);   //ask for two bytes of data
  if (Wire.available()) {
    tMSB = Wire.read();                // 2's complement int portion
    tLSB = Wire.read();                // fraction portion
    temp3231 = ((((short)tMSB << 8) | (short)tLSB) >> 6) / 4.0;  // Allows for readings below freezing: thanks to Coding Badly
  }
  else {
    temp3231 = 0;                     //if temp3231 contains zero, then you know you had a problem reading the data from the RTC!
  }
  // Turbidity Sensor
  // getTurbidity();
  int sensorValue = analogRead(TurbidityPin);// read the input on analog pin 0:
  float voltage = sensorValue * (5.0 / 1024.0); // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  float turbidity = -1120.4 * voltage * voltage + 5742.3 * voltage - 4352.9;
  // Pure water= NTU < 0.5, output should be “4.1±0.3V” when temperature is 10~50℃.
  //
  Serial.print("T-voltage: ");
  Serial.print(voltage); // print out the value you read:
  Serial.println(" V");

  Serial.print("Turbidity: ");
  Serial.print(turbidity);
  Serial.println(" NTU ");

  delay(500);

  Serial.print("RTC temp: ");
  Serial.print(temp3231);
  Serial.println(" C");

  float utc = (now.unixtime());
  // write the data to the SD card:
  File dataFile = SD.open(fileName, FILE_WRITE);  // if the file is available, write to it:
  if (dataFile) { //dgr
    dataFile.println() ;
    dataFile.print(TmeStrng); dataFile.print(","); dataFile.print(utc); dataFile.print(","); dataFile.print(temp3231);
    dataFile.print(","); dataFile.print(voltage);dataFile.print(","); dataFile.print(turbidity);
    dataFile.close();
  }
  else {
    Serial.println("Cannot save to SD card");          // if the file is not open, display an error
  }
  // Set the next alarm time
  Alarmhour = now.hour();
  Alarmminute = now.minute() + SampleIntervalMinutes;
  Alarmday = now.day();
  // check for roll-overs
  if (Alarmminute > 59) {                             // error catching the 60 rollover!
    Alarmminute = 0;
    Alarmhour = Alarmhour + 1;
    if (Alarmhour > 23) {
      Alarmhour = 0;
    }
  }
  RTC.setAlarm1Simple(Alarmhour, Alarmminute);          // set the alarm
  RTC.turnOnAlarm(1);
  if (RTC.checkAlarmEnabled(1)) {
    delay(300);
    // sleep and wait for next RTC alarm
    attachInterrupt(0, rtcISR, LOW);                       // Enable interrupt on pin2 & attach it to rtcISR function:

    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_ON);    // Enter power down state with ADC module disabled to save power
    // processor starts HERE AFTER THE RTC ALARM WAKES IT UP
    detachInterrupt(0);                                    // immediately disable the interrupt on waking
  }
}                                                        // end of the MAIN LOOP
void rtcISR() {                                          // This is the Interrupt subroutine that executes when the rtc alarm goes off
  clockInterrupt = true;
}

void initializeCard(void)
{
  Serial.print(F("Initializing SD card..."));

  // Is there even a card?
  if (!digitalRead(cardDetect))
  {
    Serial.println(F("No card detected. Waiting for card."));
    while (!digitalRead(cardDetect));
    delay(250); // 'Debounce insertion'
  }

  // Card seems to exist.  begin() returns failure
  // even if it worked if it's not the first call.
  if (!SD.begin(chipSelect) && !alreadyBegan)  // begin uses half-speed...
  {
    Serial.println(F("Initialization failed!"));
    initializeCard(); // Possible infinite retry loop is as valid as anything
  }
  else
  {
    alreadyBegan = true;
  }
  Serial.println(F("Initialization done."));

  Serial.print(fileName);
  if (SD.exists(fileName))
  {
    Serial.println(F(" exists."));
  }
  else
  {
    Serial.println(F(" doesn't exist. Creating."));
  }
  Serial.print("Opening file: ");
  Serial.println(fileName);
}
/*
  void getTurbidity()//
  {
  Turbidityval = analogRead(A0) / 1024.0 * 5.0;
  Serial.print("    Trubidity value: ");
  Serial.println( Turbidityval);//
    lcd.setCursor(0,0);
     lcd.print("Turbidity:");
     lcd.setCursor(12,0);
     lcd.println(Turbidityval);
  } */
