/* Nini_RTCtemp03.ino
  For the Mini Pearl Logger with real time clock (RTC) board and microSD card board.
  Logs temperature data from the sensor on the RTC to the microSD card.
  --  also logs temperature from a DS18S20 temperature probe
  --  also logs TDS from DFRobot TDS probe.
  Displays these data on the serial monitor in real time.
  Interval between data write events is one minute (change this near line 29: #define SampleIntervalMinutes 1)
  Sleeps some things between read/write events.
  Modified from Ed Mallon's Cave Pearl Logger sketch.
  Modified from C. Fastie, September 2017
  David Robinson, 2018-3-21
*/
#include <SdFat.h>                           // https://github.com/greiman/SdFat/
#include <SPI.h>
#include <Wire.h>
#include <OneWire.h>

#include <EEPROM.h>
#include "GravityTDS.h"
#define TdsSensorPin A1
#define VREF 3.3     // analog reference voltage(Volt) of the ADC Arduino Pro-Mini
#define SCOUNT  10           // sum of sample point - was 30
int analogBuffer[SCOUNT];    // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0, copyIndex = 0;
float averageVoltage = 0, tdsValue = 0;

int DS18S20_Pin = 6; //DS18S20 Signal pin on digital 6
//Temperature chip i/o
OneWire ds(DS18S20_Pin);  // on digital pin 6

#include "LowPower.h"                        // https://github.com/rocketscream/Low-Power
#include <RTClib.h>                          // https://github.com/MrAlvin/RTClib  
RTC_DS3231 RTC;                              // RTC will be the RTC_DS3231 object
#define DS3231_I2C_ADDRESS 0x68              // RTC address
SdFat SD;                                    // SD will be the SdFat object
const int chipSelect = 10;                   // for SD card
#define MOSIpin 11                           // for SD card
#define MISOpin 12                           // for SD card
char TmeStrng[] = "0000/00/00,00:00:00";     // string template for RTC time stamp
int RTC_INTERRUPT_PIN = 2;
byte Alarmhour;
byte Alarmminute;
byte Alarmday;
char CycleTimeStamp[ ] = "0000/00/00,00:00:00"; // template for a data/time string
#define SampleIntervalMinutes 15               // change the logging interval here
volatile boolean clockInterrupt = true;         //this flag is set to true when the RTC interrupt handler is executed
float temp3231;                                 //variables for reading the DS3231 RTC temperature register
byte tMSB = 0;
byte tLSB = 0;

void setup() {
  // Setting the SPI pins high helps some sd cards go into sleep mode
  // the following pullup resistors only need to be enabled for the stand alone logger builds - not the UNO loggers
  pinMode(chipSelect, OUTPUT); digitalWrite(chipSelect, HIGH);    //Always pullup the CS pin with the SD library
  //and you may need to pullup MOSI/MISO
  pinMode(MOSIpin, OUTPUT); digitalWrite(MOSIpin, HIGH);          //pullup the MOSI pin
  pinMode(MISOpin, INPUT); digitalWrite(MISOpin, HIGH);           //pullup the MISO pin
  pinMode(TdsSensorPin, INPUT);

  Serial.begin(9600);                    // Open serial communications
  Wire.begin();                          // initialize the I2C interface
  RTC.begin();                           // initialize the RTC

  if (!SD.begin(chipSelect)) {           // initialize the SD card and display its status:
    Serial.println("Cannot find SD card");
    //    return;
  }
  else
  {
    Serial.println("SD OK");
  }
  // Uncomment the line below and load the sketch to the Pro Mini to set the RTC time. Then the line must
  // be commented out and the sketch loaded again or the time will be wrong.
  //RTC.adjust(DateTime((__DATE__), (__TIME__)));        // sets the RTC to the time the sketch was compiled.
  // print a header to the data file:
  File dataFile = SD.open("TDS_Temp.txt", FILE_WRITE);
  if (dataFile) {                        // if the file is available, write a header to it:
    dataFile.println("Date, Time, UTC time, RTC temp C, Probe temp C, Voltage V, TDS ppm");
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

  float temperature = getTemp();
  /***** TDS code ******/
  /***** take a sequence of SCOUNT samples at 40 ms intervals */
  // Serial.print("Sampling TDS -");
  static unsigned long analogSampleTimepoint = millis();
  analogBufferIndex = 0;
  while (analogBufferIndex < SCOUNT) {
    if (millis() - analogSampleTimepoint > 40U)  //every 40 milliseconds,read the analog value from the ADC
    {
      analogSampleTimepoint = millis();
      analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);    //read the analog value and store into the buffer
      analogBufferIndex++;
    }
  }

  /** find median of the SCOUNT samples **/
  for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++)
    analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
  // read the analog value more stable by the median filtering algorithm, and convert to voltage value
  averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * (float)VREF / 1024.0;
  //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
  float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0);
  //temperature compensation
  float compensationVolatge = averageVoltage / compensationCoefficient;
  //convert voltage value to tds value
  tdsValue = (133.42 * compensationVolatge * compensationVolatge * compensationVolatge - 255.86 * compensationVolatge * compensationVolatge + 857.39 * compensationVolatge) * 0.5;
  /****** TDS Code END **********/

  Serial.print("RTC temp: ");
  Serial.print(temp3231);
  Serial.println(" C");
  Serial.print("PROBE temp: ");
  Serial.print(temperature);
  Serial.println(" C");
  Serial.print("voltage:");
  Serial.print(averageVoltage, 2);
  Serial.println(" V   ");
  Serial.print("TDS Value:");
  Serial.print(tdsValue, 0);
  Serial.println(" ppm");

  float utc = (now.unixtime());
  // write the data to the SD card:
  File dataFile = SD.open("TDS_Temp.txt", FILE_WRITE);  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println() ;
    dataFile.print(TmeStrng); dataFile.print(","); dataFile.print(utc); dataFile.print(", ");
    dataFile.print(temp3231); dataFile.print(", "); dataFile.print(temperature); dataFile.print(", "); dataFile.print(tdsValue);
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
    delay(300);                                           // you would comment out most of the message printing below
    // if your logger was actually being deployed in the field
    Serial.println();                                      // adds a carriage return
    Serial.print("Alarm set, ");
    Serial.print("will sleep for: ");
    Serial.print(SampleIntervalMinutes);
    Serial.println(" minute");
    Serial.println();                                      // adds a carriage return
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

float getTemp() {
  //returns the temperature from one DS18S20 in DEG Celsius

  byte data[12];
  byte addr[8];

  if ( !ds.search(addr)) {
    //no more sensors on chain, reset search
    ds.reset_search();
    return -1100;
  }

  if ( OneWire::crc8( addr, 7) != addr[7]) {
    Serial.println("CRC is not valid!");
    return -2000;
  }

  if ( addr[0] != 0x10 && addr[0] != 0x28) {
    Serial.print("Device is not recognized");
    return -3000;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1); // start conversion, with parasite power on at the end

  byte present = ds.reset();
  ds.select(addr);
  ds.write(0xBE); // Read Scratchpad


  for (int i = 0; i < 9; i++) { // we need 9 bytes
    data[i] = ds.read();
  }

  ds.reset_search();

  byte MSB = data[1];
  byte LSB = data[0];

  float tempRead = ((MSB << 8) | LSB); //using two's compliment
  float TemperatureSum = tempRead / 16;

  return TemperatureSum;
}

int getMedianNum(int bArray[], int iFilterLen)
{
  int bTab[iFilterLen];
  for (byte i = 0; i < iFilterLen; i++)
    bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++)
  {
    for (i = 0; i < iFilterLen - j - 1; i++)
    {
      if (bTab[i] > bTab[i + 1])
      {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0)
    bTemp = bTab[(iFilterLen - 1) / 2];
  else
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  return bTemp;
}
