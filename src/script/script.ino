#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "RTClib.h"
#include "Adafruit_SHT31.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include "Adafruit_MCP9808.h"
#include "Adafruit_TSL2591.h"
#include <SHT1x.h>
#include "SparkFunCCS811.h"
#include <SD.h>

#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11 
#define BMP_CS 10
#define dataPin  10
#define clockPin 11
#define CCS811_ADDR 0x5B //Default I2C Address

//SHT1x sht1x(dataPin, clockPin);
CCS811 mySensor(CCS811_ADDR);

Adafruit_SHT31 sht31 = Adafruit_SHT31();
Adafruit_BMP280 bmp; // I2C
Adafruit_MCP9808 tempsensor = Adafruit_MCP9808();
Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591); // pass in a number for the sensor identifier (for your use later)
RTC_PCF8523 rtc;


const int rainSensorMin = 0;     // sensor minimum
const int rainSensorMax = 1024;  // sensor maximum
const int chipSelect = 4;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

void setup() {
  Serial.begin(9600);

  while (!Serial)
    delay(10);     // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("SHT31 test");
  Serial.println(F("BMP280 test"));
  Serial.println("MCP9808 demo");
  Serial.println(F("Starting Adafruit TSL2591 Test!"));
  Serial.println("UV Test!");
  Serial.println("SHT10 Starting up");
  Serial.println("CCS811 test");
  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    while (1) delay(1);
  }
  Serial.println("card initialized.");
  
  if (!sht31.begin(0x44)) {   // Set to 0x45 for alternate i2c addr
    Serial.println("Couldn't find SHT31");
    while (1) delay(1);
  }
  if (!bmp.begin()) {  
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }
  if (!tempsensor.begin()) {
    Serial.println("Couldn't find MCP9808!");
    while (1);
  }
  if (!tsl.begin()) 
  {
    Serial.println(F("No sensor found ... check your wiring?"));
    while (1);
  } 
  configureSensorTSL();

  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }


    CCS811Core::status returnCode = mySensor.begin();
  if (returnCode != CCS811Core::SENSOR_SUCCESS)
  {
    Serial.println(".begin() returned with an error.");
    while (1); //Hang if there was a problem.
  }

}

void loop() {
    String dataString;
    CCS(dataString);
    File dataFile = SD.open("datalog.txt", FILE_WRITE);
    dataFile.println(dataString);
    dataFile.close();
    Serial.println(dataString);
//    
//  Serial.println("---SHT_HUM_TEMP---");
//  SHT();
//  Serial.println();
//  Serial.println("---BMP_PRES_TEMP_ALT---");
//  BMP();
//  Serial.println();
//  Serial.println("---MCP_TEMP_PREC---");
//  MCP();
//  Serial.println();
//  Serial.println("---TSL_LUX---");
//  TSL();
//  Serial.println();
//  Serial.println("---UV---");
//  UV();
//  Serial.println();
//  Serial.println("---SHT_GLEBA---");
////  SHT_gleba();
//  Serial.println();
//  Serial.println("---CCS---");
//  CCS();
//  Serial.println();
//  Serial.println("---RAIN---");
//  rain();
//  Serial.println();
//  Serial.println("---RTC---");
//  RTC();
  Serial.println();
  Serial.println("++++++++++++++++++++++++++++++++++++++++++++++++++");
  Serial.println("++++++++++++++++++++++++++++++++++++++++++++++++++");
  Serial.println();
  delay(10000);
}

void SHT() {
  float t = sht31.readTemperature();
  float h = sht31.readHumidity();

  if (! isnan(t)) {  // check if 'is not a number'
    Serial.print("  Temp *C = "); Serial.println(t);
  } else { 
    Serial.println("Failed to read temperature");
  }
  
  if (! isnan(h)) {  // check if 'is not a number'
    Serial.print("  Hum. % = "); Serial.println(h);
  } else { 
    Serial.println("Failed to read humidity");
  }
}

void MCP() {
  float c = tempsensor.readTempC();
  float f = c * 9.0 / 5.0 + 32;
  Serial.print("  Temp: "); Serial.print(c); Serial.print("*C\t"); 
  Serial.print(f); Serial.println("*F");
}

void BMP() {
  Serial.print(F("  Temperature = "));
  Serial.print(bmp.readTemperature());
  Serial.println(" *C");
  
  Serial.print(F("  Pressure = "));
  Serial.print(bmp.readPressure());
  Serial.println(" Pa");

  Serial.print(F("  Approx altitude = "));
  Serial.print(bmp.readAltitude(1013.25)); // this should be adjusted to your local forcase
  Serial.println(" m");
}

void TSL(void)
{
  // More advanced data read example. Read 32 bits with top 16 bits IR, bottom 16 bits full spectrum
  // That way you can do whatever math and comparisons you want!
  uint32_t lum = tsl.getFullLuminosity();
  uint16_t ir, full;
  ir = lum >> 16;
  full = lum & 0xFFFF;
  Serial.print(F("  [ ")); Serial.print(millis()); Serial.print(F(" ms ] "));
  Serial.print(F("IR: ")); Serial.print(ir);  Serial.print(F("  "));
  Serial.print(F("Full: ")); Serial.print(full); Serial.print(F("  "));
  Serial.print(F("Visible: ")); Serial.print(full - ir); Serial.print(F("  "));
  Serial.print(F("Lux: ")); Serial.println(tsl.calculateLux(full, ir), 6);
}

void UV(void) {
  float sensorVoltage; 
  float sensorValue;
 
  sensorValue = analogRead(A0);
  sensorVoltage = sensorValue/1024*3.3;
  Serial.print("  sensor reading = ");
  Serial.print(sensorValue);
  Serial.println("");
  Serial.print("  sensor voltage = ");
  Serial.print(sensorVoltage);
  Serial.println(" V");
}

//void SHT_gleba(void) {
//  float temp_c;
//  float humidity;
//    // Read values from the sensor
//  temp_c = sht1x.readTemperatureC();
//  humidity = sht1x.readHumidity();
//
//  // Print the values to the serial port
//  Serial.print("  Temperature: ");
//  Serial.print(temp_c, DEC);
//  Serial.print("C / ");
//  Serial.print("  Humidity: ");
//  Serial.print(humidity);
//  Serial.println("%");
//}

void configureSensorTSL(void)
{
  // You can change the gain on the fly, to adapt to brighter/dimmer light situations
  //tsl.setGain(TSL2591_GAIN_LOW);    // 1x gain (bright light)
  tsl.setGain(TSL2591_GAIN_MED);      // 25x gain
  //tsl.setGain(TSL2591_GAIN_HIGH);   // 428x gain
  
  // Changing the integration time gives you a longer time over which to sense light
  // longer timelines are slower, but are good in very low light situtations!
  //tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS);  // shortest integration time (bright light)
  // tsl.setTiming(TSL2591_INTEGRATIONTIME_200MS);
  tsl.setTiming(TSL2591_INTEGRATIONTIME_300MS);
  // tsl.setTiming(TSL2591_INTEGRATIONTIME_400MS);
  // tsl.setTiming(TSL2591_INTEGRATIONTIME_500MS);
  // tsl.setTiming(TSL2591_INTEGRATIONTIME_600MS);  // longest integration time (dim light)

  /* Display the gain and integration time for reference sake */  
  Serial.println(F("------------------------------------"));
  Serial.print  (F("Gain:         "));
  tsl2591Gain_t gain = tsl.getGain();
  switch(gain)
  {
    case TSL2591_GAIN_LOW:
      Serial.println(F("1x (Low)"));
      break;
    case TSL2591_GAIN_MED:
      Serial.println(F("25x (Medium)"));
      break;
    case TSL2591_GAIN_HIGH:
      Serial.println(F("428x (High)"));
      break;
    case TSL2591_GAIN_MAX:
      Serial.println(F("9876x (Max)"));
      break;
  }
  Serial.print  (F("Timing:       "));
  Serial.print((tsl.getTiming() + 1) * 100, DEC); 
  Serial.println(F(" ms"));
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));
}

void CCS(String &str)
{
  //Check to see if data is ready with .dataAvailable()
  if (mySensor.dataAvailable())
  {
    //If so, have the sensor read and calculate the results.
    //Get them later
    mySensor.readAlgorithmResults();
    str = "CO2: " + String(mySensor.getCO2()) + " tVOC: " + String(mySensor.getTVOC());
//    Serial.print("  CO2[");
//    //Returns calculated CO2 reading
//    Serial.print(mySensor.getCO2());
//    Serial.print("] tVOC[");
//    //Returns calculated TVOC reading
//    Serial.print(mySensor.getTVOC());
//    Serial.print("] millis[");
//    //Simply the time since program start
//    Serial.print(millis());
//    Serial.print("]");
//    Serial.println();
  }
}

void rain(void) {
  int sensorReading = analogRead(A1);
  int range = map(sensorReading, rainSensorMin, rainSensorMax, 0, 3);
  Serial.print("  Rain: ");
  Serial.print(sensorReading);
}

void RTC(void) {
  DateTime now = rtc.now();
  Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.println();
}

