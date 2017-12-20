#define I2C_ADDRESS 0x3C
#include "SSD1306Ascii.h"
#include "SSD1306AsciiAvrI2c.h"
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiAvrI2c.h"
#include <OneWire.h>
#include <DallasTemperature.h>

#define ARDUINO_GPS_RX 3
#define ARDUINO_GPS_TX 4
#define GPS_BAUD 9600
#define gpsPort ssGPS
#define SerialMonitor Serial
#define I2C_ADDRESS 0x3C
#define Pin 2
TinyGPSPlus tinyGPS;
SoftwareSerial ssGPS(ARDUINO_GPS_TX, ARDUINO_GPS_RX);
OneWire ourWire(Pin);
DallasTemperature sensors(&ourWire);

int led1 = 13;
int led2 = 12;
int led3 = 11;
int led4 = 10;

SSD1306AsciiAvrI2c oled;

void setup()
{
  SerialMonitor.begin(9600);
  gpsPort.begin(GPS_BAUD);
  oled.begin(&Adafruit128x64, I2C_ADDRESS);
  oled.setFont(System5x7);
  oled.clear();
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(led3, OUTPUT);
  pinMode(led4, OUTPUT);
  sensors.begin();
}

void loop()
{
  printGPSInfo();
  smartDelay(1000);
  oled.setCursor(0,0);


  if (Serial.available() > 0)
  { //cierre del segundo if
    char datos_serial = Serial.read();

    if (datos_serial == 'A')
    {
      Serial.println("enciende 1");
      digitalWrite(led1, HIGH);
      delay(2000);
      digitalWrite(led1, LOW);
    }
    if (datos_serial == 'B')
    {
      Serial.println("enciende 2");
      digitalWrite(led2, HIGH);
      delay(2000);
      digitalWrite(led2, LOW);
    }
    if (datos_serial == 'C')
    {
      Serial.println("enciende 3");
      digitalWrite(led3, HIGH);
      delay(2000);
      digitalWrite(led3, LOW);
    }
    if (datos_serial == 'D')
    {
      Serial.println("enciende4");
      digitalWrite(led4, HIGH);
      delay(2000);
      digitalWrite(led4, LOW);
    }
  }

}

void printGPSInfo()
{

  //SerialMonitor.print("Lat: "); SerialMonitor.println(tinyGPS.location.lat(), 6);
  //SerialMonitor.print("Long: "); SerialMonitor.println(tinyGPS.location.lng(), 6);
  //SerialMonitor.print("Alt: "); SerialMonitor.println(tinyGPS.altitude.feet());
  //SerialMonitor.print("Course: "); SerialMonitor.println(tinyGPS.course.deg());
  //SerialMonitor.print("Speed: "); SerialMonitor.println(tinyGPS.speed.mph());
  //SerialMonitor.print("Date: "); printDate();
  //SerialMonitor.print("Time: "); printTime();
  //SerialMonitor.print("Sats: "); SerialMonitor.println(tinyGPS.satellites.value());
  //SerialMonitor.println();

SerialMonitor.print(tinyGPS.location.lat(), 6);
SerialMonitor.print(",");
SerialMonitor.print(tinyGPS.location.lng(), 6);
SerialMonitor.print(",");
sensors.requestTemperatures();       
Serial.println(sensors.getTempCByIndex(0));
  
  oled.print("Lat: "); oled.println(tinyGPS.location.lat(), 6);
  oled.print("Long: "); oled.println(tinyGPS.location.lng(), 6);
  //oled.print("Alt: "); oled.println(tinyGPS.altitude.feet());
  //oled.print("Course: "); oled.println(tinyGPS.course.deg());
  //oled.print("Speed: "); oled.println(tinyGPS.speed.mph());
  oled.print("Date: "); printDate();
  oled.print("Time: "); printTime();
  //oled.print("Sats: "); oled.println(tinyGPS.satellites.value());
  oled.print(sensors.getTempCByIndex(0)); 
  oled.println();

}


static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {

    while (gpsPort.available())
      tinyGPS.encode(gpsPort.read());
  } while (millis() - start < ms);
}


void printDate()
{
  /*SerialMonitor.print(tinyGPS.date.day());
  SerialMonitor.print("/");
  SerialMonitor.print(tinyGPS.date.month());
  SerialMonitor.print("/");
  SerialMonitor.println(tinyGPS.date.year());*/

  oled.print(tinyGPS.date.day());
  oled.print("/");
  oled.print(tinyGPS.date.month());
  oled.print("/");
  oled.println(tinyGPS.date.year());
}

void printTime()
{
  /*SerialMonitor.print(tinyGPS.time.hour());
  SerialMonitor.print(":");
  if (tinyGPS.time.minute() < 10) SerialMonitor.print('0');
  SerialMonitor.print(tinyGPS.time.minute());
  SerialMonitor.print(":");
  if (tinyGPS.time.second() < 10) SerialMonitor.print('0');
  SerialMonitor.println(tinyGPS.time.second());*/

  oled.print(tinyGPS.time.hour());
  oled.print(":");
  if (tinyGPS.time.minute() < 10) oled.print('0');
  oled.print(tinyGPS.time.minute());
  oled.print(":");
  if (tinyGPS.time.second() < 10) oled.print('0');
  oled.println(tinyGPS.time.second());
}
