// Team: Helen Truong
// CPE301 - Final Project

#include <dht.h> //install the DHTLib library

// 3pin DHT11
dht DHT;
#define DHT11_PIN 7

// water sensor
#define POWER_PIN 6
#define SIGNAL_PIN A5

#include <Stepper.h> //Includes the Arduino Stepper Library

// step motor
const int stepsPerRevolution = 2038; // Defines the number of steps per rotation
Stepper myStepper = Stepper(stepsPerRevolution, 8, 10, 9, 11); // Creates an instance of stepper class
  // Pins entered in sequence IN1-IN3-IN2-IN4 for proper step sequence

// DC motor
int speedPin = 5;
int dir1 = 4;
int dir2 = 3;
int mSpeed = 90;

int value = 0; // variable to store the sensor value

// LCD display
#include <LiquidCrystal.h>
LiquidCrystal lcd(42, 44, 46, 48, 50, 52);

// RTC module  ex code from Adafruit RTClib github repo
#include "RTClib.h"

RTC_DS1307 rtc;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  // water sensor
  pinMode (POWER_PIN, OUTPUT); // configure D8 pin as an OUTPUT
  digitalWrite (POWER_PIN, LOW); // turn the sensor OFF

  // DC motor
  pinMode(speedPin, OUTPUT);
  pinMode(dir1, OUTPUT);
  pinMode(dir2, OUTPUT);  

  // LCD display
  lcd.begin(16, 2);
  lcd.print("Hello, World!");

  // RTC module
  #ifndef ESP8266
    while (!Serial); // wait for serial port to connect. Needed for native USB
  #endif

    if (! rtc.begin()) {
      Serial.println("Couldn't find RTC");
      Serial.flush();
      while (1) delay(10);
    }

    if (! rtc.isrunning()) {
      Serial.println("RTC is NOT running, let's set the time!");
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
}

void loop() {
  // put your main code here, to run repeatedly:
  
  /* example code for 3pin DHT11
  int chk = DHT.read11(DHT11_PIN);
  Serial.print("Temperature = ");
  Serial.println(DHT.temperature);
  Serial.print("Humidity = ");
  Serial.println(DHT.humidity);
  delay(1000);  
  */  

  /* example code for water sensor
  digitalWrite (POWER_PIN, HIGH); // turn the sensor ON
  delay(10); // wait 10 milliseconds
  value = analogRead (SIGNAL_PIN); // read the analog value from sensor
  digitalWrite (POWER_PIN, LOW); // turn the sensor OFF
  Serial.print("Sensor value: " );
  Serial.println (value);
  delay(1000); 
  */

  /* example code for step motor
  // Rotate CW slowly at 5 RPM
  myStepper.setSpeed(5);
  myStepper.step(stepsPerRevolution);
  delay(1000);
  // Rotate CCW quickly at 10 RPM
  myStepper.setSpeed(10);
  myStepper.step(-stepsPerRevolution);
  delay(1000);
  */

  /* example code for DC motor
  digitalWrite(dir1, LOW);
  digitalWrite(dir2, HIGH);
  analogWrite(speedPin, 255);
  delay(25);
  analogWrite(speedPin, mSpeed);
  delay(5000);
  */

  /* example code for LCD display  
  lcd.setCursor(0, 1);
  lcd.print(millis()/1000);
  */
  

  /* example code for RTC module data printing to LCD display
  DateTime now = rtc.now();

  lcd.setCursor(0, 0);
  lcd.print(now.year(), DEC);
  lcd.print('/');
  lcd.print(now.month(), DEC);
  lcd.print('/');
  lcd.print(now.day(), DEC);
  lcd.print("(");
  lcd.print(daysOfTheWeek[now.dayOfTheWeek()]);
  lcd.print(")");
  lcd.setCursor(0, 1);
  lcd.print(now.hour(), DEC);
  lcd.print(':');
  lcd.print(now.minute(), DEC);
  lcd.print(':');
  lcd.print(now.second(), DEC);
  //lcd.println();

  //Serial.println();
  */  
  
  delay(3000);
}
