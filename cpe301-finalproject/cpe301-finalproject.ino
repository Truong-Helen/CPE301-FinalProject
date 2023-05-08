// Team: Helen Truong
// CPE301 - Final Project

// taken from lab 7 --> for UART
#define RDA 0x80
#define TBE 0x20  

// taken from lab 5 --> UART
volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4;
volatile unsigned char *myUDR0   = (unsigned char *)0x00C6;

// taken from lab 7 --> ADC  
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;

volatile unsigned char *myTCCR1A = (unsigned char *) 0x80; // Timer/Counter 1 Control Register A
volatile unsigned char *myTCCR1B = (unsigned char *) 0x81; // Timer/Counter 1 Control Register B
volatile unsigned char *myTCCR1C = (unsigned char *) 0x82; // Timer/Counter 1 Control Register C
volatile unsigned char *myTIMSK1 = (unsigned char *) 0x6F; // Timer/Counter 1 Interrupt Mask Register
volatile unsigned int  *myTCNT1  = (unsigned  int *) 0x84; // precise form of 16-bit Timer/Counter -- accesses Timer/Counter1 counter value
volatile unsigned char *myTIFR1 =  (unsigned char *) 0x36; // Timer/Counter1 Interrupt Flag Register


volatile unsigned char* port_b = (unsigned char*) 0x25; 
volatile unsigned char* ddr_b  = (unsigned char*) 0x24; 
volatile unsigned char* pin_b  = (unsigned char*) 0x23; 

volatile unsigned char* port_h = (unsigned char*) 0x102; 
volatile unsigned char* ddr_h  = (unsigned char*) 0x101; 
volatile unsigned char* pin_h  = (unsigned char*) 0x100;

volatile unsigned char* port_e = (unsigned char*) 0x2E; 
volatile unsigned char* ddr_e  = (unsigned char*) 0x2D; 
volatile unsigned char* pin_e  = (unsigned char*) 0x2C;

volatile unsigned char* port_g = (unsigned char*) 0x34; 
volatile unsigned char* ddr_g  = (unsigned char*) 0x33; 
volatile unsigned char* pin_g  = (unsigned char*) 0x32;

volatile unsigned char* port_d = (unsigned char*) 0x2B; 
volatile unsigned char* ddr_d  = (unsigned char*) 0x2A; 
volatile unsigned char* pin_d  = (unsigned char*) 0x29; 

volatile unsigned char* port_l = (unsigned char*) 0x10B; 
volatile unsigned char* ddr_l  = (unsigned char*) 0x10A; 
volatile unsigned char* pin_l  = (unsigned char*) 0x109; 

volatile unsigned char* port_a = (unsigned char*) 0x22; 
volatile unsigned char* ddr_a  = (unsigned char*) 0x21; 
volatile unsigned char* pin_a  = (unsigned char*) 0x20; 

volatile unsigned char* port_c = (unsigned char*) 0x28; 
volatile unsigned char* ddr_c  = (unsigned char*) 0x27; 
volatile unsigned char* pin_c  = (unsigned char*) 0x26; 

// 3pin DHT11
#include <dht.h> //install the DHTLib library
dht DHT;
#define DHT11_PIN 7 //PE7 // macro for pin 7 in <pins_arduino.h>

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

// LEDS
#define yellowLED PC2 //35
#define greenLED PC4 //33
#define redLED PC6 //31
#define blueLED PA7 //29

// buttons
#define startStopButton PE3 //25
#define resetButton PE5 //23

enum State {
  DISABLED,
  IDLE,
  ERROR,
  RUNNING        
};
enum State state = IDLE;

void setup() {
  Serial.begin(9600);
  
  // water sensor
  pinMode (POWER_PIN, OUTPUT); // configure D8 pin as an OUTPUT
  digitalWrite (POWER_PIN, LOW); // turn the sensor OFF

  // DC motor
  //*ddr_e |= 0x08;//0b00001000; //pinMode(5, OUTPUT); //PE3 speedpin
  //*ddr_g |= 0x20;//0b00100000; //pinMode(4, OUTPUT); //PG5 dir1
  //*ddr_e |= 0x20;//0b00100000; //pinMode(3, OUTPUT); //PE5 dir2

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

  // LEDs
  *ddr_c |= (1 << yellowLED); //pinMode(yellowLED, OUTPUT); 
  *ddr_c |= (1 << greenLED); //pinMode(greenLED, OUTPUT); 
  *ddr_c |= (1 << redLED); //pinMode(redLED, OUTPUT); 
  *ddr_a |= (1 << blueLED); //pinMode(blueLED, OUTPUT); 
  
  // buttons
  *ddr_a &= ~(1 << startStopButton); //pinMode(startStopButton, INPUT);
  *ddr_a &= ~(1 << resetButton); //pinMode(resetButton, INPUT);  
  
}

void loop() {
  /*
  Serial.println("running DC motor");
  *port_g &= 0xDF;//0b11011111; //digitalWrite(dir1, LOW);
  *port_e |= 0x20;//0b00100000; //digitalWrite(dir2, HIGH);
  analogWrite(5, 255);
  delay(25);
  analogWrite(5, mSpeed);
  my_delay(305, port_e, 5);

  if (digitalRead(stopButton) == HIGH) {
    StopButton();
  }
  */
  if (state != DISABLED){
    changeState(DISABLED);
  }  
  //displayTempAndHumidity(); 
  //updateScreen();
  //*port_c |= (1 << yellowLED);
  //*port_c |= (1 << greenLED);  
  //*port_c |= (1 << redLED); 
  //*port_a |= (1 << blueLED); 
  
   
  //delay(1000);  
}

void changeState(enum State newState) {
  // turn all LEDs off
  *port_c &= ~(1 << yellowLED);
  *port_c &= ~(1 << greenLED);  
  *port_c &= ~(1 << redLED); 
  *port_a &= ~(1 << blueLED); 

  switch(newState) {
    case DISABLED:
      state = DISABLED;
      lcd.setCursor(0, 0);
      lcd.print("    DISABLED    ");
      *port_c |= (1 << yellowLED); // turn LED on
      break;

  }
}

void StopButton() {
  if (digitalRead(dir2) == HIGH) {
    Serial.println("stopButton pressed");    
    digitalWrite(dir2, LOW);
    delay(5000);
    
  }
  //digitalWrite(stopButton, LOW);
}

void displayTempAndHumidity() {
  

  
  int chk = DHT.read11(DHT11_PIN);
  lcd.setCursor(0, 0);
  lcd.print("    Temp = ");
  lcd.print(DHT.temperature);
  lcd.print("  ");
  lcd.setCursor(0, 1);
  lcd.print("Humidity = ");
  lcd.print(DHT.humidity);  
  /*
  int chk = DHT.read11(DHT11_PIN);
  
  Serial.print("    Temp = ");
  Serial.println(DHT.temperature);
  Serial.print("Humidity = ");
  Serial.println(DHT.humidity);
  */
}

void updateScreen() { 

  unsigned long currentTime = millis();
  static unsigned long previousTime = 0;
  const unsigned long interval = 60000;  // Every minute
  if (currentTime - previousTime >= interval) {
    previousTime += interval;
    displayTempAndHumidity();
  }
}

// taken from lab 7 --> ADC
void adc_init()
{
  // setup the A register
  *my_ADCSRA |= 0b10000000; // set bit   7 to 1 to enable the ADC
  *my_ADCSRA &= 0b11011111; // clear bit 6 to 0 to disable the ADC trigger mode
  *my_ADCSRA &= 0b11110111; // clear bit 5 to 0 to disable the ADC interrupt
  *my_ADCSRA &= 0b11111000; // clear bit 0-2 to 0 to set prescaler selection to slow reading
  // setup the B register
  *my_ADCSRB &= 0b11110111; // clear bit 3 to 0 to reset the channel and gain bits
  *my_ADCSRB &= 0b11111000; // clear bit 2-0 to 0 to set free running mode
  // setup the MUX Register
  *my_ADMUX  &= 0b01111111; // clear bit 7 to 0 for AVCC analog reference
  *my_ADMUX  |= 0b01000000; // set bit   6 to 1 for AVCC analog reference
  *my_ADMUX  &= 0b11011111; // clear bit 5 to 0 for right adjust result
  *my_ADMUX  &= 0b11100000; // clear bit 4-0 to 0 to reset the channel and gain bits
}

// taken from lab 7 --> ADC
unsigned int adc_read(unsigned char adc_channel_num)
{
  // clear the channel selection bits (MUX 4:0)
  *my_ADMUX  &= 0b11100000;
  // clear the channel selection bits (MUX 5)
  *my_ADCSRB &= 0b11110111;
  // set the channel number
  if(adc_channel_num > 7)
  {
    // set the channel selection bits, but remove the most significant bit (bit 3)
    adc_channel_num -= 8;
    // set MUX bit 5
    *my_ADCSRB |= 0b00001000;
  }
  // set the channel selection bits
  *my_ADMUX  += adc_channel_num;
  // set bit 6 of ADCSRA to 1 to start a conversion
  *my_ADCSRA |= 0x40;
  // wait for the conversion to complete
  while((*my_ADCSRA & 0x40) != 0);
  // return the result in the ADC data register
  return *my_ADC_DATA;
}

// taken from lab 5 --> UART
void U0init(int U0baud)
{
 unsigned long FCPU = 16000000;
 unsigned int tbaud;
 tbaud = (FCPU / 16 / U0baud - 1);
 // Same as (FCPU / (16 * U0baud)) - 1;
 *myUCSR0A = 0x20;
 *myUCSR0B = 0x18;
 *myUCSR0C = 0x06;
 *myUBRR0  = tbaud;
}

// taken from lab 5 --> UART
unsigned char U0kbhit()
{
  return *myUCSR0A & RDA;
}

// taken from lab 5 --> UART
unsigned char U0getchar()
{
  return *myUDR0;
}

// taken from lab 5 --> UART
void U0putchar(unsigned char U0pdata)
{
  while((*myUCSR0A & TBE)==0);
  *myUDR0 = U0pdata;
}

// taken from lab 4 --> Timers
void my_delay(unsigned int freq, volatile unsigned char* port, int portIndex)
{

  double period = 1.0/double(freq); // calc period
  double half_period = period/ 2.0f; // 50% duty cycle
  double clk_period = 0.0000000625; // clock period def
  unsigned int ticks = half_period / clk_period; // calc ticks

  while (freq > 0) {

    *port |= (1 << portIndex);  

    *myTCCR1B &= 0xF8; // stop the timer
    *myTCNT1 = (unsigned int) (65536 - ticks); // set the counts
    *myTCCR1B |= 0b00000001; // start the timer 
    while((*myTIFR1 & 0x01)==0); // wait for overflow // 0b 0000 0000
    *myTCCR1B &= 0xF8;   // stop the timer // 0b 0000 0000
    *myTIFR1 |= 0x01; // reset TOV

    *port &= ~(1 << portIndex);

    *myTCCR1B &= 0xF8; // stop the timer
    *myTCNT1 = (unsigned int) (65536 - ticks); // set the counts
    * myTCCR1B |= 0b00000001; // start the timer 
    while((*myTIFR1 & 0x01)==0); // wait for overflow // 0b 0000 0000
    *myTCCR1B &= 0xF8;   // stop the timer // 0b 0000 0000
    *myTIFR1 |= 0x01; // reset TOV
  }
}
