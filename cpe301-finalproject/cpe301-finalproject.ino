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

// taken from lab 4 --> Timers
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
#define DHT11_PIN PH7 // pin 7

// water sensor
#define POWER_PIN PH3 // pin 6
#define SIGNAL_PIN PF5 // pin A5
int value = 0; // variable to store the sensor value

// step motor
#include <Stepper.h> //Includes the Arduino Stepper Library
const int stepsPerRevolution = 2038; // Defines the number of steps per rotation
Stepper myStepper = Stepper(stepsPerRevolution, 8, 10, 9, 11); // Creates an instance of stepper class
  // Pins entered in sequence IN1-IN3-IN2-IN4 for proper step sequence

// DC motor (fan)
#define speedPin PE3 // pin 5
#define dir1 PG5 // pin 4
#define dir2 PE5 // pin 3
bool fanRunning = false;

// LCD display
#include <LiquidCrystal.h>
LiquidCrystal lcd(42, 44, 46, 48, 50, 52);

// RTC module  ex code from Adafruit RTClib github repo
#include "RTClib.h"
RTC_DS1307 rtc;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

// LEDS
#define yellowLED PC2 // pin 35
#define greenLED PC4 // pin 33
#define redLED PC6 // pin 31
#define blueLED PA7 // pin 29

// buttons
#define resetButton PA1 // pin 23
#define startStopButton PA3 // pin 25
#define ventButton PA5 // pin 27

enum State {
  DISABLED,
  IDLE,
  ERROR,
  RUNNING        
};
enum State state = IDLE;

int waterThreshold = 100;
int waterLevel;
int tempThreshold = 24;
int temp;

bool startStopButtonPressed = false;

void setup() {
  U0init(9600);
  
  adc_init();

  // water sensor
  *ddr_h |= (1 << POWER_PIN); //pinMode (POWER_PIN, OUTPUT); // configure D6 pin as an OUTPUT
  *port_h &= ~(1 << POWER_PIN); //digitalWrite (POWER_PIN, LOW); // turn the sensor OFF  

  // DC motor
  *ddr_e |= (1 << speedPin); //pinMode(5, OUTPUT); //PE3 speedpin
  *ddr_g |= (1 << dir1); //pinMode(4, OUTPUT); //PG5 dir1
  *ddr_e |= (1 << dir2); //pinMode(3, OUTPUT); //PE5 dir2

  *port_g &= ~(1 << dir1); //digitalWrite(dir1, LOW);
  *port_e |= (1 << dir2); //digitalWrite(dir2, HIGH);

  // LCD display
  lcd.begin(16, 2);
  lcd.print("Hello, World!");

  // RTC module
  #ifndef ESP8266
    while (!Serial); // wait for serial port to connect. Needed for native USB
  #endif

    if (! rtc.begin()) {
      my_println("Couldn't find RTC\n");
      Serial.flush();
      while (1) delay(10);
    }

    if (! rtc.isrunning()) {
      my_println("RTC is NOT running, let's set the time!");
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
  *ddr_a &= ~(1 << ventButton); //pinMode(ventButton, INPUT);  
  
  changeState(IDLE); // makes first state IDLE so that 
  *port_a |= (1 << startStopButton); // enables pullup resistor for startStopButton
  
}


void loop() {  

  
  if (*pin_a & (1 << startStopButton)) { // check if button is pressed
    startStopButtonPressed = true;
    if(fanRunning) {
      fanOff();
    }
    if (state == DISABLED) {
      state = IDLE;
      Timestamp("IDLE State");
    } else {
      state = DISABLED;
    }
    changeState(state);
  }
  if (state != DISABLED) {
    
    // can only monitor temperature and water level in all states except DISABLED
    temp = getTemp();
    waterLevel = getWaterLevel();

    if (state == ERROR) {
      if (*pin_a & (1 << resetButton)) { // check if button is pressed
        state = IDLE;
        Timestamp("IDLE State");      
      }
    } else if (state == IDLE) {
      if (temp > tempThreshold) {
        state = RUNNING;
        fanOn();
      }
      if (waterLevel <= waterThreshold) {
        state = ERROR;
        if(fanRunning) {
          fanOff();
        } 
      }   
    } else if (state == RUNNING) {
      if (temp <= tempThreshold) {
        state = IDLE;
        Timestamp("IDLE State");
        if(fanRunning) {
          fanOff();
        }
      }
      if (waterLevel < waterThreshold) {
        state = ERROR;
        if(fanRunning) {
          fanOff();
        }
      }
    }
    changeState(state);
  }
   
}

// function to change state and do all the operations for that state
void changeState(enum State newState) {
  // turn all LEDs off
  *port_c &= ~(1 << yellowLED);
  *port_c &= ~(1 << greenLED);  
  *port_c &= ~(1 << redLED); 
  *port_a &= ~(1 << blueLED); 

  switch(newState) {
    case DISABLED: {
      *port_c |= (1 << yellowLED); // turn yellow LED on
      lcd.setCursor(0, 0);
      lcd.print("    DISABLED    ");
      lcd.setCursor(0, 1);
      lcd.print("                    ");            
      break;
    }
    case IDLE: {
      *port_c |= (1 << greenLED); // turn green LED on
      //Timestamp();
      displayTempAndHumidity(); 
      changeVentAngle();
      break;      
    }
    case ERROR: {
      *port_c |= (1 << redLED); // turn red LED on
      lcd.setCursor(0, 0);
      lcd.print("   Water level      ");
      lcd.setCursor(0, 1);
      lcd.print("   is too low       "); 
      changeVentAngle();         
      //displayTempAndHumidity(); 
      break;
    }
    case RUNNING: {
      *port_a |= (1 << blueLED); // turn blue LED on
      displayTempAndHumidity();    
      changeVentAngle();
      break;
    }

  }
}

// turns on fan motor
void fanOn() {
  // sets speedPin to HIGH
  *port_e |= (1 << speedPin); //analogWrite(5, 255);
  fanRunning = true;

  Timestamp("Fan Motor turned ON");  
}

// turns off fan motor
void fanOff() {
  // sets speedPin to LOW
  *port_e &= ~(1 << speedPin);
  fanRunning = false;

  Timestamp("Fan Motor turned OFF");
}

void changeVentAngle() {
  // if button is held down, 'vent' turns     
  myStepper.setSpeed(5);  
  if (*pin_a & (1 << ventButton)) { // check if button is pressed
    myStepper.step(5);
  }  
}

// function using the UART function to print out a full string
void my_println(String word) {
  for (char c : word) { // "for each character in word"
    U0putchar(c);    
  }
}

// time.timestamp() taken from example in library
void Timestamp(String subject) {
  DateTime time = rtc.now();
  my_println(time.timestamp(DateTime::TIMESTAMP_FULL)+" : "+subject+"\n");
}

// based on code from CPE301_Sensors slides
int getWaterLevel() {
  *port_h |= (1 << POWER_PIN); //digitalWrite (POWER_PIN, HIGH); // turn the sensor ON
  value = adc_read (5); // read the analog value from sensor
  *port_h &= ~(1 << POWER_PIN); //digitalWrite (POWER_PIN, LOW); // turn the sensor OFF
  return value;
}

int getTemp() {
  int chk = DHT.read11(DHT11_PIN);
  return DHT.temperature;
}

// based on code from CPE301_Sensors slides
void displayTempAndHumidity() {
  
  int chk = DHT.read11(DHT11_PIN);
  lcd.setCursor(0, 0);
  lcd.print("    Temp = ");
  lcd.print(DHT.temperature);
  lcd.print("  ");
  lcd.setCursor(0, 1);
  lcd.print("Humidity = ");
  lcd.print(DHT.humidity); 
 
}


// looked at Lecture-14_Interrupt slides and example looked similar to this
ISR(TIMER1_0VF_vect) {
  startStopButtonPressed = true;
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

// taken from lab 3 --> GPIO
void my_delay(unsigned int freq)
{  
  // calc period
  double period = 1.0/double(freq);
  // 50% duty cycle
  double half_period = period/ 2.0f;
  // clock period def
  double clk_period = 0.0000000625;
  // calc ticks
  unsigned int ticks = half_period / clk_period;
  // stop the timer
  *myTCCR1B &= 0xF8;
  // set the counts
  *myTCNT1 = (unsigned int) (65536 - ticks);
  // start the timer
  * myTCCR1B |= 0b00000001;
  // wait for overflow
  while((*myTIFR1 & 0x01)==0);
  // stop the timer
  *myTCCR1B &= 0xF8;
  // reset TOV           
  *myTIFR1 |= 0x01;
}

