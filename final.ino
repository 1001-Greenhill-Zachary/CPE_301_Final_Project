/**************************************************************
 * Name:	      Zach Greenhill
 * Assignment:	Final Project
 * File name:   final.ino
 * Date:	      12/13/22
 * Description: Final project for CPE 301. Implements a system
                to control a swap cooler.

* Notes:        Some code was taken from previous Labs
 **************************************************************/
////////// SYSTEM OPERATION VARIABLES //////////
// Change to effect the monitored thresholds for operation
const volatile int WATER_LEVEL_LIMIT = 200;
const volatile float TEMP_LIMIT = 80; // Temprature measured in fahrenheit

////////// LIBRARIES //////////
// Libraries for RTC
#include <Wire.h>
#include "RTClib.h"

// Library for temp/humdity sensor
#include "DHT.h"

// Library for LCD
#include <LiquidCrystal.h>

// Library for Stepper Motor
#include <Stepper.h>

////////// Parameters for Libraries //////////

// Variables for RTC
RTC_DS1307 rtc;

// Variables and definitions for DHT sensor
#define DHTPIN 8
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// Parameters for LCD
const int rs = 33, en = 31, d4 = 29, d5 = 27, d6 = 25, d7 = 23;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// Parameters for stepper motor
// Stepper(Steps per revoultion, IN1, IN3, IN2, IN4)
const int stepsPerRevolution = 2038;
Stepper myStepper = Stepper(stepsPerRevolution, 13, 11, 12, 10);

////////// Direct Memory Addresses /////////

// ADC addresses 
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;

// Serial input for water sensor
volatile unsigned char* port_f = (unsigned char*) 0x31;
volatile unsigned char* ddr_f  = (unsigned char*) 0x30;
volatile unsigned char* pin_f  = (unsigned char*) 0x2F;

// Output for fan motor and LEDs
volatile unsigned char *port_h = (unsigned char*) 0x102;
volatile unsigned char *ddr_h  = (unsigned char*) 0x101;
volatile unsigned char *pin_h  = (unsigned char*) 0x100;

//Output for additional LEDs
volatile unsigned char *port_g = (unsigned char*) 0x34;
volatile unsigned char *ddr_g  = (unsigned char*) 0x33;
volatile unsigned char *pin_g  = (unsigned char*) 0x32;

// Pin2 for On/Off monitoring (PE4)
volatile unsigned char *ddr_e = (unsigned char *) 0x2D;
volatile unsigned char *port_e = (unsigned char *) 0x2E;

// Other variables
volatile bool systemOn = 0;
volatile bool moveVent = 0;
volatile int currentState = 0;
volatile int updateTime = 0;
char stateMessages[5][12] = {"DISABLED", "ERROR", "IDLE", "RUNNING", "MOVE_VENT"};

// Setup function
void setup() {
  // Set baud rate
  Serial.begin(9600);

  // Initialize the RTC
  setupRTC();

  // Initialize the ADC for water level monitoring
  adc_init();

  // Initialize DHT
  dht.begin();

  // Set up Stepper motor
  myStepper.setSpeed(8);

  // Set pin2 (PE4) to input for on/off interrupts
  *ddr_e &= 0b11101111;

  // Set up analog pin for water level sensor on PF0 (pin A0)
  *ddr_f &= 0b11111110;

  // Set up output for fan motor and LEDs
  //*ddr_h |= 0b01000000; // Fan motor only
  *ddr_h |= 0b01011000;

  // Set output for additional LEDs
  *ddr_e |= 0b00001000;
  *ddr_g |= 0b00100000;

  // Setup interrupts
  // 0 is pin_2, 1 is pin_3
  attachInterrupt(0, system_ISR, RISING);
  attachInterrupt(5, stepper_ISR, RISING);

  // Set up initial staus and displays
  printTime(currentState);
  displaySystemOff();
  ledOff();
  *port_e |= 0b00001000;
} // End Setup

void loop() {
  // delay to prevent rapid toggling of time display on button press
  delay(250);

  // Check to see if state has changed, update Serial with time if TRUE
  int newState = getState();
  if (newState != currentState) {
    printTime(newState);
    currentState = newState;

    // Update LEDs
    ledOff();
    switch (newState) {
      case 0: *port_e |= 0b00001000; break;
      case 1: *port_g |= 0b00100000; break;
      case 2: *port_h |= 0b00001000; break;
      case 3: *port_h |= 0b00010000; break;
    }

    // Update display
    lcd.clear();
    // Display for system off
    if (newState == 0) displaySystemOff();
    // Display for error
    else if (newState == 1) displayError();
    // Display temp and humidity
    else {
      updateTime = rtc.now().minute();
      displayUpdateTemp();
    }
  }

  // Select the correct functionallity
  switch(currentState) {
    case 0: fanOff(); break; 
    case 1: fanOff(); break; // Turn fan off
    case 2: fanOff(); break; // Turn fan off
    case 3: fanOn();  break; // Turn fan ON
    default: break;
  }

  // Check for error and off states, does not enter if true
  if (currentState > 1){
    // Check if it is time to update LCD
    if (updateTime != rtc.now().minute()) {
      lcd.clear();
      displayUpdateTemp();
      updateTime = rtc.now().minute();
    }

    // Check for stepper motor change
    if (moveVent) {
      printTime(4);
	    myStepper.step(stepsPerRevolution / 4);
      moveVent = 0;
    }
  }
  // Reset moveVent if button were pressed in a invalid state
  else moveVent = 0;
} // END MAIN LOOP

////////// RUN TIME OPERATIONS //////////

// Check conditions and returns the current working state
int getState() {

  //If we have entered an error state stay there until reset
  if (currentState == 1) return currentState;
  
  // System is off
  // DISABLED
  if (!systemOn) return 0;

  // Check to see if water levels are low
  // ERROR
  else if (waterLevelLow()) return 1;

  // Check to see if temp is below threshold
  // IDLE
  else if (tempLow()) return 2;

  // else turn on fan
  // RUNNING  
  else return 3;
}

// Returns true if water level falls below threshold
bool waterLevelLow() {
  if (adc_read(*pin_f) < WATER_LEVEL_LIMIT) return true;
  else return false;  
}

// Returns true if temrature is below threshold
// True = fan off
bool tempLow() {
  if (dht.readTemperature(true) < TEMP_LIMIT) return true;
  else return false;
}

// Functions to turn fan on/off
void fanOn() {
  *port_h |= 0b01000000;
}
void fanOff() {
  *port_h &= 0b10111111;
}

// LED functions
void ledOff() {
  *port_g &= 0b11011111;
  *port_e &= 0b11110111;
  *port_h &= 0b11100111;
}

///////// ISR's ////////
void system_ISR() {
  systemOn = !systemOn;
  moveVent = 0;
}

void stepper_ISR() {
  moveVent = 1;
}

////////// LCD Display //////////
void displaySystemOff() {
  lcd.begin(16, 2);
  lcd.print("System is off");
}

void displayError() {
  lcd.begin(16, 2);
  lcd.print("ERROR! Water low");
  lcd.setCursor(0,1);
  lcd.print("Refill and reset");
}

void displayUpdateTemp() {
  // Get current temprature and humidity
  int temp = (int)dht.readTemperature(true);
  int humidity = (int)dht.readHumidity();

  // Display to screen
  // Temprature
  lcd.begin(16, 2);
  lcd.print("Temp: ");
  lcd.print(temp);
  lcd.print(" F");
  
  // Humidity
  lcd.setCursor(0, 1);
  lcd.print("Humidity: ");
  lcd.print(humidity);
  lcd.print("%");      
}

///////// TIME AND DATE /////////
// Setup RTC
void setupRTC(){
  // Check for RTC
  if (!rtc.begin()) {
    Serial.println("RTC not detected");
    while (true);
  }
  // Check to see if RTC is running
  if (! rtc.isrunning()) {
    Serial.println("Error with RTC");
  }
  // Set RTC
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
}

void printTime(int i) {
  // Get date and time from RTC
  DateTime now = rtc.now();
  // Print message to Serial Monitor
  Serial.println(stateMessages[i]);
  Serial.print(now.year());
  Serial.print('/');
  Serial.print(now.month());
  Serial.print('/');
  Serial.print(now.day());
  Serial.print(" at ");
  Serial.print(now.hour());
  Serial.print(':');
  Serial.print(now.minute());
  Serial.print(':');
  Serial.print(now.second());
  Serial.println("\n");
}

////////// ADC //////////
// Initialize the ADC
void adc_init() {
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

// Get value from ADC
unsigned int adc_read(unsigned char adc_channel_num) {
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