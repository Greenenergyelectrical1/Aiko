/* aiko_node.pde
 * ~~~~~~~~~~~~~
 * Please do not remove the following notices.
 * Copyright (c) 2009 by Geekscape Pty. Ltd.
 * Documentation:  http://groups.google.com/group/aiko-platform
 * Documentation:  http://geekscape.org/static/arduino.html
 * License: GPLv3. http://geekscape.org/static/arduino_license.html
 * Version: 0.2
 * ----------------------------------------------------------------------------
 * See Google Docs: "Project: Aiko: Stream protocol specification"
 * Currently requires an Aiko-Gateway.
 * ----------------------------------------------------------------------------
*/

#include <AikoEvents.h>
#include <AikoSExpression.h>

#include <OneWire.h>
#include <DallasTemperature.h>

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

using namespace Aiko;

//To use or not use the Aiko events system
//#define IS_AIKO
#define NOT_AIKO

//#define HAS_SERIAL_MIRROR

#define DEFAULT_NODE_NAME "mote"

#define DEFAULT_BAUD_RATE     38400

// Digital Input/Output pins
#define PIN_SERIAL_RX       0
#define PIN_SERIAL_TX       1
#define PIN_ONE_WIRE        5 // OneWire or CANBus
#define PIN_RELAY           7
#define PIN_LED_STATUS     13 // Standard Arduino flashing LED !

#include <avr/sleep.h>
#include <avr/wdt.h>
#define PIN_ZIGBEE_SLEEP    7 // Signal to Zibee pins 4, 6 and 7 to sleep.  High on, Low sleepy.


// Analogue Input pins
#define PIN_CURRENT_SENSOR_1  0 // Electrical monitoring
// Digital Input/Output pins
// TODO - Synch this with pebble design.

#define PIN_LIGHT_SENSOR      0
#define PIN_SOLAR_VOLTAGE     1
#define PIN_BATTERY_VOLTAGE   2

#include <PString.h>
char globalBuffer[250]; // Used to manage dynamically constructed strings
PString globalString(globalBuffer, sizeof (globalBuffer));

int skip_count = 0;

volatile boolean f_wdt = 1;

OneWire oneWire(PIN_ONE_WIRE); // Maxim DS18B20 temperature sensor
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);


void setup() {
  Serial.begin(DEFAULT_BAUD_RATE);

    // CPU Sleep Modes
    // SM2 SM1 SM0 Sleep Mode
    // 0    0  0 Idle
    // 0    0  1 ADC Noise Reduction
    // 0    1  0 Power-down
    // 0    1  1 Power-save
    // 1    0  0 Reserved
    // 1    0  1 Reserved
    // 1    1  0 Standby(1)

  cbi(SMCR, SE); // sleep enable, power down mode
  cbi(SMCR, SM0); // power down mode
  sbi(SMCR, SM1); // power down mode
  cbi(SMCR, SM2); // power down mode

  setup_watchdog(8);
}

void loop() {

    if (f_wdt == 1) { // wait for timed out watchdog / flag is set when a watchdog timeout occurs
        f_wdt = 0; // reset flag
        //Serial.println("About to nap...");
        delay(100);
        system_sleep(); // have nap
        flash();
        //Serial.println("");
        //Serial.println("...yawn");
    }

    if (skip_count > 5) {
      globalString.begin();
      skip_count = 0;
      
      temperatureSensorHandler();   
      lightSensorHandler();
      batteryVoltageHandler();
      solarVoltageHandler();
      
      sendMessage(globalString);      
      
      flash_sequence();

    }
    
    skip_count ++;
}

/* --------------------------------------------------------------------------
 ** Battery and Solar Voltage - for 3.7 v lipos direct to the device.
 */

void batteryVoltageHandler() {
    globalString += "(battery ";
    globalString += get_voltage(PIN_BATTERY_VOLTAGE);
    globalString += ")";   
}

void solarVoltageHandler() {
    globalString += "(solar ";
    globalString += get_voltage(PIN_SOLAR_VOLTAGE);
    globalString += ")";   
}

float get_voltage(int pin) {
  float this_adc = 0;
  float this_voltage = 0.0;
  this_adc = analogRead(pin); 
  this_voltage = (this_adc / 1024) * 5.04;
  return this_voltage;
}


/* --------------------------------------------------------------------------
 ** Light Sensor Handler
 */

int lightValue = 0;

void lightSensorHandler(void) {
    lightValue = analogRead(PIN_LIGHT_SENSOR);
    
    // Calibrate the light value now... TODO
   
    globalString += "(light ";
    globalString += lightValue;
    globalString += ")";    
    
}

/* --------------------------------------------------------------------------
 ** Temperature Sensor Handler
 */

void temperatureSensorHandler(void) {

  int this_index = 0;
  int resultant_temp = 0;
  int bus_devices = 0;
  byte written = false;
  sensors.begin();
      
  bus_devices = sensors.getDeviceCount();
  sensors.requestTemperatures();
      
  while (this_index < bus_devices) {     
    if (sensors.getTempCByIndex(this_index) == -127) break;
    
      globalString += "(temperature_";
      globalString += this_index + 1;
      globalString += " ";
      globalString += sensors.getTempCByIndex(this_index);
      globalString += ")";         
  
      this_index += 1; 
    }
}


//****************************************************************
// set system into the sleep state
// system wakes up when wtchdog is timed out

void system_sleep() {
    cbi(ADCSRA, ADEN); // switch Analog to Digitalconverter OFF
    set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here
    sleep_enable();
    sleep_mode(); // System sleeps here
    sleep_disable(); // System continues execution here when watchdog timed out
    sbi(ADCSRA, ADEN); // switch Analog to Digitalconverter ON
}

//****************************************************************
// 0=16ms, 1=32ms,2=64ms,3=128ms,4=250ms,5=500ms
// 6=1 sec,7=2 sec, 8=4 sec, 9= 8sec

void setup_watchdog(int ii) {

    byte bb;
    int ww;
    if (ii > 9) ii = 9;
    bb = ii & 7;
    if (ii > 7) bb |= (1 << 5);
    bb |= (1 << WDCE);
    ww = bb;
    //Serial.println(ww);

    MCUSR &= ~(1 << WDRF);
    // start timed sequence
    WDTCSR |= (1 << WDCE) | (1 << WDE);
    // set new watchdog timeout value
    WDTCSR = bb;
    WDTCSR |= _BV(WDIE);
}
//****************************************************************
// Watchdog Interrupt Service / is executed when  watchdog timed out

ISR(WDT_vect) {
    f_wdt = 1; // set global flag
}

/* --------------------------------------------------------------------------
 ** The Message Sender
 */

void sendMessage(const char* message) {

    Serial.print("(node ");
    Serial.print(DEFAULT_NODE_NAME);
    Serial.print(" ? ");
    Serial.print(message);
    Serial.println(")");
}


/* --------------------------------------------------------------------------
 ** The Clock
 */

byte second = 0;
byte minute = 0;
byte hour = 0;

void clockHandler(void) {
    if ((++second) == 60) {
        second = 0;
        if ((++minute) == 60) {
            minute = 0;
            if ((++hour) == 100) hour = 0; // Max: 99 hours, 59 minutes, 59 seconds
        }
    }
}

void resetClockCommand(void) {
    second = minute = hour = 0;
}


/* --------------------------------------------------------------------------
 ** Blink routine, flashing
 */

byte blinkInitialized = false;
byte blinkStatus = LOW;

void blinkInitialize(void) {
    pinMode(PIN_LED_STATUS, OUTPUT);

    blinkInitialized = true;
}

void blinkHandler(void) {
    if (blinkInitialized == false) blinkInitialize();

    blinkStatus = !blinkStatus;
    digitalWrite(PIN_LED_STATUS, blinkStatus);
    delay(200);
    blinkStatus = !blinkStatus;
    digitalWrite(PIN_LED_STATUS, blinkStatus);
}

void flash(void) {
    if (blinkInitialized == false) blinkInitialize();

    blinkStatus = !blinkStatus;
    digitalWrite(PIN_LED_STATUS, blinkStatus);
    delay(50);
    blinkStatus = !blinkStatus;
    digitalWrite(PIN_LED_STATUS, blinkStatus);
}

void flash_sequence(){
 flash();
 delay(100);
 flash();
 delay(100);
 flash();
 delay(100);
 flash();
 delay(100);
 flash();  
}

