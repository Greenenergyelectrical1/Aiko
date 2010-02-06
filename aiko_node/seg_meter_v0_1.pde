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
 *
 * Third-Party libraries
 * ~~~~~~~~~~~~~~~~~~~~~
 * These libraries are not included in the Arduino IDE and
 * need to be downloaded and installed separately.
 *
 * - LiquidCrystal
 *   http://arduino.cc/en/Reference/LiquidCrystal
 *
 * - One-Wire
 *   http://www.arduino.cc/playground/Learning/OneWire
 *
 * - NewSoftSerial
 *   http://arduiniana.org/libraries/newsoftserial
 *
 * - PString
 *   http://arduiniana.org/libraries/pstring
 *
 * To Do
 * ~~~~~
 * - Put protocol version into boot message to Aiko-Gateway.
 * - Verify protocol version in the Aiko-Gateway boot message.
 * - Default baud rate 38,400 and auto-baud to 115,200.
 * - Fix temperature data acquisition should work every time, not every second time !
 * - Temperature sensor won't need 750 ms, if using permanent 5 VDC.
 * - Handle "(node= name)", where "name" greater than 40 characters.
 *   - "name" parameter should be delimited by double-quotes.
 * - Complete serialHandler() communications.
 * - Think about what happens when reusing "SExpressionArray commandArray" ?
 * - Implement: addCommandHandler() and removeCommandHandler().
 *   - This will be neater than the current ugly #ifdef / #endif arrangement.
 * - Implement: (update_rate SECONDS UNIT)
 * - Implement: (error on) (error off)
 * - Implement: (display_title= STRING) --> LCD position (0,0)
 * - Implement: (device_update NAME VALUE UNIT) or (NAME= VALUE)
 * - Implement: (profile)
 * - Implement: (clock yyyy-mm-ddThh:mm:ss)
 * - Improve error handling.
 */

#include <AikoEvents.h>
#include <AikoSExpression.h>

#include <OneWire.h>


#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

using namespace Aiko;


#define IS_STONE
//#define IS_3_CHANNEL
//#define IS_STONE_DEPRECATED

//#define IS_SLEEPY
#define NOT_SLEEPY

//#define HAS_SERIAL_MIRROR

#define DEFAULT_NODE_NAME "seg_meter_3"
//#define DEFAULT_NODE_NAME "seg_meter_3"
#define DEFAULT_TRANSMIT_RATE    5  // seconds
//#define STONE_DEBUG  // Enable capture and dump of all sampled values

//#define CALIBRATION  8  // Hall Effect with 3 passes thru opening
//#define CALIBRATION  24  // Hall Effect with 1 pass thru opening
#define CALIBRATION  32  // 10 mV per Amp, analog reference = internal (1 VDC)
//#define CALIBRATION   144.45  // 10 mV per Amp, analog reference = default  (5 VDC)
#define CHANNELS          3   // Current Clamp(s)
#define SAMPLES        2000   // Current samples to take
#define AVERAGES         10   // Number of RMS values to average

#define DEFAULT_BAUD_RATE     38400

// Digital Input/Output pins
#define PIN_SERIAL_RX       0
#define PIN_SERIAL_TX       1
#define PIN_ONE_WIRE        5 // OneWire or CANBus
#define PIN_RELAY           6
#define PIN_LED_STATUS     13 // Standard Arduino flashing LED !

#ifdef IS_SLEEPY
#include <avr/sleep.h>
#include <avr/wdt.h>
#define PIN_ZIGBEE_SLEEP    7 // Signal to Zibee pins 4, 6 and 7 to sleep.  High on, Low sleepy.
#endif

// Analogue Input pins
#define PIN_CURRENT_SENSOR_1  0 // Electrical monitoring
// Digital Input/Output pins
// TODO - Synch this with pebble design.

#ifdef IS_3_CHANNEL
// Analogue Input pins not used for 6 channel sensing
#define PIN_LIGHT_SENSOR    3
#define PIN_VOLTAGE_SENSOR  4
#endif

#include <PString.h>
char globalBuffer[200];  // Used to manage dynamically constructed strings
PString globalString(globalBuffer, sizeof(globalBuffer));

void (*commandHandlers[])() = {
  baudRateCommand,
  nodeCommand,
  relayCommand,
  resetClockCommand,
  transmitRateCommand
};

char* commands[] = {
  "baud=",
  "node=",
  "relay",
  "reset_clock",
  "transmit="
};

char* eepromKeyword[] = {
  0,  // "bd",
  "nd",
  0,
  0,
  0,
  0   // "tr"
};

byte parameterCount[] = {  // ToDo: Change this to incorporate parameter type ?
  1,  // baud rate       (integer)
  1,  // node name       (string)
  1,  // relay state     (boolean)
  0,  // reset clock     (none)
  1   // transmit rate   (integer seconds)
};

int skipCount = 0;
byte commandCount = sizeof(commands) / sizeof(*commands);
volatile boolean f_wdt=1;

SExpression parameter;

void setup() {
// no ref means that it's at the 5Volts, perfect for the voltage handler.
//analogReference(EXTERNAL);
analogReference(INTERNAL);

  // handlers set to 1 milisecond to use sleep mode
  Events.addHandler(serialHandler,  2000);  // Sufficient for 38,400 baud
  Events.addHandler(blinkHandler,   2000);
  Events.addHandler(nodeHandler,    2000);
  
#ifdef IS_SLEEPY
  
    // CPU Sleep Modes 
  // SM2 SM1 SM0 Sleep Mode
  // 0    0  0 Idle
  // 0    0  1 ADC Noise Reduction
  // 0    1  0 Power-down
  // 0    1  1 Power-save
  // 1    0  0 Reserved
  // 1    0  1 Reserved
  // 1    1  0 Standby(1)

  cbi( SMCR,SE );      // sleep enable, power down mode
  cbi( SMCR,SM0 );     // power down mode
  sbi( SMCR,SM1 );     // power down mode
  cbi( SMCR,SM2 );     // power down mode

  setup_watchdog(7);

#endif



#ifdef HAS_SENSORS
  // Delay is 1 to deal with the sleepy sleep
  Events.addHandler(lightSensorHandler,       2000);
  Events.addHandler(temperatureSensorHandler, 2000);
  Events.addHandler(voltageSensorHandler,     2000);
#endif

#ifdef HAS_SERIAL_MIRROR
  Events.addHandler(serialMirrorHandler, 30);  // Sufficient for 38,400 baud
#endif

//Events.addHandler(currentSensorHandler, 1000 * DEFAULT_TRANSMIT_RATE);
  Events.addHandler(currentClampHandler,  1);
  Events.addHandler(powerOutputHandler,   1000 * DEFAULT_TRANSMIT_RATE);

}

void loop() {
  
#ifdef IS_SLEEPY
  if (f_wdt==1) {   // wait for timed out watchdog / flag is set when a watchdog timeout occurs
    f_wdt=0;        // reset flag
    
    if (skipCount == 10) {
      skipCount = 0;
      
      digitalWrite(PIN_ZIGBEE_SLEEP, HIGH);
      delay(100);    // time to power up
      Events.loop();  // do stuff
      delay(500);    // to allow the send of the data over serial
      digitalWrite(PIN_ZIGBEE_SLEEP, LOW);
      delay(100);    // time to power down
    }
    else {
      skipCount += 1;
    }
    
    system_sleep(); // have nap
  }
#endif

#ifdef NOT_SLEEPY
  Events.loop();  // do stuff
#endif
}

/* -------------------------------------------------------------------------- 
** Current measurement oscillating around zero volts.
*/

#ifdef IS_STONE
byte currentClampInitialized = false;

float watts[CHANNELS];                  // Instantaneous
float wattHoursSum[CHANNELS];           // Cumulative
float wattHoursSumLast[CHANNELS];       // For calculating instantaneous wattHours
unsigned long wattHoursTime[CHANNELS];  // Time of last calculation

void currentClampInitialize(void) {
//analogReference(INTERNAL);

  for (int channel = 0; channel < CHANNELS; channel ++) {
    watts[channel]            = 0.0;
    wattHoursSum[channel]     = 0.0;
    wattHoursSumLast[channel] = 0.0;
    wattHoursTime[channel]    = millis();
  }

  currentClampInitialized = true;
}

void currentClampHandler() {
  if (currentClampInitialized == false) currentClampInitialize();

  for (int channel = 0; channel < CHANNELS; channel ++) {
    int   pin = PIN_CURRENT_SENSOR_1 + channel;
    int   sample;
    float watts_sum = 0;

    for (int average = 0; average < AVERAGES; average ++) {
      float rms = 0;

      for (int index = 0; index < SAMPLES; index ++) {
        sample = analogRead(pin);
        rms    = rms + sq((float) sample);
      }

      rms = sqrt(rms / (SAMPLES / 2) );
      if (rms < 0.5) rms = 0;
      
      watts_sum = watts_sum + (rms * CALIBRATION);
    }

    watts[channel] = watts_sum / AVERAGES;

    unsigned long timeNow = millis();

    if (timeNow > wattHoursTime[channel]) {  // Sanity check, in case millis() wraps around
      unsigned long duration = timeNow - wattHoursTime[channel];

      wattHoursSum[channel] = wattHoursSum[channel] + (watts[channel] * (duration / 1000.0 / 3600.0));
    }

    wattHoursTime[channel] = timeNow;
/*
    Serial.print("Watts ");
    Serial.print(channel + 1);
    Serial.print(": ");
    Serial.println(watts[channel]);

    Serial.print("WattHoursSum ");
    Serial.print(channel + 1);
    Serial.print(": ");
    Serial.println(wattHoursSum[channel]);
 */
  }
}

void powerOutputHandler() {
  globalString.begin();

  for (int channel = 0; channel < CHANNELS; channel ++) {
    float wattHours = wattHoursSum[channel] - wattHoursSumLast[channel];

    wattHoursSumLast[channel] = wattHoursSum[channel];

    globalString += "(power_";
    globalString += channel + 1;
    globalString += "_w ";
    globalString += watts[channel];
    globalString += " W)";

    globalString += "(energy_";
    globalString += channel + 1;
    globalString += "_wh ";
    globalString += wattHours;
    globalString += " Wh)";
  }

  sendMessage(globalString);

/*  globalString.begin();

  for (int channel = 0; channel < CHANNELS; channel ++) {
    globalString += "(energy_sum_";
    globalString += channel + 1;
    globalString += "_whs ";
    globalString += wattHoursSum[channel];
    globalString += " Wh)";
  }

  sendMessage(globalString);
*/
}
#endif

/* -------------------------------------------------------------------------- 
** Hall effect sensor wriggling around the middle
*/

byte relayInitialized = false;

void relayInitialize(void) {
  pinMode(PIN_RELAY,   OUTPUT);
#ifdef HAS_SPEAKER
  pinMode(PIN_SPEAKER, OUTPUT);
#endif

  relayInitialized = true;
}

boolean relay_state = false;

void relayCommand(void) {
  if (relayInitialized == false) relayInitialize();

  if (parameter.isEqualTo("on")) {
relay_state = true;
    digitalWrite(PIN_RELAY, HIGH);
    Serial.println("(relay is on)");
#ifdef HAS_SPEAKER
    playTune();
#endif
  }
  else if (parameter.isEqualTo("off")) {
relay_state = false;
    digitalWrite(PIN_RELAY, LOW);
    Serial.println("(relay is off)");
  }
  else {
//  sendMessage("(error parameterInvalid)");
  }
}



/* -------------------------------------------------------------------------- 
** Current sensor handler, Not sure exactly, but I think this is deprecatable
*/

#ifdef IS_STONE_DEPRECATED
byte currentSensorInitialized = false;

#define CURRENT_SIZE   10

#ifdef STONE_DEBUG
#define SAMPLE_SIZE 100
#else
#define SAMPLE_SIZE  5000
#endif

float current_average[CURRENT_SIZE];
int   current_index = 0;

void currentSensorInitialize(void) {
//analogReference(INTERNAL);

  currentSensorInitialized = true;

  for (int index = 0;  index < CURRENT_SIZE;  index ++) {
    current_average[index] = 0.0;
  }
}

void currentSensorHandler(void) {
  if (currentSensorInitialized == false) currentSensorInitialize();

  long raw_average = 0;
  int  raw_minimum = 2048;
  int  raw_maximum = 0;
  int  sample;
  long timer;
#ifdef STONE_DEBUG
  int  samples[SAMPLE_SIZE];
  long timers[SAMPLE_SIZE];
#endif

  float rms_current = 0.0;
  float runtime_average = 0.0;

  for (int index = 0;  index < SAMPLE_SIZE;  index ++) {
    timer = micros();
    sample = analogRead(PIN_CURRENT_SENSOR_1);
#ifdef STONE_DEBUG
    timers[index] = timer;
    samples[index] = sample;
#endif
    if (sample < raw_minimum) raw_minimum = sample;
    if (sample > raw_maximum) raw_maximum = sample;

    // Should dynamically use average, replace hard-coded "537"!
    rms_current = rms_current + sq((float) (sample - 537));
    raw_average = raw_average + sample;

#ifdef STONE_DEBUG
    delayMicroseconds(10000 - micros() + timer - 8);  // 100 samples per second
#else
    delayMicroseconds(200 - micros() + timer - 8);  // 5,000 samples per second
#endif
  }

#ifdef STONE_DEBUG
  for (int index = 0;  index < SAMPLE_SIZE;  index ++) {
    Serial.print(samples[index]);
    Serial.print(",");
    Serial.println(timers[index] - timers[0]);
  }
#endif

  rms_current = sqrt(rms_current / (SAMPLE_SIZE));
  raw_average = raw_average / SAMPLE_SIZE;

#ifdef STONE_DEBUG
  Serial.println("----------");
  Serial.print("RMS current (pre-correction): ");
  Serial.println(rms_current);
#endif

  // Hard-coded correction factor, replace "32.6" :(
  float correction_factor = 32.6;
  if (rms_current < 30.00) correction_factor = 34.09;
  rms_current = rms_current / correction_factor;

  current_average[current_index] = rms_current;
  current_index = (current_index + 1) % CURRENT_SIZE;

  for (int index = 0;  index < CURRENT_SIZE;  index ++) {
    runtime_average = runtime_average + current_average[index];
  }

  runtime_average = runtime_average / CURRENT_SIZE;

  int watts = rms_current * 248;
  if (watts < 70) watts = 0;

//watts = relay_state  ?  100  :  222;

  char unitName[] = { '1', '2', '3', '4', '5' };

  globalString.begin();

  for (int unit = 0; unit < sizeof(unitName); unit ++) {
//  globalString += "(power ";
    globalString += "(hvac_";
    globalString += unitName[unit];
    globalString += "w ";
    globalString += watts + unit;
    globalString += " W)";

    globalString += "(hvac_";
    globalString += unitName[unit];
    globalString += "wh ";
    globalString += watts + unit + 300;
    globalString += " Wh)";
  }

  sendMessage(globalString);

#ifdef STONE_DEBUG
  Serial.println("----------");
  Serial.print("Raw average: ");
  Serial.println(raw_average);
  Serial.print("Raw minimum: ");
  Serial.println(raw_minimum);
  Serial.print("Raw maximum: ");
  Serial.println(raw_maximum);
  Serial.print("RMS Current: ");
  Serial.println(rms_current);
  // Assume Power Factor of 1.0 for the moment
  Serial.print("RMS Current (average): ");
  Serial.println(runtime_average);
#endif
}
#endif


#ifdef IS_SLEEPY
//****************************************************************  
// set system into the sleep state 
// system wakes up when wtchdog is timed out

void system_sleep() {
  cbi(ADCSRA,ADEN);                    // switch Analog to Digitalconverter OFF
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here
  sleep_enable();
  sleep_mode();                        // System sleeps here
  sleep_disable();                     // System continues execution here when watchdog timed out 
  sbi(ADCSRA,ADEN);                    // switch Analog to Digitalconverter ON
}

//****************************************************************
// 0=16ms, 1=32ms,2=64ms,3=128ms,4=250ms,5=500ms
// 6=1 sec,7=2 sec, 8=4 sec, 9= 8sec
void setup_watchdog(int ii) {

  byte bb;
  int ww;
  if (ii > 9 ) ii=9;
  bb=ii & 7;
  if (ii > 7) bb|= (1<<5);
  bb|= (1<<WDCE);
  ww=bb;
  Serial.println(ww);

  MCUSR &= ~(1<<WDRF);
  // start timed sequence
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  // set new watchdog timeout value
  WDTCSR = bb;
  WDTCSR |= _BV(WDIE);
}
//****************************************************************  
// Watchdog Interrupt Service / is executed when  watchdog timed out
ISR(WDT_vect) {
  f_wdt=1;  // set global flag
}

#endif

#ifdef IS_3_CHANNEL

/* -------------------------------------------------------------------------- 
** Voltage Sensor
*/

float voltageRaw = 0;
float voltageValue = 0;

void voltageSensorHandler(void) {
  
  voltageValue = 0;
  
  voltageRaw = analogRead(PIN_VOLTAGE_SENSOR);
  
  // Divide out the ratio from the ADC and multiply by Aref
  // Then multiply by 3 (note R1 = 200 and R2 = 100)
  voltageValue = (voltageRaw/1024) * 5.15 * 3;

  //Serial.print("(voltage ");
  //Serial.print(voltageValue);
  //Serial.println(" V)");
  
  globalString.begin();
  globalString += "(voltage ";
  globalString += voltageValue;
  globalString += " V)";
  
  sendMessage(globalString);  
 }
 
 #endif

/* -------------------------------------------------------------------------- 
** Blink routine, flashing
*/

byte blinkInitialized = false;
byte blinkStatus      = LOW;

void blinkInitialize(void) {
  pinMode(PIN_LED_STATUS, OUTPUT);

  blinkInitialized = true;
}

void blinkHandler(void) {
  if (blinkInitialized == false) blinkInitialize();

  blinkStatus = ! blinkStatus;
  digitalWrite(PIN_LED_STATUS, blinkStatus);
  delay(200);
  blinkStatus = ! blinkStatus;
  digitalWrite(PIN_LED_STATUS, blinkStatus);
}

/* -------------------------------------------------------------------------- 
** The Clock
*/

byte second = 0;
byte minute = 0;
byte hour   = 0;

void clockHandler(void) {
  if ((++ second) == 60) {
    second = 0;
    if ((++ minute) == 60) {
      minute = 0;
      if ((++ hour) == 100) hour = 0;  // Max: 99 hours, 59 minutes, 59 seconds
    }
  }
}

void resetClockCommand(void) {
  second = minute = hour = 0;
}

/* -------------------------------------------------------------------------- 
** Node handler
*/

char nodeName[40] = DEFAULT_NODE_NAME;

void nodeHandler(void) {
  sendMessage("");
}

void nodeCommand(void) {
  char* parameterString = parameter.head();

  for (byte index = 0; index < sizeof(nodeName); index ++) {
    if (index == parameter.size()) {
      nodeName[index] = '\0';
      break;
    }

    nodeName[index] = *parameterString ++;
  }
}

void sendMessage(const char* message) {
  Serial.print("(node ");
  Serial.print(nodeName);
  Serial.print(" ? ");
  Serial.print(message);
  Serial.println(")");
}

#ifdef IS_3_CHANNEL

/* -------------------------------------------------------------------------- 
** Light Sensor Handler
*/


int lightValue = 0;

void lightSensorHandler(void) {
  lightValue = analogRead(PIN_LIGHT_SENSOR);

  globalString.begin();
  globalString  = "(light_lux ";
  globalString += lightValue;
  globalString += " lux)";
  sendMessage(globalString);
}

#endif

/* -------------------------------------------------------------------------- 
** Temperature Sensor
*/

OneWire oneWire(PIN_ONE_WIRE);  // Maxim DS18B20 temperature sensor

byte oneWireInitialized = false;

#define ONE_WIRE_COMMAND_READ_SCRATCHPAD  0xBE
#define ONE_WIRE_COMMAND_START_CONVERSION 0x44
#define ONE_WIRE_COMMAND_MATCH_ROM        0x55
#define ONE_WIRE_COMMAND_SKIP_ROM         0xCC

#define ONE_WIRE_DEVICE_18B20  0x28
#define ONE_WIRE_DEVICE_18S20  0x10

int temperature_whole = 0;
int temperature_fraction = 0;

/*
void processOneWireListDevices(void) {
  byte address[8];
 
  oneWire.reset_search();
 
  while (oneWire.search(address)) {
    if (OneWire::crc8(address, 7) == address[7]) {
      if (address[0] == ONE_WIRE_DEVICE_18B20) {
// Display device details
      }
    }
  }
}
*/
void temperatureSensorHandler(void) {  // total time: 33 milliseconds
  byte address[8];
  byte data[12];
  byte index;

  if (! oneWire.search(address)) {  // time: 14 milliseconds
//  Serial.println("(error 'No more one-wire devices')");
    oneWire.reset_search();         // time: <1 millisecond
    return;
  }
/*
  Serial.print("OneWire device: ");
  for (index = 0; index < 8; index ++) {
    Serial.print(address[index], HEX);
    Serial.print(" ");
  }
  Serial.println();
 */
  if (OneWire::crc8(address, 7) != address[7]) {
//  sendMessage("(error 'Address CRC is not valid')");
    return;
  }

  if (address[0] != ONE_WIRE_DEVICE_18B20) {
//  sendMessage("(error 'Device is not a DS18B20')");
    return;
  }

  if (oneWireInitialized) {
    byte present = oneWire.reset();                   // time: 1 millisecond
    oneWire.select(address);                          // time: 5 milliseconds
    oneWire.write(ONE_WIRE_COMMAND_READ_SCRATCHPAD);  // time: 1 millisecond

    for (index = 0; index < 9; index++) {             // time: 5 milliseconds
      data[index] = oneWire.read();
    }
/*
    Serial.print("Scratchpad: ");
    Serial.print(present, HEX);
    Serial.print(" ");
    for (index = 0; index < 9; index++) {
      Serial.print(data[index], HEX);
      Serial.print(" ");
    }
    Serial.println();
 */
    if (OneWire::crc8(data, 8) != data[8]) {
//    sendMessage("(error 'Data CRC is not valid')");
      return;
    }

    int temperature = (data[1] << 8) + data[0];
    int signBit     = temperature & 0x8000;
    if (signBit) temperature = (temperature ^ 0xffff) + 1;  // 2's complement

    int tc_100 = (6 * temperature) + temperature / 4;  // multiply by 100 * 0.0625

    temperature_whole    = tc_100 / 100;
    temperature_fraction = tc_100 % 100;

    globalString.begin();
    globalString  = "(temperature ";
    if (signBit) globalString += "-";
    globalString += temperature_whole;
    globalString += ".";
    if (temperature_fraction < 10) globalString += "0";
    globalString += temperature_fraction;
    globalString += " C)";
    sendMessage(globalString);
  }

  // Start temperature conversion with parasitic power
  oneWire.reset();                                      // time: 1 millisecond
  oneWire.select(address);                              // time: 5 milliseconds
  oneWire.write(ONE_WIRE_COMMAND_START_CONVERSION, 1);  // time: 1 millisecond

  // Must wait at least 750 milliseconds for temperature conversion to complete
  oneWireInitialized = true;
}



/* -------------------------------------------------------------------------- 
** Baud Rate Handler
*/

int baudRate = DEFAULT_BAUD_RATE;

void baudRateCommand(void) {
  char* parameterString = parameter.head();
}

/* -------------------------------------------------------------------------- 
** Transmit rate command
*/

int transmitRate = DEFAULT_TRANSMIT_RATE;  // seconds

void transmitRateCommand(void) {
  char* parameterString = parameter.head();
}

/* -------------------------------------------------------------------------- */
/*
 * Arduino serial buffer is 128 characters.
 * At 115,200 baud (11,520 cps) the buffer is filled 90 times per second.
 * Need to run this handler every 10 milliseconds.
 *
 * At 38,400 baud (3,840 cps) the buffer is filled 30 times per second.
 * Need to run this handler every 30 milliseconds.
 */

SExpressionArray commandArray;

byte serialHandlerInitialized = false;

void serialHandlerInitialize(void) {
  Serial.begin(DEFAULT_BAUD_RATE);

  serialHandlerInitialized = true;
}

void serialHandler(void) {
  static char buffer[32];
  static byte length = 0;
  static long timeOut = 0;

  if (serialHandlerInitialized == false) serialHandlerInitialize();

  unsigned long timeNow = millis();
  int count = Serial.available();

  if (count == 0) {
    if (length > 0) {
      if (timeNow > timeOut) {
//      sendMessage("(error timeout)");
        length = 0;
      }
    }
  }
  else {
/*  globalString.begin();
    globalString  = "(info readCount ";
    globalString += count;
    globalString += ")";
    sendMessage(globalString);
 */
    for (byte index = 0; index < count; index ++) {
      char ch = Serial.read();

      if (length >= (sizeof(buffer) / sizeof(*buffer))) {
//      sendMessage("(error bufferOverflow)");
        length = 0;
      }
      else if (ch == '\n'  ||  ch == ';') {
        buffer[length] = '\0';  // TODO: Check this working correctly, seems to be some problems when command is longer than buffer length ?!?

        char* result = commandArray.parse(buffer);  // TODO: Error handling when result == null
/*
        for (int index = 0; index < commandArray.length(); index ++) {  // TODO: Check failure cases
          Serial.print(index);
          Serial.print(": ");
          Serial.println(commandArray[index].head());
        }
 */
        int commandIndex = 0;

        while (commandIndex < commandCount) {
          if (commandArray[0].isEqualTo(commands[commandIndex])) {
            if (parameterCount[commandIndex] != (commandArray.length() - 1)) {
//            sendMessage("(error parameterCount)");
            }
            else {  // execute command
              if (parameterCount[commandIndex] > 0) parameter = commandArray[1];
              (commandHandlers[commandIndex])();
            }
            break;
          }

          commandIndex ++;
        }

//      if (commandIndex >= commandCount) sendMessage("(error unknownCommand)");

        length = 0;
      }
      else {
        buffer[length ++] = ch;
      }
    }

    timeOut = timeNow + 5000;
  }
}

/* -------------------------------------------------------------------------- */

#ifdef HAS_SERIAL_MIRROR
#include <NewSoftSerial.h>

#define SERIAL_MIRROR_RX_PIN 2
#define SERIAL_MIRROR_TX_PIN 3

byte serialMirrorInitialized = false;

NewSoftSerial serialMirror =  NewSoftSerial(SERIAL_MIRROR_RX_PIN, SERIAL_MIRROR_TX_PIN);

#define SERIAL_MIRROR_BUFFER_SIZE 128

void serialMirrorInitialize(void) {
  serialMirror.begin(DEFAULT_BAUD_RATE);

  serialMirrorInitialized = true;
}

void serialMirrorHandler(void) {
  static char serialMirrorBuffer[SERIAL_MIRROR_BUFFER_SIZE];
  static byte serialMirrorLength = 0;
  static long serialMirrorTimeOut = 0;

  if (serialMirrorInitialized == false) serialMirrorInitialize();

  unsigned long timeNow = millis();
  int count = serialMirror.available();

  if (count == 0) {
    if (serialMirrorLength > 0) {
      if (timeNow > serialMirrorTimeOut) {
        sendMessage("(error serialMirrorTimeout)");
        serialMirrorLength = 0;
      }
    }
  }
  else {
/*  globalString.begin();
    globalString  = "(info readCount ";
    globalString += count;
    globalString += ")";
    sendMessage(globalString);
 */
    for (byte index = 0; index < count; index ++) {
      char ch = serialMirror.read();
      if (ch == '\n') continue;

      if (serialMirrorLength >= (sizeof(serialMirrorBuffer) / sizeof(*serialMirrorBuffer))) {
        sendMessage("(error serialMirrorBufferOverflow)");
        serialMirrorLength = 0;
      }
      else if (ch == '\r') {
        serialMirrorBuffer[serialMirrorLength] = '\0';  // TODO: Check this working correctly, seems to be some problems when command is longer than buffer length ?!?
        Serial.println(serialMirrorBuffer);
        serialMirrorLength = 0;
      }
      else {
        serialMirrorBuffer[serialMirrorLength ++] = ch;
      }
    }

    serialMirrorTimeOut = timeNow + 5000;
  }
}
#endif



