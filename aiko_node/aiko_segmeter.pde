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

#define IS_SEGMETER
// the folowing def is to use some other analog inputs.
//#define IS_3_CHANNEL

//#define IS_SLEEPY
#define NOT_SLEEPY

//To use or not use the Aiko events system
//#define IS_AIKO
#define NOT_AIKO

//#define HAS_SERIAL_MIRROR

#define DEFAULT_NODE_NAME "seg_meter_10"
#define DEFAULT_TRANSMIT_RATE    15  // seconds
//#define STONE_DEBUG  // Enable capture and dump of all sampled values

// Sensor Types
#define SENSOR_CSLT 1
#define SENSOR_SCT_013_030 2
#define SENSOR_SCT_013_060 3
#define SENSOR_JAYCAR_CLIP_AREF_1 4
#define SENSOR_JAYCAR_CLIP_AREF_5 5

// Calibration factors
#define CALIBRATION_CSLT 25
#define CALIBRATION_SCT_013_030 9.1
#define CALIBRATION_SCT_013_060 25.8
#define CALIBRATION_JAYCAR_CLIP_AREF_1 33.333
#define CALIBRATION_JAYCAR_CLIP_AREF_5 144.45

// Deprecatable...
//#define CALIBRATION  8  // Hall Effect with 3 passes thru opening
//#define CALIBRATION  22  // Hall Effect with 1 pass thru opening
//#define CALIBRATION  9.1  // SCT-013-30 amp, 1v output
//#define CALIBRATION  25.8  // SCT-013-60 amp, 1v output
//#define CALIBRATION  33.333  // 10 mV per Amp, 1 turn, analog reference = internal (1 VDC)
//#define CALIBRATION   144.45  // 10 mV per Amp, analog reference = default  (5 VDC)

//#define MULTI_PHASE
#define SINGLE_PHASE
//#define SUM_CHANNELS

#define CHANNELS          6   // Current Clamp(s)
#define SAMPLES        1500   // Current samples to take
#define AVERAGES          5   // Number of RMS values to average
#define CYCLES            3   // Number of times to cycle through the calculations

#define DEFAULT_BAUD_RATE     38400
#define ONE_SHOT_TIME         180000
#define QUICK_FLASH_TIME      20

// Digital Input/Output pins
#define PIN_SERIAL_RX       0
#define PIN_SERIAL_TX       1
#define PIN_ONE_WIRE        5 // OneWire or CANBus
#define PIN_RELAY           7
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


// Analogue Input pins not used for 6 channel sensing
#define PIN_LIGHT_SENSOR    3
#define PIN_VOLTAGE_SENSOR  4


#include <PString.h>
char globalBuffer[250]; // Used to manage dynamically constructed strings
PString globalString(globalBuffer, sizeof (globalBuffer));

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
    0, // "bd",
    "nd",
    0,
    0,
    0,
    0 // "tr"
};

byte parameterCount[] = {// ToDo: Change this to incorporate parameter type ?
    1, // baud rate       (integer)
    1, // node name       (string)
    1, // relay state     (boolean)
    0, // reset clock     (none)
    1 // transmit rate   (integer seconds)
};

int skipCount = 0;
byte commandCount = sizeof (commands) / sizeof (*commands);
volatile boolean f_wdt = 1;

SExpression parameter;

void setup() {
    Serial.begin(DEFAULT_BAUD_RATE);
    // no ref means that it's at the 5Volts, perfect for the voltage handler.
    //analogReference(EXTERNAL);
    analogReference(INTERNAL);


#ifdef IS_AIKO
    // handlers set to 1 milisecond to use sleep mode
    Events.addHandler(serialHandler,      10); // Sufficient for 38,400 baud
    Events.addHandler(blinkHandler,      2000);
    Events.addHandler(quickFlashHandler,  100);
   
    Events.addHandler(nodeHandler,        15000);
    Events.addHandler(oneShotHandler,     1000);
    Events.addHandler(relayStateHandler,  15000);

//    Events.addHandler(segMeterHandler, 1);
//    Events.addHandler(powerOutputHandler, 1);
#endif

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

    cbi(SMCR, SE); // sleep enable, power down mode
    cbi(SMCR, SM0); // power down mode
    sbi(SMCR, SM1); // power down mode
    cbi(SMCR, SM2); // power down mode

    setup_watchdog(7);
#endif

#ifdef HAS_SERIAL_MIRROR
    Events.addHandler(serialMirrorHandler, 30); // Sufficient for 38,400 baud
#endif

}

#ifdef NOT_AIKO
// Loop timers
unsigned long this_mili = 50000;
unsigned long last_mili = 0;
float seconds = 0;
#endif

void loop() {

#ifdef IS_SLEEPY
    if (f_wdt == 1) { // wait for timed out watchdog / flag is set when a watchdog timeout occurs
        f_wdt = 0; // reset flag

        if (skipCount == 10) {
            skipCount = 0;

            digitalWrite(PIN_ZIGBEE_SLEEP, HIGH);
            delay(100); // time to power up
            Events.loop(); // do stuff
            delay(500); // to allow the send of the data over serial
            digitalWrite(PIN_ZIGBEE_SLEEP, LOW);
            delay(100); // time to power down
        } else {
            skipCount += 1;
        }

        system_sleep(); // have nap
    }
#endif

#ifdef IS_AIKO
    Events.loop();  // do stuff
#endif

#ifdef NOT_AIKO

    segMeterHandler();
    nodeHandler();
    powerOutputHandler();
#endif
}

/* --------------------------------------------------------------------------
 ** Blink routine, flashing
 */

byte blinkInitialized = false;
byte blinkStatus = LOW;
unsigned long lastQuickFlash;


void blinkInitialize(void) {
    pinMode(PIN_LED_STATUS, OUTPUT);
    blinkInitialized = true;
    lastQuickFlash = 0;
}

void blinkHandler(void) {
    if (blinkInitialized == false) blinkInitialize();
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

/* --------------------------------------------------------------------------
 ** Current measurement oscillating around zero volts.
 */

#ifdef IS_SEGMETER
byte segMeterInitialised = false;

float watts[CHANNELS]; // Instantaneous
float energySum[CHANNELS]; // Cumulative for the cycles
float energySumNow[CHANNELS]; // Cumulative for the current cycle
float energySumLast[CHANNELS]; // For calculating instantaneous energySum

// Tell it what sensors are on which channels
int channelSensors[CHANNELS];

// Tell it how many primary turns, usually 1
int channelPrimaryTurns[CHANNELS];

float currentKWOut;
float energySumOut;

float currentKW_1;
float energySum_1;

float currentKW_2;
float energySum_2;

unsigned long energySumTime[CHANNELS]; // Time of last calculation
unsigned long timeNow;
unsigned long duration;

int pin = 0;
int sample = 0;
float watts_sum = 0;
float rms = 0;

float energyNow = 0;

void segMeterInitialise(void) {
    // Init arrays
    for (int channel = 0; channel < CHANNELS; channel++) {
        watts[channel] = 0.0;
        energySum[channel] = 0.0;
        energySumNow[channel] = 0.0;
        energySumLast[channel] = 0.0;
        currentKW_1 = 0.0;
        energySum_1 = 0.0;
        currentKW_2 = 0.0;
        energySum_2 = 0.0;
        energySumTime[channel] = millis();
    }

    channelSensors[0] = SENSOR_SCT_013_060;
    channelSensors[1] = SENSOR_CSLT;
    channelSensors[2] = SENSOR_SCT_013_060;
    channelSensors[3] = SENSOR_SCT_013_060;
    channelSensors[4] = SENSOR_SCT_013_060;
    channelSensors[5] = SENSOR_SCT_013_060;

    channelPrimaryTurns[0] = 1;
    channelPrimaryTurns[1] = 3;
    channelPrimaryTurns[2] = 1;
    channelPrimaryTurns[3] = 3;
    channelPrimaryTurns[4] = 1;
    channelPrimaryTurns[5] = 1;

    segMeterInitialised = true;
}

void segMeterHandler() {
    if (segMeterInitialised == false) segMeterInitialise();
    // Accumulate
    for (int cycle = 0; cycle < CYCLES; cycle++) {
        collectChannels(); // Go get those dataz!
    }
}

void collectChannels() {
    for (int channel = 0; channel < CHANNELS; channel++) {
        pin = PIN_CURRENT_SENSOR_1 + channel;
        flash(); // a little message that we are collecting.
        energySumNow[channel] = 0;
        collectChannel(channel);
        energySum[channel] += energySumNow[channel];
    }
}

void collectChannel(int channel) {
    watts_sum = 0;
    for (int average = 0; average < AVERAGES; average++) {
        rms = 0;
        for (int index = 0; index < SAMPLES; index++) {
            sample = analogRead(pin);
            rms += sq((float) sample);
        }
        rms = sqrt(rms / (SAMPLES / 2));
        watts_sum += calibrateRMS(channelSensors[channel], rms, channelPrimaryTurns[channel]);
    }
    watts[channel] = watts_sum / AVERAGES;
    timeNow = millis(); // Mr Wolf.
    if (timeNow > energySumTime[channel]) { // Sanity check, in case millis() wraps around
        duration = timeNow - energySumTime[channel];
        energySumNow[channel] = energySumNow[channel] + (watts[channel] * (duration / 1000.0 / 3600.0));
    }
    energySumTime[channel] = timeNow;
}

float calibrateRMS(int sensor_type, float this_rms, int turns) {
    float output = 0.0;

    if (this_rms > 0.5) {
        switch (sensor_type) {
            case SENSOR_SCT_013_030:
                output = this_rms * CALIBRATION_SCT_013_030;
                break;
            case SENSOR_SCT_013_060:
                this_rms = this_rms * CALIBRATION_SCT_013_060;
                output = nonLinearSCT_030_060_Calibration(this_rms);
                break;
            case SENSOR_CSLT:
                this_rms = this_rms * CALIBRATION_CSLT;
                output = nonLinearCSLT_Calibration(this_rms);
                break;
            case SENSOR_JAYCAR_CLIP_AREF_1:
                output = this_rms * CALIBRATION_JAYCAR_CLIP_AREF_1;
                break;
            case SENSOR_JAYCAR_CLIP_AREF_5:
                output = this_rms * CALIBRATION_JAYCAR_CLIP_AREF_5;
                break;
            default:
                output = this_rms;
                break;
        }
        output = output / turns;
    } else {
        output = 0.0;
    }

    return output;
}

float nonLinearSCT_030_060_Calibration(float this_rms) {
    // Designed now for the non linear behaviour of the SCT_013_060 sensor
    float output = 0.0;
    if (this_rms <= 4300) {
        if (this_rms <= 160) {
            output = this_rms * 0.93;
        } else {
            // y = 1.44x - 25.52
            output = (this_rms + 25.52) / 1.44;
        }
    } else if (this_rms > 4300 && this_rms <= 7600) {
        // linear_1,           # y = 1.4x - 1720
        output = (this_rms + 1720) / 1.4;
    } else if (this_rms > 7600 && this_rms <= 11870) {
        // linear_2,           # y = 2.25x - 8200
        output = (this_rms + 8200) / 2.25;
    } else {
        // linear_3,          # y = 6.24x - 55580
        output = (this_rms + 55580) / 6.24;
    }
    return output;
}

float nonLinearCSLT_Calibration(float this_rms) {
    // Designed now for the non linear behaviour of the SCT_013_060 sensor
    float output = 0.0;
    if (this_rms <= 108) {
        // Subtract a reverse curve
        output = this_rms + ((108 - this_rms) * 0.09);
    } else if (this_rms > 108 && this_rms <= 325) {
        // linear,           # y = 1.36x - 35.5
        output = (this_rms + 35) / 1.35;
    } else {
        // linear,           # y = 1.338x - 90.58
        output = (this_rms + 90.58) / 1.338;
    }
    return output;
}

void powerOutputHandler() {
#ifdef MULTI_PHASE
    globalString.begin();

    currentKW_1 = 0.0;
    energySum_1 = 0.0;

    currentKW_2 = 0.0;
    energySum_2 = 0.0;

    for (int channel = 0; channel < CHANNELS; channel++) {
        if (channel == 0 || channel == 1 || channel == 2) {
            currentKW_1 += watts[channel];
            energySum_1 += energySum[channel];
        } else {
            currentKW_2 += watts[channel];
            energySum_2 += energySum[channel];
        }
    }

    globalString += "(power_1 ";
    globalString += currentKW_1;
    globalString += " W)";

    globalString += "(energy_1 ";
    globalString += energySum_1;
    globalString += " Wh)";

    globalString += "(power_2 ";
    globalString += currentKW_2;
    globalString += " W)";

    globalString += "(energy_2 ";
    globalString += energySum_2;
    globalString += " Wh)";
#endif

#ifdef SINGLE_PHASE
    globalString.begin();

    for (int channel = 0; channel < CHANNELS; channel++) {
        globalString += "(power_";
        globalString += channel + 1;
        globalString += " ";
        globalString += watts[channel];
        globalString += " W)";

        globalString += "(energy_";
        globalString += channel + 1;
        globalString += " ";
        globalString += energySum[channel];
        globalString += " Wh)";

        // some debug things
        //Serial.print(watts[channel]);
        //Serial.print(", ");
    }
    //Serial.println("");
#endif

#ifdef SUM_CHANNELS
    currentKWOut = 0.0;
    energySumOut = 0.0;

    // Sum the puppies
    for (int channel = 0; channel < CHANNELS; channel++) {
        currentKWOut += watts[channel];
        energySumOut += energySum[channel];
    }

    globalString += "(power ";
    globalString += currentKWOut;
    globalString += " W)";

    globalString += "(energy ";
    globalString += energySumOut;
    globalString += " Wh)";
#endif

    sendMessage(globalString);

    // Lets reseet the output arrays for the next time through.
    for (int channel = 0; channel < CHANNELS; channel++) {
        watts[channel] = 0;
        energySum[channel] = 0;
    }
}

#endif

/* --------------------------------------------------------------------------
 ** Works out if to shut the relay off or not
 */

byte oneShotInitialised = false;
byte relayInitialized = false;
byte relayState;
unsigned long relayTurnedOn = 0;
unsigned long relayTimeNow = 0;

void oneShotInitialise(void) {
    oneShotInitialised = true;
}

void oneShotHandler(void) {
    if (oneShotInitialised == false) oneShotInitialise();

    relayState = digitalRead(PIN_RELAY);
    relayTimeNow = millis();
    if (relayState == HIGH) {
        if (relayTimeNow > relayTurnedOn) {
            if ((relayTimeNow - relayTurnedOn) > ONE_SHOT_TIME) {
                turnRelayOff();
            }
        } else {
            // We have a rollover, so turn eet off
            turnRelayOff();
        }
    }
}

void turnRelayOff(void) {
    digitalWrite(PIN_RELAY, LOW);
    relayMessager("(relay off)");
}

/* --------------------------------------------------------------------------
 **  Reports the relay status
 */

byte relayStateInitialized = false;
byte thisRelayState = 0;

void relayStateInitialize(void) {
    relayStateInitialized = true;
}

void relayStateHandler(void) {
    if (relayStateInitialized == false) relayStateInitialize();

    // Just report the relay's state

    if (digitalRead(PIN_RELAY) == HIGH) {
        relayMessager("(relay 1 number)");
    } else {
        relayMessager("(relay 0 number)");
    }
}

/* --------------------------------------------------------------------------
 ** Relay handler
 */

void relayInitialize(void) {
    pinMode(PIN_RELAY, OUTPUT);
    relayInitialized = true;
}

void relayCommand(void) {
    if (relayInitialized == false) relayInitialize();

    if (parameter.isEqualTo("on")) {
        digitalWrite(PIN_RELAY, HIGH);
        relayTurnedOn = millis();
        relayMessager("(relay is on)");

    } else if (parameter.isEqualTo("off")) {
        digitalWrite(PIN_RELAY, LOW);
        relayTurnedOn = 0;
        relayMessager("(relay is off)");
    } else {
        //  Do nothing, sending stuff here messes the mesh.
        //  sendMessage("(error parameterInvalid)");
    }
}

void quickFlashHandler(void) {
    
    if (digitalRead(PIN_RELAY) == HIGH) {
      if ((millis() - lastQuickFlash) > QUICK_FLASH_TIME) {
          blinkHandler();
          lastQuickFlash = millis();
      }
    }
}

void relayMessager(char* message) {
    globalString.begin();
    globalString = message;
    sendMessage(globalString);
}

#ifdef IS_SLEEPY
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
    Serial.println(ww);

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

#endif

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
    voltageValue = (voltageRaw / 1024) * 5.15 * 3;

    //Serial.print("(voltage ");
    //Serial.print(voltageValue);
    //Serial.println(" V)");

    globalString.begin();
    globalString += "(voltage ";
    globalString += voltageValue;
    globalString += " V)";

    sendMessage(globalString);
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
 ** Node handler
 */

char nodeName[40] = DEFAULT_NODE_NAME;

void nodeHandler(void) {
    sendMessage("");
}

void nodeCommand(void) {
    char* parameterString = parameter.head();

    for (byte index = 0; index < sizeof (nodeName); index++) {
        if (index == parameter.size()) {
            nodeName[index] = '\0';
            break;
        }

        nodeName[index] = *parameterString++;
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
    globalString = "(light_lux ";
    globalString += lightValue;
    globalString += " lux)";
    sendMessage(globalString);
}

#endif

/* --------------------------------------------------------------------------
 ** Temperature Sensor
 */

OneWire oneWire(PIN_ONE_WIRE); // Maxim DS18B20 temperature sensor

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
void temperatureSensorHandler(void) { // total time: 33 milliseconds
    byte address[8];
    byte data[12];
    byte index;

    if (!oneWire.search(address)) { // time: 14 milliseconds
        //  Serial.println("(error 'No more one-wire devices')");
        oneWire.reset_search(); // time: <1 millisecond
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
        byte present = oneWire.reset(); // time: 1 millisecond
        oneWire.select(address); // time: 5 milliseconds
        oneWire.write(ONE_WIRE_COMMAND_READ_SCRATCHPAD); // time: 1 millisecond

        for (index = 0; index < 9; index++) { // time: 5 milliseconds
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
        int signBit = temperature & 0x8000;
        if (signBit) temperature = (temperature ^ 0xffff) + 1; // 2's complement

        int tc_100 = (6 * temperature) + temperature / 4; // multiply by 100 * 0.0625

        temperature_whole = tc_100 / 100;
        temperature_fraction = tc_100 % 100;

        globalString.begin();
        globalString = "(temperature ";
        if (signBit) globalString += "-";
        globalString += temperature_whole;
        globalString += ".";
        if (temperature_fraction < 10) globalString += "0";
        globalString += temperature_fraction;
        globalString += " C)";
        sendMessage(globalString);
    }

    // Start temperature conversion with parasitic power
    oneWire.reset(); // time: 1 millisecond
    oneWire.select(address); // time: 5 milliseconds
    oneWire.write(ONE_WIRE_COMMAND_START_CONVERSION, 1); // time: 1 millisecond

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

int transmitRate = DEFAULT_TRANSMIT_RATE; // seconds

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
    } else {
        /*  globalString.begin();
         globalString  = "(info readCount ";
         globalString += count;
         globalString += ")";
         sendMessage(globalString);
         */
        for (byte index = 0; index < count; index++) {
            char ch = Serial.read();

            if (length >= (sizeof (buffer) / sizeof (*buffer))) {
                //      sendMessage("(error bufferOverflow)");
                length = 0;
            } else if (ch == '\n' || ch == ';') {
                buffer[length] = '\0'; // TODO: Check this working correctly, seems to be some problems when command is longer than buffer length ?!?

                char* result = commandArray.parse(buffer); // TODO: Error handling when result == null
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
                        } else { // execute command
                            if (parameterCount[commandIndex] > 0) parameter = commandArray[1];
                            (commandHandlers[commandIndex])();
                        }
                        break;
                    }

                    commandIndex++;
                }

                //      if (commandIndex >= commandCount) sendMessage("(error unknownCommand)");

                length = 0;
            } else {
                buffer[length++] = ch;
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

NewSoftSerial serialMirror = NewSoftSerial(SERIAL_MIRROR_RX_PIN, SERIAL_MIRROR_TX_PIN);

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
    } else {
        /*  globalString.begin();
         globalString  = "(info readCount ";
         globalString += count;
         globalString += ")";
         sendMessage(globalString);
         */
        for (byte index = 0; index < count; index++) {
            char ch = serialMirror.read();
            if (ch == '\n') continue;

            if (serialMirrorLength >= (sizeof (serialMirrorBuffer) / sizeof (*serialMirrorBuffer))) {
                sendMessage("(error serialMirrorBufferOverflow)");
                serialMirrorLength = 0;
            } else if (ch == '\r') {
                serialMirrorBuffer[serialMirrorLength] = '\0'; // TODO: Check this working correctly, seems to be some problems when command is longer than buffer length ?!?
                Serial.println(serialMirrorBuffer);
                serialMirrorLength = 0;
            } else {
                serialMirrorBuffer[serialMirrorLength++] = ch;
            }
        }

        serialMirrorTimeOut = timeNow + 5000;
    }
}
#endif



