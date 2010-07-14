/* aiko_segmeter.pde
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
 * Usage and configuration for smartenergygroups.com, SEG
 * ~~~~~~~~~~~~~~~~~~~~~
 * There are lots of things in here, and here are some basics.
 * If you can get through all this, you may well disover some new functions, and
 * you may even implement some of your own!  So Rock on!
 *
 * 1. Set the default node name, unique for your user id in SEG
 * 2. Make sure either single phase or multi phase is selected (one or the other commented out)
 * 3. Select the number of channels
 * 4. Adjust samples, averages and cycles to an appropriate setting.
 * 5. Decide whether to use the Aiko Events system or not.
 * 6. Go to the method segMeterInitialise and setup some stuff:
 *    - Sensor type on each channel
 *    - Primary turns on each channel (default is 1)
 *
 * 7. If you have any Q's on this, pop me a note here:
 *    Twitter: @samotage
 *    Message: https://smartenergygroups.com/questions/ask
 *
 * Good luck!
 * Sam.
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

//To use or not use the Aiko events system
//#define IS_AIKO
#define NOT_AIKO

//#define HAS_SERIAL_MIRROR

#define DEFAULT_NODE_NAME "pb"

// Transmit rate is only used for IS_AIKO option
#define DEFAULT_TRANSMIT_RATE    15  // seconds

//#define MULTI_PHASE
#define SINGLE_PHASE
//#define COMBINED_PHASE
//#define COMBINED_INDEX 2

/* 
** COMBINED_PHASE 
** Based on the combined index, the first channels are added together, the rest treated as single phases
*/

//#define SUM_CHANNELS
//#define SUBTRACT_CHANNELS

// Setup the aref to use, one or the other.
//#define IS_5V_AREF
#define IS_1V_AREF

// For Australian conditions, 230V is the voltage to use becase of fluctionations.
#define MEASUREMENT_VOLTAGE 230.0


#define CHANNELS          6   // Current Clamp(s)
#define SAMPLES        2000   // Current samples to take
#define AVERAGES          5   // Number of RMS values to average
#define CYCLES            3   // Number of times to cycle through the calculations

#define DEFAULT_BAUD_RATE     38400
#define ONE_SHOT_TIME         180000
#define QUICK_FLASH_TIME      20

// Sensor Types
#define SENSOR_CSLT 1
#define SENSOR_SCT_013_030 2
#define SENSOR_SCT_013_060 3
#define SENSOR_JAYCAR_CLIP_AREF_1 4
#define SENSOR_JAYCAR_CLIP_AREF_5 5
#define SENSOR_CRMAG_200 6

// Calibration factors
#define CALIBRATION_CSLT 25
#define CALIBRATION_SCT_013_030 9.1
#define CALIBRATION_SCT_013_060 25.8
#define CALIBRATION_JAYCAR_CLIP_AREF_1 33.333
#define CALIBRATION_JAYCAR_CLIP_AREF_5 144.45
#define CALIBRATION_CRMAG_200 1

// Digital Input/Output pins
#define PIN_SERIAL_RX       0
#define PIN_SERIAL_TX       1
#define PIN_ONE_WIRE        5 // OneWire or CANBus
#define PIN_RELAY           7
#define PIN_LED_STATUS     13 // Standard Arduino flashing LED !

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

float myAref = 5.0;
SExpression parameter;

void setup() {
    Serial.begin(DEFAULT_BAUD_RATE);
    // no ref means that it's at the 5Volts, perfect for the voltage handler.

#ifdef IS_1V_AREF
    //analogReference(EXTERNAL);
    analogReference(INTERNAL);
    myAref = 1.1;
#endif

#ifdef IS_AIKO
    // handlers set to 1 milisecond to use sleep mode
    Events.addHandler(serialHandler, 10); // Sufficient for 38,400 baud
 //   Events.addHandler(serialMirrorHandler, 30); // Sufficient for 38,400 baud
    Events.addHandler(blinkHandler, 1000);
    Events.addHandler(quickFlashHandler, 100);

    Events.addHandler(nodeHandler, 15000);
 //   Events.addHandler(temperatureSensorHandler, 60000);
 // Events.addHandler(oneShotHandler, 1000);
 // Events.addHandler(relayStateHandler, 60000);

    Events.addHandler(segMeterHandler, 15000);
    Events.addHandler(powerOutputHandler, 15000);
#endif
}

#ifdef NOT_AIKO
// Loop timers
unsigned long this_mili = 50000;
unsigned long last_mili = 0;
float seconds = 0;
#endif

void loop() {

#ifdef IS_AIKO
    Events.loop(); // do stuff
#endif

#ifdef NOT_AIKO
    segMeterHandler();
    nodeHandler();
    powerOutputHandler();
    temperatureSensorHandler();
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
    channelSensors[2] = SENSOR_CSLT;
    channelSensors[3] = SENSOR_CSLT;
    channelSensors[4] = SENSOR_CSLT;
    channelSensors[5] = SENSOR_SCT_013_060;

    channelPrimaryTurns[0] = 1;
    channelPrimaryTurns[1] = 1;
    channelPrimaryTurns[2] = 1;
    channelPrimaryTurns[3] = 1;
    channelPrimaryTurns[4] = 1;
    channelPrimaryTurns[5] = 1;

    segMeterInitialised = true;
}

void segMeterHandler() {
    if (segMeterInitialised == false) segMeterInitialise();
    // Accumulate
    for (int cycle = 0; cycle < CYCLES; cycle++) {
        collectChannels(); // Go get those dataz!
#ifdef HAS_SERIAL_MIRROR
        // Listen and send on the serial mirror.
        serialMirrorHandler();
#endif
    }
}

void collectChannels() {
    for (int channel = 0; channel < CHANNELS; channel++) {
        pin = PIN_CURRENT_SENSOR_1 + channel;
        flash(); // a little message that we are collecting.
        energySumNow[channel] = 0;
        // TODO Magic here to deermine collection method

        if (channelSensors[channel] != SENSOR_CRMAG_200) {
            collectChannelRMS(channel);
        } else {
            collectChannelTransduced(channel);
        }
        energySum[channel] += energySumNow[channel];
    }
}

void collectChannelRMS(int channel) {
    // For RMS sampling of an analog waveform
    watts_sum = 0.0;
    for (int average = 0; average < AVERAGES; average++) {
        rms = 0.0;
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

void collectChannelTransduced(int channel) {
    // For voltage output from an RMS processing sensor
    watts_sum = 0.0;
    for (int average = 0; average < AVERAGES; average++) {
        rms = 0.0;
        for (int index = 0; index < SAMPLES; index++) {
            //rms += 1023.0;
            rms += (float) analogRead(pin);
        }
        rms = rms / SAMPLES;
        // Then work out in relation to the 200 amp sensor.
        watts_sum += (((rms * (myAref / 1024)) / 5) * 200) * MEASUREMENT_VOLTAGE;
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

    if (this_rms > 0.01) {
        switch (sensor_type) {
            case SENSOR_SCT_013_030:
                output = this_rms * CALIBRATION_SCT_013_030;
                break;
            case SENSOR_SCT_013_060:
                this_rms = this_rms * CALIBRATION_SCT_013_060;
                output = nonLinearSCT_030_060_Calibration(this_rms);
                output = correctNonLinearity(output);
                break;
            case SENSOR_CSLT:
                this_rms = this_rms * CALIBRATION_CSLT;
                output = nonLinearCSLT_Calibration(this_rms);
                output = correctNonLinearity(output);
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
        // Factor for turns and measurement voltage.  110V should be less than 240V for the same current.
        output = (output / turns) * (MEASUREMENT_VOLTAGE / 240);
    } else {
        output = 0.0;
    }

    return output;
}

float nonLinearSCT_030_060_Calibration(float this_rms) {
    // Designed now for the non linear behaviour of the SCT_013_060 sensor
    float output = 0.0;
    if (this_rms <= 160) {
        output = this_rms * 0.93;
    } else {
        output = (this_rms + 25.52) / 1.44;
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

float correctNonLinearity(float this_rms) {
    // It appears the way the Arduino is sensing, is that an positive nonlinearity is experienced as the current increases.
    float output = 0.0;
    if (this_rms <= 3599) {
        // Linear
        output = this_rms;
    } else if (this_rms > 3599 && this_rms <= 7708) {
        // linear,           # y = 2.14x - 4100
        output = (this_rms + 4100) / 2.14;
    } else if (this_rms > 7709) {
        // linear,           # y = 1.86x - 4755
        output = (this_rms + 4755) / 1.86;
    }
    return output;
}

void powerOutputHandler() {

#ifdef MULTI_PHASE
    // 

    globalString.begin();

    currentKW_1 = 0.0;
    energySum_1 = 0.0;

    currentKW_2 = 0.0;
    energySum_2 = 0.0;

    for (int channel = 0; channel < CHANNELS; channel++) {
        if (channel == 0 || channel == 1 || channel == 2) {
            currentKW_1 += watts[channel];
            energySum_1 += energySum[channel];
        } else if (CHANNELS > 3 ) {
            currentKW_2 += watts[channel];
            energySum_2 += energySum[channel];
        }
    }

    globalString += "(p_1 ";
    globalString += currentKW_1;
    globalString += ")";

    globalString += "(e_1 ";
    globalString += energySum_1;
    globalString += ")";

    globalString += "(p_2 ";
    globalString += currentKW_2;
    globalString += ")";

    globalString += "(e_2 ";
    globalString += energySum_2;
    globalString += ")";

#endif

#ifdef COMBINED_PHASE
    // 

    globalString.begin();

    currentKW_1 = 0.0;
    energySum_1 = 0.0;

    currentKW_2 = 0.0;
    energySum_2 = 0.0;

    for (int channel = 0; channel < CHANNELS; channel++) {
        if (channel <  COMBINED_INDEX) {
            // Aggregate it
            currentKW_1 += watts[channel];
            energySum_1 += energySum[channel];
    }

    globalString += "(p_1 ";
    globalString += currentKW_1;
    globalString += ")";

    globalString += "(e_1 ";
    globalString += energySum_1;
    globalString += ")";
    
    // Now do the rest!
    
    for (int channel = COMBINED_INDEX; channel < CHANNELS; channel++) {

        globalString += "(p_";
        globalString += channel + 1;
        globalString += " ";
        globalString += watts[channel];
        globalString += ")";

        globalString += "(e_";
        globalString += channel + 1;
        globalString += " ";
        globalString += energySum[channel];
        globalString += ")";
    }

#endif

#ifdef SINGLE_PHASE
    globalString.begin();

    for (int channel = 0; channel < CHANNELS; channel++) {

        globalString += "(p_";
        globalString += channel + 1;
        globalString += " ";
        globalString += watts[channel];
        globalString += ")";

        globalString += "(e_";
        globalString += channel + 1;
        globalString += " ";
        globalString += energySum[channel];
        globalString += ")";
    }

#endif

#ifdef SUM_CHANNELS
    currentKWOut = 0.0;
    energySumOut = 0.0;
    
    // Sum the puppies
    for (int channel = 0; channel < CHANNELS; channel++) {
        
        currentKWOut += watts[channel];
        energySumOut += energySum[channel];
    }
    
     globalString += "(p ";
     globalString += currentKWOut;
     globalString += ")";

     globalString += "(e ";
     globalString += energySumOut;
     globalString += ")";
#endif

#ifdef SUBTRACT_CHANNELS
    currentKWOut = 0.0;
    energySumOut = 0.0;
    
    // Sum the puppies to make channel number 7,
    // channel 7 = Channel 1 - ( Channels 2 thru 6)
    // Because this is a computer, we use 0 instead of 1...
    
    for (int channel = 1; channel < CHANNELS; channel++) {
        
        currentKWOut += watts[channel];
        energySumOut += energySum[channel];
    }
    
     currentKWOut = watts[0] - currentKWOut;
     energySumOut = energySum[0] - energySumOut;
     
     // Sanitisation
     
     if ( currentKWOut < 0.0 ) {
       currentKWOut = 0;
       energySumOut = 0;
     }
    
     globalString += "(p_7 ";
     globalString += currentKWOut;
     globalString += ")";

     globalString += "(e_7 ";
     globalString += energySumOut;
     globalString += ")";
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

#define SERIAL_MIRROR_BUFFER_SIZE 256

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
         
         globalString.begin();
         globalString  = "(info readCount ";
         globalString += count;
         globalString += ")";
         sendMessage(globalString);
         
         
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

        serialMirrorTimeOut = timeNow + 20000;
    }
}
#endif




