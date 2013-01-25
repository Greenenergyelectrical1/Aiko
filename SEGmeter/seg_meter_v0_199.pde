/* 
 * ~~~~~~~~~~~~~
 * Please do not remove the following notices.
 * Copyright (c) 2009 by Geekscape Pty. Ltd.
 * Documentation:  http://groups.google.com/group/aiko-platform
 * Documentation:  http://geekscape.org/static/arduino.html
 * License: GPLv3. http://geekscape.org/static/arduino_license.html
  * ----------------------------------------------------------------------------
 * See Google Docs: "Project: Aiko: Stream protocol specification"
 * Currently requires an Aiko-Gateway.
 * ----------------------------------------------------------------------------
 *
 *  v0.16 - 12 channel support, and ultrasonic sensor
 *  v0.15 - controllability, and Arduino 0021
 *  v0.16, 0.17 - stuff, new sensors etc
 *  v0.18 - Added the Dallas library for multiple temperature sensors
 *  v0.19 - 16 channel support using Mega
 *
 * Usage and configuration for smartenergygroups.com, SEG
 * ~~~~~~~~~~~~~~~~~~~~~
 * FIRST Time using this?
 * make sure to read up on the installation guides here:
 * 
 * https://github.com/samotage/Aiko/blob/master/README.markdown
 *
 * There are lots of things in here, and here are some basics.
 * If you can get through all this, you may well disover some new functions, and
 * you may even implement some of your own!  So Rock on!
 *
 * 1. Set the default node name, unique for your user id in SEG
 * 2. Make sure either single phase, multi phase or grouped is selected (one or the other commented out)
 * 3. Select the number of channels
 * 4. Adjust samples, averages and cycles to an appropriate setting for the frequency of data, every 45 seconds is good.
 * 5. Decide whether to use the Aiko Events system or not.
 * 6. Go to the method segMeterInitialise and setup some stuff:
 *    - Sensor type on each channel
 *    - Trim for each channel, a multiplier for each channel, good for calibrations and parallel feeds.
 *
 * 7. If you have any Q's on this, pop me a note here:
 *    Twitter: @samotage
 *    Message: https://smartenergygroups.com/questions/ask
 *
 * Good luck!
 * Sam.
 */
 
#define DEFAULT_NODE_NAME "segmeter"
#define BOOT_MESSAGE "(restart 0.199)"
 /*
 *
 * Third-Party libraries
 * ~~~~~~~~~~~~~~~~~~~~~
 * These libraries are not included in the Arduino IDE and
 * need to be downloaded and installed separately.
 *
 * - LiquidCrystal
 *   http://arduino.cc/en/Reference/LiquidCrystal
 *
 * - NewSoftSerial
 *   http://arduiniana.org/libraries/newsoftserial
 *
 * - PString
 *   http://arduiniana.org/libraries/pstring
 *
 * To Do
 * ~~~~~
 * - Fix temperature data acquisition should work every time, not every second time !
 * - Temperature sensor won't need 750 ms, if using permanent 5 VDC.
 * - Handle "(node= name)", where "name" greater than 40 characters.
 */

#include <PString.h>
#include <AikoSExpression.h>
#include <OneWire.h>
#include <DallasTemperature.h>
using namespace Aiko;


//#define DEBUGGABLE

//#define HAS_BUTTON  // If the device has a properly built momentary button on PIN_BUTTON normally 8

/*
 ** Phase Options
 ** Only one of the following phase options can be seleced
 ** SINGLE_PHASE - All phases treated as single
 ** MULTI_PHASE - Channels 1-3 are Feed 1 and Channels 4-6 are Feed 2
 ** GROUPED - For other magic, see below.
 */

#define SINGLE_PHASE
//#define MULTI_PHASE
//#define GROUPED

#define OUTPUT_GROUP 1  // To send power output in smaller chunks

/*
 ** Group together phases see above for option selection.
 **
 ** GROUPS - defines the number of groups
 ** GROUP_SIZE define the channels in the group
 **
 ** Examples:
 ** GROUPS = 3
 ** GROUP_SIZE = 2
 **   will groups together (ch1 + ch2) (ch3 + ch4) (ch5 + ch6) into 3 summed outputs
 ** GROUPS = 2
 ** GROUP_SIZE = 2
 **   will groups together (ch1 + ch2) (ch3 + ch4) intop 2 summed output
 **   ch5 and ch6 will be output as single channels
 **
 ** Caveat, playing funning buggers with more groups and group sizes than channels
 **  will result in non-optimal behaviour.
 */

#define GROUPS 1
#define GROUP_SIZE 3

#define SUB_GROUPS 0
#define SUB_GROUP_SIZE 0


/*
 ** COMBINED_PHASE
 ** Based on the combined index, the first channels are added together, the rest treated as single phases
 */
//#define COMBINED_PHASE
//#define COMBINED_INDEX   2

/*
 ** SUM_CHANNELS - this creates an extra channel based on the sum of all the proceeding channels
 ** SUBTRACT_CHANNELS - Based on the formula Channel 1 - (Sum Channels 2 thru 6) Create a new channel)
 ** ... note Channel 1 is really indexable as 0
 */

//#define SUM_CHANNELS  // setting to sum all and output total as _a
//#define SUBTRACT_CHANNELS // setting to subtract with math:  _s = ch0 - (all other channels)

// Setup the aref to use, one or the other.  Most ADC is done in the 1.1 volt internal ARef range.
//#define IS_5V_AREF
#define IS_1V_AREF

// For Australian conditions, 240V, 110 for US, and 230 for Europe
#define MEASUREMENT_VOLTAGE 240.0

// Post calibrations multiplier for ALL channels.  For individual channels, see trim below.
#define OUTPUT_MULTIPLIER 1.0

#define CHANNELS          1  // Current sensor(s)
#define SAMPLES        1000   // 1000 Samples to take
#define AVERAGES          3  // 3 Number of RMS values to average
#define CYCLES           60  // 20 Number of times to cycle through the calculations

#define DEFAULT_BAUD_RATE     38400
#define ONE_SHOT_TIME         180000
#define QUICK_FLASH_TIME      20

// Sensor Types
#define SENSOR_CSLT 1

#define SENSOR_SCT_013_030 2
#define SENSOR_SCT_013_060 3

#define SENSOR_JAYCAR_CLIP_AREF_1 4
#define SENSOR_JAYCAR_CLIP_AREF_1_LOW 5
#define SENSOR_JAYCAR_CLIP_AREF_5 6

#define SENSOR_CRMAG_200 7
#define SENSOR_JD_100 8
#define SENSOR_JD_250 9
#define SENSOR_JD_500 10

int channelSensors[CHANNELS];  // Tell it what sensors are on which channels
float channelTrim[CHANNELS];   // To trim the output

void define_sensors() {
  
  // Deck 1
  channelSensors[0] = SENSOR_SCT_013_060;
  channelSensors[1] = SENSOR_SCT_013_060;
  channelSensors[2] = SENSOR_SCT_013_060;

  channelSensors[3] = SENSOR_SCT_013_060;
  channelSensors[4] = SENSOR_SCT_013_060;
  channelSensors[5] = SENSOR_SCT_013_060;
    
    // Skip pins 6 and 7 on Seed Mega for a 12 channel, otherwise it keeps on truckin!
  
  // Deck 2  
  channelSensors[6]  = SENSOR_SCT_013_060;
  channelSensors[7]  = SENSOR_SCT_013_060;
  channelSensors[8]  = SENSOR_SCT_013_060;
  
  channelSensors[9]  = SENSOR_SCT_013_060;
  channelSensors[10] = SENSOR_SCT_013_060;
  channelSensors[11] = SENSOR_SCT_013_060;
  

  // Deck 3
  channelSensors[12]  = SENSOR_SCT_013_060;
  channelSensors[13]  = SENSOR_SCT_013_060;
  channelSensors[14]  = SENSOR_SCT_013_060;
  
  channelSensors[15]  = SENSOR_SCT_013_060;
  
  
  // Channel trim factor, multiply the power reading by the factor
    
  // Deck 1
  channelTrim[0]  = 1.0;
  channelTrim[1]  = 1.0;
  channelTrim[2]  = 1.0;
  channelTrim[3]  = 1.0;
  channelTrim[4]  = 1.0;
  channelTrim[5]  = 1.0;
  
  // Deck 2
  channelTrim[6]  = 1.0;
  channelTrim[7]  = 1.0;
  channelTrim[8]  = 1.0;
  channelTrim[9]  = 1.0;
  channelTrim[10] = 1.0;
  channelTrim[11] = 1.0;
 
  // Deck 3
  channelTrim[12] = 1.0;
  channelTrim[13] = 1.0;
  channelTrim[14] = 1.0;
  channelTrim[15] = 1.0;
}


// Digital Input/Output pins
#define PIN_SERIAL_RX       0
#define PIN_SERIAL_TX       1
#define PIN_ONE_WIRE        5 // OneWire or CANBus
#define PIN_ULTRASONIC      6
#define PIN_RELAY           7
#define PIN_BUTTON          8

#define PIN_LED_STATUS     13 // Standard Arduino flashing LED !

// Analogue Input pins
#define PIN_CURRENT_SENSOR_1  0 // Electrical monitoring
// Digital Input/Output pins
// TODO - Synch this with pebble design.

// Analogue Input pins not used for 6 channel sensing
#define PIN_LIGHT_SENSOR    3
#define PIN_VOLTAGE_SENSOR  4

#define SEG_STRING_SIZE 15
#define BUFFER_SIZE 450

char globalBuffer[BUFFER_SIZE]; // Used to manage dynamically constructed strings
PString globalString(globalBuffer, sizeof (globalBuffer));

void (*commandHandlers[])() = {
    baudRateCommand,
    nodeCommand,
    relay_command,
    switch_3g,
    resetClockCommand
};

char* commands[] = {
    "baud=",
    "node=",
    "relay",
    "switch_3g",
    "set_point",
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

byte initialised = false;
byte parameterCount[] = {// ToDo: Change this to incorporate parameter type ?
    1, // baud rate       (integer)
    1, // node name       (string)
    4, // relay           (<node name> (relay <on or off> < ? or timeout seconds> <SEG command id>));
    0, // reset clock     (none)
    1 // transmit rate   (integer seconds)
};

SExpressionArray commandArray;
SExpressionArray seg_commands;
SExpression parameter;

int skipCount = 0;
byte commandCount = sizeof (commands) / sizeof (*commands);
volatile boolean f_wdt = 1;

float myAref = 5.0;

OneWire oneWire(PIN_ONE_WIRE); // Maxim DS18B20 temperature sensor
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);


byte relayOn = false;
unsigned long relayTurnedOn = 0;
unsigned long relayTimeNow = 0;

String energise;
unsigned long  energise_time;
String seg_command_id;
String command_response;

void setup() {
    Serial.begin(DEFAULT_BAUD_RATE);
    pinMode(PIN_BUTTON, INPUT);
    pinMode(PIN_RELAY, OUTPUT);
    
    digitalWrite(PIN_RELAY, LOW);
    
    // no ref means that it's at the 5Volts, perfect for the voltage handler.

    #ifdef IS_1V_AREF
      //analogReference(INTERNAL1V1);  // ATmega1280
      analogReference(INTERNAL);  // ATmega328
      myAref = 1.1;
    #endif

}

void loop() {
  
    if (initialised != true) {
      globalString.begin();      
      sendMessage(globalString);
      globalString.begin();      
      globalString = BOOT_MESSAGE;
      sendMessage(globalString);
      globalString.begin();      
      initialised = true;
    }
    
    subEvents();  
    segMeterHandler();
    powerOutputHandler();
    subEvents();
    
    // Other measurements are sent on their own global string after the power output.
    globalString.begin();
    temperatureSensorHandler();
    sendMessage(globalString);
    subEvents();
}

/* --------------------------------------------------------------------------
 ** These things are called inside the guts of the main energy sampling for
 ** implementing control and switching.
 */
void subEvents() {
    serialHandler();
    one_shot_handler();
    
    #ifdef HAS_BUTTON 
      buttonHandler();
    #endif
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
    delay(180);
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
    delay(40);
    blinkStatus = !blinkStatus;
    digitalWrite(PIN_LED_STATUS, blinkStatus);
}

void flash_long(void) {

    if (blinkInitialized == false) blinkInitialize();

    blinkStatus = !blinkStatus;
    digitalWrite(PIN_LED_STATUS, blinkStatus);
    delay(140);
    blinkStatus = !blinkStatus;
    digitalWrite(PIN_LED_STATUS, blinkStatus);
}

void quickFlashHandler(void) {

    if (digitalRead(PIN_RELAY) == HIGH) {
        if ((millis() - lastQuickFlash) > QUICK_FLASH_TIME) {

            blinkHandler();
            lastQuickFlash = millis();
        }
    }
}

/* --------------------------------------------------------------------------
 **  Ultrasonic sensor handler, perfect for water tanks.
 */
 
void ultrasonic_handler() {
  
  // establish variables for duration of the ping, 
  // and the distance result in inches and centimeters:
  long duration, cm;

  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(PIN_ULTRASONIC, OUTPUT);
  digitalWrite( PIN_ULTRASONIC, LOW);
  delayMicroseconds(2);
  digitalWrite( PIN_ULTRASONIC, HIGH);
  delayMicroseconds(15);
  digitalWrite( PIN_ULTRASONIC, LOW);
  delayMicroseconds(20);
  // The same pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode( PIN_ULTRASONIC, INPUT);
  duration = pulseIn( PIN_ULTRASONIC, HIGH);

  // convert the time into a distance
  
  cm =  (duration / 29) / 2;

  globalString += "(level ";
  globalString += cm;
  globalString += ")";
} 
 
 
/* --------------------------------------------------------------------------
 ** Current measurement oscillating around zero volts.
 */

byte segMeterInitialised = false;

float watts[CHANNELS]; // Instantaneous
float energySum[CHANNELS]; // Cumulative for the cycles
float energySumNow[CHANNELS]; // Cumulative for the current cycle
float energySumLast[CHANNELS]; // For calculating instantaneous energySum

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

// Calibration factors
#define CALIBRATION_CSLT 25

#define CALIBRATION_SCT_013_030 9.1
#define CALIBRATION_SCT_013_060 25.8

#define CALIBRATION_JAYCAR_CLIP_AREF_1 33.333
#define CALIBRATION_JAYCAR_CLIP_AREF_1_LOW 280.0  // For low output range
#define CALIBRATION_JAYCAR_CLIP_AREF_5 144.45

#define CALIBRATION_CRMAG_200 1

// With the 390 kOhm it's 1.1
/*
**  J&D Sensor calibration notes.  
**  For carded range, the 390K resistor is used, with calibration 1.1
**  For 50% greater range, the 590k resistor is used with calibration is a higher number
*/


#define CALIBRATION_JD_100 1.04  // 390K On site calibrations at i9
#define CALIBRATION_JD_250 0.99  // Second calibration at Atchinson
#define CALIBRATION_JD_500 0.99  // Second calibration at Atchinson

// 590K On site calibrations on the 2.4.2011 at Target Doncaster

//#define CALIBRATION_JD_100 1.47 // Calibrated at 7 Eleven
//#define CALIBRATION_JD_250 1.43 // with 590k this was calbirated at 1.43
//#define CALIBRATION_JD_500 1.32 // with 590k

#define GROUPED_METHOD 1
#define SINGLE_METHOD 2

int grouping_method = 0;

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
    
    define_sensors();
    
    // Set the grouping method
    
    grouping_method = SINGLE_METHOD;
    
    #ifdef GROUPED
      grouping_method = GROUPED_METHOD;
    #endif    
    
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
      
        // For channel counts above 6 and less than 12 using ATmega1280 which we skip pins 6, 7 and go from 8.
        // 12 and above don't skip!
        
        if (CHANNELS <= 6 || CHANNELS > 12) {
          pin = PIN_CURRENT_SENSOR_1 + channel;
        } else {
          // We are on a 12 channel so need to Skip two pins 6 and 7
          if (channel < 6) {
            pin = PIN_CURRENT_SENSOR_1 + channel;
          } else {
            pin = PIN_CURRENT_SENSOR_1 + channel + 2;
          }
        }
        
        flash(); // a little message that we are collecting for this channel/
        energySumNow[channel] = 0;
        // TODO Magic here to deermine collection method
        
        //Serial.print("working on pin: ");
        //Serial.println(pin);
        
        switch (channelSensors[channel]) {
          
            case SENSOR_CRMAG_200:
              collectChannelTransduced(channel, channelSensors[channel]);
              break;
            case SENSOR_JD_500:
              collectChannelTransduced(channel, channelSensors[channel]);
              break;
            case SENSOR_JD_250:
              collectChannelTransduced(channel, channelSensors[channel]);
              break;
            case SENSOR_JD_100:
              collectChannelTransduced(channel, channelSensors[channel]);
              break;
            default:
                collectChannelRMS(channel);
                break;
        }
        
        energySum[channel] += energySumNow[channel];
    }
}

void collectChannelTransduced(int channel, int sensor_type) {
    // For voltage output from an RMS processing sensor
    watts_sum = 0.0;
    for (int average = 0; average < AVERAGES; average++) {
        // Process sub events and commands
        subEvents();
        
        rms = 0.0;
        for (int index = 0; index < SAMPLES; index++) {
            //rms += 1023.0;
            rms += (float) analogRead(pin);           
        }
        rms = rms / SAMPLES;
        
        //Serial.println(rms);
        
        switch (sensor_type) {
            case SENSOR_CRMAG_200:
              // Then work out in relation to the 200 amp sensor.
              //watts_sum +=  ( rms / 1024) * myAref * 500 * (float) MEASUREMENT_VOLTAGE *  (float) OUTPUT_MULTIPLIER * CALIBRATION_JD_500;
              watts_sum +=  ( rms / 1024) * myAref * 200 * (float) MEASUREMENT_VOLTAGE *  (float) OUTPUT_MULTIPLIER * CALIBRATION_JD_500;
              break;
            case SENSOR_JD_500:
            // Then work out in relation to the 500 amp sensor, using a voltage divider to take from 5V to 1V.
              watts_sum +=  ( rms / 1024) * myAref * 500 * (float) MEASUREMENT_VOLTAGE *  (float) OUTPUT_MULTIPLIER * CALIBRATION_JD_500;
              break;
            case SENSOR_JD_250:
            // Then work out in relation to the 250 amp sensor, using a voltage divider to take from 5V to 1V.
              watts_sum += ( rms / 1024) * myAref * 250 * (float) MEASUREMENT_VOLTAGE *  (float) OUTPUT_MULTIPLIER * CALIBRATION_JD_250;
              break;
            case SENSOR_JD_100:
            // Then work out in relation to the 100 amp sensor, using a voltage divider to take from 5V to 1V.
              watts_sum +=  ( rms / 1024) * myAref * 100 * (float) MEASUREMENT_VOLTAGE *  (float) OUTPUT_MULTIPLIER * CALIBRATION_JD_100;
              break;
            default:
                watts_sum = -1.0;
                break;
        }
        
    }

    //Serial.println(channelTrim[channel]);     
    watts[channel] = (watts_sum / AVERAGES) * channelTrim[channel];

    // For debuggerising
    
    #ifdef DEBUGGABLE
      if (channel < 3) {
        watts[channel] = 1000;
      } else {  
        watts[channel] = channel + 1;
      }
    #endif
    
    
    timeNow = millis(); // Mr Wolf.
    if (timeNow > energySumTime[channel]) { // Sanity check, in case millis() wraps around
        duration = timeNow - energySumTime[channel];
        energySumNow[channel] = energySumNow[channel] + (watts[channel] * (duration / 1000.0 / 3600.0));
    }
    energySumTime[channel] = timeNow;
}

void collectChannelRMS(int channel) {
    // For RMS sampling of an analog waveform
    watts_sum = 0.0;
    for (int average = 0; average < AVERAGES; average++) {
        // Process sub events and commands
        subEvents();
      
        rms = 0.0;
        for (int index = 0; index < SAMPLES; index++) {
            sample = analogRead(pin);
            rms += sq((float) sample);
        }
       
        rms = sqrt(rms / (SAMPLES / 2));
        watts_sum += calibrateRMS(channelSensors[channel], rms);        
    }
        
    // Apply the channel trim.
    watts[channel] = (watts_sum / AVERAGES) * channelTrim[channel];
    
    if (watts[channel] < 0) {
        watts[channel] = 0;
    }
    
    // For debugging
    #ifdef DEBUGGABLE
      watts[channel] = channel + 1;
    #endif

    timeNow = millis(); // Mr Wolf.
    if (timeNow > energySumTime[channel]) { // Sanity check, in case millis() wraps around

        duration = timeNow - energySumTime[channel];
        energySumNow[channel] = energySumNow[channel] + (watts[channel] * (duration / 1000.0 / 3600.0));
    }
    energySumTime[channel] = timeNow;
}

float calibrateRMS(int sensor_type, float this_rms) {
    float output = 0.0;

    if (this_rms > 0.01) {
        switch (sensor_type) {
            case SENSOR_SCT_013_030:
                //Serial.println("SENSOR_SCT_013_030");
                // Because these have a max output of 1V, adjust for 5V aRefs
                this_rms = this_rms * (myAref / 1.1);
                if (myAref == 5.0) {
                  this_rms = this_rms * 1.025;
                }
                output = this_rms * CALIBRATION_SCT_013_030;
                break;
            case SENSOR_SCT_013_060:
                //Serial.println("SENSOR_SCT_013_060");
                // Because these have a max output of 1V, adjust for 5V aRefs
                this_rms = this_rms * (myAref / 1.1);
                if (myAref == 5.0) {
                  this_rms = this_rms * 1.025;
                }
                this_rms = this_rms * CALIBRATION_SCT_013_060;
                output = nonLinearSCT_030_060_Calibration(this_rms);
                output = correctNonLinearity(output);
                output = correct_SCT_030_060(output);
                break;
            case SENSOR_CSLT:
                //Serial.println("SENSOR_CSLT");
                this_rms = this_rms * CALIBRATION_CSLT;
                output = nonLinearCSLT_Calibration(this_rms);  
                output = correctNonLinearity(output); 
                break;
            case SENSOR_JAYCAR_CLIP_AREF_1:
                //Serial.println("SENSOR_JAYCAR_CLIP_AREF_1");
                output = this_rms * CALIBRATION_JAYCAR_CLIP_AREF_1;
                break;
            case SENSOR_JAYCAR_CLIP_AREF_1_LOW:
                //Serial.println("SENSOR_JAYCAR_CLIP_AREF_1_LOW");
                output = this_rms * CALIBRATION_JAYCAR_CLIP_AREF_1_LOW;
                break;
            case SENSOR_JAYCAR_CLIP_AREF_5:
                //Serial.println("SENSOR_JAYCAR_CLIP_AREF_1");
                output = this_rms * CALIBRATION_JAYCAR_CLIP_AREF_5;
                break;
            default:
                output = this_rms;
                break;
        }
        // Factor for measurement voltage.  110V should be less than 240V for the same current.
        output = output * ((float) MEASUREMENT_VOLTAGE / (float) 240.0) * (float) OUTPUT_MULTIPLIER;
    } else {
        output = 0.0;
    }
    return output;
}

float nonLinearSCT_030_060_Calibration(float this_rms) {
    // Designed now for the non linear behaviour of the SCT_013_060 sensor
    float output = 0.0;
    if (this_rms <= 160.0) {
        output = this_rms * 0.93;
    } else {
        output = (this_rms + 25.52) / 1.44;
    }
    return output;
}

float correct_SCT_030_060(float this_rms) {
    // Post calibtration routine to correct variances.
    float output = 0.0;
   
    if (this_rms <= 80.0) {
        output = this_rms;
    } else if (this_rms > 80.0 && this_rms <= 100.0) {
        // y = 1.5x - 40 where y is the output
        output = this_rms * 1.5 - 40;
    } else if (this_rms > 100.0 && this_rms <= 250.0) {
        // y = x + 10  where y is the output
        output = this_rms + 10.0;
    } else if (this_rms > 250.0 && this_rms <= 320.0) {
        // y = 1.43x -97.1  where y is the output
        output = this_rms * 1.43 - 97.1;
    } else {
        output = this_rms + 40.0;
    }
    
    return output;
}

float nonLinearCSLT_Calibration(float this_rms) {
    // Designed now for the non linear behaviour
    float output = 0.0;
    if (this_rms <= 108.0) {
        // Subtract a reverse curve
        output = this_rms + ((108.0 - this_rms) * 0.09);
    } else if (this_rms > 108.0 && this_rms <= 325.0) {
        // linear,           # y = 1.36x - 35.5
        output = (this_rms + 35.0) / 1.35;
    } else {
        // linear,           # y = 1.338x - 90.58

        output = (this_rms + 90.58) / 1.338;
    }
    return output;
}

float correctNonLinearity(float this_rms) {
    // It appears the way the Arduino is sensing, is that an positive nonlinearity is experienced as the current increases.
    float output = 0.0;
    if (this_rms <= 3599.0) {
        // Linear
        output = this_rms;
    } else if (this_rms > 3599.0 && this_rms <= 7708.0) {
        // linear,           # y = 2.14x - 4100
        output = (this_rms + 4100.0) / 2.14;
    } else if (this_rms > 7709.0) {
        // linear,           # y = 1.86x - 4755

        output = (this_rms + 4755.0) / 1.86;
    }
    return output;
}

void powerOutputHandler() {
  
    int this_output = 1;
  
    flash_long(); // a longer flash that we are outputting datas now.
    
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
            } else if (CHANNELS > 3) {
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
        
        globalString += "(p_s ";
        globalString += currentKW_1 - currentKW_2;
        globalString += ")";
    
        globalString += "(e_s ";
        globalString += energySum_1 - energySum_2;
        globalString += ")";
    
        
    
    #endif

    #ifdef GROUPED
        //
    
        globalString.begin();
    
        float this_power = 0.0;
        float this_energy = 0.0;
        
        int channel_count = 1;
    
        for (int channel = 0; channel < (GROUPS * GROUP_SIZE); channel++) {
    
            // Lets group first
    
            this_power += watts[channel];
            this_energy += energySum[channel];
    
            if (channel_count == GROUP_SIZE) {
                // Lets output
                globalString += "(p_";
                globalString += channel + 1;
                globalString += " ";
                globalString += this_power;
                globalString += ")";
    
    
                globalString += "(e_";
                globalString += channel + 1;
                globalString += " ";
                globalString += this_energy;
                globalString += ")";
                
                if (this_output >= OUTPUT_GROUP) {
                  // Output now!
                  this_output = 1;
                  sendMessage(globalString);
                  globalString.begin();
                  
                } else {
                  this_output ++;
                }
    
                channel_count = 1;
                this_power = 0.0;
                this_energy = 0.0;
            } else {
                channel_count += 1;
            }
        }
    
        // Output the sub grouped channels
    
        for (int channel = (GROUPS * GROUP_SIZE); channel < ((GROUPS * GROUP_SIZE) + (SUB_GROUPS * SUB_GROUP_SIZE)); channel++) {
            // Special sub grouped
    
            this_power += watts[channel];
            this_energy += energySum[channel];
    
            if (channel_count == SUB_GROUP_SIZE) {
                // Lets output
                globalString += "(p_";
                globalString += channel + 1;
                globalString += " ";
                globalString += this_power;
                globalString += ")";
    
    
                globalString += "(e_";
                globalString += channel + 1;
                globalString += " ";
                globalString += this_energy;
                globalString += ")";
                
                if (this_output >= OUTPUT_GROUP) {
                  // Output now!
                  this_output = 1;
                  sendMessage(globalString);
                  globalString.begin();
                  
                } else {
                  this_output ++;
                }
    
                channel_count = 1;
                this_power = 0.0;
                this_energy = 0.0;
            } else {
                channel_count += 1;
            }   
        }
            
        // Now thats done, output the remaining single channels
    
        for (int channel = ((GROUPS * GROUP_SIZE) + (SUB_GROUPS * SUB_GROUP_SIZE)); channel < CHANNELS; channel++) {
      
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
              

          sendMessage(globalString);
          globalString.begin();

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
            
            if (this_output >= OUTPUT_GROUP) {
              // Output now!
              this_output = 1;
              sendMessage(globalString);
              globalString.begin();
                  
            } else {
              this_output ++;
            } 
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
    
        globalString += "(p_a ";
        globalString += currentKWOut;
        globalString += ")";
    
        globalString += "(e_a ";
        globalString += energySumOut;
        globalString += ")";
    #endif
    
    #ifdef SUBTRACT_CHANNELS
        currentKWOut = 0.0;
        energySumOut = 0.0;
    
        // Sum the puppies to make channel number 7,
        // channel 7 = Channel 1 - ( Channels 2 thru 6)
        // Because this is a computer, we use 0 instead of 1...
        
        float group_power = 0.0;
        float group_energy = 0.0;
            
        float remaining_power = 0.0;
        float remaining_energy = 0.0;
        
        if (grouping_method == GROUPED_METHOD) {
            // Re-do the group calcs
            #ifdef GROUPED        
              int channel_count = 1;
          
              for (int channel = 0; channel < GROUP_SIZE; channel++) {
                  // Lets group first
                  group_power += watts[channel];
                  group_energy += energySum[channel];
              }
          
              // Now thats done, output the remaining single channels
          
              for (int channel = GROUP_SIZE; channel < CHANNELS; channel++) {
                  remaining_power += watts[channel];
                  remaining_energy += energySum[channel];      
              }
  
              currentKWOut = group_power - remaining_power;
              energySumOut = group_energy - remaining_energy;            
            #endif

            
        } else {
            for (int channel = 1; channel < CHANNELS; channel++) {
              currentKWOut += watts[channel];
              energySumOut += energySum[channel];
            }
            currentKWOut = watts[0] - currentKWOut;
            energySumOut = energySum[0] - energySumOut;
        }
    
        // Sanitisation
    
        //if (currentKWOut <= 0.0) {
        //    currentKWOut = 0;
        //    energySumOut = 0;
        //}
    
        globalString += "(p_s ";
        globalString += currentKWOut;
        globalString += ")";
    
        globalString += "(e_s ";
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

/* -------------------------------------------------------------------------- */
/*
 * Arduino serial buffer is 128 characters.
 * At 115,200 baud (11,520 cps) the buffer is filled 90 times per second.
 * Need to run this handler every 10 milliseconds.
 *
 * At 38,400 baud (3,840 cps) the buffer is filled 30 times per second.
 * Need to run this handler every 30 milliseconds.
 */

byte serialHandlerInitialized = false;

void serialHandlerInitialize(void) {
    Serial.begin(DEFAULT_BAUD_RATE);
    serialHandlerInitialized = true;
}

void serialHandler(void) {
    static char buffer[64];
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
                // Serial.print("Hot in from the buffer: ");
                //Serial.println(buffer);
                
                buffer[length] = '\0'; // TODO: Check this working correctly, seems to be some problems when command is longer than buffer length ?!?
                parseCommand(buffer);
                length = 0;
            } else {
                buffer[length++] = ch;
            }
        }

        timeOut = timeNow + 5000;
    }
}

void parseCommand(char* buffer) {

    char* result = commandArray.parse(buffer); // TODO: Error handling when result == null
    /*
        for (int index = 0; index < commandArray.length(); index++) { // TODO: Check failure cases
            Serial.println(index);
            Serial.print("head: ");
            Serial.println(commandArray[index].head());
            Serial.println("Testing chop-----");
            Serial.println(chop_string(commandArray[index].head(), commandArray[index].size()));
            Serial.println("Done chop testing-----");
        }
    */ 
    
    byte is_for_me = false;
    if (commandArray[0].isEqualTo(nodeName)) {
      is_for_me = true;
    } else if (commandArray[0].isEqualTo("all_nodes")) {
      is_for_me = true;
    }
    
    if (is_for_me == true) {
        // Prse out the SEG commands
        seg_commands.parse(commandArray[1]);
        String choppd = chop_string(seg_commands.head(), seg_commands.size());
                      
        if (choppd == "relay")  {
            relay_command();
        } else if (choppd == "switch_3g")  {
            switch_3g();
        } else if (choppd == "display_3g")  {
            display_3g();
        } else {
            //Serial.println("...not commanding");
        }

    } else {
        // Serial.println("...not for this node");
    }
}

String chop_string(char* input, int chop_at) {
    String output;
    for (int index = 0; index < chop_at; index++) {        
        if (input[index] != '(') {
            if (input[index] == ' ') break;
            output.concat(input[index]);
        }
        if ((input[index] == ' ') && (index > 0)) {
            break;
        }
        if (input[index] == ')') {
            break;
        }
    }    
    return output.trim();  // Remove white spaces ad return
}

/* --------------------------------------------------------------------------
 ** Relay handler
 */


void relay_command(void) {

    command_response = "error";

    set_relay_parameters();

    //Serial.println(energise);
    //Serial.println(energise_time);
    //Serial.println(seg_command_id);
    //Serial.println(command_response);

    if (energise == "on") {
        turn_relay_on_notify();
        command_response = "complete";
    } else if (energise == "off") {
        turn_relay_off_notify();
        command_response = "complete";
    } else {
        //  Do nothing, sending stuff here messes the mesh.
        //  Rebroadcast the command and check at the start wether it's for this node.
        //  sendMessage("(error parameterInvalid)");
    }
   complete_relay_command();
}

void switch_3g(){  
  //Serial.println("About to hit the relay");
    energise_time = 4000;
    turn_relay_on_silent();
    //turn_relay_on_notify();
    relay_messager("(3g 1)");   
}

void display_3g(){   
    energise_time = 800;
    turn_relay_on_silent();
}

void turn_relay_on_silent() {
    digitalWrite(PIN_RELAY, HIGH);
    relayTurnedOn = millis();   
    relayOn = true;
    relay_messager("");   // For some silly reason this needs to send a message...
}

void turn_relay_off_silent() {
    digitalWrite(PIN_RELAY, LOW);
    relayOn = false;
    energise_time = 0;
    relay_messager("");   // For some silly reason this needs to send a message...   
}

void turn_relay_on_notify() {
    digitalWrite(PIN_RELAY, HIGH);
    relayTurnedOn = millis();  
    relayOn = true;
    relay_messager("(relay on)");
}

void turn_relay_off_notify() {
    digitalWrite(PIN_RELAY, LOW);
    relayOn = false;
    energise_time = 0;
    relay_messager("(relay off)");   
}


void set_relay_parameters() {

    String choppd;

    for (int index = 0; index < seg_commands.length(); index++) {

        /*
        Serial.print("In the relay parameters: ");
        Serial.println(index);
        Serial.print("head: ");
        Serial.println(seg_commands[index].head());
        Serial.println("Testing chop-----");
        Serial.println(chop_string(seg_commands[index].head(), seg_commands[index].size()));
        Serial.println("Done chop testing-----");
        */
      
        choppd = chop_string(seg_commands[index].head(), seg_commands[index].size());
        //Serial.println(choppd);

        switch (index) {
            case 0:
                // Command name nothing to do here
                break;
            case 1:
                // Energise setting
                energise = choppd;
                break;
            case 2:
                //Serial.println(choppd);

                if (choppd.startsWith('?')) {
                   energise_time = 0;  // 0 Implies that the relay will remain ON, ie. is_one_shot test == 0 for false
                } else {
                  char this_char[choppd.length() + 1];
                  choppd.toCharArray(this_char, (choppd.length() + 1));
                  energise_time = atol(this_char);
                }
                energise_time =  energise_time * 1000;
                break;

            case 3:
            
                seg_command_id = choppd;
                break;
            default:
                break;
        }
    }
}

void complete_relay_command() {

    globalString.begin();

    globalString += "(command ";
    globalString += seg_command_id;
    globalString += " ";
    globalString += command_response;
    globalString += ")";

    sendMessage(globalString);
}


/* --------------------------------------------------------------------------
 ** Works out if to shut the relay off or not
 */

void one_shot_handler(void) {
    /*
    Serial.print("one_shot_handler relay status is: ");
    Serial.print(relayOn);
    Serial.print(" energise_time: ");
    Serial.println(energise_time);
    */
          
    if (energise_time > 0) {
      
      relayTimeNow = millis();
      
      if (relayOn == true) {
          /*
          Serial.print("relay turned on for ");
          Serial.print(energise_time);
          Serial.print(" at time: ");
          Serial.print(relayTurnedOn);
          Serial.print(" it's now: ");
          Serial.println(relayTimeNow);
          */
          
          if (relayTimeNow > relayTurnedOn) {
                
              if ((relayTimeNow - relayTurnedOn) > energise_time) {
                  // No need to tell SEG that shot has expired.  Otherwise the device will de-energise...
                  //Serial.println("About to turn off silently...");
                  turn_relay_off_silent();
              }
          } else {
              // No need to tell SEG that shot has expired.  Otherwise the device will de-energise...
              // We have a rollover, so turn eet off
              turn_relay_off_silent();
          }
      }
    }
}

/*
**------------------------------------------------------------------------
**  Button Handler, this is a little button that toggles the relay ON or OFF
**  And then tells SEG the new state of the relay
*/

byte buttonInitialised = false;

int buttonState = 0;         // variable for reading the pushbutton status
int thisButtonState = LOW; 
int lastButtonState = LOW; 

byte changedState = false;

long lastDebounceTime = 0;  // the last time the output pin was toggled
long debounceDelay = 200;    // the debounce time; increase if the output flickers

void buttonInitialise(void) {
    changedState = false;
    buttonInitialised = true;
}

void buttonHandler()  {
  
  if (buttonInitialised == false) buttonInitialise();
  
   // read the state of the switch into a local variable:
  thisButtonState = digitalRead(PIN_BUTTON);

  // If the switch changed, due to noise or pressing:
  if (thisButtonState != lastButtonState) {
    // reset the debouncing timer, and the changeable state
    lastDebounceTime = millis();
    changedState = false;  
  } else {
    // The button is in the same state, is it debounceable?
    if ((millis() - lastDebounceTime) > debounceDelay) {
      // whatever the reading is at, it's been there for longer
      // than the debounce delay, so take it as the actual current state:
   
      // Toggle the Relay State, only do this once while the button state is still high!
      if (thisButtonState == HIGH) {
        
        if (changedState == false) {
          toggleRelayState();
          changedState = true;
        }
      }
    }
  }
  lastButtonState = thisButtonState;
}

void toggleRelayState() {
  if (relayOn == true) {
    turn_relay_off_notify();
  } else {
    turn_relay_on_notify();
  }
}


/* --------------------------------------------------------------------------
 **  Reports the relay status
 */

byte relayStateInitialized = false;

void relayStateInitialize(void) {
    relayStateInitialized = true;
}

void relayStateHandler(void) {
    if (relayStateInitialized == false) relayStateInitialize();

    // Just report the relay's state
    if (digitalRead(PIN_RELAY) == HIGH) {
        relay_messager("(relay on)");
    } else {

        relay_messager("(relay off)");
    }
}

void relay_messager(char* message) {

    globalString.begin();
    globalString += message;
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
      globalString += ") ";         
  
      this_index += 1; 
    }
}


/* --------------------------------------------------------------------------
 ** Baud Rate Handler
 */

int baudRate = DEFAULT_BAUD_RATE;

void baudRateCommand(void) {

    char* parameterString = parameter.head();
}
