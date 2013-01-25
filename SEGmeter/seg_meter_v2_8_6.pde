/* 
* ~~~~~~~~~~~~~
* Please do not remove the following notices.q
* Copyright (c) 2011 by Smart Energy Groups Pty. Ltd.
* License: GPLv3
*
*  v0_19   last stable version of the V1, used to craft this code
*  v2_1    fail version, not stable, based on v0_18
*  v2_2    SEGmeter V2 codes version, made from v0_19.  Note v2_2 is not commandable...
*  v2_3    commandable, DSP
*  v2_4    Special serial control to work with Dragino
*  v2_5    Deprecated the old SEpression parser to something simpler.
*  v2_6    Adding subtotals for solar measurement shennanigans
*  v2_8_4  First prime time v2 version
*  v2_8_5  Support for ampd rogowskis and more flexible greeen energy outputting
*  v2_8_6  Adjusted channel trim to int for switching - memory issues.
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
* 7. To get SEGmeter to send data it needs the command send:
*    (all_nodes (start_data));
*    or replace all_nodes with this node name
*
* 8. If you have any Q's on this, pop me a note here:
*    Twitter: @samotage
*    Message: https://smartenergygroups.com/contact
*
* Good luck!
* Sam.
*/

#include <PString.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define NODE_NAME "segmeter"
#define BOOT_MESSAGE "(restart 2.8.6)"
#define HAS_BUTTON 
//#define NOT_DRAGINO 
//#define DEBUGGABLE

// Phase Options
#define SINGLE_PHASE
//#define GROUPED // For multiphase, see below

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
**   will groups together (ch1 + ch2) (ch3 + ch4) intop 2 summed outputs, and:
**   ch5 and ch6 will be output as single channels
**
** SUB_GROUPS extends this, with a second group after the first.
**
** After all this magic, the remaining channels are single phases.
** 
** Caveat, playing funning buggers with more groups and group sizes than channels will result in non-optimal behaviour.
*/

#define GROUPS 0
#define GROUP_SIZE 0

#define SUB_GROUPS 0
#define SUB_GROUP_SIZE 0

//#define SUBTOTAL_AT 12 // Sums all the channels to this point and creates a subtotal, which is 
                      // then used by the sum or subtract below...
                      // HINT, use the negative trim and flip afterwards in SEG for solars :D
                      // Also, Solar goes on the LAST channel :)
                      // AND if the subtotal is for all channels up to say 1 thru 6 set this to 6
                      
//#define GREEN         // Summs all the channels where the multiplier is less than 1 - i.e. green energy to output as  single total.                      
#define SUM_CHANNELS  // setting to sum all and output total as _a//

//#define SUBTRACT_CHANNELS // setting to subtract with math:  _s = ch0 - (all other channels)
//#define SUBTRACTABLE_AT   // Where the subtration virtual is to be calculated from a series of channels ending before the last channel.  
                            // I.e. where the SEGmeter is measuring across boards.
                            // HINT:  if the last channel is 12, set this to 12.


#define MEASUREMENT_VOLTAGE 240.0  // For Australian conditions, 240V, 110 for US, and 230 for Europe
#define OUTPUT_MULTIPLIER 1.0  // Post calibrations multiplier for ALL channels.  For individual channels, see trim below.

#define CHANNELS          2 // Current sensor(s)
#define SAMPLES          110 //220  // Samples to take
#define AVERAGES          4  // Number of RMS values to average
#define CYCLES            4  // Number of times to cycle through the calculations

// TODO, work out a sample time that will calculate the above, say 50 seconds, this back manipulates etc...

#define DISCARD_COUNT   15  // Samples to discard when switching channels for moar wins
#define CROSSINGS       4
#define CROSSINGS_UPPER 6

// Sensor Types
#define SENSOR_NONE 0
#define SENSOR_CSLT 1

#define SENSOR_SCT_013_030 2
#define SENSOR_SCT_013_060 3

#define SENSOR_JAYCAR_CLIP_AREF_1 4
#define SENSOR_JAYCAR_CLIP_AREF_1_LOW 5
#define SENSOR_JAYCAR_CLIP_AREF_5 6

#define SENSOR_CRMAG_200  7
#define SENSOR_JD_50      8
#define SENSOR_JD_100     9
#define SENSOR_JD_250    10
#define SENSOR_JD_500    11
#define SENSOR_JRF_80     12

int channelSensors[CHANNELS];  // Tell it what sensors are on which channels
int channelTrim[CHANNELS];   // To trim the output

void define_sensors() {
 // Base Deck  
 channelSensors[0]  = SENSOR_JD_500;
 channelSensors[1]  = SENSOR_JD_500;
 channelSensors[2]  = SENSOR_JD_500;
 channelSensors[3]  = SENSOR_JD_500;
 channelSensors[4]  = SENSOR_JD_500;
 channelSensors[5]  = SENSOR_JD_500;
 channelSensors[6]  = SENSOR_JD_500;
 channelSensors[7]  = SENSOR_JD_500;

 // Expansion Deck  
 channelSensors[8]   = SENSOR_JRF_80;
 channelSensors[9]   = SENSOR_JRF_80;
 channelSensors[10]  = SENSOR_JRF_80;
 channelSensors[11]  = SENSOR_JRF_80;
 channelSensors[12]  = SENSOR_JD_500;
 channelSensors[13]  = SENSOR_JD_500;
 channelSensors[14]  = SENSOR_JD_500;
 channelSensors[15]  = SENSOR_JD_500;

 // Base Deck, channel trim factor, multiply the power reading by the factor
 channelTrim[0]  = 1;
 channelTrim[1]  = 1;
 channelTrim[2]  = 1;
 channelTrim[3]  = 1;
 channelTrim[4]  = 1;
 channelTrim[5]  = 1;
 channelTrim[6]  = 1;
 channelTrim[7]  = 1;

 // Expansion Deck
 channelTrim[8]  = 1;
 channelTrim[9]  = 1;
 channelTrim[10] = 1;
 channelTrim[11] = 1;
 channelTrim[12] = 1;
 channelTrim[13] = 1;
 channelTrim[14] = 1;
 channelTrim[15] = 1;
}

#define NOISE_OFFSET 2

// Calibration factors
#define CALIBRATION_NONE 0
#define CALIBRATION_CSLT 16.87

#define CALIBRATION_SCT_013_030   8.63   // Estimated
#define CALIBRATION_SCT_013_060   15.89  // Re-calibrated at the SEGshop

#define CALIBRATION_JAYCAR_CLIP_AREF_1     33.333     
#define CALIBRATION_JAYCAR_CLIP_AREF_1_LOW 280.0      // At Carey
#define CALIBRATION_JAYCAR_CLIP_AREF_5     144.45     

#define CALIBRATION_JD_50    1.04
#define CALIBRATION_JD_100   1.04
#define CALIBRATION_JD_250   1.04
#define CALIBRATION_JD_500   1.04

//#define CALIBRATION_JRF80 252.53  // CoC 10x gain Calibration factor for the new Rogowski coil high-current sensors, LW 03/11/2011
#define CALIBRATION_JRF80    194.55  // 20x gain Calibration factor for the new Rogowski coil high-current sensors

// 590K On site calibrations on the 2.4.2011 at Target Doncaster

//#define CALIBRATION_JD_100 1.47 // Calibrated at 7 Eleven
//#define CALIBRATION_JD_250 1.43 // with 590k this was calbirated at 1.43
//#define CALIBRATION_JD_500 1.32 // with 590k


// Pin assignment
#define PIN_SERIAL_RX         0
#define PIN_SERIAL_TX         1
#define PIN_ONE_WIRE          5 // OneWire or CANBus
#define PIN_LED_STATUS        3 // Standard Arduino flashing LED !
#define PIN_DC_VOLTAGE_SENSOR 0 // Arduino Pins.

// Pin Microchip MCP3208 8-channel 12-bit SPI-interfaced ADC over SPI. 
// MOSI, MISO and SCK should not be changed, but CS refrences the ADC Bank
// Base bank is 9
// Expansion is 10

#define CS   9
#define CS_E 10   

//#define CS   10  // Swap these for deck flipping
//#define CS_E 9   

#define MOSI 11
#define MISO 12
#define SCK  13

#define OUTPUT_BUFFER 70
#define INPUT_BUFFER 50

char globalBuffer[OUTPUT_BUFFER]; // Used to manage dynamically constructed strings
PString globalString(globalBuffer, sizeof (globalBuffer));

OneWire oneWire(PIN_ONE_WIRE); // Maxim DS18B20 temperature sensor
DallasTemperature sensors(&oneWire);

// Variable setupping
int sample_array[SAMPLES];
float mean_sample = 2048;
float min_sample = 4096;
float max_sample = 0;

byte send_data = false;
byte initialised = false;

byte relayOn = false;
unsigned long relayTurnedOn = 0;
unsigned long relayTimeNow = 0;

String seg_command;
byte is_for_me = false;

String energise;
unsigned long  energise_time;
String seg_command_id;
String command_response;

#define PIN_RELAY_1         6
#define PIN_RELAY_2         7
#define PIN_RELAY_3         8

#define PIN_BUTTON         4

void setup() {
  
   pinMode(CS,   OUTPUT); 
   pinMode(CS_E, OUTPUT); 
   pinMode(MOSI, OUTPUT); 
   pinMode(MISO, INPUT); 
   pinMode(SCK,  OUTPUT); 

   pinMode(PIN_LED_STATUS, OUTPUT); 
   pinMode(PIN_BUTTON, INPUT);
   pinMode(PIN_RELAY_1, OUTPUT);
   pinMode(PIN_RELAY_2, OUTPUT);
   pinMode(PIN_RELAY_3, OUTPUT);
    
   digitalWrite(PIN_RELAY_1, LOW);
   digitalWrite(PIN_RELAY_2, LOW);
   digitalWrite(PIN_RELAY_3, LOW);
   digitalWrite(PIN_BUTTON, LOW);

   digitalWrite(CS,   HIGH); // Base     ADC is disabled. Remember, low = enabled.
   digitalWrite(CS_E, HIGH); // Expanded ADC is disabled. Remember, low = enabled.
   digitalWrite(MOSI, LOW); 
   digitalWrite(SCK,  LOW); 
   
   flash_long();
   delay(500);
   flash_long();
   delay(500);
   flash_long();   
   delay(10000);
   Serial.begin(38400);
   //send_reboot_message();
}

void loop() {
   
   seg_meter_handler();
   sub_events();
   
   if (send_data == true) {
     
       globalString.begin();
       //dc_voltage_sensor();
       temperature_sensor();
       send_message(globalString);
       
       globalString.begin();
       power_output_machine();
       // Other measurements are sent on their own global string after the power output.      
   }   
}

/* --------------------------------------------------------------------------
** These things are called inside the guts of the main energy sampling for
** implementing control and switching.power_
*/
void sub_events() {
    serial_handler();
    one_shot_handler();
    
    #ifdef HAS_BUTTON 
      button_handler();
    #endif
}

void send_reboot_message() {
     //if (initialised != true) {
      globalString.begin();      
      send_message(globalString);
      globalString.begin();      
      globalString = BOOT_MESSAGE;
      send_message(globalString);
      globalString.begin();      
      initialised = true;
    //} 
}

void send_message(const char* message) {
   if (message) {
     Serial.print("(node ");
     Serial.print(NODE_NAME);
     Serial.print(" ? ");
     Serial.print(message);
     Serial.println(")");   
   }
   delay(60);  // So the Zigbees don't get flooded?
   sub_events();
}


/* --------------------------------------------------------------------------------
** Serial Handler, and related command codes
*/

void serial_handler(void) {
    static char buffer[INPUT_BUFFER];
    static byte length = 0;
    static long timeOut = 0;

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
                
                buffer[length] = '\0'; 
                parse_command(buffer);
                length = 0;
            } else {
                buffer[length++] = ch;
            }
        }

        timeOut = timeNow + 5000;
    }
}

void parse_command(char* buffer) {
      
    seg_command = get_command_element(buffer, 0);
    
    if (seg_command == NODE_NAME) {
      is_for_me = true;
    } else if (seg_command == "all_nodes") {
      is_for_me = true;
    } else {
      is_for_me = false;
    }
    
    if (is_for_me == true) {
        // Prse out the SEG commands
        
        seg_command = get_command_element(buffer, 1);
                      
        if (seg_command == "start_data")  {
            start_data();          
        }  else if (seg_command == "stop_data")  {
            stop_data();              
        }  else if (seg_command == "relay")  {
            relay_command(buffer);
        } else if (seg_command == "switch_3g")  {
            switch_3g();
        } else if (seg_command == "display_3g")  {
            display_3g();
        } else {
            //Serial.println("...not commanding");
        }

    } else {
        // Serial.println("...not for this node");
    }
}

String get_command_element(char* input, int target_element) {
  
    // example command  "(all_nodes (start_data));"
    // CAVEAT if there are multiple spaces, this wil fail!
    // Serial.println(input);
    
    int current_index = 0;    
    String output_string;
    
    for (int index = 0; index < INPUT_BUFFER; index++) {        
      if (input[index] == '\0') {
        break;
      }
      if ((input[index] != '(' ) && (input[index] != ')' )) {
        if (input[index] == ' ') {
          
          if (current_index == target_element) {
            break;
          } else {
            output_string = "";
            current_index += 1;
          }
        } else {
          output_string.concat(input[index]);
        }         
      }  
    }  
    
    //Serial.println(output_string);
    // Serial.println("");
    return output_string;
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
 ** Various command handlers
 */

void start_data(){  
  send_data = true;
}

void stop_data(){  
  send_data = false;
}

void relay_command(char* buffer) {

    command_response = "error";

    set_relay_parameters(buffer);

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
        //  send_message("(error parameterInvalid)");
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
    digitalWrite(PIN_RELAY_1, HIGH);
    relayTurnedOn = millis();   
    relayOn = true;
    relay_messager("");   // For some silly reason this needs to send a message...
}

void turn_relay_off_silent() {
    digitalWrite(PIN_RELAY_1, LOW);
    relayOn = false;
    energise_time = 0;
    relay_messager("");   // For some silly reason this needs to send a message...   
}

void turn_relay_on_notify() {
    digitalWrite(PIN_RELAY_1, HIGH);
    relayTurnedOn = millis();  
    relayOn = true;
    relay_messager("(relay on)");
}

void turn_relay_off_notify() {
    digitalWrite(PIN_RELAY_1, LOW);
    relayOn = false;
    energise_time = 0;
    relay_messager("(relay off)");   
}


void set_relay_parameters(char* buffer) {
      
    energise       = get_command_element(buffer, 2);
    seg_command    = get_command_element(buffer, 3);  // For energise_time
    seg_command_id = get_command_element(buffer, 4);
    
    
    if (seg_command.startsWith('?')) {
       energise_time = 0;  // 0 Implies that the relay will remain ON, ie. is_one_shot test == 0 for false
    } else {
      // Buggery for atoi...
      char this_char[seg_command.length() + 1];
      seg_command.toCharArray(this_char, (seg_command.length() + 1));
      energise_time = atol(this_char) * 1000;
    }
    seg_command = "";
}

void complete_relay_command() {

    globalString.begin();
    globalString += "(command ";
    globalString += seg_command_id;
    globalString += " ";
    globalString += command_response;
    globalString += ")";

    send_message(globalString);
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
long debounceDelay = 70;    // the debounce time; increase if the output flickers

void buttonInitialise(void) {
    changedState = false;
    buttonInitialised = true;
}

void button_handler()  {
  
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
          toggle_relay_state();
        }
      }
    }
  }
  lastButtonState = thisButtonState;
}

void toggle_relay_state() {
  changedState = true;
  if (relayOn == true) {
    turn_relay_off_silent();
    relay_messager("(switched off)");   
  } else {
    turn_relay_on_silent();
    relay_messager("(switched on)");   
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
    if (digitalRead(PIN_RELAY_1) == HIGH) {
        relay_messager("(relay on)");
    } else {

        relay_messager("(relay off)");
    }
}

void relay_messager(char* message) {

    globalString.begin();
    globalString += message;
    send_message(globalString);
}




/* --------------------------------------------------------------------------------
** Current measurement oscillating in the middle of the ADC range of 4096/2 = 2048
** NOTE - the analog ref is set to 4.096 V by the special regulator.
*/

byte seg_meter_ready = false;

float watts[CHANNELS]; // Instantaneous
float energySum[CHANNELS]; // Cumulative for the cycles
float energySumNow[CHANNELS]; // Cumulative for the current cycle
float energySumLast[CHANNELS]; // For calculating instantaneous energySum

float currentKWOut;
float energySumOut;

unsigned long energySumTime[CHANNELS]; // Time of last calculation
unsigned long timeNow;
unsigned long duration;

int pin = 0;
int sample = 0;
float these_watts = 0;
float this_rms = 0;

float energyNow = 0;

#define GROUPED_METHOD 1
#define SINGLE_METHOD 2

int grouping_method = 0;

void seg_meter_init(void) {
   // Init arrays
   for (int channel = 0; channel < CHANNELS; channel++) {

       watts[channel] = 0.0;
       energySum[channel] = 0.0;
       energySumNow[channel] = 0.0;
       energySumLast[channel] = 0.0;
       energySumTime[channel] = millis();
   }

   define_sensors();
   set_grouped_method();

   seg_meter_ready = true;
}

void set_grouped_method() {
   grouping_method = SINGLE_METHOD;

   #ifdef GROUPED
     grouping_method = GROUPED_METHOD;
   #endif     
}

void seg_meter_handler() {
   if (seg_meter_ready == false) seg_meter_init();
   // Accumulate
   for (int cycle = 0; cycle < CYCLES; cycle++) {
       collect_channels(); // Go get those dataz!
   }
}

void collect_channels() {
   for (int channel = 0; channel < CHANNELS; channel++) {
       pin = channel;
       flash(); // a little message that we are collecting for this channel/
       energySumNow[channel] = 0;
       // TODO Magic here to deermine collection method

       //Serial.print("working on pin: ");
       //Serial.println(pin);

       switch (channelSensors[channel]) {

           case SENSOR_JD_500:
             collect_channel_transduced(channel, channelSensors[channel]);
             break;
           case SENSOR_JD_250:
             collect_channel_transduced(channel, channelSensors[channel]);
             break;
           case SENSOR_JD_100:
             collect_channel_transduced(channel, channelSensors[channel]);
             break;
           case SENSOR_JD_50:
             collect_channel_transduced(channel, channelSensors[channel]);
             break; 
           case SENSOR_JRF_80:
             collect_channel_zero_rms(channel);
             break; 
           default:
             collect_channel_true_rms(channel);
             break;
       }

       energySum[channel] += energySumNow[channel];
   }
}


void fill_sample_array(int channel) {  

  byte filled = false;
  
  while (true) {
    int i = 0;
   
    for(i = 0; i < DISCARD_COUNT; i++) {   
      read_adc(channel);
    }
  
    for(i = 0; i < SAMPLES; i++) {   
      sample_array[i] = read_adc(channel);
    }
   
    // Correct the ADC for sag.
    for(i = 0; i < SAMPLES; i++) {
     sample_array[i] = (int) ((float) sample_array[i] + (0.022 * (float) i));
  
    }
    
    // DSP the samples, alpha disabled till worked out more
    // dsp_sample_array(channel);
    break;
  }
}

byte dsp_sample_array(int channel) {
 // processes the samples, and works out some stuff
 // Zero crossing is the dead band between 2050 and 2038
 
  int i = 0;
  int zero_crossings = 0;
  int last_crossing  = 0;
  byte top           = false;
  byte dead_band     = false;
  byte bottom        = false;
  byte filled        = true;
  
  
  for(i = 0; i < SAMPLES; i++) {    
    // Serial.print(sample_array[i], DEC);
    if (sample_array[i] > (2048 - NOISE_OFFSET) && sample_array[i] < (2048 + NOISE_OFFSET)) {
      // in the dead band
      bottom    = false;
      top       = false;
      dead_band = true; 
      // Serial.println(" dead");
    }
    else if (sample_array[i] <= (2048 - NOISE_OFFSET)) {
      // in the bottoms
      if ((dead_band == true) || (top == true)) {
        // Serial.print(" falling");
        zero_crossings += 1;
      }
      
      bottom    = true;
      top       = false;
      dead_band = false; 
      // Serial.println(" bottom");
    } else {
      // by logic we are in the tops
      if ((dead_band == true) || (bottom == true)) {
        // Serial.print(" rising");
        zero_crossings += 1;
      }
      bottom    = false;
      top       = true;
      dead_band = false;       
      // Serial.println(" top");      
    }
    if (zero_crossings == 0) {
      sample_array[i] = 2048;      
    }
    
    if (zero_crossings > 4 ) {
      sample_array[i] = 2048;            
    }
    
  }
  
    
  if ((zero_crossings > CROSSINGS_UPPER) || (zero_crossings < CROSSINGS)) {
    // Crud samples, so clear them
    
    // print_sample_array(channel);
    reset_sample_array();
    Serial.print("zapped data crossings: ");
    Serial.print(zero_crossings);
    Serial.print(" for channel: ");
    Serial.println(channel);
    filled = false;      

  }
  return filled;  
}

void calculate_true_rms(int channel) {

 // print_sample_array(channel);
 this_rms = 0; 

 float mean = calculate_mean();

 for(int i = 0; i < SAMPLES; i++) {
   sample = mean - sample_array[i];
   this_rms += sq((float) sample );
 }

 this_rms = sqrt(this_rms / SAMPLES);
 if (this_rms < 0) {
   this_rms = 0.0;
 }
 
}

void print_sample_array_ends(int channel) {  
  Serial.print("Start and end of sample array for channel: ");
  Serial.print(channel + 1);
  Serial.print(" ");
  Serial.print(sample_array[0]);
  Serial.print(" - "); 
  Serial.println(sample_array[SAMPLES - 1]);
}


void print_sample_array(int channel) {  
  Serial.print("Printing sample array for channel: ");
  Serial.println(channel + 1);
  for(int i = 0; i < SAMPLES; i++) {
    Serial.println(sample_array[i]);
  }
  Serial.println();
  Serial.print("Done printing sample array for channel: ");
  Serial.println(channel + 1);
  delay(3000);
}

void reset_sample_array() {
  for (int index = 0; index < SAMPLES; index++) {   
   sample = sample_array[index] = 2048;    
 }       
}

void reset_statistics(){
 mean_sample = 2048.0;
 min_sample = 4096.0;
 max_sample = 0.0;  
}

float calculate_mean() {

 reset_statistics();
 float this_mean = 0.0;   

 for (int index = 0; index < SAMPLES; index++) {

   sample = sample_array[index];    
   if (sample < min_sample) {
     min_sample = sample;
   }
   if (sample > max_sample) {
     max_sample = sample;
   }
 }

 this_mean =  min_sample + ((max_sample - min_sample) / 2);   
 
 return this_mean; 
}


float collect_true_rms(int channel) {  
 reset_sample_array();
 fill_sample_array(channel);
 calculate_true_rms(channel);
 return this_rms;
}

void collect_channel_true_rms(int channel) {
   // For True RMS sampling the v2 ways, oscillating between a mean point somewhere in the ADC range

   these_watts = 0.0;
   
   if (channelSensors[channel] != SENSOR_NONE) {
     for (int average = 0; average < AVERAGES; average++) {
         // Process sub events and commands
         sub_events();
         this_rms = collect_true_rms(channel);
         //Serial.println(this_rms);
         if (this_rms < NOISE_OFFSET) {
           this_rms = 0;
         }
         //Serial.println(this_rms);
         these_watts += calibrateRMS(channelSensors[channel], this_rms);        
     }
  
     // Apply the channel trim.
  
     watts[channel] = (these_watts / AVERAGES) * (float) channelTrim[channel];
     //print_sample_array(channel);
  
     
   } else {
     watts[channel] = 0;
   }

   // For debugging
   #ifdef DEBUGGABLE
     watts[channel] = (channel + 1) * (float) channelTrim[channel];
   #endif

   timeNow = millis(); // Mr Wolf.
   if (timeNow > energySumTime[channel]) { // Sanity check, in case millis() wraps around

       duration = timeNow - energySumTime[channel];
       energySumNow[channel] = energySumNow[channel] + (watts[channel] * (duration / 1000.0 / 3600.0));
   }
   energySumTime[channel] = timeNow;
}

void collect_channel_zero_rms(int channel) {
   // For True RMS sampling the v2 ways oscilating around zero, with just the positive side sampled

   these_watts = 0.0;
   
   if (channelSensors[channel] != SENSOR_NONE) {
     for (int average = 0; average < AVERAGES; average++) {
         // Process sub events and commands
         sub_events();
         this_rms = collect_true_rms(channel);
         //Serial.println(this_rms);
         if (this_rms < NOISE_OFFSET) {
           this_rms = 0;
         }
         //Serial.println(this_rms);
         these_watts += calibrateRMS(channelSensors[channel], this_rms);        
     }
  
     // Apply the channel trim.
  
     watts[channel] = (these_watts / AVERAGES) * (float) channelTrim[channel];
     //print_sample_array(channel);
  
     
   } else {
     watts[channel] = 0;
   }

   // For debugging
   #ifdef DEBUGGABLE
     watts[channel] = (channel + 1) * (float) channelTrim[channel];
   #endif

   timeNow = millis(); // Mr Wolf.
   if (timeNow > energySumTime[channel]) { // Sanity check, in case millis() wraps around

       duration = timeNow - energySumTime[channel];
       energySumNow[channel] = energySumNow[channel] + (watts[channel] * (duration / 1000.0 / 3600.0));
   }
   energySumTime[channel] = timeNow;
}

float calibrateRMS(int sensor_type,  int rms_to_calibrate) {
   float output = 0.0;

   switch (sensor_type) {
     case SENSOR_SCT_013_030:
       output = rms_to_calibrate * CALIBRATION_SCT_013_030;
       break;
     case SENSOR_SCT_013_060:        
       output = rms_to_calibrate * CALIBRATION_SCT_013_060;
       output = correct_low_range(output);
       break;
     case SENSOR_CSLT:
       output = rms_to_calibrate * CALIBRATION_CSLT;
       output = correct_low_range(output);
       break;
     case SENSOR_JAYCAR_CLIP_AREF_1:
       output = rms_to_calibrate * CALIBRATION_JAYCAR_CLIP_AREF_1;
       break;
     case SENSOR_JAYCAR_CLIP_AREF_1_LOW:
       output = rms_to_calibrate * CALIBRATION_JAYCAR_CLIP_AREF_1_LOW;
       break;
     case SENSOR_JAYCAR_CLIP_AREF_5:
       output = rms_to_calibrate * CALIBRATION_JAYCAR_CLIP_AREF_5;
       break;
       case SENSOR_JRF_80:
       output = rms_to_calibrate * CALIBRATION_JRF80;
       break;
     default:
       output = rms_to_calibrate;
       break;
    }
    // Factor for measurement voltage.  110V should be less than 240V for the same current.
    output = output * ((float) MEASUREMENT_VOLTAGE / (float) 240.0) * (float) OUTPUT_MULTIPLIER;
    return output;
}


float correct_low_range(float correctable) {
   // Designed now for the non linear behaviour
   
   if (correctable != 0) {
     if (correctable <= 1.4) {
         // y = 1.024x - 0.167, solve for x
         correctable = (correctable + 0.167) / 1.024;
     } else if (correctable > 1.4 && correctable <= 2.17) {
         // y = 1.262x - 0.531, solve for x
         correctable = (correctable + 0.531) / 1.262;
     } else {
         // y = x, do nothing!
         correctable = correctable;
     }
   }
   //Serial.print(" .. ");
   //Serial.println(output);
   return correctable;
}

void  collect_channel_transduced(int channel, int sensor_type) {
   // For voltage output from an RMS processing sensor - Note this isn't really supported on the v2 yet...
   these_watts = 0.0;
   int this_sample = 0;
   for (int average = 0; average < AVERAGES; average++) {
       // Process sub events and commands
       sub_events();
       // xxx
       this_rms = 0.0;
       for (int index = 0; index < SAMPLES; index++) {
           this_sample = (float) read_adc(channel);
           this_rms += this_sample;
       }
       
       this_rms = this_rms / SAMPLES;
        
       switch (sensor_type) {
           case SENSOR_JD_500:
             // 1mV is 1 step is 0.1 amp from 5v at 500A, note the full range isn't measured as the Aref is 4.096v
             these_watts += 0.1 * (float) this_rms * (float) MEASUREMENT_VOLTAGE *  (float) OUTPUT_MULTIPLIER * (float) CALIBRATION_JD_50;
             break;
           case SENSOR_JD_250:
             // 1mV is 1 step is 0.05 amp from 5v at 250A, note the full range isn't measured as the Aref is 4.096v
             these_watts += 0.05 * (float) this_rms * (float) MEASUREMENT_VOLTAGE *  (float) OUTPUT_MULTIPLIER * (float) CALIBRATION_JD_50;
             break;
           case SENSOR_JD_100:
             // 1mV is 1 step is 0.02 amp from 5v at 100A, note the full range isn't measured as the Aref is 4.096v
             these_watts += 0.02 * (float) this_rms * (float) MEASUREMENT_VOLTAGE *  (float) OUTPUT_MULTIPLIER * (float) CALIBRATION_JD_50;
             break;
           case SENSOR_JD_50:
             // 1mV is 1 step is 0.01 amp from 5v at 50A, note the full range isn't measured as the Aref is 4.096v
             these_watts += 0.01 * (float) this_rms * (float) MEASUREMENT_VOLTAGE *  (float) OUTPUT_MULTIPLIER * (float) CALIBRATION_JD_50;
             //these_watts += this_rms;

             break;             
           default:
               these_watts = -1.0;
               break;
       }   

   }
   
   watts[channel] = (these_watts / AVERAGES) * (float) channelTrim[channel];
  

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


/*
** The ugly power_output_machine
*/


void power_output_machine() {

   int this_output = 1;
   flash_long(); // a longer flash that we are outputting datas now.

   #ifdef GROUPED
       float this_power = 0.0;
       float this_energy = 0.0;
       int channel_count = 1;

       for (int channel = 0; channel < (GROUPS * GROUP_SIZE); channel++) {

           // Lets group first

           this_power += watts[channel];
           this_energy += energySum[channel];

           if (channel_count == GROUP_SIZE) {
               // Lets output
               if (channelSensors[channel] != SENSOR_NONE) {
                 globalString.begin();
                 globalString += "(p_";
                 globalString += channel + 1;
                 globalString += " ";
                 globalString += this_power;
                 globalString += ")";
                 send_message(globalString);
  
                 globalString.begin();
                 globalString += "(e_";
                 globalString += channel + 1;
                 globalString += " ";
                 globalString += this_energy;
                 globalString += ")";
                 send_message(globalString);
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
           
           if (channelSensors[channel] != SENSOR_NONE) {
             this_power += watts[channel];
             this_energy += energySum[channel];
  
             if (channel_count == SUB_GROUP_SIZE) {
                 globalString.begin();
                 globalString += "(p_";
                 globalString += channel + 1;
                 globalString += " ";
                 globalString += this_power;
                 globalString += ")";
                 send_message(globalString);
  
                 globalString.begin();
                 globalString += "(e_";
                 globalString += channel + 1;
                 globalString += " ";
                 globalString += this_energy;
                 globalString += ")";
                 send_message(globalString);
  
                 channel_count = 1;
                 this_power = 0.0;
                 this_energy = 0.0;
             } else {
                 channel_count += 1;
             }
           }
       }

       // Now thats done, output the remaining single channels

       for (int channel = ((GROUPS * GROUP_SIZE) + (SUB_GROUPS * SUB_GROUP_SIZE)); channel < CHANNELS; channel++) {
         
         if (channelSensors[channel] != SENSOR_NONE) {
           globalString.begin();
           globalString += "(p_";
           globalString += channel + 1;
           globalString += " ";
           globalString += watts[channel];
           globalString += ")";
           send_message(globalString);
  
           globalString.begin();
           globalString += "(e_";
           globalString += channel + 1;
           globalString += " ";
           globalString += energySum[channel];
           globalString += ")";
           send_message(globalString);
         }
      }
   #endif
   
   #ifdef SINGLE_PHASE
       for (int channel = 0; channel < CHANNELS; channel++) {
         if (channelSensors[channel] != SENSOR_NONE) {
           globalString.begin();
           globalString += "(p_";
           globalString += channel + 1;
           globalString += " ";
           globalString += watts[channel];
           globalString += ")";
           send_message(globalString);
          
           globalString.begin();
           globalString += "(e_";
           globalString += channel + 1;
           globalString += " ";
           globalString += energySum[channel];
           globalString += ")";
           send_message(globalString);
         }
       }
   #endif
   
   #ifdef SUBTOTAL_AT
       currentKWOut = 0.0;
       energySumOut = 0.0;

       // Sum the puppies
       for (int channel = 0; channel < SUBTOTAL_AT; channel++) {
         if (channelSensors[channel] != SENSOR_NONE) {
           currentKWOut += watts[channel];
           energySumOut += energySum[channel];
         }
       }

       globalString.begin();
       globalString += "(p_st ";
       globalString += currentKWOut;
       globalString += ")";
       send_message(globalString);

       globalString.begin();
       globalString += "(e_st ";
       globalString += energySumOut;
       globalString += ")";
       send_message(globalString);
   
   #endif

   #ifdef SUM_CHANNELS
       currentKWOut = 0.0;
       energySumOut = 0.0;

       // Sum the puppies
       for (int channel = 0; channel < CHANNELS; channel++) {

           currentKWOut += watts[channel];
           energySumOut += energySum[channel];
       }

       globalString.begin();
       globalString += "(p_a ";
       globalString += currentKWOut;
       globalString += ")";
       send_message(globalString);

       globalString.begin();
       globalString += "(e_a ";
       globalString += energySumOut;
       globalString += ")";
       send_message(globalString);
   #endif
   
     #ifdef SUBTRACTABLE_AT
   
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
             for (int channel = GROUP_SIZE; channel < SUBTRACTABLE_AT + 1; channel++) {
                 remaining_power += watts[channel];
                 remaining_energy += energySum[channel];
             }
             currentKWOut = group_power - remaining_power;
             energySumOut = group_energy - remaining_energy;
           #endif
       } else {
           for (int channel = 1; channel < SUBTRACTABLE_AT + 1; channel++) {
             currentKWOut += watts[channel];
             energySumOut += energySum[channel];
           }
           currentKWOut = watts[0] - currentKWOut;
           energySumOut = energySum[0] - energySumOut;
       }

       globalString.begin();
       globalString += "(p_s ";
       globalString += currentKWOut;
       globalString += ")";
       send_message(globalString);

       globalString.begin();
       globalString += "(e_s ";
       globalString += energySumOut;
       globalString += ")";
       send_message(globalString);
       
   
   #endif
   
   
   #ifdef GREEN
       currentKWOut = 0.0;
       energySumOut = 0.0;

       // Sum the puppies
       for (int channel = 0; channel < CHANNELS; channel++) {
         
           if (channelTrim[channel] < 0) {
             // i.e. it's solar!  Note it's again flipped to output a positive value for SEG
             currentKWOut += watts[channel] * -1.0;
             energySumOut += energySum[channel] * -1.0;
           }
       }

       globalString.begin();
       globalString += "(p_g ";
       globalString += currentKWOut;
       globalString += ")";
       send_message(globalString);

       globalString.begin();
       globalString += "(e_g ";
       globalString += energySumOut;
       globalString += ")";
       send_message(globalString);
   #endif

   #ifdef SUBTRACT_CHANNELS
       currentKWOut = 0.0;
       energySumOut = 0.0;
       
       //Serial.println("IN subtract channels");

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

       globalString.begin();
       globalString += "(p_s ";
       globalString += currentKWOut;
       globalString += ")";
       send_message(globalString);

       globalString.begin();
       globalString += "(e_s ";
       globalString += energySumOut;
       globalString += ")";
       send_message(globalString);
   #endif
   // Reset the output arrays for the next time through.
   reset_energy_arrays();
   blinkHandler();
}  // End of power_output_machine


void reset_energy_arrays() {
   for (int channel = 0; channel < CHANNELS; channel++) {
       watts[channel] = 0;
       energySum[channel] = 0;
   }  
   
   // Turn sending off until next time!
   stop_data();
}

int read_adc(int channel) {
  if (channel < 8) {
     return get_adc(CS, channel);
   } else {
     return get_adc(CS_E, (channel - 8));
   }
}

int get_adc(int cs_pin, int channel)
{
 // Samples the expanded ADC via SPI

 //Serial.print("reading ADC for channel: ");
 //Serial.println(channel + 1);
 int adc_value = 0;
 byte command = B11000000; // Command byte: start (1), mode (1), channel (3), dont care (3).
 command |= ((channel) << 3); // Set the channel bits to the channel we want.
 digitalWrite(cs_pin, LOW); // Enable chip.

 for(int i = 7; i >= 3; i--)
 {
   digitalWrite(MOSI, (command & 1 << i)); // Write out the SPI command byte to the ADC.
   digitalWrite(SCK, HIGH); // Cycle SPI clock.
   digitalWrite(SCK, LOW);    
 }

 digitalWrite(SCK, HIGH);
 digitalWrite(SCK, LOW);
 digitalWrite(SCK, HIGH);  
 digitalWrite(SCK, LOW);

 for(int i = 11; i >= 0; i--) // Read 12-bit output value from the ADC.
 {
   adc_value += (digitalRead(MISO) << i);
   digitalWrite(SCK, HIGH); // Cycle SPI clock.
   digitalWrite(SCK, LOW);
 }

 digitalWrite(cs_pin, HIGH); // Chip de-selected.
 return adc_value;
}


/* --------------------------------------------------------------------------
 ** Voltage Sensor
 */

float voltageRaw = 0;
float voltageValue = 0;

void dc_voltage_sensor(void) {

    voltageValue = 0.0;

    // Divide out the ratio from the ADC and multiply by Aref
    // Then multiply by 3.125 (note R1 = 220k and R2 = 100k)
    voltageValue = ((float) analogRead(PIN_DC_VOLTAGE_SENSOR) / 1024) * 5.15 * 3.125;

    globalString += "(voltage ";
    globalString += voltageValue;
    globalString += " V)";
}


/* --------------------------------------------------------------------------
** Temperature Sensor Handler
*/


void temperature_sensor(void) {

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

