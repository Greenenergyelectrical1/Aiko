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
*  v2_8_6  Adjusted channel trim to int for switching - memory issues - last good v2.1x version
*  SMD win!
*  v25_1_1 First SMD based SEGmeter
*  v25_1_2 Revision for JRF-80 to use normal wriggle RMS sampling
*  v25_1_3 Adding true power measurement with voltage sampling
*  v25_1_4 Adding phase shift and refined Rogowskis with the SEG integrator circuitry and Light Sensor
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


* -- Executing commands from SEG:
  -- smartenergygroups.com can send commands to commandable SEGmeters or nodes
  -- the command comes on the response of the sending data :put command in the form
  -- (node_name (relay relay_state shot_seconds command_id));
  -- or for other relays
  -- (node_name (relay_1 relay_state shot_seconds command_id));
  -- (node_name (relay_2 relay_state shot_seconds command_id));
  -- (node_name (relay_3 relay_state shot_seconds command_id));
  --
  -- where:
  -- node_name = the name of your SEG device (refer above)
  -- relay = a fixed string from SEG telling the device this is a relay command
  -- relay_state = on or off - self explainatory ;)
  -- shot_seconds = the following values:
  --     ? or 0 remain on indefinately or
  --     some value in seconds to remain in the state of on
  -- command_id = the SEG command id, for reciepting the completion of the command
  --
  -- The SEGmeter device will take this command and execute it, and return a message
  -- for sending back to SEG for completion of the command like so:
  --
  -- (node node_name time_stamp (command command_id command_status))
  --
  -- where:
  -- command = a keyword to identify the information being sent to SEG is a command receipt
  -- command_status = the status of the command, usually complete, but may be error.
  --
* Good luck!
* Sam, @samotage
*/
  
#include <PString.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define NODE_NAME   "segmeter"
#define CHANNELS    8 // Analog sensor(s)
#define IS_SMD
#define TRUE_POWER  0 // Toggle 0 disabled or 1 to enable
#define SINGLE_PHASE
//#define GROUPED // For multiphase, see below


#define HAS_BUTTON
#define COMMANDABLE
#define IS_DRAGINO
//#define DEBUGGABLE
//#define PRINTABLE

#define BOOT_MESSAGE "(restart 25.1.4)"

// Phase Options


//#define CHOP_TO_WAVEFORM        // Controls chopping of samples to a waveform in dsp samples
#define CROSSINGS          4   // DSP waves are half this number.

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

#define GROUPS 2
#define GROUP_SIZE 3

#define SUB_GROUPS 0
#define SUB_GROUP_SIZE 0

//#define SUBTOTAL_AT 6 // Sums all the channels to this point and creates a subtotal, which is
                      // then used by the sum or subtract below...
                      // HINT, use the negative trim and flip afterwards in SEG for solars :D
                      // Also, Solar goes on the LAST channel :)
                      // AND if the subtotal is for all channels up to say 1 thru 6 set this to 6
                      
//#define SUBTOTAL_2_AT 14  // A second subtotal to be reported for channels between SUBTOTAL_AT and this index.


//#define GREEN         // Summs all the channels where the multiplier is less than 1 - i.e. green energy to output as  single total.
#define SUM_CHANNELS  // setting to sum all and output total as _a//

//#define SUBTRACT_CHANNELS // setting to subtract with math:  _s = ch0 - (all other channels)
//#define SUBTRACTABLE_AT   // Where the subtration virtual is to be calculated from a series of channels ending before the last channel.
                            // I.e. where the SEGmeter is measuring across boards.
                            // HINT:  if the last channel is 12, set this to 12.


#define MEASUREMENT_VOLTAGE 240.0  // For Australian conditions, 240V, 110 for US, and 230 for Europe
#define OUTPUT_MULTIPLIER 1.0  // Post calibrations multiplier for ALL channels.  For individual channels, see trim below.

#ifdef TRUE_POWER == 1
  #define SAMPLES         108 // three waves at 36 samples a wave for 2ch sampling at 50 Hz
  #define PHASE_SHIFT     9  // samples to phase shift forward 36/4 for 90 degrees 
#else
  #define SAMPLES         110 // enough samples for a single waveform at 50hz  110 for current only sampling
  #define PHASE_SHIFT     0
#endif

#define AVERAGES          4  // Number of RMS values to average
#define CYCLES            4  // Number of times to cycle through the calculations

// TODO, work out a sample time that will calculate the above, say 50 seconds, this back manipulates etc...

#define DISCARD_COUNT   30  // Samples to discard when switching channels for moar wins

int actual_samples = SAMPLES;  // To take into account how many usable samples after DSP

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
#define SENSOR_JRF_80    12

#define SENSOR_SEG_20_DC 13
#define SENSOR_CSLT_DC   14

#define SENSOR_100V_DC   15

#define SENSOR_240V_AC   16
#define SENSOR_LDR       17  // Light sensor outputting magics for lux and irradiance


int channelSensors[CHANNELS];  // Tell it what sensors are on which channels
float channelTrim[CHANNELS];   // To trim the output

int first_voltage_channel = 0;
int first_current_channel = 1;
float true_power          = 0.0;

/*
**----------------------------------------------------------------------------------------------------------------------------------------------------
*/

void define_sensors() {
  
     // Base Deck
     
     channelSensors[0]  = SENSOR_SCT_013_060;
     channelSensors[1]  = SENSOR_SCT_013_060;
     channelSensors[2]  = SENSOR_SCT_013_060;
     channelSensors[3]  = SENSOR_SCT_013_060;
     channelSensors[4]  = SENSOR_SCT_013_060;
     channelSensors[5]  = SENSOR_SCT_013_060;
     channelSensors[6]  = SENSOR_SCT_013_060;
     channelSensors[7]  = SENSOR_SCT_013_060;
    
     // Expansion Deck
    
     channelSensors[8]  = SENSOR_CSLT;
     channelSensors[9]  = SENSOR_CSLT;
     channelSensors[10]  = SENSOR_CSLT;
     channelSensors[11]  = SENSOR_SCT_013_060;
     channelSensors[12]  = SENSOR_SCT_013_060;
     channelSensors[13]  = SENSOR_CSLT;
     channelSensors[14]  = SENSOR_NONE;
     channelSensors[15]  = SENSOR_NONE;
 
 
     // Base Deck, channel trim factor, multiply the power reading by the factor
     channelTrim[0]  = 1.0;
     channelTrim[1]  = 1.0;
     channelTrim[2]  = 1.0;
     channelTrim[3]  = 1.0;
     channelTrim[4]  = 1.0;
     channelTrim[5]  = 1.0;
     channelTrim[6]  = 1.0;
     channelTrim[7]  = 1.0;

     // Expansion Deck
     channelTrim[8]  = 1.0;
     channelTrim[9]  = 1.0;
     channelTrim[10] = 1.0;
     channelTrim[11] = 1.0;
     channelTrim[12] = 1.0;
     channelTrim[13] = 1.0;
     channelTrim[14] = 1.0;
     channelTrim[15] = 1.0;

     if (TRUE_POWER == 1) {

        #ifdef SINGLE_PHASE
             first_voltage_channel = 0;
             first_current_channel = 1;
         #endif

         #ifdef GROUPED
             first_voltage_channel = 0;
             first_current_channel = GROUP_SIZE;
         #endif
     }

}

#define NOISE_OFFSET 2

// Calibration factors
#define CALIBRATION_NONE 0


#define CALIBRATION_SCT_013_030   8.63   // Estimated

/*  As I think the actual_samples will fix this ;)
#ifdef CHOP_TO_WAVEFORM             // Because chopped waveforms have less values to crunch
  #define CALIBRATION_CSLT          21.93
  #define CALIBRATION_SCT_013_060   19.77  // Re-calibrated at the SEGshop
#else
  #define CALIBRATION_CSLT          16.87
  #define CALIBRATION_SCT_013_060   15.89  // Re-calibrated at the SEGshop
#endif
*/

#define CALIBRATION_CSLT          16.87
#define CALIBRATION_SCT_013_060   15.89  // Re-calibrated at the SEGshop

#define CALIBRATION_JAYCAR_CLIP_AREF_1     33.333
#define CALIBRATION_JAYCAR_CLIP_AREF_1_LOW 280.0      // At Carey
#define CALIBRATION_JAYCAR_CLIP_AREF_5     144.45

#define CALIBRATION_JD_50    1.04
#define CALIBRATION_JD_100   1.04
#define CALIBRATION_JD_250   1.04
#define CALIBRATION_JD_500   1.04
#define CALIBRATION_JRF80    178.4  // 10x gain on the OP Amp Calibration factor for the new Rogowski coil high-current sensors (Melb Uni)

#define CALIBRATION_SEG_20_DC   0.255
#define CALIBRATION_CSLT_DC     0.03265

#define CALIBRATION_100V_DC     0.02683

// True RMS Calibrations
#define CALIBRATION_240V_AC_VOLT    0.457  // from a sample to a volt

//As SCT 013 060 :
#define CALIBRATION_TRUE_POWER_AMP 0.067  // from a sample to an amp

// For JRF 80 : 
//#define CALIBRATION_TRUE_POWER_AMP 0.491  // from a sample to an amp

// For SCT 013 060 :
//#define CALIBRATION_TRUE_POWER_AMP 0.994  // from a sample to an amp


// Pin assignment
#define PIN_SERIAL_RX         0
#define PIN_SERIAL_TX         1
#define PIN_ONE_WIRE          5 // OneWire or CANBus
#define PIN_LED_STATUS        3 // Standard Arduino flashing LED !
#define PIN_DC_VOLTAGE_SENSOR 0 // Arduino Pins.

// Pin Microchip MCP3208 8-channel 12-bit SPI-interfaced ADC over SPI.
// MOSI, MISO and SCK should not be changed, but CS refrences the ADC Bank selected

// V2.0, 2.1 
// CS   9
// CS_E 10

// V2.5
// CS   10
// CS_E  9

#ifdef IS_SMD
  #define CS     10
  #define CS_E   9
#else
  #define CS     9
  #define CS_E   10
#endif


#define MOSI 11
#define MISO 12
#define SCK  13

#define DRAGON_BOOT_WAIT 12000  // milliseconds

#define OUTPUT_BUFFER 70
#define INPUT_BUFFER 50

char globalBuffer[OUTPUT_BUFFER]; // Used to manage dynamically constructed strings
PString globalString(globalBuffer, sizeof (globalBuffer));

OneWire oneWire(PIN_ONE_WIRE); // Maxim DS18B20 temperature sensor
DallasTemperature sensors(&oneWire);

int i       = 0;  // General purpose iterator
int sample  = 0;
int current_samples[SAMPLES];
int voltage_samples[SAMPLES * TRUE_POWER];  //ie no voltage sample array if true power is not active.
int buffer_samples[PHASE_SHIFT * TRUE_POWER];

float mean_sample = 2048;
float min_sample = 4096;
float max_sample = 0;

byte send_data = false;
byte initialised = false;

#define RELAYS 3
byte relay_on[RELAYS];
unsigned long relay_turned_on[RELAYS];
unsigned long relay_energise_time[RELAYS];

String seg_command;
byte is_for_me = false;

String energise;
String seg_command_id;
String command_response;

#define PIN_RELAY_1         6
#define PIN_RELAY_2         7
#define PIN_RELAY_3         8

#define PIN_BUTTON          A0

/*
**----------------------------------------------------------------------------------------------------------------------------------------------------
*/

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

   digitalWrite(CS,   HIGH); // Base     ADC is disabled. Remember, low = enabled.
   digitalWrite(CS_E, HIGH); // Expanded ADC is disabled. Remember, low = enabled.
   digitalWrite(MOSI, LOW);
   digitalWrite(SCK,  LOW);

   flash_long();
   delay(100);
   flash_long();
   delay(100);
   flash_long();
   
   #ifdef IS_DRAGINO
     delay(DRAGON_BOOT_WAIT);  /// to prevent Dragon bootloader failing, no boot message :( 
     Serial.begin(38400);
   #else
     Serial.begin(38400);
     //send_reboot_message();
   #endif   
}

/*
**----------------------------------------------------------------------------------------------------------------------------------------------------
*/

void loop() {
   seg_meter_handler();
   sub_events();

   if (send_data == true) {
       globalString.begin();
       temperature_sensor();
       send_message(globalString);
       
       globalString.begin();
       power_output_machine();
       // Other measurements are sent on their own global string after the power output.
       light_sensor();
   }
}

/* ----------------------------------------------------------------------------------------------------------------------------------------------------
** These things are called inside the guts of the main energy sampling for
** implementing control and switching.power_
*/
void sub_events() {
    // Serial.println("in sub events");
    #ifdef COMMANDABLE
      serial_handler();
      one_shot_handler();
    #endif

    #ifdef HAS_BUTTON
      button_handler();
    #endif
}

/*
**----------------------------------------------------------------------------------------------------------------------------------------------------
*/

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

/*
**----------------------------------------------------------------------------------------------------------------------------------------------------
*/

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


/* ----------------------------------------------------------------------------------------------------------------------------------------------------
** Light Sensor
*/


void light_sensor() {
     for (int channel = 0; channel < CHANNELS; channel++) {
       //Serial.println("in light_sensor");       
       switch (channelSensors[channel]) {
             case SENSOR_LDR:
                  determine_light_levels(channel);
             default:
                   //Serial.print("BOMBED in light_sensor for type: ");       
                   //Serial.println(channelSensors[channel]);
                 break;
       }      
    }
}

void determine_light_levels(int channel) {
 
  //Serial.println("in the determine_light_levels");
  
   // Read the ADC a bunch of times
  float cumulative = 0.0;
  for(i = 0; i < SAMPLES; i++) {
    cumulative += (float) read_adc(channel);
  }

  cumulative = cumulative / SAMPLES;
  
  //  Now work out the lux from the formula derived from measurements and maths
  //  http://mathonweb.com/help_ebook/html/expoapps_3.htm
  //
  //  lux = 0.00717 * e^(0.0036334 * sample)  
  //  and eulers numebr = 2.718281828459
  
  cumulative = 0.0036334 * cumulative;
  
  //Serial.println(cumulative_adc);
  
  cumulative = 0.00717 * pow(2.718281828459, cumulative); 
  
  globalString.begin();
  globalString += "(lux_";
  globalString += channel + 1;
  globalString += " "; 
  globalString += cumulative;
  globalString += ")";
  send_message(globalString);
  
  //Serial.print(cumulative);
  //Serial.print(" Lux, Illuminance is: ");  
  
  // Convert to W/cm2
  // 1,000,000 lux = 0.146 W/cm2 = 1460 W/m2
  // and there are 10,000 sq cms in a m2, so see above
  // 1 lux = 0.00146 W/m2
  
  cumulative = cumulative * 0.00146;
  
  globalString.begin();
  globalString += "(irradiance_";
  globalString += channel + 1;
  globalString += " "; 
  globalString += cumulative;
  globalString += ")";
  send_message(globalString);
  
  
  //Serial.print(cumulative);
  //Serial.println(" W/m2 ");  
    
}

/*
** ----------------------------------------------------------------------------------------------------------------------------------------------------------
**  Current measurement oscillating in the middle of the ADC range of 4096/2 = 2048 (roughly...)
**  NOTE - the analog ref is set to 4.096 V by the special regulator.
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
float these_watts = 0;
float this_rms = 0;

float energyNow = 0;

#define GROUPED_METHOD 1
#define SINGLE_METHOD 2

int grouping_method = 0;


/*
**----------------------------------------------------------------------------------------------------------------------------------------------------
*/

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

/*
**----------------------------------------------------------------------------------------------------------------------------------------------------
*/

void set_grouped_method() {
   grouping_method = SINGLE_METHOD;

   #ifdef GROUPED
     grouping_method = GROUPED_METHOD;
   #endif
}

/*
**----------------------------------------------------------------------------------------------------------------------------------------------------
*/

void seg_meter_handler() {
   if (seg_meter_ready == false) seg_meter_init();
   // Accumulate
   for (int cycle = 0; cycle < CYCLES; cycle++) {
       collect_channels(); // Go get those dataz!
   }
}

/*
**----------------------------------------------------------------------------------------------------------------------------------------------------
*/

void collect_channels() {
   for (int channel = 0; channel < CHANNELS; channel++) {
       pin = channel;
       flash(); // a little message that we are collecting for this channel/
       energySumNow[channel] = 0;
       
       
       if (TRUE_POWER == 1) {


          switch (channelSensors[channel]) {
            
            case SENSOR_240V_AC:
              collect_channel_true_rms_voltage(channel);
              break;
            default:
              collect_channel_true_power(channel);
              energySum[channel] += energySumNow[channel];
              break;                      
          }
         
       } else {
         
         switch (channelSensors[channel]) {
           case SENSOR_JD_500:
                collect_channel_transduced(channel, channelSensors[channel]);
                energySum[channel] += energySumNow[channel];
                break;
           case SENSOR_JD_250:
                collect_channel_transduced(channel, channelSensors[channel]);
                energySum[channel] += energySumNow[channel];
                break;
           case SENSOR_JD_100:
                collect_channel_transduced(channel, channelSensors[channel]);
                energySum[channel] += energySumNow[channel];
                break;
           case SENSOR_JD_50:
                collect_channel_transduced(channel, channelSensors[channel]);
                energySum[channel] += energySumNow[channel];
                break;
           case SENSOR_SEG_20_DC:
                collect_dc_energy(channel);
                energySum[channel] += energySumNow[channel];
                break;
           case SENSOR_100V_DC:
                collect_dc_voltage(channel);
                energySum[channel] = energySumNow[channel];  // No accumulation on this hack which is for amps 
                break;
           default:
                collect_channel_true_rms(channel);
                energySum[channel] += energySumNow[channel];
                break;           
         }
      }
   }
}

/*
**----------------------------------------------------------------------------------------------------------------------------------------------------
*/

float collect_dc_voltage(int channel) {
    //Get the voltage as watts channel, this will be outputted in the power output machine as volts.

    watts[channel] = calculate_samples_average(channel);
    watts[channel] = calibrate_sensor(channelSensors[channel], watts[channel]);
    watts[channel] = watts[channel] * (float) channelTrim[channel];
    
    /*
    Serial.print("collect_dc_voltage channel: ");
    Serial.print(channel + 1);
    Serial.print("voltage: ");
    Serial.print(watts[channel]);
    */
    
    // Note, Will Use the other energy units for the amps, found from the channel's current pair
    
    energySumNow[channel] = calculate_samples_average(channel - 1);
    energySumNow[channel] = calibrate_sensor(channelSensors[channel - 1], energySumNow[channel]);
    energySumNow[channel] = energySumNow[channel] * (float) channelTrim[channel - 1];

    /*
    Serial.print(" current: ");
    Serial.print(channel + 1);
    Serial.print(" is ");
    Serial.println(energySumNow[channel]);
     */
     
    energySumTime[channel] = millis();
    return watts[channel];
}

/*
**----------------------------------------------------------------------------------------------------------------------------------------------------
*/

void collect_dc_energy(int channel) {
    // Get the current in the DC sensor
    
    watts[channel] = calculate_samples_average(channel);
    watts[channel] = calibrate_sensor(channelSensors[channel], watts[channel]);    

    /*
    Serial.print("collect_dc_energy channel: ");
    Serial.print(channel + 1);
    Serial.print(" current: ");
    Serial.print(watts[channel]);
    */

    // Then get the voltage ( from this channel + 1) and multiply
    // The convention here is P =  Amps * Volts
    float voltage = 0.0;
    voltage = collect_dc_voltage(channel + 1);

    watts[channel] = voltage * watts[channel] * (float) channelTrim[channel];
    
    /*
    Serial.print(" power: ");
    Serial.print(watts[channel]);
    */
    
    timeNow = millis(); // Mr Wolf.
    if (timeNow > energySumTime[channel]) { // Sanity check, in case millis() wraps around
       duration = timeNow - energySumTime[channel];
       energySumNow[channel] = energySumNow[channel] + (watts[channel] * (duration / 1000.0 / 3600.0));
    }
    
    /*
    Serial.print(" energy: ");
    Serial.println(energySumNow[channel]);
    */
    
    energySumTime[channel] = timeNow;
}

/*
**----------------------------------------------------------------------------------------------------------------------------------------------------
*/


void collect_channel_true_rms_voltage(int channel) {
  
}


/*
**----------------------------------------------------------------------------------------------------------------------------------------------------
*/


void collect_channel_true_power(int channel) {
   // For True voltage and current sampling on dual waveforms
   these_watts = 0.0;

   //Serial.println("about to call process_true_power");

   if (channelSensors[channel] != SENSOR_NONE) {
     for (int average = 0; average < AVERAGES; average++) {
         // Process sub events and commands
         sub_events();
         process_true_power(channel);
         these_watts += true_power;
     }
     watts[channel] = (these_watts / AVERAGES) * (float) channelTrim[channel];

   } else {
     watts[channel] = 0.0;
   }

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

/*
**----------------------------------------------------------------------------------------------------------------------------------------------------
*/

void process_true_power(int current_channel) {
  
    reset_samples(voltage_samples);
    reset_samples(current_samples);

    int this_voltage_channel = 0;

    #ifdef SINGLE_PHASE
        this_voltage_channel = first_voltage_channel;
    #endif

    #ifdef GROUPED
        // Use the awesome modulo function % to divide the current channel by group size to get the voltage channel #FTW
        this_voltage_channel = ((first_current_channel + current_channel) % GROUP_SIZE);
    #endif

    //Serial.println("about to call read_voltage_power_channels");
    read_voltage_power_channels(this_voltage_channel, current_channel);
    dsp_true_power_samples();
    
    if (channelSensors[current_channel] == SENSOR_JRF_80) {
      shift_samples(current_samples);  // Shifting because of the integration offsetting
      invert_samples(current_samples); // As there is an negative integrating op amp in the circiut.
    }

    calculate_true_power(current_channel);
    
    #ifdef PRINTABLE
      print_voltage_current_samples(this_voltage_channel, current_channel);
    #endif    
}


/*
**----------------------------------------------------------------------------------------------------------------------------------------------------
*/

float calculate_true_power(int channel) {

    true_power = 0.0;
    // Multiply out the samples, then divide.
    for(i = 0; i < SAMPLES; i++) {

        true_power += ((float) voltage_samples[i] * CALIBRATION_240V_AC_VOLT) * ((float) current_samples[i] * CALIBRATION_TRUE_POWER_AMP);
    }
    true_power = true_power / actual_samples;
    /*
    Serial.print(true_power);
    Serial.print(" channel ");
    Serial.print(channel);
    Serial.print(", actual_samples ");    
    Serial.println(actual_samples);
    */
}

/*
**----------------------------------------------------------------------------------------------------------------------------------------------------
*/


void shift_samples(int *samples) {
  // Shifting the a samples, note 
  // caveats in that SAMPLES must be more than 2 x PHASE_SHIFT ;)
  
  for(int i = 0; i < SAMPLES; i++) {
    if ( i < PHASE_SHIFT) {
      // Buffer samples
      buffer_samples[i] = samples[i];
    } 
    
    if ((i + PHASE_SHIFT) < SAMPLES) {
      // Move samples
      samples[i] = samples[i + PHASE_SHIFT];
    } else {
      break;
    }
  }
   
  for(int i = 0; i < PHASE_SHIFT; i++) {
    // Backfill    
    samples[(SAMPLES - PHASE_SHIFT) + i] = buffer_samples[i];
  }
  

}

/*
**----------------------------------------------------------------------------------------------------------------------------------------------------
*/

void invert_samples(int *samples)  {  
  for(int i = 0; i < SAMPLES; i++) {
     // Now invert the samples
     samples[i] = samples[i] * -1;
  }  
}


/*
**----------------------------------------------------------------------------------------------------------------------------------------------------
*/

void collect_channel_true_rms(int channel) {
   // For True RMS sampling the v2 ways, oscillating between a mean point somewhere in the ADC range
   these_watts = 0.0;

   if (channelSensors[channel] != SENSOR_NONE) {
     for (int average = 0; average < AVERAGES; average++) {
         // Process sub events and commands
         sub_events();
         this_rms = process_true_rms(channel);
         if (this_rms < NOISE_OFFSET) {
           this_rms = 0;
         }
         these_watts += calibrate_rms(channelSensors[channel], this_rms);
     }
     watts[channel] = (these_watts / AVERAGES) * (float) channelTrim[channel];
     
   } else {
     watts[channel] = 0;
   }

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

/*
**----------------------------------------------------------------------------------------------------------------------------------------------------
*/

float process_true_rms(int channel) {
    reset_samples(current_samples);
    fill_current_samples(channel);
    calculate_true_rms(channel);
    return this_rms;
}

/*
**----------------------------------------------------------------------------------------------------------------------------------------------------
*/

float calibrate_sensor(int sensor_type,  int rms_to_calibrate) {
    float output = 0.0;

    
    switch (sensor_type) {
        case SENSOR_SEG_20_DC:
            output = (float) rms_to_calibrate - 2490.0;  // 2490 is 2.5V ish and middle of the hall sensor range
                        
            if (((output * output) / output) < NOISE_OFFSET + 1) {
              output = 0;
            }
            
            output = output * CALIBRATION_SEG_20_DC;
            break;
            
        case SENSOR_CSLT_DC:
            output = (float) rms_to_calibrate - 2500.0;  // 2500 is 2.5V ish and middle of the hall sensor range
            
            if (((output * output) / output) < NOISE_OFFSET + 1) {
              output = 0;
            }
            
            output = output * CALIBRATION_CSLT_DC;
            break;  
            
        case SENSOR_100V_DC:
            output = rms_to_calibrate * CALIBRATION_100V_DC;
            break;

        default:
            output = rms_to_calibrate;
            break;
    }
    return output;
}

/*
**----------------------------------------------------------------------------------------------------------------------------------------------------
*/

float calibrate_rms(int sensor_type,  int rms_to_calibrate) {
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

/*
**----------------------------------------------------------------------------------------------------------------------------------------------------
*/

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
   return correctable;
}

/*
**----------------------------------------------------------------------------------------------------------------------------------------------------
*/

void  collect_channel_transduced(int channel, int sensor_type) {
   // For voltage output from an RMS processing sensor - Note this isn't really supported on the v2 yet...
   these_watts = 0.0;
   int this_sample = 0;
   for (int average = 0; average < AVERAGES; average++) {
       // Process sub events and commands
       sub_events();
       this_rms = 0.0;
       for (i = 0; i < SAMPLES; i++) {
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
**----------------------------------------------------------------------------------------------------------------------------------------------------
*/

void calculate_true_rms(int channel) {
     // print_current_samples(channel);
     this_rms = 0;
     dsp_samples(current_samples);


     for(i = 0; i < SAMPLES; i++) {
       sample = current_samples[i];
       this_rms += sq((float) sample );
     }

     this_rms = sqrt(this_rms / actual_samples);
     if (this_rms < 0) {
       this_rms = 0.0;
     }
}

/*
**----------------------------------------------------------------------------------------------------------------------------------------------------
*/

float calculate_samples_average(int channel) {
    this_rms = 0;
    reset_samples(current_samples);
    fill_current_samples(channel);
    //print_current_samples(channel);
    this_rms = calculate_average();
    return this_rms;
}

/*
**----------------------------------------------------------------------------------------------------------------------------------------------------
*/

void read_voltage_power_channels(int voltage_channel, int current_channel) {  
  /*
  Serial.print("reading voltage channel: ");
  Serial.print(voltage_channel);
  Serial.print("     current channel: ");
  Serial.println(current_channel);
  */

  for(i = 0; i < DISCARD_COUNT; i++) {
    read_adc(voltage_channel);
    read_adc(current_channel);
  }

  for(i = 0; i < SAMPLES; i++) {
    voltage_samples[i] = read_adc(voltage_channel);
    current_samples[i] = read_adc(current_channel);
  }
}

/*
**----------------------------------------------------------------------------------------------------------------------------------------------------
*/

void fill_current_samples(int channel) {
    for(i = 0; i < DISCARD_COUNT; i++) {
      read_adc(channel);
    }
    for(i = 0; i < SAMPLES; i++) {
      current_samples[i] = read_adc(channel);
    }
}

/*
**----------------------------------------------------------------------------------------------------------------------------------------------------
*/

void dsp_samples(int *samples) {
 // processes the samples, and works out some stuff
 // Zero crossing point is calculated for the samples.

  actual_samples = SAMPLES;

  #ifdef CHOP_TO_WAVEFORM
      int zero_crossings = 0;
      int last_crossing  = 0;
      byte top           = false;
      byte dead_band     = false;
      byte bottom        = false;
      byte filled        = true;

      calculate_mean(samples);

      for(i = 0; i < SAMPLES; i++) {
        if (samples[i] > (mean_sample - NOISE_OFFSET) && samples[i] < (mean_sample + NOISE_OFFSET)) {
          bottom    = false;
          top       = false;
          dead_band = true;

        }
        else if (samples[i] <= (mean_sample - NOISE_OFFSET)) {
          // in the bottoms
          if ((dead_band == true) || (top == true)) {
            zero_crossings += 1;
          }

          bottom    = true;
          top       = false;
          dead_band = false;
        } else {
          // by logic we are in the tops
          if ((dead_band == true) || (bottom == true)) {
            zero_crossings += 1;
          }
          bottom    = false;
          top       = true;
          dead_band = false;
        }

        if (zero_crossings == 0) {
          samples[i] = 0;
          actual_samples--;
        }

        if (zero_crossings > CROSSINGS ) {
          samples[i] = 0;
          actual_samples--;
        }
      }
  #endif

  calculate_mean(samples);
  offset_samples(samples);
}

/*
**----------------------------------------------------------------------------------------------------------------------------------------------------
*/

void dsp_true_power_samples() {
 // processes the samples, and works out some stuff
 // Zero crossing point is calcualted for the samples.
 
 actual_samples = SAMPLES;

  #ifdef CHOP_TO_WAVEFORM
      int zero_crossings = 0;
      int last_crossing  = 0;
      byte top           = false;
      byte dead_band     = false;
      byte bottom        = false;
      byte filled        = true;

      calculate_mean(voltage_samples);

      for(i = 0; i < SAMPLES; i++) {
        // Serial.print(sample_array[i], DEC);
        if (voltage_samples[i] > (mean_sample - NOISE_OFFSET) && voltage_samples[i] < (mean_sample + NOISE_OFFSET)) {
          // in the dead band
          bottom    = false;
          top       = false;
          dead_band = true;

        }
        else if (voltage_samples[i] <= (mean_sample - NOISE_OFFSET)) {
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
          current_samples[i] = 0;
          voltage_samples[i] = 0;
          actual_samples--;
        }

        if (zero_crossings > CROSSINGS ) {
          current_samples[i] = 0;
          voltage_samples[i] = 0;
          actual_samples--;
        }
      }
  #endif

  calculate_mean(voltage_samples);
  offset_samples(voltage_samples);
  calculate_mean(current_samples);
  offset_samples(current_samples);
}

/*
**----------------------------------------------------------------------------------------------------------------------------------------------------
*/

void print_current_samples_ends(int channel) {
      Serial.print("Start and end of sample array for channel: ");
      Serial.print(channel + 1);
      Serial.print(" ");
      Serial.print(current_samples[0]);
      Serial.print(" - ");
      Serial.println(current_samples[SAMPLES - 1]);
}

/*
**----------------------------------------------------------------------------------------------------------------------------------------------------
*/

void print_voltage_current_samples(int voltage_channel, int current_channel) {


      for(int i = 0; i < SAMPLES; i++) {
        Serial.print(voltage_samples[i]);
        Serial.print(",");
        Serial.println(current_samples[i]);
      }

      Serial.print("Printed above voltage ch: ");
      Serial.print(voltage_channel);
      Serial.print(" current ch: ");
      Serial.println(current_channel);
      delay(15000);
}

/*
**----------------------------------------------------------------------------------------------------------------------------------------------------
*/

void print_current_samples(int channel) {
      Serial.print("Printing sample array for channel: ");
      Serial.println(channel + 1);
      for(i = 0; i < SAMPLES; i++) {
        Serial.println(current_samples[i]);
      }
      Serial.println();
      Serial.print("Done printing sample array for channel: ");
      Serial.println(channel + 1);
      delay(20000);
}

/*
**----------------------------------------------------------------------------------------------------------------------------------------------------
*/

void reset_samples(int *samples) {
    for (i = 0; i < SAMPLES; i++) {
        sample = samples[i] = 2048;
    }
}

/*
**----------------------------------------------------------------------------------------------------------------------------------------------------
*/

void reset_statistics(){
     mean_sample = 2048.0;
     min_sample = 4096.0;
     max_sample = 0.0;
}

/*
**----------------------------------------------------------------------------------------------------------------------------------------------------
*/

float calculate_average() {
 unsigned long summary_value = 0;
 for (i = 0; i < SAMPLES; i++) {
    summary_value += current_samples[i];
    //Serial.print("after add: ");
    //Serial.println(sample);
 }
 //Serial.print("Sum of all samples: ");
 //Serial.println(sample);
 return (float) (summary_value / SAMPLES);
}

/*
**----------------------------------------------------------------------------------------------------------------------------------------------------
*/

void calculate_mean(int *samples) {

 reset_statistics();

 for (i = 0; i < SAMPLES; i++) {
    sample = samples[i];

    if (sample != 0) {
      if (sample < min_sample) {
          min_sample = sample;
      }
      if (sample > max_sample) {
          max_sample = sample;
      }
    }
 }
 
 mean_sample = (float) (min_sample + ((max_sample - min_sample) / 2.0));
 
}

/*
**----------------------------------------------------------------------------------------------------------------------------------------------------
*/

void offset_samples(int *samples) {
    for ( i = 0; i < SAMPLES; i++) {
        if (samples[i] != 0) {
            samples[i] = samples[i] - mean_sample;

            if ((samples[i] < NOISE_OFFSET) and (samples[i] > (NOISE_OFFSET * -1))) {
              samples[i] = 0;
            }

        }
    }
}




/*
** -------------------------------------------------------------------
** The forever ugly power_output_machine
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
           
           switch(channelSensors[channel])  {
             case SENSOR_NONE:
               break;
             
             case SENSOR_240V_AC:
               break;

             
             default:
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
               break;
           }

           
       }

       // Output the sub grouped channels

       for (int channel = (GROUPS * GROUP_SIZE); channel < ((GROUPS * GROUP_SIZE) + (SUB_GROUPS * SUB_GROUP_SIZE)); channel++) {
           // Special sub grouped
           
           switch(channelSensors[channel])  {
             case SENSOR_NONE:
               break;
             
             case SENSOR_240V_AC:
               break;
               
             default:
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
             
               break;
           }
       }

       // Now thats done, output the remaining single channels
       

       for (int channel = ((GROUPS * GROUP_SIZE) + (SUB_GROUPS * SUB_GROUP_SIZE)); channel < CHANNELS; channel++) {

           switch(channelSensors[channel])  {
             case SENSOR_NONE:
               break;
             
             case SENSOR_240V_AC:
               break;
               
             default:
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
               
               break;
           }
      }
   #endif

   #ifdef SINGLE_PHASE
       for (int channel = 0; channel < CHANNELS; channel++) {

            switch(channelSensors[channel])  {
                case SENSOR_NONE:
                    // Do nothing...
                    break;
                case SENSOR_100V_DC:
                
                    globalString.begin();
                    globalString += "(a_";
                    globalString += channel;  // Amps come from the channel pair.
                    globalString += " ";
                    globalString += energySum[channel];
                    globalString += ")";
                    send_message(globalString);
                                    
                    globalString.begin();
                    globalString += "(v_";
                    globalString += channel + 1;
                    globalString += " ";
                    globalString += watts[channel];
                    globalString += ")";
                    send_message(globalString);
                    
                    break;
                    
                case SENSOR_240V_AC:
                    break;

                default:
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
                    break;
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
   
    #ifdef SUBTOTAL_2_AT
       currentKWOut = 0.0;
       energySumOut = 0.0;

       // Sum the puppies
       for (int channel = SUBTOTAL_AT; channel < SUBTOTAL_2_AT; channel++) {
         if (channelSensors[channel] != SENSOR_NONE) {
           currentKWOut += watts[channel];
           energySumOut += energySum[channel];
         }
       }

       globalString.begin();
       globalString += "(p_st_2 ";
       globalString += currentKWOut;
       globalString += ")";
       send_message(globalString);

       globalString.begin();
       globalString += "(e_st_2 ";
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


/*
**----------------------------------------------------------------------------------------------------------------------------------------------------
*/

void reset_energy_arrays() {
   for (int channel = 0; channel < CHANNELS; channel++) {
       watts[channel] = 0;
       energySum[channel] = 0;
   }

   // Turn sending off until next time!
   stop_data();
}

/*
**----------------------------------------------------------------------------------------------------------------------------------------------------
*/

int read_adc(int channel) {
  if (channel < 8) {
     return get_adc(CS, channel);
   } else {
     return get_adc(CS_E, (channel - 8));
   }
}

/*
**----------------------------------------------------------------------------------------------------------------------------------------------------
*/

int get_adc(int cs_pin, int channel)
{
 // Samples the expanded ADC via SPI

 //Serial.print("reading ADC for channel: ");
 //Serial.println(channel + 1);
 sample = 0;
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
   sample += (digitalRead(MISO) << i);
   digitalWrite(SCK, HIGH); // Cycle SPI clock.
   digitalWrite(SCK, LOW);
 }

 digitalWrite(cs_pin, HIGH); // Chip de-selected.
 return sample;
}


/* ----------------------------------------------------------------------------------------------------------------------------------------------------
** Temperature Sensor Handler
*/

void temperature_sensor(void) {

 i = 0;
 int resultant_temp = 0;
 int bus_devices = 0;
 byte written = false;
 sensors.begin();

 bus_devices = sensors.getDeviceCount();
 sensors.requestTemperatures();

 while (i < bus_devices) {
   if (sensors.getTempCByIndex(i) == -127) break;

     globalString += "(temperature_";
     globalString += i + 1;
     globalString += " ";
     globalString += sensors.getTempCByIndex(i);
     globalString += ") ";

     i += 1;
   }
}


/* ----------------------------------------------------------------------------------------------------------------------------------------------------------
** Serial Handler, and related command codes
*/

#ifdef COMMANDABLE

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
            for (byte index = 0; index < count; index++) {
                char ch = Serial.read();
                if (length >= (sizeof (buffer) / sizeof (*buffer))) {
                    length = 0;
                } else if (ch == '\n' || ch == ';') {
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
            // Work out out the SEG commands
            seg_command = get_command_element(buffer, 1);

            if (seg_command == "start_data")  {
                start_data();
            }  else if (seg_command == "stop_data")  {
                stop_data();
            }  else if (seg_command == "relay")  {
                //  The default state is to toggle ALL relays
                relay_command(1, buffer);
                relay_command(2, buffer);
                relay_command(3, buffer);    
            }  else if (seg_command == "relay_all")  {
                //  The default state is to toggle ALL relays
                relay_command(1, buffer);
                relay_command(2, buffer);
                relay_command(3, buffer);                    
            }  else if (seg_command == "relay_1")  {
                relay_command(1, buffer);
            }  else if (seg_command == "relay_2")  {
                relay_command(2, buffer);
            }  else if (seg_command == "relay_3")  {
                relay_command(3, buffer);
            } else {
                // Serial.println("...doing nothing with this command");
            }
        }
    }

    String get_command_element(char* input, int target_element) {
          // example command  "(all_nodes (start_data));"
          // CAVEAT if there are multiple spaces, this wil fail!
  
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

 
    /* ----------------------------------------------------------------------------------------------------------------------------------------------------
     ** Various command handlers
     */

    void start_data(){
      //Serial.println("commanded to start data output");
      send_data = true;
    }

    void stop_data(){
      send_data = false;
    }

    byte relays_initialised = false;

    void initialise_relays() {

        for (i = 0; i < RELAYS; i++) {
            relay_on[i]             = false;
            relay_turned_on[i]      = 0;
            relay_energise_time[i]  = 0;
        }
        relays_initialised = true;
    }

    void relay_command(int relay_commanded, char* buffer) {

        if (relays_initialised != true ){
            initialise_relays();
        }
        command_response = "error";

        set_relay_parameters(relay_commanded, buffer);

        if (energise == "on") {
            turn_relay_on(relay_commanded, true);
            command_response = "complete";
        } else if (energise == "off") {
            turn_relay_off(relay_commanded, true);
            command_response = "complete";
        } else {
            //  Do nothing, sending stuff here messes the mesh.
            //  Rebroadcast the command and check at the start wether it's for this node.
            //  send_message("(error parameterInvalid)");
        }
       complete_relay_command();
    }

    void set_relay_parameters(int relay_commanded, char* buffer) {

        energise       = get_command_element(buffer, 2);
        seg_command    = get_command_element(buffer, 3);  // For energise_time
        seg_command_id = get_command_element(buffer, 4);

        if (seg_command.startsWith('?')) {
            relay_energise_time[relay_commanded - 1] = 0;
        } else {
            // Buggery for atoi...
            char this_char[seg_command.length() + 1];
            seg_command.toCharArray(this_char, (seg_command.length() + 1));
            relay_energise_time[relay_commanded - 1] = atol(this_char) * 1000;
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

    /* ----------------------------------------------------------------------------------------------------------------------------------------------------
     ** Works out if to shut the relay off or not
     */

    void one_shot_handler(void) {

        // Check the shot time for all 3 relays

        for (i = 0; i < RELAYS; i++) {
            if (relay_energise_time[i] > 0) {
                  if (relay_on[i] == true) {
                      if (millis() > relay_turned_on[i]) {
                          if ((millis() - relay_turned_on[i]) > relay_energise_time[i]) {
                              // No need to tell SEG that shot has expired.  Otherwise the device will de-energise...
                              //Serial.println("About to turn off silently...");
                              turn_relay_off((i + 1), false);
                          }
                      } else {
                          // No need to tell SEG that shot has expired.  Otherwise the device will de-energise...
                          // We have a rollover, so turn eet off
                          turn_relay_off((i + 1), false);
                      }
                  }
            }
        }
    }

#endif // COMMANDABLE

#ifdef HAS_BUTTON

    /*
    **------------------------------------------------------------------------
    **  Button Handler, this is a little button that toggles the relay 1, 2 or 3 ON or OFF
    **  It runs from a single analog input, and there is a voltage divider that does magic to
    **  Adjust the values seen on the ADC.
    **  Ranges from the ADC are
    **  Relay 1:  +/- 50 for a nominal of  346
    **  Relay 2:  +/- 50 for a nominal of  703
    **  Relay 3:  +/- 50 for a nominal of 1023
    **
    **  And then tells SEG the new state of the appropriate relay
    */

    byte buttons_initialised = false;
    byte button_changed_state = false;
    int this_button_state = 0;
    int last_button_state = 0;

    int button_pressed = 0;
    int last_button_pressed = 0;

    long last_debounce_time = 0;  // the last time the output pin was toggled
    long debounce_delay     = 70;    // the debounce time; increase if the output flickers

    void initialise_buttons() {
        buttons_initialised    = true;
        button_changed_state  = false;
    }

    void button_handler()  {
      if (buttons_initialised == false) {
        initialise_buttons();
      }

      // read the state of the switch into a local variable:
      button_pressed = determine_relay_button_pressed();

      if (button_pressed != 0) {
        if ( button_pressed == last_button_pressed) {
          if (button_changed_state == false) {
             button_changed_state = true;
            toggle_relay_state(button_pressed);
          }
        } else {
          button_changed_state = false;
        }
      }
      last_button_pressed = button_pressed;
    }

    int determine_relay_button_pressed() {
       int pressed_button = 0;  // returns 1, 2, or 3

       /*
       **  Relay 1:   +/- 50 for a nominal of 346
       **  Relay 2:   +/- 50 for a nominal of 703
       **  Relay 3:   +/- 50 for a nominal of 1023
       **
       ** This little function also works out the debouncing.
       */

        this_button_state = analogRead(PIN_BUTTON);
        
        //Serial.println(this_button_state);

        if ( (this_button_state > 10) and (((this_button_state + 20)  > last_button_state) and ((this_button_state  -20) < last_button_state))) {
           // The button is in the same state, is it debounceable?
            if ((millis() - last_debounce_time) > debounce_delay) {
                 // whatever the reading is at, it's been there for longer
                 // than the debounce delay, so take it as the actual current state:
                // Work out what button has been pressed

                if (this_button_state < 400) {
                    // Its relay 1
                    pressed_button = 1;
                    //Serial.println("  pressed button 1");

                } else if (this_button_state > 600 and this_button_state < 800) {
                    // Its relay 2
                    pressed_button = 2;
                    //Serial.println("  pressed button 2");

                } else {
                    // Its relay 3
                    pressed_button = 3;
                    //Serial.println("  pressed button 3");

                }                
            }
        } else {
            // reset the debouncing timer, and the changeable state
            last_debounce_time = millis();
        }
        last_button_state = this_button_state;
        return pressed_button;
    }

    void toggle_relay_state(int relay_to_toggle) {
        if (relay_on[relay_to_toggle - 1] == true) {
            turn_relay_off(relay_to_toggle, false );

            globalString.begin();
            globalString += "(switched relay_";
            globalString += relay_to_toggle;
            globalString += " off)";

            send_message(globalString);

        } else {
            turn_relay_on(relay_to_toggle, false );

            globalString.begin();
            globalString += "(switched relay_";
            globalString += relay_to_toggle;
            globalString += " on)";

            send_message(globalString);
        }
    }

#endif  // For HAS_BUTTON

void relay_messenger(char* message) {
    globalString.begin();
    globalString += message;
    send_message(globalString);
}

void turn_relay_on(int relay_id, bool send_a_message ) {

    bool operated = false;

    switch (relay_id) {
        case 1:
            digitalWrite(PIN_RELAY_1, HIGH);
            operated = true;
            break;
        case 2:
            digitalWrite(PIN_RELAY_2, HIGH);
            operated = true;
            break;
        case 3:
            digitalWrite(PIN_RELAY_3, HIGH);
            operated = true;
            break;
        default:
            // Do nothing as no button pressed
            break;
    }

    relay_turned_on[relay_id - 1] = millis();
    relay_on[relay_id - 1]        = true;

    if ((operated == true) and (send_a_message == true)) {

        globalString.begin();
        globalString += "(relay_";
        globalString += relay_id;
        globalString += " on)";
        send_message(globalString);

    }
}

void turn_relay_off(int relay_id, bool send_a_message) {

    bool operated = false;

    switch (relay_id) {
        case 1:
            digitalWrite(PIN_RELAY_1, LOW);
            operated          = true;
            break;
        case 2:
            digitalWrite(PIN_RELAY_2, LOW);
            operated          = true;
            break;
        case 3:
            digitalWrite(PIN_RELAY_3, LOW);
            operated          = true;
            break;
        default:
            // Do nothing as no button pressed
            break;
    }

    relay_on[relay_id - 1]            = false;
    relay_turned_on[relay_id - 1]     = 0;
    relay_energise_time[relay_id - 1] = 0;

    if ((operated == true) and (send_a_message == true)) {
        globalString.begin();
        globalString += "(relay_";
        globalString += relay_id;
        globalString +=  " off)";
        send_message(globalString);
    }
}



/* ----------------------------------------------------------------------------------------------------------------------------------------------------
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
