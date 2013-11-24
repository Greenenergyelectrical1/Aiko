/*
* ~~~~~~~~~~~~~
* Please do not remove the following notices.
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
*  v25_sleepy_1  A special sleepy version cut down.
*
* Sam, @samotage
*/

#include <LowPower.h>
#include <Wire.h> 
#include <RTClib.h>

#include <PString.h>
#include <OneWire.h>
#include <DallasTemperature.h>

RTC_DS1307 RTC;

#define NODE_NAME   "segmeter_sleepy"
#define CHANNELS    1 // Analog sensor(s)
#define IS_SMD
#define SLEEPS      2     // 8 seconds per sleep
#define SLEEP_TIME  8     // Seconds

//#define  START_UNIX    1363219199  //   Wed Mar 13 23:59:59 +0000 2013 an end of day date time in UTC
#define  START_UNIX    1363204799    //   Above adjusted back four hours to correct the output time.
#define  TZ_OFFSET     36000       //   10 hours, i.e. 10 * 3600


#define MEASUREMENT_VOLTAGE 240.0  // For Australian conditions, 240V, 110 for US, and 230 for Europe
#define OUTPUT_MULTIPLIER 1.0  // Post calibrations multiplier for ALL channels.  For individual channels, see trim below.

#define SAMPLES         110 // enough samples for a single waveform at 50hz  110 for current only sampling
#define AVERAGES        4  // Number of RMS values to average
#define CYCLES          1  // Measurements per data point
#define DATA_LENGTH     93 // How many data points to store
#define DATA_INTERVAL   28800 // 8 hours in seconds 3600 * 8 in seconds between intervals

#define AWAKE_FOR       20000   // Stay awake for these miliseconds waiting for a command before auto resleep


#define DISCARD_COUNT   30  // Samples to discard when switching channels for moar wins

#define NOISE_OFFSET 2
#define SEC_HOUR  3600

// Pin assignment
#define PIN_SERIAL_RX         0
#define PIN_SERIAL_TX         1
#define PIN_ONE_WIRE          5 // OneWire or CANBus
#define PIN_LED_STATUS        3 // Flashing LED !
#define PIN_DC_VOLTAGE_SENSOR 0 // Arduino Pins.
#define WAKE_PIN              2


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

#define INPUT_BUFFER 30

OneWire oneWire(PIN_ONE_WIRE); // Maxim DS18B20 temperature sensor
DallasTemperature sensors(&oneWire);

int index   = 0;  // General purpose iterator
int sample  = 0;
int current_samples[SAMPLES];

int sleep = 0;
float mean_sample = 2048;
float min_sample = 4096;
float max_sample = 0;

byte has_data = false;

String seg_command;
byte is_for_me = false;

String seg_command_id;
String command_response;

byte awoken      = false;
byte asleep      = false;
byte send_data   = false;
byte data_stored = false;


int this_sleep = 0;
unsigned long  seconds_now  = 0;
unsigned long  seconds_last = 0;
unsigned long  seconds_remaining = 0;
unsigned long  awoken_at    = 0;
long           time_diff = 0.0;    // For the maths to work

unsigned long unix_time[DATA_LENGTH];
long          data_value[DATA_LENGTH];

long    watts[CHANNELS];          // Instantaneous average watts
int     data_index        = 0;           // The current data index
int     cycle             = 0;
float   this_rms          = 0.0;
long   these_watts       = 0.0;
long   running_total     = 0;
int     channelSensors[CHANNELS];  // Tell it what sensors are on which channels
int     throttle          = 0;
#define THROTTLE_SETTING    10

int     flash_count       = 0;
#define FLASH_THROTTLE    2

int     data_count         = 0;
#define DATA_THROTTLE      90


/*
**----------------------------------------------------------------------------------------------------------------------------------------------------
*/

#define SENSOR_SCT_013_060 0
#define SENSOR_DS18B20     1

#define CALIBRATION_SCT_013_060   15.89  // Re-calibrated at the SEGshop

void define_sensors() {
     // Base Deck
     channelSensors[0]  = SENSOR_SCT_013_060;
     // channelSensors[0]  = SENSOR_DS18B20;
}


/*
**----------------------------------------------------------------------------------------------------------------------------------------------------
*/

void setup() {

    pinMode(WAKE_PIN, INPUT);
    pinMode(CS,   OUTPUT);
    pinMode(CS_E, OUTPUT);
    pinMode(MOSI, OUTPUT);
    pinMode(MISO, INPUT);
    pinMode(SCK,  OUTPUT);

    pinMode(PIN_LED_STATUS, OUTPUT);


    digitalWrite(CS,   HIGH); // Base     ADC is disabled. Remember, low = enabled.
    digitalWrite(CS_E, HIGH); // Expanded ADC is disabled. Remember, low = enabled.
    digitalWrite(MOSI, LOW);
    digitalWrite(SCK,  LOW);
    
    status_on();

    Wire.begin();
    RTC.begin();


    seconds_now = the_time_now();
    seconds_last = seconds_now;
    
    initialise_data_arrays();
    
    Serial.begin(38400);
    Serial.flush();
    Serial.println();
    
    attachInterrupt(0, wake_me_up, HIGH); // use interrupt 0 (on pin 2) and run wake_me_up

    if (! RTC.isrunning()) {        
        // following line sets the RTC to the date & time this sketch was compiled
        send_message("(rtc 0)");           
        RTC.adjust(DateTime(__DATE__, __TIME__));
    } else {
        send_message("(rtc 1)");                
        time_and_status();  
    }

    seg_meter_init();
    
    send_message("ready");

    awoken_at = 0;
    boot_flash();
    status_off();
}


void boot_flash() {
  
      status_off();
      delay(10);
      status_on();
      delay(10);
      status_off();
      delay(10);
      status_on();
      delay(10);
      status_off();
      delay(10);
      status_on();
      delay(10);
      status_off();
      delay(10);
      status_on();
      delay(10);
      status_off();
      delay(10);
      status_on();
      delay(1000);
}

void loop() {
  
    // Serial.println("start of main loop");
    // time_and_status();
    flash();    
    
    if (awoken == true) {
      Serial.flush();
      awoken_at = millis();       
      send_message("(awoken 1)");              
      awoken = false; 
      throttle = 0;
    }    
    
    serial_handler();

    seg_meter_handler();   // Accumulates power for the period
     //temperature_sensor();  // Accumulates temperature for the period
   
    serial_handler();
    
    if (send_data == true) {
      data_output();
    }
    
    if (awoken_at > 0) {
        if ((millis() - awoken_at) > AWAKE_FOR) {
            awoken_at = 0;
        } else {
            if (throttle > THROTTLE_SETTING) {
                if (send_data != true)  {              
                    send_message("(awoken 0)");           
                }
                
                throttle = 0;
            } else {
                throttle++;          
            }            
        }
    }
        
    if (awoken_at == 0) { 
      serial_handler();
      sleep_now();
    }           
}

/* ----------------------------------------------------------------------------------------------------------------------------------------------------
** This interrupt handler controlls the sending of Datas
*/

void wake_me_up() {

    delay(200);
    awoken = true;
    asleep = false;    
    detachInterrupt(0);
}

void print_date(unsigned long unix_time) {  
    DateTime dt (unix_time);
    Serial.print(dt.year(), DEC);
    Serial.print('-');
    Serial.print(dt.month(), DEC);
    Serial.print('-');
    Serial.print(dt.day(), DEC);
    Serial.print('T');
    Serial.print(dt.hour(), DEC);
    Serial.print(':');
    Serial.print(dt.minute(), DEC);
    Serial.print(':');
    Serial.print(dt.second(), DEC);
}

/*
**----------------------------------------------------------------------------------------------------------------------------------------------------
*/

void sleep_now()  {
  
  int sleep_index = 0;
  attachInterrupt(0, wake_me_up, HIGH); // use interrupt 0 (on pin 2) and run wake_me_up
  delay(200);     // A small to check not awoekn
  
  if (awoken == false) {
      //send_message("(sleeping 1)");  
      blink();
      delay(200);     // A small delay so the serial isn't broken...  
 
      while (sleep_index < SLEEPS) {
        
        if (awoken == true) {
          break;
        }
        send_data = false;
        asleep = true;
        LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_ON);
        sleep_index ++;
      }  
  }
  
  asleep = false;
  
}


/*
**----------------------------------------------------------------------------------------------------------------------------------------------------
*/

void store_data_point() {
    // this takes the values and makes a data point entry.
    // Serial.print("inside store_data_point() watts times now,last and difference: ");

    seconds_now = the_time_now();
    
    /*
    Serial.print(seconds_now);
    Serial.print(" - ");
    Serial.print(seconds_last);
    Serial.print(" - ");    
    */
    
    for (int channel = 0; channel < CHANNELS; channel++) {
        if (data_index == DATA_LENGTH) {
            data_index = 0;  // A data rollover event
        }
        
        // Work out a normalised time at the end of an interval
        
        time_diff  = (the_time_now() - (START_UNIX + TZ_OFFSET)) / DATA_INTERVAL;
        unix_time[data_index] = (time_diff * DATA_INTERVAL) +  (START_UNIX + TZ_OFFSET);        
        unix_time[data_index] = unix_time[data_index] + DATA_INTERVAL; // Add another data interval to bring it to the current time.
        
        // the following fuckery is to overcome floating point math fail.
        time_diff = (seconds_now - seconds_last) * 10000;  // in ten thousanths of a second now.
        time_diff = (time_diff / SEC_HOUR);                   // hour fraction times 10,000        
        
        
        data_value[data_index] = (watts[channel] / cycle);         
        
        if (channelSensors[channel] != SENSOR_DS18B20) {
          // Multiply by time, - otherwise it's average temperature.
          data_value[data_index] = data_value[data_index] * time_diff;  // Note this is still multiplied by 10,000 at this time.  
          running_total += data_value[data_index];                      // Same as above.
        } else {
          running_total += 0;
        }
        
        
        
        /*
        Serial.print(" time diff: ");
        Serial.print(time_diff);        
        Serial.print(" hours, watts: ");
        Serial.print(watts[channel]); 
        Serial.print(" for cycles: ");
        Serial.print(cycle);
       
               
        Serial.print(" data element: ");
        Serial.print(data_index);
        
        Serial.print(" time: ");
        Serial.print(unix_time[data_index]);
        
        
        Serial.print(" value: ");
        Serial.println(data_value[data_index]);
        */
        
        has_data = true;    
        data_index ++;       // Increment the index
        watts[channel] = 0.0;  // Reset this accumulator

        
    }    
    data_stored = true;
    seconds_last = seconds_now;        
}

/*
**----------------------------------------------------------------------------------------------------------------------------------------------------
*/

void data_output()  {
    // Outputs the data in the data array!    
      has_data = true;
    
      if (data_count > DATA_THROTTLE) {
          status_on();
          if (has_data == true) {            
              data_count = 0;
              // calculate and send the total as a checksum
              these_watts = 0;
              
            //  for (index = 0; index < DATA_LENGTH; index++) {
            //        unix_time[index] = 1000000;
            //        data_value[index] = millis() * random(300);
            //  }
   
              for (index = 0; index < DATA_LENGTH; index++) {
                    if (unix_time[index] > 0 ) {
                        these_watts += data_value[index];
                    }
              }
              
              Serial.print("(node ");
              Serial.print(NODE_NAME);
              Serial.print(" checksum ");
              Serial.print((float) these_watts / (float) 10000);
              Serial.println(")");
              
              for (int channel = 0; channel < CHANNELS; channel++) {                  
                  if (channelSensors[channel] != SENSOR_DS18B20) {
                      for (index = 0; index < DATA_LENGTH; index++) {
                          if (unix_time[index] > 0 ) {
                              Serial.print("(node ");
                              Serial.print(NODE_NAME);
                              Serial.print(" ");
                              //Serial.print(unix_time[index]);
                              print_date(unix_time[index]);
                              
                              Serial.print(" ");
                              Serial.print("(e_1 ");
                              Serial.print( (float) data_value[index] / (float) 10000);
                              Serial.println("))");
                          }
                          
                          serial_handler();
                      }
                  } else {
                      for (index = 0; index < DATA_LENGTH; index++) {
                          if (unix_time[index] > 0 ) {
                              Serial.print("(node ");
                              Serial.print(NODE_NAME);
                              Serial.print(" ");
                              Serial.print(unix_time[index]);
                              Serial.print(" ");
                              Serial.print("(temperature_1 ");
                              Serial.print( (float) data_value[index] / (float) 10);
                              Serial.println("))");
                          }
                          
                          serial_handler();
                      }                    
                    
                  }
              }                                                  
    
              Serial.print("(node ");
              Serial.print(NODE_NAME);
              Serial.print(" data_sent ");
              Serial.print((float) these_watts / (float) 10000);
              Serial.println(")");
          
              
              Serial.print("(node ");
              Serial.print(NODE_NAME);
              Serial.print(" ");
              Serial.print(the_time_now());
              Serial.print(" ");
              Serial.print("(e_total ");
              Serial.print((float)running_total / (float) 10000);
              Serial.println("))");

      
          }
      
          get_ram();
          status_off();
        
    } else {
      data_count ++;
    }

    

}

//----------------------------------------------------------------------------------------------------------------------------------------------------

void send_message(const char* message) {
   if (message) {
     Serial.print("(node ");
     Serial.print(NODE_NAME);
     Serial.print(" ");
     Serial.print(the_time_now());
     Serial.print(" ");
     Serial.print(message);
     Serial.println(")");
   }
   delay(20);
}

/* ----------------------------------------------------------------------------------------------------------------------------------------------------*/

void initialise_data_arrays() {
    for (index = 0; index < DATA_LENGTH; index ++) {
        unix_time[index]   = 0;
        data_value[index]  = 0.0;
    }
    running_total  = 0;
    has_data = false;
}

/* ----------------------------------------------------------------------------------------------------------------------------------------------------------
**  Current measurement oscillating in the middle of the ADC range of 4096/2 = 2048 (roughly...)
**  NOTE - the analog ref is set to 4.096 V by the special regulator.  */


void seg_meter_init(void) {
   // Init arrays
   for (int channel = 0; channel < CHANNELS; channel++) {
       watts[channel] = 0.0;
   }
   define_sensors();
}

// ----------------------------------------------------------------------------------------------------------------------------------------------------

void seg_meter_handler() {

    collect_channels(); // Go get those dataz!
    cycle ++;

    if (send_data == false) {
        if (store_data_now()) {
            store_data_point();
            cycle = 0;
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------------------------

boolean store_data_now()  {
  
  time_diff  = RTC.now().unixtime() - (START_UNIX + TZ_OFFSET);  
  seconds_remaining = DATA_INTERVAL - (time_diff % DATA_INTERVAL);  
  //Serial.print("seconds_remaining: ");
  //Serial.print(seconds_remaining);
    
  if ((seconds_remaining >= (DATA_INTERVAL - ((SLEEPS + 2) * SLEEP_TIME))  ) || (seconds_remaining <= ((SLEEPS + 2) * SLEEP_TIME))) {     
    if (data_stored == false) {  
         return true;         
     } 
  } else {
    data_stored = false;
  }    
  return false;
}

// ----------------------------------------------------------------------------------------------------------------------------------------------------

void collect_channels() {

    for (int channel = 0; channel < CHANNELS; channel++) {        
        if (channelSensors[channel] == SENSOR_DS18B20) {            
            collect_temperature(channel);
        } else {            
            collect_channel_true_rms(channel);
        }
    }
}

// ----------------------------------------------------------------------------------------------------------------------------------------------------
// Temperature Sensor Handler

void collect_temperature(int channel) {       
  
     index = 0;
     int bus_devices = 0;
     sensors.begin();
     bus_devices = sensors.getDeviceCount();
     sensors.requestTemperatures();
    
     while (index < bus_devices) {
         these_watts = 0;
         if (sensors.getTempCByIndex(index) == -127) break;
         
         these_watts = (long) (sensors.getTempCByIndex(index) * 10);
//         Serial.print("temperature is: ");
//         Serial.println(these_watts);           
         
         watts[channel] += these_watts;                  
         break;
     }
}

//----------------------------------------------------------------------------------------------------------------------------------------------------

void collect_channel_true_rms(int channel) {
   // For True RMS sampling the v2 ways, oscillating between a mean point somewhere in the ADC range
   
   these_watts = 0;
   
    for (int average = 0; average < AVERAGES; average++) {
        // Process sub events and commands
        
        serial_handler();

        process_true_rms(channel);
                     
        if (this_rms < NOISE_OFFSET) {
            this_rms = 0;
        }
        these_watts += calibrate_rms(channelSensors[channel], this_rms);
        //Serial.println(these_watts);
        
    }           
    watts[channel] += (these_watts / AVERAGES);   
    // Serial.print("    and accumulated is: ");
    //Serial.println(watts[channel]);           
}


// ----------------------------------------------------------------------------------------------------------------------------------------------------


long calibrate_rms(int sensor_type,  float rms_to_calibrate) {
    long output = 0.0;

    switch (sensor_type) {
        case SENSOR_SCT_013_060:
            output = rms_to_calibrate * CALIBRATION_SCT_013_060;
            output = correct_low_range(output);
            break;

        default:
            output = rms_to_calibrate;
            break;
    }
    // Factor for measurement voltage.  110V should be less than 240V for the same current.
    output = output * ((float) MEASUREMENT_VOLTAGE / (float) 240.0) * (float) OUTPUT_MULTIPLIER; 
   
    //Serial.println(output);
    
    return output;
}

/*
**----------------------------------------------------------------------------------------------------------------------------------------------------
*/

float correct_low_range(float correctable) {
   // Designed now for the non linear behaviour at the low range.

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

void process_true_rms(int channel) {
    reset_samples(current_samples);
    fill_current_samples(channel);
    calculate_true_rms(channel);

}

/*
**----------------------------------------------------------------------------------------------------------------------------------------------------
*/

void reset_samples(int *samples) {
    for (index = 0; index < SAMPLES; index++) {
        sample = samples[index] = 2048;
    }
}

/*
**----------------------------------------------------------------------------------------------------------------------------------------------------
*/

void fill_current_samples(int channel) {
    for(index = 0; index < DISCARD_COUNT; index++) {
      read_adc(channel);
    }
    for(index = 0; index < SAMPLES; index++) {
      current_samples[index] = read_adc(channel);
    }
}

/*
**----------------------------------------------------------------------------------------------------------------------------------------------------
*/

void calculate_true_rms(int channel) {
     // print_current_samples(channel);
     this_rms = 0;
     dsp_samples(current_samples);

     for(index = 0; index< SAMPLES; index++) {
       sample = current_samples[index];
       this_rms += sq((float) sample );
     }

     this_rms = sqrt(this_rms / SAMPLES);
     if (this_rms < 0) {
       this_rms = 0.0;
     }
}


/*
**----------------------------------------------------------------------------------------------------------------------------------------------------
*/

void dsp_samples(int *samples) {
    // Simplified routine.
    calculate_mean(samples);
    offset_samples(samples);
}


/*
**----------------------------------------------------------------------------------------------------------------------------------------------------
*/

void calculate_mean(int *samples) {
    reset_statistics();
    for (index = 0; index < SAMPLES; index++) {
        sample = samples[index];
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

void reset_statistics(){
     mean_sample = 2048.0;
     min_sample = 4096.0;
     max_sample = 0.0;
}

/*
**----------------------------------------------------------------------------------------------------------------------------------------------------
*/

void offset_samples(int *samples) {
    for ( index = 0; index < SAMPLES; index++) {
        if (samples[index] != 0) {
            samples[index] = samples[index] - mean_sample;
            if ((samples[index] < NOISE_OFFSET) and (samples[index] > (NOISE_OFFSET * -1))) {
              samples[index] = 0;
            }
        }
    }
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

 for(int adc_index = 7; adc_index >= 3; adc_index--)
 {
   digitalWrite(MOSI, (command & 1 << adc_index)); // Write out the SPI command byte to the ADC.
   digitalWrite(SCK, HIGH); // Cycle SPI clock.
   digitalWrite(SCK, LOW);
 }

 digitalWrite(SCK, HIGH);
 digitalWrite(SCK, LOW);
 digitalWrite(SCK, HIGH);
 digitalWrite(SCK, LOW);

 for(int adc_index = 11; adc_index >= 0; adc_index--) // Read 12-bit output value from the ADC.
 {
   sample += (digitalRead(MISO) << adc_index);
   digitalWrite(SCK, HIGH); // Cycle SPI clock.
   digitalWrite(SCK, LOW);
 }

 digitalWrite(cs_pin, HIGH); // Chip de-selected.
 return sample;
}






/* ----------------------------------------------------------------------------------------------------------------------------------------------------
** Time and status
*/

void time_and_status() {

    DateTime now = RTC.now();

    Serial.print("(node ");
    Serial.print(NODE_NAME);
    Serial.print(" ");
    Serial.print(now.unixtime());
    Serial.print(" ");
    Serial.print("(ram ");
    Serial.print(free_ram());
    Serial.print(")(time ");
    
    Serial.print(now.year(), DEC);
    Serial.print('-');
    Serial.print(now.month(), DEC);
    Serial.print('-');
    Serial.print(now.day(), DEC);
    Serial.print('T');
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);    
    Serial.println("))");
    
    delay(100);    
}




/* ----------------------------------------------------------------------------------------------------------------------------------------------------
** What's the time
*/

unsigned long the_time_now() {
  return RTC.now().unixtime();
}

/* ----------------------------------------------------------------------------------------------------------------------------------------------------
** How much memory is left?
*/

void rams_left() {
    Serial.print("(node ");
    Serial.print(NODE_NAME);
    Serial.print(" ");
    Serial.print(the_time_now());
    Serial.print(" ");
    Serial.print("(ram ");
    Serial.print(free_ram());
    Serial.println("))");
 
}

int free_ram() {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}

/* ----------------------------------------------------------------------------------------------------------------------------------------------------
** Flash routine
*/

void flash() {
   if (flash_count > FLASH_THROTTLE) {
       digitalWrite(PIN_LED_STATUS, HIGH);
       delay(5);
       digitalWrite(PIN_LED_STATUS, LOW);
       flash_count = 0;
   } else {
       flash_count ++;
   }     
}

void blink() {  
    digitalWrite(PIN_LED_STATUS, HIGH);
    delay(5);
    digitalWrite(PIN_LED_STATUS, LOW);
}



void status_on(void) {
   digitalWrite(PIN_LED_STATUS, HIGH);
}

void status_off(void) {
   digitalWrite(PIN_LED_STATUS, LOW);
}


void serial_handler(void) {

    // Serial.println("In the serial handler");
    static char buffer[INPUT_BUFFER];
    static byte length = 0;

    int count = Serial.available();
    if (count == 0) {

    } else {
        //Serial.println("Parsing out the serials");
        for (byte index = 0; index < count; index++) {
            char ch = Serial.read();
            if (length >= (sizeof (buffer) / sizeof (*buffer))) {
                length = 0;
            } else if (ch == '\n' || ch == ';') {
                buffer[length] = '\0';
                
//                Serial.print("buffer is: ");
//                Serial.println(buffer);                
                parse_command(buffer);
                length = 0;
            } else {
                buffer[length++] = ch;
            }
        }
    }
    
}

void parse_command(char* buffer) {

 
    seg_command = get_command_element(buffer, 0);

//    Serial.print("seg_command: ");
//    Serial.println(seg_command);
    

    if (seg_command == NODE_NAME) {
      is_for_me = true;
    } else if (seg_command == "all_nodes") {
      is_for_me = true;
    } else if (seg_command == "all") {
      is_for_me = true;
    } else {
      is_for_me = false;
    }

    if (is_for_me == true) {
        // Work out out the SEG commands
        seg_command = get_command_element(buffer, 1);
        
//        Serial.print("seg_command: ");
//        Serial.println(seg_command);
        

        if (seg_command == "start_data")  {
            start_data();
        } else if (seg_command == "start")  {
            start_data();
        } else if (seg_command == "get_data")  {
            start_data();
        } else if (seg_command == "stop_data")  {
            stop_data();
        } else if (seg_command == "stop")  {
            stop_data();
        } else if (seg_command == "reset_data")  {
            reset_data();
        } else if (seg_command == "rams")  {
            get_ram();
        } else {
          send_message("unknown command");
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

void start_data() {
  //Serial.println("commanded to start data output");
  if (has_data == true) {
    data_count = DATA_THROTTLE + 1;
    send_data = true;
    send_message("data_requested");
  } else {
    send_message("no_data");
  }
}

/* ----------------------------------------------------------------------------------------------------------------------------------------------------*/


void stop_data() {
    send_message("data_stopping");
    send_data = false;
}

/* ----------------------------------------------------------------------------------------------------------------------------------------------------*/

void reset_data() {
    initialise_data_arrays();
    send_data = false;
    status_on();
    delay(8000);
    status_off();
    send_message("data_reset");
}


/* ----------------------------------------------------------------------------------------------------------------------------------------------------*/

void get_ram() {
    rams_left();
}


