/* 
 * ~~~~~~~~~~~~~
 * Please do not remove the following notices.
 * Copyright (c) 2011 by Smart Energy Groups Pty. Ltd.
 * 
 * This a little Arduino program to watch the electrons being used by and electric bike!
 * It measures the bulk DC current going to the controller, and the voltage.
 * From these, it does some calculating and outputs instantaneous:
 * - voltage
 * - power
 * - amps
 * 
 * and accumulated:
 * - watt hours
 * - amp hours
 *
 * and the run time since the Arduino was booted up.
 *
 * All this is then displayed on a two line, 16 character LCD
 *
 * It may well also do some other stuff!
 *
 * Under the hood it also uses some components from @geekscape 's Aiko Event driven libraries.
 */

#include <PString.h>
#include <AikoEvents.h>
#include <AikoSExpression.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <LiquidCrystal.h>
using namespace Aiko;

LiquidCrystal lcd(2, 3, 4, 5, 6, 7);

#define DEFAULT_NODE_NAME "bike"

//#define HAS_BUTTON  // If the device has a properly built momentary button on PIN_BUTTON normally 8 this could do some magics.

#define SAMPLES         800   // Current samples to take

#define DEFAULT_BAUD_RATE     38400
#define ONE_SHOT_TIME         180000
#define QUICK_FLASH_TIME      20

int display_toggle = 0;
float voltage_value = 0;
float charge_percentage = 111.0;

float current_value = 0;
float power_value = 0;

float energy_value = 0.0;
float ah_value = 0.0;
unsigned long ah_last_time = 0;
unsigned long wh_last_time = 0;

byte second = 0;
byte minute = 0;
byte hour = 0;

// Digital Input/Output pins
#define PIN_SERIAL_RX       0
#define PIN_SERIAL_TX       1
#define PIN_ONE_WIRE        5 // OneWire or CANBus
#define PIN_RELAY           7
#define PIN_BUTTON          8

#define PIN_LED_STATUS     13 // Standard Arduino flashing LED !

// Analogue Input pins not used for 6 channel sensing
#define PIN_VOLTAGE_SENSOR  1
#define PIN_DC_CURRENT_SENSOR  0

#define SEG_STRING_SIZE 15
#define BUFFER_SIZE 450

char globalBuffer[BUFFER_SIZE]; // Used to manage dynamically constructed strings
PString globalString(globalBuffer, sizeof (globalBuffer));
PString lcd_string(globalBuffer, sizeof (globalBuffer));

#define RELAY_COMMAND "relay"

SExpressionArray commandArray;
SExpressionArray seg_commands;
SExpression parameter;

OneWire oneWire(PIN_ONE_WIRE); // Dallas DS18B20 temperature sensor
// Pass our oneWire reference to Dallas Temperature library 
DallasTemperature sensors(&oneWire);


void setup() {
  
  Serial.begin(DEFAULT_BAUD_RATE);
  
  Events.addHandler(clockHandler,  1000);
  Events.addHandler(calculate_stuff,  300);
  Events.addHandler(display_stuff,  700);

  lcd.begin(16, 2);
  lcd.print("bike STARTUP!");
  delay(2000);
}

void loop() {
  Events.loop();    
}

void display_stuff() {
  
  if (display_toggle > 10 && display_toggle < 17) {
    lcd_energy_display();
  } else {
    lcd_power_display();
  }
  
  display_toggle ++;
  
  if (display_toggle >= 17) {
    display_toggle = 0;
  }
}

void lcd_power_display() {
   
  lcd.clear();
  
  lcd_string.begin();
  
  lcd_string = voltage_value;
  lcd_string += "v";  
  
  lcd.setCursor(0, 0);
  lcd.print(lcd_string); 
  
  
  lcd_string.begin();
  lcd_string += (int) hour;
  lcd_string += ":";
  lcd_string += (int) minute;
  lcd_string += ":";
  lcd_string += (int) second;
  
  lcd.setCursor(9, 0);
  lcd.print(lcd_string); 
  
 
  lcd_string.begin();  
  
  lcd_string = power_value;
  lcd_string += "w ";  
  
  lcd.setCursor(0, 1);
  lcd.print(lcd_string); 
  
  lcd_string.begin();  
  lcd_string = current_value;
  lcd_string += "a";    
  
  lcd.setCursor(9, 1);
  lcd.print(lcd_string);

}


void lcd_energy_display() {
  
  lcd.clear();
  
    
  lcd_string.begin();
  
  lcd_string = charge_percentage;
  lcd_string += "%";  
  
  lcd.setCursor(0, 0);
  lcd.print(lcd_string); 
  
 
  lcd_string.begin();
  lcd_string += (int) hour;
  lcd_string += ":";
  lcd_string += (int) minute;
  lcd_string += ":";
  lcd_string += (int) second;
  
  lcd.setCursor(9, 0);
  lcd.print(lcd_string); 
  
  lcd_string.begin();  
  
  lcd_string = energy_value;
  lcd_string += "wh ";  
  
  lcd.setCursor(0, 1);
  lcd.print(lcd_string);   
  
  lcd_string.begin();  
  lcd_string = ah_value;
  lcd_string += "ah";    
  
  lcd.setCursor(9, 1);
  lcd.print(lcd_string);  
}

void calculate_stuff() {
    sense_voltage();
    determine_charge_percentage();
    sense_current();
    calculate_power();
    accumulate_watt_hours();    
    accumulate_amp_hours();  
    ah_last_time = millis();
    wh_last_time = millis();
    blinkHandler(); 
}

void determine_charge_percentage() {
  
  if (voltage_value > 52.5) {
    charge_percentage = 111.0;
    
  } else if (voltage_value > 51.2 && voltage_value <= 52.5 ) {
    charge_percentage = 109.0;    
    
  } else if (voltage_value > 50.88 && voltage_value <= 51.2) {
    charge_percentage = 106.0;    
    
  } else if (voltage_value > 44.5 && voltage_value <= 50.88) {
    /* 
     * should be in the range of 100% to 10%
     * Equation, where y = percentage and x = voltage
     * y = 14.107x - 617.67
     */  
     charge_percentage = (14.107 * voltage_value) - 617.76;
          
  } else if (voltage_value > 40 && voltage_value <= 44.5) {
    /* 
     * should be in the range of 10% to 0%
     * Equation, where y = percentage and x = voltage
     * y = 2.247x - 89.98
     */
     charge_percentage = (2.247 * voltage_value) - 89.89;
     
  } else {
    // Depleted
    charge_percentage = -999.0;        
  }   
}

void accumulate_watt_hours() {
  
  // Calculate the elapsed time
  
  unsigned long wh_duration = millis() - wh_last_time;
  
  /*
  Serial.print("Teh energies duration: ");
  Serial.print(wh_duration);
  Serial.print(" power_value: ");
  Serial.print(power_value);
  Serial.print(" before energy: ");
  Serial.print(energy_value);
  */
  
  energy_value += power_value * (( wh_duration / 1000.0) / 3600.0);
  
  //Serial.print(" after energy: ");
  //Serial.println(energy_value);
  
}

void accumulate_amp_hours() {
  
  // Calculate the elapsed time
  
  unsigned long ah_duration = millis() - ah_last_time;
  
  /*
  Serial.print("Teh electrons duration: ");
  Serial.print(ah_duration);
  Serial.print(" current_value: ");
  Serial.print(current_value);
  Serial.print(" ah: ");
  Serial.print(ah_value);
  */
  
  ah_value += current_value * (( ah_duration / 1000.0) / 3600.0);
  
  //Serial.print(" after ah: ");
  //Serial.println(ah_value);
  
}

void calculate_power() {
   power_value = current_value * voltage_value;
}

void sense_current(void) {
  current_value = 0.0;
  long sample = analogRead(PIN_DC_CURRENT_SENSOR);
  float delta = 0;
  
  for (long index = 0; index < SAMPLES; index++) {
     delta += sq(507.0 - (float) sample);
  }
  
  delta = sqrt(delta/SAMPLES);
  
  delta = delta - 1.5;
   
  //Serial.println(delta);
  
  if (delta < 0.5) {
    delta = 0.0;
  }
 current_value = delta * 0.5;
 //Serial.print("bike_current_handler: ");
 //Serial.println(current_value);
}

/* --------------------------------------------------------------------------
 ** Voltage Sensor
 */

void sense_voltage(void) {
    int sample = analogRead(PIN_VOLTAGE_SENSOR);
    
    //Serial.println(sample);
    
    voltage_value = ( (float) sample / 1024) * 56.374;
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
                Serial.println(buffer);
                
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
    if (commandArray[0].isEqualTo(nodeName)) {
        // Serial.println("...we have a node winner!");

        // Make the SEG commands
        seg_commands.parse(commandArray[1]);
        String choppd = chop_string(seg_commands.head(), seg_commands.size());
        if (choppd.equals("relay") == 0)  {
            //Serial.println("...we going to command the node now.");
            relayCommand();
        } else {
            //Serial.println("...not commanding");
        }

    } else {
        // Serial.println("...not for this node");
    }
}

String chop_string(char* input, int chop_at) {

    /*
    Serial.print("Chop String input: ");
    Serial.println(input);
    Serial.print("Chop at: ");
    Serial.println(chop_at);
    */
    
    String output;
    
    for (int index = 0; index < chop_at; index++) {
        //Serial.println(input[index]);
        if (input[index] != '(') {
            output.concat(input[index]);
        }
        if ((input[index] == ' ') && (index > 0)) {
            break;
        }
        if (input[index] == ')') {
            break;
        }
    }
    /*
    Serial.print("Chop String output: ");
    Serial.println(output);
    */
    return output;
   
}

/* --------------------------------------------------------------------------
 ** Relay handler
 */
byte oneShotInitialised = false;
byte relayInitialized = false;
byte relayOn = false;
unsigned long relayTurnedOn = 0;
unsigned long relayTimeNow = 0;

String energise;
unsigned long  energise_time;
String seg_command_id;
String command_response;

void relayInitialize(void) {
    relayInitialized = true;
}

void relayCommand(void) {

    command_response = "error";

    if (relayInitialized == false) relayInitialize();

    setRelayParameters();

    //Serial.println(energise);
    //Serial.println(energise_time);
    //Serial.println(seg_command_id);
    //Serial.println(command_response);

    if (energise.equals("on")) {
        turnRelayOn(true);
        relayTurnedOn = millis();
        command_response = "complete";
    } else if (energise.equals("off")) {
        turnRelayOff(true);
        relayTurnedOn = millis();
        command_response = "complete";
    } else {
        //  Do nothing, sending stuff here messes the mesh.
        //  Rebroadcast the command and check at the start wether it's for this node.
        //  sendMessage("(error parameterInvalid)");
    }
   completeRelayCommand();
}

void setRelayParameters() {

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

void completeRelayCommand() {

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

void oneShotInitialise(void) {
    oneShotInitialised = true;
}

void oneShotHandler(void) {
    if (oneShotInitialised == false) oneShotInitialise();

    if (energise_time > 0) {
      
      relayTimeNow = millis();
      if (relayOn == true) {
         
          if (relayTimeNow > relayTurnedOn) {
                
              if ((relayTimeNow - relayTurnedOn) > energise_time) {
                  // No need to tell SEG that shot has expired.  Otherwise the device will de-energise...
                  turnRelayOff(false);
              }
          } else {
              // No need to tell SEG that shot has expired.  Otherwise the device will de-energise...
              // We have a rollover, so turn eet off
              turnRelayOff(false);
          }
      }
    }
}

void turnRelayOff(byte tell_seg) {
    digitalWrite(PIN_RELAY, LOW);
    relayOn = false;
    if (tell_seg) {
      relayMessager("relay off"); 
    }
    
}

void turnRelayOn(byte tell_seg) {
    digitalWrite(PIN_RELAY, HIGH);
    relayOn = true;
    if (tell_seg) {
      relayMessager("relay on");
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
  //Serial.println("in buttonHandler...");
  
   // read the state of the switch into a local variable:
  thisButtonState = digitalRead(PIN_BUTTON);
  //Serial.print("This button state: ");
  //Serial.println(thisButtonState);

  // check to see if you just pressed the button 

  // If the switch changed, due to noise or pressing:
  if (thisButtonState != lastButtonState) {
    // reset the debouncing timer, and the changeable state
    lastDebounceTime = millis();
    changedState = false;  
    //Serial.println("Reset debounce timer");
    
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
  //Serial.println("Changing the relay state");
  
  
  if (relayOn == true) {
    turnRelayOff(true);
  } else {
    turnRelayOn(true);
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
        relayMessager("(relay 1 number)");
    } else {

        relayMessager("(relay 0 number)");
    }
}

void relayMessager(char* message) {

    globalString.begin();
    globalString += "(";
    globalString += message;
    globalString += ")";

    sendMessage(globalString);
}


/* --------------------------------------------------------------------------
 ** The Clock
 */

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

