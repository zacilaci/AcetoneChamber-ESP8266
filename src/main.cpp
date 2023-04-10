#define USE_LITTLEFS

#include <Arduino.h>

// "Factory" libraries
//https://hieromon.github.io/AutoConnect/gettingstarted.html
#include <ESP8266WiFi.h>          // Replace with WiFi.h for ESP32
#include <ESP8266WebServer.h>     // Replace with WebServer.h for ESP32
#include <AutoConnect.h>
#include <LittleFS.h>

#include <EEPROM.h>

#include <OneWire.h>
#include <DallasTemperature.h>
#include <QuickPID.h>

// User written libraries
#include "motor.h"
#include "heatpad.h"


// Defines the custom data should be stored in EEPROM.
typedef struct {
  char  fan_speed[8];
  char  temperature[8];
  char  time_on[8];
  char  time_off[8];
  char  cycles[8];
  char  fan_off[8]; // just a bool value, if true, then the fan will be turned off during non-heating blocks
} EEPROM_CONFIG_t;


/*#############################################################################################################################*/
// Compiler settings
#define DEBUG_MODE false // prints messages through Serial port if set to true

// defines
#define PIN_MOTOR 13 // D7, GPIO13
#define PIN_ONEWIRE_TEMP 4 // D2, GPIO4
#define PIN_HEATING 15 // D8, GPIO15

#define FREQUENCY_MOTOR 1000 // 1kHz, the PWM frequency of the DC motor, Valid values are from 100Hz to 40000Hz.
#define BAUDRATE 115200 // for debug mode

#define LOOP_PERIOD_LOGIC 60000 // standard period of 1 minute

#define PERIOD_PID 50 // ms, the Time period of the PID loop.

/*#############################################################################################################################*/
// PID Tuning parameters (Not tuned yet!)
#define KP 50
#define KI 50
#define KD 10

/*#############################################################################################################################*/
// Function Definitions
void tempControlLoop(void); // Control loop for the temperature control (PID loop)

// webPages
void onStart(AutoConnectButtonJson& me, AutoConnectAux& page); // The function to perform on the click of the Start button
void onSave(AutoConnectButtonJson& me, AutoConnectAux& page); // The function to perform on the click of the Save button

// Logic behind it all
void logic(void); // The whole logic behind the Acetone chamber heating and ventilation in a given time period.

/*#############################################################################################################################*/
// Variable Definitions
float pid_set_point; // Desired value of PID
float pid_input; // Current value of PID
float pid_output; // Response of the PID system

unsigned long long current_loop_pid, previous_loop_pid=0; // timer variables, for the PID loop

unsigned long long current_cycle_loop, previous_cycle_loop=0; // timer variables for the logic loop

uint8_t fan_speed, temperature, time_on, time_off, cycles, current_cycle = 0, no_of_heating_periods = 0, no_of_off_periods = 0;// values for the state variables 
bool run_current_cycle_loop = false; // Helper for the logic loop, if true then the system is active
bool heating_period = false; // heating period variable
bool turn_fan_off = false; // variable to store if the fan should be turned off during non-heating periods

int totalPeriods, currentPeriod; // in minutes, variables to help count the total and current period 

/*#############################################################################################################################*/
// Class Definitions
OneWire oneWireTemp; // One Wire

Motor motor; // DC Motor

DallasTemperature sens_temp; // Dallas BMP180 One Wire Temperature

HeatPad heatpad; // Heatpad

QuickPID pid; // PID Control loop

ESP8266WebServer Server(80); // Web server
AutoConnect Portal(Server); // Web Portal
AutoConnectConfig Config; // Web Configuration

/*#############################################################################################################################*/
// Main Setup
void setup() 
{

  // Turn the bulitin led On.
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  #if DEBUG_MODE
    Serial.begin(BAUDRATE);// Initialize Serial port
    Serial.setDebugOutput(true); // Turn on debugging
    Serial.println("\n"); // go to next-next line just for clarity.
  #endif

  oneWireTemp = OneWire(PIN_ONEWIRE_TEMP); // Initialize the one wire bus for the temperature sensor

  motor = Motor(PIN_MOTOR, FREQUENCY_MOTOR); // Initialize the DC motor which will be driven by PWM

  sens_temp = DallasTemperature(&oneWireTemp); // Initialize the one wire temperature sensor

  sens_temp.begin(); // Start the temp sensor

  heatpad = HeatPad(PIN_HEATING); // Initialize the heatpad

  pid = QuickPID(&pid_input, &pid_output, &pid_set_point); // initialize the PID loop

  pid.SetTunings(KP, KI, KD); // Parametrize the PID loop

  pid_set_point = 0; // For now set the desired value to be 0 (no heating)

  pid.SetMode(QuickPID::Control::automatic); // ?

  if(!LittleFS.begin()){// Initialize the LittleFS, where the webpage elements are stored in a json format.
    #if DEBUG_MODE
      Serial.println("An Error has occurred while mounting LittleFS");
    #endif
    return;
  }

  File mainPage = LittleFS.open("/pages.json", "r"); // load the main page json from LittleFS
  if (Portal.load(mainPage))
  {
    #if DEBUG_MODE
      Serial.println("Successfully loaded pages!");
    #endif
  }
  else
  {
    #if DEBUG_MODE
      Serial.println("Pages load unsuccessfull!!!");  
    #endif
  }

  mainPage.close(); // Close the file
  
  // Server settings
  Config.autoReconnect = true; // On system start
  Config.menuItems = AC_MENUITEM_OPENSSIDS | AC_MENUITEM_RESET | AC_MENUITEM_CONFIGNEW | AC_MENUITEM_DELETESSID | AC_MENUITEM_DISCONNECT; // Which elements to include in webpage header

  EEPROM_CONFIG_t eeprom_data; // Initialize the struct for eeprom data

  Config.boundaryOffset = sizeof(eeprom_data); // Important: USE WITH AUTOCONNECT to offset the memory table by the amount used by autoconnect, otherwise the data stored in EEPROM can get mixed up between user and autoconnect data.

  Portal.config(Config); // Apply the configuration

  AutoConnectAux& page_home = Portal.locate("/"); // where our homepage is located
  
  // elements located on our home page, with these we can use them later (read and write data for it)
  // Element names which are given in quotation ("") marks must be the same names as in data/pages.json
  AutoConnectButtonJson& button_start = page_home["start"].as<AutoConnectButtonJson>();
  AutoConnectButtonJson& button_save = page_home["save"].as<AutoConnectButtonJson>();

  AutoConnectRangeJson& range_fan_speed = page_home["fanSpeed"].as<AutoConnectRangeJson>();
  AutoConnectRangeJson& range_temperature = page_home["temperature"].as<AutoConnectRangeJson>();

  AutoConnectInputJson& input_time_on = page_home["timeOn"].as<AutoConnectInputJson>();
  AutoConnectInputJson& input_time_off = page_home["timeOff"].as<AutoConnectInputJson>();
  AutoConnectInputJson& input_cycles = page_home["cycles"].as<AutoConnectInputJson>();

  AutoConnectCheckboxJson& checkbox_fan_off = page_home["fanOff"].as<AutoConnectCheckboxJson>();

  if (Portal.begin()) // begin hosting the webpage
  {
    #if DEBUG_MODE
      Serial.println("WiFi connected: " + WiFi.localIP().toString());
    #endif
  }

  EEPROM.begin(Portal.getEEPROMUsedSize()); // Initialize the EEPROM
  EEPROM.get<EEPROM_CONFIG_t>(0,eeprom_data); // Load values from EEPROM
  EEPROM.end(); // terminate EEPROM for now

  range_fan_speed.value = atoi(eeprom_data.fan_speed); // give the values to the webserver, which were loaded from the EEPROM
  range_temperature.value = atoi(eeprom_data.temperature);

  #if DEBUG_MODE    
    Serial.print("EEPROM fan_speed:\t");
    Serial.println(atoi(eeprom_data.fan_speed));

    Serial.print("EEPROM temperature:\t");
    Serial.println(atoi(eeprom_data.temperature));

    Serial.print("EEPROM time_on:\t");
    Serial.println(atoi(eeprom_data.time_on));

    Serial.print("EEPROM time_off:\t");
    Serial.println(eeprom_data.time_off);

    Serial.print("EEPROM cycles:\t");
    Serial.println(eeprom_data.cycles);

    Serial.print("EEPROM fan_off:\t");
    Serial.println(eeprom_data.fan_off);
  #endif

  input_time_on.value = eeprom_data.time_on;
  input_time_off.value = eeprom_data.time_off;

  input_cycles.value = eeprom_data.cycles;

  checkbox_fan_off.checked = eeprom_data.fan_off ? 1:0;

  button_start.on(onStart);
  button_save.on(onSave);

  #if DEBUG_MODE
    Serial.println("Starting loop.");
  #endif

  digitalWrite(LED_BUILTIN, HIGH);// Turn off the onboard led.
}

/*#############################################################################################################################*/
// Main loop
void loop() 
{
  Portal.handleClient(); // This is a must for the autoconnect library, so with this the library can handle requests and everything.
  
  // PID loop
  if (sens_temp.requestTemperatures() != DEVICE_DISCONNECTED_C) // if the device is not disconnected, then everything works as it should.
  { 
    tempControlLoop(); // Control the temperature of the environment.
  }
  else
  {
    heatpad.TurnOff(); // in case of no sensor can be found, the heatpad is automatically turned off.
  }

  #if DEBUG_MODE
    if (sens_temp.requestTemperatures() != DEVICE_DISCONNECTED_C)
    { 
      Serial.print("T:\t");
      Serial.print(pid_input);
      Serial.print(" Celsius\tOutput:\t");
      Serial.println(pid_output);
    }
    else
    {
      Serial.println("Device seems to be disconnected!");
    }
  #endif
  


  if (run_current_cycle_loop == true) // to run the logic for the acetone chamber.
  {
    #if DEBUG_MODE
      Serial.println("Logic");
    #endif
    logic();
  }
  

}

/*#############################################################################################################################*/
// PID loop for temperature control of heating pad
void tempControlLoop(void)
{ 
  current_loop_pid = millis();

  if (current_loop_pid - previous_loop_pid >= PERIOD_PID)
  {
    sens_temp.requestTemperatures();
    pid_input = sens_temp.getTempCByIndex(0);

    pid.Compute();
    heatpad.TurnOn(pid_output);   

    previous_loop_pid = millis();
  }

}


/*#############################################################################################################################*/
// Give Basic values
// Start the cycles
void onStart(AutoConnectButton& me, AutoConnectAux& page) // what to do when the start button is clicked on the webpage
{
  AutoConnectRange& range_fan_speed = page["fanSpeed"].as<AutoConnectRangeJson>();
  AutoConnectRange& range_temperature = page["temperature"].as<AutoConnectRangeJson>();

  AutoConnectInput& input_time_on = page["timeOn"].as<AutoConnectInputJson>();
  AutoConnectInput& input_time_off = page["timeOff"].as<AutoConnectInputJson>();
  AutoConnectInput& input_no_of_cycles = page["cycles"].as<AutoConnectInputJson>();

  AutoConnectCheckboxJson& checkbox_fan_off = page["fanOff"].as<AutoConnectCheckboxJson>();

  fan_speed = map(range_fan_speed.value, 0, 100, 0, 255);
  temperature = range_temperature.value;
  time_on = input_time_on.value.toInt();
  time_off = input_time_off.value.toInt();
  cycles = input_no_of_cycles.value.toInt();
  turn_fan_off = checkbox_fan_off.checked;

  motor.setSpeed(fan_speed); // since pwm duty cycle can be set between 0-255.

  pid_set_point = range_temperature.value; // Change the PID desired value, to the amount set on the webpage

  #if DEBUG_MODE
    Serial.print("Fan speed set to:\t");
    Serial.println(map(fan_speed);

    Serial.print("Time On Value:\t");
    Serial.println(input_time_on.value.toInt());
  #endif

  // these are all which are necessary, to be set in order for the main logic loop to run accordingly
  run_current_cycle_loop = true;

  heating_period = true;

  no_of_heating_periods = 0;

  totalPeriods = (cycles*(time_on + time_off)) + 2; // Number of cycles, +2 is needed (did not look into it yet why, but it works like this)

  currentPeriod = 0;

  digitalWrite(LED_BUILTIN, HIGH);

}

void onSave(AutoConnectButton& me, AutoConnectAux& page) // what to do when the save button is clicked on the webpage
{ // Basically this save the values entered on the GUI into EEPROM
  #if DEBUG_MODE
    Serial.println("Entered onSave.");
    Serial.println("Writing to EEPROM.");
  #endif

  EEPROM_CONFIG_t eeprom_data; // allocate eeprom_data

  char _fan[8], _temperature[8]; // helper variables

  // get the GUI objects
  AutoConnectRange& range_fan_speed = page["fanSpeed"].as<AutoConnectRangeJson>();
  AutoConnectRange& range_temperature = page["temperature"].as<AutoConnectRangeJson>();

  AutoConnectCheckboxJson& checkbox_fan_off = page["fanOff"].as<AutoConnectCheckboxJson>();

  itoa(range_fan_speed.value, _fan, 10); // converts the integer values to a char array
  itoa(range_temperature.value, _temperature, 10);

  // copies the values of the char arrays into the eeprom_data, for EEPROM storage
  strncpy(eeprom_data.fan_speed, _fan, sizeof(EEPROM_CONFIG_t::fan_speed)); 
  strncpy(eeprom_data.temperature, _temperature, sizeof(EEPROM_CONFIG_t::temperature));
  strncpy(eeprom_data.time_on, page["timeOn"].value.c_str(), sizeof(EEPROM_CONFIG_t::time_on));
  strncpy(eeprom_data.time_off, page["timeOff"].value.c_str(), sizeof(EEPROM_CONFIG_t::time_off));
  strncpy(eeprom_data.cycles, page["cycles"].value.c_str(), sizeof(EEPROM_CONFIG_t::cycles));
  String isChecked = checkbox_fan_off.checked ? "1":"0";
  strncpy(eeprom_data.fan_off, isChecked.c_str(), sizeof(EEPROM_CONFIG_t::fan_off));
  
  // Initialize, write, and commit changes to EEPROM.
  EEPROM.begin(Portal.getEEPROMUsedSize());
  EEPROM.put<EEPROM_CONFIG_t>(0,eeprom_data);
  EEPROM.commit();
  EEPROM.end();

  #if DEBUG_MODE
    Serial.println("Wrote fan speed to EEPROM.");
    Serial.println("Wrote temperature to EEPROM.");
    Serial.println("Wrote time on to EEPROM.");
    Serial.println("Wrote time off to EEPROM.");
    Serial.println("Wrote cycles on to EEPROM.");
    Serial.println("Wrote fan_off on to EEPROM.");

    Serial.print("web fan_speed:\t");
    Serial.println(_fan);

    Serial.print("web temperature:\t");
    Serial.println(_temperature);

    Serial.print("web time_on:\t");
    Serial.println(page["timeOn"].value.c_str());

    Serial.print("web time_off:\t");
    Serial.println(page["timeOff"].value.c_str());

    Serial.print("web cycles:\t");
    Serial.println(page["cycles"].value.c_str());

    Serial.print("web fan_off:\t");
    Serial.println(isChecked.c_str());

    Serial.println("committed changes to EEPROM.");
  #endif
}

void logic(void) // main logic for the acetone chamber
{
  /*
    On the GUI you can give the temperature to set, the fan speed to use, on-off cycle durations and the number of times to run these cycles.
    The handling of timing, and turning everything on and off is handled here accordingly.
  */
  current_cycle_loop = millis();

  #if DEBUG_MODE
    Serial.print("current_cycle_loop:\t");
    Serial.println(current_cycle_loop);

    Serial.print("previous_cycle_loop:\t");
    Serial.println(previous_cycle_loop);
  #endif

  if (current_cycle_loop - previous_cycle_loop >= LOOP_PERIOD_LOGIC) // Main logic periodic loop
  {
    #if DEBUG_MODE
      Serial.println("Entered logic loop.");
    #endif

    if(heating_period) // when the heating is applied, and the motor is running
    {
      #if DEBUG_MODE
        Serial.println("In heating period.");
      #endif

      if (no_of_heating_periods >= time_on) // if we have reached the desired heating minutes, we go into the non-heating period
      {
        pid_set_point = 0;

        if (turn_fan_off == true)
        {
          motor.stopMotor();   
        }
        no_of_heating_periods = 0;
        heating_period = false;
        #if DEBUG_MODE
          Serial.print("turn_fan_off:\t");
          Serial.println(turn_fan_off);
          Serial.println("Heating off.");
        #endif
      }
      else // since the main loop is executed every minute, but we might not want to only heat our  chamber for 1 minute only
      {
        no_of_heating_periods += 1;
        #if DEBUG_MODE
          Serial.print("no_of_heating_periods:\t");
          Serial.println(no_of_heating_periods);
        #endif
      }
    }
    else// non-heating period
    {
      #if DEBUG_MODE
        Serial.println("In off period.");
      #endif
      if (no_of_off_periods >= time_off-1)// end of non-heating period, go into heating period
      {
        
        pid_set_point = temperature;

        motor.setSpeed(fan_speed);

        no_of_off_periods = 0;
        heating_period = true;

        #if DEBUG_MODE
          Serial.println("Heating on.");
        #endif
      }
      else
      {
        no_of_off_periods += 1;
        #if DEBUG_MODE
          Serial.print("no_of_off_periods:\t");
          Serial.println(no_of_off_periods);
        #endif
      }
    }

    currentPeriod += 1; // counter to help, how many minutes have passed.

    #if DEBUG_MODE
      Serial.print("currentPeriod:\t");
      Serial.println(currentPeriod);
    #endif

    if(currentPeriod > totalPeriods) // if we have reached the end of our periods then turn everything off, and turn the builtin led on.
    {
      run_current_cycle_loop = false;

      motor.stopMotor();
      pid_set_point = 0;

      digitalWrite(LED_BUILTIN, LOW);

      #if DEBUG_MODE
        Serial.println("Finished Everything.");
      #endif
    }

    previous_cycle_loop = millis();
  }

}