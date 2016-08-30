///
/// @mainpage	Trail-Counter-Simblee
///
/// @details	This is the Simblee side of the trail counter
/// @n
/// @n
/// @n @a		Developed with [embedXcode+](http://embedXcode.weebly.com)
///
/// @author		Charles McClelland
/// @author		Charles McClelland
/// @date		5/10/16 9:13 AM
/// @version	<#version#>
///
/// @copyright	(c) Charles McClelland, 2016
/// @copyright	GNU General Public Licence
///
/// @see		ReadMe.txt for references
///


///
/// @file		Trail_Counter_Simblee.ino
/// @brief		Main sketch
///
/// @details	<#details#>
/// @n @a		Developed with [embedXcode+](http://embedXcode.weebly.com)
///
/// @author		Charles McClelland
/// @author		Charles McClelland
/// @date		5/10/16 9:13 AM
/// @version	<#version#>
///
/// @copyright	(c) Charles McClelland, 2016
/// @copyright	GNU General Public Licence
///
/// @see		ReadMe.txt for references
/// @n
///


// Core library for code-sense - IDE-based
#if defined(WIRING) // Wiring specific
    #include "Wiring.h"
#elif defined(MAPLE_IDE) // Maple specific
    #include "WProgram.h"
#elif defined(ROBOTIS) // Robotis specific
    #include "libpandora_types.h"
    #include "pandora.h"
#elif defined(MPIDE) // chipKIT specific
    #include "WProgram.h"
#elif defined(DIGISPARK) // Digispark specific
    #include "Arduino.h"
#elif defined(ENERGIA) // LaunchPad specific
    #include "Energia.h"
#elif defined(LITTLEROBOTFRIENDS) // LittleRobotFriends specific
    #include "LRF.h"
#elif defined(MICRODUINO) // Microduino specific
    #include "Arduino.h"
#elif defined(TEENSYDUINO) // Teensy specific
    #include "Arduino.h"
#elif defined(REDBEARLAB) // RedBearLab specific
    #include "Arduino.h"
#elif defined(RFDUINO) // RFduino specific
    #include "Arduino.h"
#elif defined(SPARK) || defined(PARTICLE) // Particle / Spark specific
    #include "application.h"
#elif defined(ESP8266) // ESP8266 specific
    #include "Arduino.h"
#elif defined(ARDUINO) // Arduino 1.0 and 1.5 specific
    #include "Arduino.h"
#else // error
    #   error Platform not defined
#endif // end IDE


// Set parameters
//These defines let me change the memory map without hunting through the whole program
#define VERSIONNUMBER 7       // Increment this number each time the memory map is changed
#define WORDSIZE 8            // For the Word size
#define PAGESIZE 4096         // Memory size in bytes / word size - 256kb FRAM
// First Word - 8 bytes for setting global values
#define DAILYOFFSET 2        // First word of daily counts
#define HOURLYOFFSET 30        // First word of hourly counts (remember we start counts at 1)
#define DAILYCOUNTNUMBER 28    // used in modulo calculations - sets the # of days stored
#define HOURLYCOUNTNUMBER 4064 // used in modulo calculations - sets the # of hours stored - 256k (4096-14-2)
#define VERSIONADDR 0x0       // Memory Locations By Name not Number
#define SENSITIVITYADDR 0x1   // For the 1st Word locations
#define DEBOUNCEADDR 0x2        // Two bytes for debounce
#define DAILYPOINTERADDR 0x4    // One byte for daily pointer
#define HOURLYPOINTERADDR 0x5   // Two bytes for hourly pointer
#define CONTROLREGISTER 0x7     // This is the control register acted on by both Simblee and Arduino
//Second Word - 8 bytes for storing current counts
#define CURRENTHOURLYCOUNTADDR 0x8
#define CURRENTDAILYCOUNTADDR 0xA
#define CURRENTCOUNTSTIME 0xC
//These are the hourly and daily offsets that make up the respective words
#define DAILYDATEOFFSET 1         //Offsets for the value in the daily words
#define DAILYCOUNTOFFSET 2        // Count is a 16-bt value
#define DAILYBATTOFFSET 4
#define HOURLYCOUNTOFFSET 4         // Offsets for the values in the hourly words
#define HOURLYBATTOFFSET 6


// Include application, user and local libraries
#include <Arduino.h>
#include <Wire.h>               //http://arduino.cc/en/Reference/Wire (included with Arduino IDE)
#include "DS3232RTC.h"          //http://github.com/JChristensen/DS3232RTC
#include "Time.h"               //http://www.arduino.cc/playground/Code/Time
#include "MAX17043.h"           // Drives the LiPo Fuel Gauge
#include "Adafruit_FRAM_I2C.h"   // Note - had to comment out the Wire.begin() in this library
#include "SimbleeForMobile.h"
#include <stdlib.h>
#include <stdio.h>
#include "FRAMcommon.h"     // Where I put all the common FRAM read and write extensions
#include "SimbleeForMobileClient.h"
#include "SimbleeCloud.h"


// Prototypes
// Prototypes From the included libraries
MAX17043 batteryMonitor;                      // Init the Fuel Gauge
SimbleeForMobileClient client;
SimbleeCloud cloud(&client);



// Prototypes for General Functions
void BlinkForever(); // Ends execution if there is an error
int sprintf ( char * str, const char * format, ... );



// Prototypes for Date and Time Functions
time_t findMidnight(time_t unixT); // Need to break at midnight
void toArduinoTime(time_t unixT); // Puts time in format for reporting



// Prototypes for the Simblee
void SimbleeForMobile_onConnect();   // Actions to take once we get connected
void SimbleeForMobile_onDisconnect();    // Can clean up resources once we disconnect
void ui();   // The function that defines the iPhone UI
void ui_event(event_t &event);   // This is where we define the actions to occur on UI events
void createCurrentScreen(); // This is the screen that displays current status information
void updateCurrentScreen(); // Since we have to update this screen three ways: create, menu bar and refresh button
void createDailyScreen(); // This is the screen that displays current status information
void createHourlyScreen(); // This is the screen that displays current status information
void createAdminScreen(); // This is the screen that you use to administer the device
void printEvent(event_t &event);     // Utility method to print information regarding the given event
int AlarmInteruptHandler(uint32_t dummyPin); // This is where we will take action when we detect an alarm
int wakeUpAlarm(uint32_t dummyButton); // Function to let us know we are waking up
volatile boolean alarmInterrupt = false;


// Define variables and constants
// Pin Value Variables
int SCLpin = 13;    // Simblee i2c Clock pin
int SDApin = 14;    // Simblee i2c Data pin
int AlarmPin = 20;  // This is the open-drain line for signaling an Alarm

// Battery monitor
float stateOfCharge = 0;    // Initialize state of charge

// FRAM and Unix time variables
tmElements_t tm;        // Time elements (such as tm.Minues
time_t t;               // UNIX time format (not note 32 bit number unless converted)
int lastHour = 0;  // For recording the startup values
int lastDate = 0;   // For providing dat break counts
unsigned int hourlyPersonCount = 0;  // hourly counter
unsigned int dailyPersonCount = 0;   //  daily counter
// Interface vairables
unsigned int lastupdate = 0;    // For when we are on the current screen
int updateFrequency = 1000;     // How often will we update the current screen
int adminAccessKey = 27617;     // This is the code you need to enter to get to the admin field
int adminAccessInput = 0;       // This is the user's input

boolean adminUnlocked = false;  // Start with the Admin tab locked
const char* releaseNumber = "1.21";

// Variables for Simblee Display
uint8_t ui_adminLockIcon;       // Shows whether the admin tab is unlocked
uint8_t ui_adminAccessField;    // Where the Admin access code is entered
uint8_t ui_StartStopSwitch; // Start stop button ID on Admin Tab
uint8_t ui_StartStopStatus; // Text field ID for start stop status on Admin Tab
uint8_t ui_EraseMemSwitch; // Erase Memory button ID on Admin Tab
uint8_t ui_EraseMemStatus; // Text field ID for Erase Memory status on Admin Tab
uint8_t ui_DebounceSlider;  // Slider ID for adjusting debounce on Admin Tab
uint8_t ui_SensitivitySlider;   // Slider ID for adjusting sensitivity on Admin Tab
uint8_t ui_LEDswitch;   // Used to turn on and off the LEDs
uint8_t ui_UpdateButton;  // Update button ID on Admin Tab
uint8_t dateTimeField;  // Text field on Current Tab
uint8_t hourlyField;    // Hour field ID for Houly Tab
uint8_t dailyField;     // Date feild ID for Hourly Tab
uint8_t chargeField;    // State of charge field ID on Current Tab
uint8_t menuBar;        // ID for the tabbed Menu Bar
const char *titles[] = { "Current", "Daily", "Hourly", "Admin" };
const char *fields[] = { "Hr","Min","Sec","Yr","Mon","Sec"};
int currentScreen; // The ID of the current screen being displayed
uint8_t ui_setYear, ui_setMonth,ui_setDay,ui_setHour,ui_setMinute,ui_setSecond; // Element which displays date and time values on Admin Tab
uint8_t ui_hourStepper, ui_minStepper, ui_secStepper, ui_yearStepper, ui_monthStepper, ui_dayStepper;   // Stepper IDs for adjusting on Admin Tab

// Variables for Simblee Cloud
unsigned int userID = 0xe983942d; //Enter your assigned userID here
unsigned int destESN = 0x00000001; // This is the destination from the Simblee Cloud Admin Site
uint8_t ui_sendCloudSwitch;

// Accelerometer Values
int debounce;               // This is a minimum debounce value - additional debounce set using pot or remote terminal
int accelInputValue;            // Raw sensitivity input (0-9);
byte accelSensitivity;               // Hex variable for sensitivity

// Variables for the control byte
// Control Register  (8 - 6 Reserved, 5-Clear Counts, 4-Toggle LEDs, 3-Start / Stop Test, 2-Set Sensitivity, 1-Set Delay)
byte signalDebounceChange = B0000001;  // Mask for accessing the debounce bit
byte signalSentitivityChange = B00000010;   // Mask for accessing the sensitivity bit
byte toggleStartStop = B00000100;   // Mask for accessing the start / stop bit
byte toggleLEDs = B00001000;        // Mask for accessing the LED on off bit
byte signalClearCounts = B00010000; // Flag to have the Arduino clear current counts
byte controlRegisterValue;  // Current value of the control register

// include newlib printf float support (%f used in sprintf below)
asm(".global _printf_float");

// Add setup code
void setup()
{
    Wire.beginOnPins(SCLpin,SDApin);  // Not sure if this talks to the i2c bus or not - just to be safe...
    Serial.begin(9600);
    printf("Module ESN is 0x%08x\n", cloud.myESN);
    cloud.userID = userID;
    Serial.println("Startup delay...");
    delay(500); // This is to make sure that the Arduino boots first as it initializes the various devices
    
    // Unlike Arduino Simblee does not pre-define inputs
    pinMode(TalkPin, INPUT);  // Shared Talk line
    pinMode(The32kPin, INPUT);   // Shared 32kHz line from clock
    pinMode(AlarmPin,INPUT);    // Shared DS3231 Alarm Pin
    attachPinInterrupt(20,AlarmInteruptHandler,LOW);    // this will trigger an alarm that will put Simblee in low power mode until morning
    
    TakeTheBus(); // Need the bus as we access the FRAM
        if (fram.begin()) {  // you can stick the new i2c addr in here, e.g. begin(0x51);
            Serial.println("Found I2C FRAM");
        }
        else {
            Serial.println("No I2C FRAM found ... check your connections\r\n");
            BlinkForever();
        }
    GiveUpTheBus(); // OK, from now on we will use the FRAM functions which have buss management built in

    // Check to see if the memory map in the sketch matches the data on the chip
    if (FRAMread8(VERSIONADDR) != VERSIONNUMBER) {
        Serial.print(F("FRAM Version Number: "));
        Serial.println(FRAMread8(VERSIONADDR));
        Serial.read();
        Serial.println(F("Memory/Sketch mismatch! Erase FRAM? (Y/N)"));

        while (!Serial.available());
        switch (Serial.read()) {    // Give option to erase and reset memory
            case 'Y':
                ResetFRAM();
                break;
            case 'y':
                ResetFRAM();
                break;
            default:
                Serial.println(F("Cannot proceed"));
                BlinkForever();
        }
    }
    
    // Set up the Simblee Mobile App
    SimbleeForMobile.deviceName = "Umstead";          // Device name
    SimbleeForMobile.advertisementData = "Dev" ;  // Name of data service
    SimbleeForMobile.begin();
    
    Serial.println("Ready to go....");
}

// Add loop code
void loop()
{
    SimbleeForMobile.process(); // process must be called in the loop for SimbleeForMobile
    cloud.process();            // process must be called in the loop for Simblee Cloud
    if (alarmInterrupt)
    {
        delay(100);         // Keeps us from hammering the i2c bus
        alarmInterrupt = false;
        TakeTheBus();
            t = RTC.get();      // We will check the time to make sure it is the evening alarm
            stateOfCharge = batteryMonitor.getSoC();    // Then we will check the battery
        GiveUpTheBus();
        if (hour(t) == 22) {     // Only going to sleep at night - Requires a 10PM alarm
            Serial.println("Going to Sleep");
            delay(1000);
        
            if (stateOfCharge > 75)
            {
                Simblee_ULPDelay(HOURS(8)); // Sleeps until 6am if the battery is 75% charged
            }
            else
            {
                Simblee_ULPDelay(HOURS(14)); // Sleeps until noon if the battery is less than 75% charged
            }
            // If we want to wake on a pin interrupt - these are the lines of code
            // Simblee_pinWakeCallback(20, LOW, wakeUpAlarm); // configures pin 20 to wake up device on a Low signal
            // Simblee_systemOff();  // Very low power - only comes back with interrupt
        }
    }
    if (SimbleeForMobile.connected && SimbleeForMobile.screen == 1)
    {
        if (millis() >= lastupdate + updateFrequency)
        {
            lastupdate = millis();
            if (SimbleeForMobile.updatable)
            {
                updateCurrentScreen();
            }
        }
    }
}

int wakeUpAlarm(uint32_t dummyButton)  // Function to let us know we are waking up
{
    Simblee_resetPinWake(20); // reset state of pin that caused wakeup
    Serial.println("Waking Up");
}

void SimbleeForMobile_onConnect()   // Actions to take once we get connected.
{
    currentScreen = -1;     // Reset the current screen to non being displayed

}


void SimbleeForMobile_onDisconnect()    // Can clean up resources once we disconnect
{
    adminUnlocked = false;      // Clear the Admin Unlock values on disconnect
    adminAccessInput = 0;
    controlRegisterValue = FRAMread8(CONTROLREGISTER);
    if (toggleLEDs & controlRegisterValue == 8)       // Check to see if the LED bit is set on
    {
        FRAMwrite8(CONTROLREGISTER, toggleLEDs ^ controlRegisterValue); // If so, it will turn off the LED
    }
}

void ui()   // The function that defines the iPhone UI
{
    if(SimbleeForMobile.screen == currentScreen) return;    // If we are on the right screen then we are all set
    currentScreen = SimbleeForMobile.screen;    // If not, let's capture the current screen number
    switch(SimbleeForMobile.screen)
    {
        case 1:
            createCurrentScreen();  // Create the current screen
            updateCurrentScreen();
            break;

        case 2:
            createDailyScreen(); // This screen is created and populated in one step
            break;

        case 3:
            createHourlyScreen(); // This screen is created and populated in one step
            break;
            
        case 4:
            createAdminScreen(); // Create the current screen
            if ((controlRegisterValue & toggleStartStop) >> 2) {    // Populate the fields with values
                SimbleeForMobile.updateText(ui_StartStopStatus, "Running");
            }
            else SimbleeForMobile.updateText(ui_StartStopStatus, "Stopped");
            debounce = FRAMread16(DEBOUNCEADDR);
            SimbleeForMobile.updateValue(ui_DebounceSlider, debounce);
            accelInputValue = FRAMread8(SENSITIVITYADDR);
            SimbleeForMobile.updateValue(ui_SensitivitySlider, accelInputValue);
            break;
            
        default:
            Serial.print("ui: Uknown screen requested: ");  // Don't think we can get here using the menubar but just in case
            Serial.println(SimbleeForMobile.screen);
    }
}

void ui_event(event_t &event)   // This is where we define the actions to occur on UI events
{
    printEvent(event);
    if (event.id == menuBar)                                          // This is the event handler for the menu bar
    {
        switch(event.value)
        {
            case 0: // This is the current tab
                SimbleeForMobile.showScreen(1);
                break;
                
            case 1: // This is the Daily Tab
                SimbleeForMobile.showScreen(2);
                break;
                
            case 2: // This is the Hourly Tab
                SimbleeForMobile.showScreen(3);
                break;
                
            case 3: // This is the Admin Tab
                if (adminUnlocked) SimbleeForMobile.showScreen(4);
                else if (currentScreen == 1) SimbleeForMobile.updateValue(menuBar, 0);
                else SimbleeForMobile.showScreen(1);
                break;
        }
    }
    else if (event.id == ui_adminAccessField)   // Where you enter the Admin code
    {
        adminAccessInput = event.value;
        if (adminAccessInput == adminAccessKey)
        {
            SimbleeForMobile.updateColor(ui_adminLockIcon,GREEN);
            adminUnlocked = true;
        }
        else if (adminAccessInput != adminAccessKey)
        {
            adminAccessInput = 0;
            SimbleeForMobile.updateValue(ui_adminAccessField,0);
            SimbleeForMobile.updateColor(ui_adminLockIcon,RED);
            adminUnlocked = false;
            
        }
    }
    else if (event.id == ui_LEDswitch) // LED on off switch on the Current Status Screen
    {
        controlRegisterValue = FRAMread8(CONTROLREGISTER);
        FRAMwrite8(CONTROLREGISTER, toggleLEDs ^ controlRegisterValue); // Toggle the LED bit
        Serial.println("Toggled the LED bit");
    }
     else if (event.id == ui_sendCloudSwitch)
    {
        if(cloud.connect())
        {
            Serial.println("Simblee Cloud Connected");
        }
        else Serial.println("Simblee Cloud Not Connected");
        if (cloud.active())
        {
            // send start message (ie: start the timer on the receiving side)
            cloud.send(destESN, "S", 1);
            
            for (int i = 1; i < 5; i++)
            {
                // send a '1' message
                cloud.send(destESN, "1", 1);
                // send a '0' message
                cloud.send(destESN, "0", 1);
            }
            
            // send end message (ie: stop the timer on the receiving side)
            cloud.send(destESN, "E", 1);
            Serial.println("Cloud data sent");
        }
        else Serial.println("Simblee Cloud not Active - no data sent");
    }
    else if (event.id == ui_StartStopSwitch && event.type == EVENT_RELEASE) // Start / Stop Button handler from the Admin screen
    {
        controlRegisterValue = FRAMread8(CONTROLREGISTER);
        FRAMwrite8(CONTROLREGISTER,toggleStartStop ^ controlRegisterValue);  // Toggle the start stop bit
        if ((controlRegisterValue & toggleStartStop) >> 2) {
            SimbleeForMobile.updateText(ui_StartStopStatus, "Stopped");
        }
        else SimbleeForMobile.updateText(ui_StartStopStatus, "Running");
    }
    else if (event.id == ui_EraseMemSwitch && event.type == EVENT_RELEASE)  // This button allows us to erase the FRAM from the Admin screen
    {
        SimbleeForMobile.updateText(ui_EraseMemStatus,"Started");
        ResetFRAM();
        SimbleeForMobile.updateText(ui_EraseMemStatus,"Erased");
        controlRegisterValue = FRAMread8(CONTROLREGISTER);
        FRAMwrite8(CONTROLREGISTER,signalClearCounts ^ controlRegisterValue);  // Toggle the start stop bit
        
    }
    else if (event.id == ui_DebounceSlider && event.type == EVENT_RELEASE) // Moving the debounce slider on the Admin Tab
    {
        debounce = event.value;
    }
    else if (event.id == ui_SensitivitySlider && event.type == EVENT_RELEASE)  // Moving the sensitivity  slider on the Admin Tab
    {
        accelInputValue = event.value;
    }
    else if (event.id == ui_UpdateButton && event.type == EVENT_RELEASE)  // A bit more complicated - Update botton event on Admin Tab
    {
        if (FRAMread16(DEBOUNCEADDR) != debounce)    // Check to see if debounce value needs to be updated
        {
            FRAMwrite16(DEBOUNCEADDR, debounce);    // If so, write to FRAM
            controlRegisterValue = signalDebounceChange ^ controlRegisterValue; // Set the signal change bit
            FRAMwrite8(CONTROLREGISTER,controlRegisterValue);    // Then set the flag so Arduino will apply new setting
            Serial.println("Updating debounce");    // Let the console know
        }
        if (FRAMread8(SENSITIVITYADDR) != accelInputValue) // Same as debounce above - but now for sensitivity
        {
            FRAMwrite8(SENSITIVITYADDR, accelInputValue);
            controlRegisterValue = signalSentitivityChange ^ controlRegisterValue; // Then set the flag so Arduino will apply new setting
            FRAMwrite8(CONTROLREGISTER,controlRegisterValue);
            Serial.println("Updating sensitivity");
        }
        if (tm.Year >> 0 && tm.Month >> 0 && tm.Day >> 0) // Will only update the update if these values are all non-zero
        {
            t= makeTime(tm);
            TakeTheBus();  // Clock is an i2c device
                RTC.set(t);             //use the time_t value to ensure correct weekday is set
                setTime(t);
            GiveUpTheBus();
            Serial.println("Updating time");
        }
        controlRegisterValue = FRAMread8(CONTROLREGISTER);  // Read and display the control register after these updates
        Serial.print("Control Register Value =");
        Serial.println(controlRegisterValue);
    }
    else if (event.id == ui_hourStepper)    // Used to increment / decrement values for setting clock on the Admin Screen
    {
        tm.Hour = event.value;
        SimbleeForMobile.updateValue(ui_setHour, tm.Hour);
    }
    else if (event.id == ui_minStepper)
    {
        tm.Minute = event.value;
        SimbleeForMobile.updateValue(ui_setMinute, tm.Minute);
    }
    else if (event.id == ui_secStepper)
    {
        tm.Second = event.value;
        SimbleeForMobile.updateValue(ui_setSecond, tm.Second);
    }
    else if (event.id == ui_yearStepper)
    {
        tm.Year = CalendarYrToTm(event.value);
        SimbleeForMobile.updateValue(ui_setYear, tm.Year+1970);
    }
    else if (event.id == ui_monthStepper)
    {
        tm.Month = event.value;
        SimbleeForMobile.updateValue(ui_setMonth, tm.Month);
    }
    else if (event.id == ui_dayStepper)
    {
        tm.Day = event.value;
        SimbleeForMobile.updateValue(ui_setDay, tm.Day);
    }
    controlRegisterValue = FRAMread8(CONTROLREGISTER);
    Serial.print("ControlRegisterValue = ");
    Serial.println(controlRegisterValue);
}


void createCurrentScreen() // This is the screen that displays current status information
{

    SimbleeForMobile.beginScreen(WHITE, PORTRAIT); // Sets orientation
    menuBar = SimbleeForMobile.drawSegment(20, 70, 280, titles, countof(titles));
    SimbleeForMobile.updateValue(menuBar, 0);

    // all we are doing here is laying out the screen - updates are in a separate function
    dateTimeField = SimbleeForMobile.drawText(40, 140, " ");
    SimbleeForMobile.drawText(40, 160, "Hourly Count:");
    hourlyField = SimbleeForMobile.drawText(200,160," ");
    SimbleeForMobile.drawText(40, 180, "Daily Count:");
    dailyField =   SimbleeForMobile.drawText(200,180," ");
    SimbleeForMobile.drawText(40, 200, "State of Charge:");
    chargeField = SimbleeForMobile.drawText(200,200," ");
    SimbleeForMobile.drawText(40, 220, "Counter Status:");
    ui_StartStopStatus = SimbleeForMobile.drawText(200, 220, " ");
    ui_adminLockIcon = SimbleeForMobile.drawText(40,290,"Admin Code:",RED);
    ui_adminAccessField = SimbleeForMobile.drawTextField(132,285,80,adminAccessInput);
    SimbleeForMobile.drawText(40,356, "Indicator Lights:");
    ui_LEDswitch = SimbleeForMobile.drawSwitch(170,350);
    SimbleeForMobile.setEvents(ui_LEDswitch,EVENT_PRESS);
    ui_sendCloudSwitch = SimbleeForMobile.drawButton(70,400,150,"Send to Cloud");
    SimbleeForMobile.setEvents(ui_sendCloudSwitch,EVENT_PRESS);
    SimbleeForMobile.drawText(10,(SimbleeForMobile.screenHeight-20),"Version:");
    SimbleeForMobile.drawText(80,(SimbleeForMobile.screenHeight-20),releaseNumber);
    SimbleeForMobile.endScreen();
}

void updateCurrentScreen() // Since we have to update this screen three ways: create, menu bar and refresh button
{
    char battBuffer[4];   // Should be enough 3 digits plus % symbol
    
    TakeTheBus();
        t = RTC.get();
        stateOfCharge = batteryMonitor.getSoC();
    GiveUpTheBus();
    
    toArduinoTime(t);  // Update Time Field on screen and in the console
    Serial.println("");
    
    if (stateOfCharge >= 105)   // Update the value and color of the state of charge field
    {
        SimbleeForMobile.drawText(210,200,"Error");
    }
    else if (stateOfCharge > 75) {
        snprintf(battBuffer, 5,"%1.0f%%",stateOfCharge);   // Puts % next to batt from 0-100
        SimbleeForMobile.updateText(chargeField,battBuffer);
        SimbleeForMobile.updateColor(chargeField,GREEN);
    }
    else if (stateOfCharge > 50) {
        snprintf(battBuffer, 5,"%1.0f%%",stateOfCharge);   // Puts % next to batt from 0-100
        SimbleeForMobile.updateText(chargeField,battBuffer);
        SimbleeForMobile.updateColor(chargeField,MAGENTA);
    }
    else {
        snprintf(battBuffer, 5,"%1.0f%%",stateOfCharge);   // Puts % next to batt from 0-100
        SimbleeForMobile.updateText(chargeField,battBuffer);
        SimbleeForMobile.updateColor(chargeField,RED);
    }

    SimbleeForMobile.updateValue(hourlyField, FRAMread16(CURRENTHOURLYCOUNTADDR)); // Populate the hourly and daily fields with values
    SimbleeForMobile.updateValue(dailyField, FRAMread16(CURRENTDAILYCOUNTADDR));
    
    controlRegisterValue = FRAMread8(CONTROLREGISTER);          // Update the Start and Stop fields with the latest status
    if ((controlRegisterValue & toggleStartStop) >> 2) {
        SimbleeForMobile.updateText(ui_StartStopStatus, "Running");
    }
    else SimbleeForMobile.updateText(ui_StartStopStatus, "Stopped");
    
    if (adminUnlocked) {
        SimbleeForMobile.updateValue(ui_adminAccessField,adminAccessInput);
        SimbleeForMobile.updateColor(ui_adminLockIcon,GREEN);
    }
    
    if ((controlRegisterValue & toggleLEDs) >> 3) {    // Show the state of the LED switch
        SimbleeForMobile.updateValue(ui_LEDswitch,1);
    }
    else SimbleeForMobile.updateValue(ui_LEDswitch,0);
}

void createDailyScreen() // This is the screen that displays current status information
{
    int xAxis = 30;
    int yAxis = 110;
    int rowHeight = 15;
    int columnWidth = 5;
    int row = 1;
    int dailyCount = 0;
    char battBuffer[4];   // Should be enough 3 digits plus % symbol

    SimbleeForMobile.beginScreen(WHITE, PORTRAIT); // Sets orientation
    menuBar = SimbleeForMobile.drawSegment(20, 70, 280, titles, countof(titles));
    SimbleeForMobile.updateValue(menuBar, 1);
    
    SimbleeForMobile.drawText(xAxis, yAxis,"Date");
    SimbleeForMobile.drawText(xAxis+21*columnWidth, yAxis,"Count");
    SimbleeForMobile.drawText(xAxis+41*columnWidth, yAxis,"Batt %");
    SimbleeForMobile.endScreen();       // So, everything below this is not cached
    

    for (int i=0; i < DAILYCOUNTNUMBER; i++) {
        int pointer = (DAILYOFFSET + (i+FRAMread8(DAILYPOINTERADDR)) % DAILYCOUNTNUMBER)*WORDSIZE;
        dailyCount = FRAMread16(pointer+DAILYCOUNTOFFSET);
        if (dailyCount > 0) {
            yAxis = yAxis + rowHeight;
            SimbleeForMobile.drawText(xAxis, yAxis,FRAMread8(pointer));
            SimbleeForMobile.drawText(xAxis+3*columnWidth, yAxis,"/");
            SimbleeForMobile.drawText(xAxis+5*columnWidth, yAxis,FRAMread8(pointer+DAILYDATEOFFSET));
            SimbleeForMobile.drawText(xAxis+15*columnWidth, yAxis," - ");
            SimbleeForMobile.drawText(xAxis+22*columnWidth, yAxis,dailyCount);
            SimbleeForMobile.drawText(xAxis+34*columnWidth, yAxis,"  -  ");
            snprintf(battBuffer, 5,"%u%%",FRAMread8(pointer+DAILYBATTOFFSET));   // Puts % next to batt from 0-100
            SimbleeForMobile.drawText(xAxis+42*columnWidth, yAxis,battBuffer);
        }
    }
    SimbleeForMobile.endScreen();       // So, everything below this is not cached
}

void createHourlyScreen() // This is the screen that displays today's hourly counts
{
    int xAxis = 30;
    int yAxis = 110;
    int rowHeight = 15;
    int columnWidth = 5;
    int row = 1;
    int hourIndex = FRAMread16(HOURLYPOINTERADDR);
    char battBuffer[4];   // Should be enough 3 digits plus % symbol
    char hourBuffer[5];   // Should be enough for 4 hour and :00

    TakeTheBus();
        t = RTC.get();                  // First we will establish the time
    GiveUpTheBus();
    time_t mighnightUnixT = findMidnight(t);   // This is the midnight that preceeds today's day
    
    SimbleeForMobile.beginScreen(WHITE, PORTRAIT); // Sets orientation
    menuBar = SimbleeForMobile.drawSegment(20, 70, 280, titles, countof(titles));
    SimbleeForMobile.updateValue(menuBar, 2);
    
    SimbleeForMobile.drawText(xAxis, yAxis,"Hour");
    SimbleeForMobile.drawText(xAxis+21*columnWidth, yAxis,"Count");
    SimbleeForMobile.drawText(xAxis+41*columnWidth, yAxis,"Batt %");
    SimbleeForMobile.endScreen();       // So, everything below this is not cached
    

    for (int i=1; i <= 24; i++) {           // The most hours we can have in a day is 24 - right?
        hourIndex = (hourIndex ? hourIndex : HOURLYCOUNTNUMBER) -1; // Modeled on i = (i ? i : range) - 1;
        int pointer = (HOURLYOFFSET + hourIndex) * WORDSIZE;
        yAxis = yAxis + rowHeight;
        time_t unixTime = FRAMread32(pointer);
        if (unixTime < mighnightUnixT) {       // Let's make sure we are still on today's date
            Serial.println("That is all for today");
            break;
        }
        snprintf(hourBuffer, 6, "%i:00",hour(unixTime));
        SimbleeForMobile.drawText(xAxis, yAxis,hourBuffer);
        SimbleeForMobile.drawText(xAxis+15*columnWidth, yAxis," - ");
        SimbleeForMobile.drawText(xAxis+22*columnWidth, yAxis,FRAMread16(pointer+HOURLYCOUNTOFFSET));
        SimbleeForMobile.drawText(xAxis+34*columnWidth, yAxis,"  -  ");
        snprintf(battBuffer, 5,"%u%%",FRAMread8(pointer+HOURLYBATTOFFSET));   // Puts % next to batt from 0-100
        SimbleeForMobile.drawText(xAxis+42*columnWidth, yAxis,battBuffer);
    }
    SimbleeForMobile.endScreen();       // So, everything below this is not cached
}


void createAdminScreen() // This is the screen that displays current status information
{
    SimbleeForMobile.beginScreen(WHITE, PORTRAIT); // Sets orientation
    menuBar = SimbleeForMobile.drawSegment(20, 70, 280, titles, countof(titles));
    SimbleeForMobile.updateValue(menuBar, 3);

    // The first control will be a switch
    controlRegisterValue = FRAMread8(CONTROLREGISTER);

    ui_EraseMemStatus = SimbleeForMobile.drawText(190, 140, " ");
    
    ui_EraseMemSwitch = SimbleeForMobile.drawButton(60,130,120, "Reset Mem",RED);
    SimbleeForMobile.setEvents(ui_EraseMemSwitch, EVENT_RELEASE);
    
    ui_StartStopStatus = SimbleeForMobile.drawText(190, 190, " ");

    ui_StartStopSwitch = SimbleeForMobile.drawButton(60,180,120, "Start/Stop");
    SimbleeForMobile.setEvents(ui_StartStopSwitch, EVENT_RELEASE);
    
    SimbleeForMobile.drawText(20,240,"0 sec          Debounce           1 sec");
    ui_DebounceSlider = SimbleeForMobile.drawSlider(20,260,270,0,1000);
 
    SimbleeForMobile.drawText(20,300,"Max            Sensitivity           Min");
    ui_SensitivitySlider = SimbleeForMobile.drawSlider(20,320,270,0,16);

    //SimbleeForMobile.drawText(20,340,"Set Date");
    ui_setYear = SimbleeForMobile.drawTextField(35,370,60,tm.Year);
    ui_setMonth = SimbleeForMobile.drawTextField(135,370,45,tm.Month);
    ui_setDay = SimbleeForMobile.drawTextField(240,370,40,tm.Day);

    //SimbleeForMobile.drawText(20,400,"Set Time");
    ui_setHour = SimbleeForMobile.drawTextField(40,430,40,tm.Hour);
    ui_setMinute = SimbleeForMobile.drawTextField(135,430,45,tm.Minute);
    ui_setSecond = SimbleeForMobile.drawTextField(230,430,40,tm.Second);

    ui_hourStepper = SimbleeForMobile.drawStepper(15,460,20,0,24);
    ui_minStepper = SimbleeForMobile.drawStepper(110,460,20,0,60);
    ui_secStepper = SimbleeForMobile.drawStepper(205,460,20,0,60);
    ui_yearStepper = SimbleeForMobile.drawStepper(15,400,20,2016,2017);
    ui_monthStepper = SimbleeForMobile.drawStepper(110,400,20,1,12);
    ui_dayStepper = SimbleeForMobile.drawStepper(205,400,20,1,31);
    
    SimbleeForMobile.updateText(ui_setHour,"HH");
    SimbleeForMobile.updateText(ui_setMinute,"MM");
    SimbleeForMobile.updateText(ui_setSecond,"SS");
    SimbleeForMobile.updateText(ui_setYear,"YYYY");
    SimbleeForMobile.updateText(ui_setMonth,"MM");
    SimbleeForMobile.updateText(ui_setDay,"DD");
    
    ui_UpdateButton = SimbleeForMobile.drawButton(100,510,120, "Update");
    SimbleeForMobile.setEvents(ui_UpdateButton, EVENT_RELEASE);
    
    SimbleeForMobile.endScreen();
    
}

void printEvent(event_t &event)     // Utility method to print information regarding the given event
{
    switch (currentScreen) {
        case 1: Serial.print("On Current screen "); break;
        case 2: Serial.print("On Daily screen ");   break;
        case 3: Serial.print("On Hourly screen ");  break;
        case 4: Serial.print("On Admin screen ");   break;
        default:                                    break;
    }
    switch (event.id) {
        case 0: Serial.print("the Menu bar");           break;
        case 2: Serial.print("the Mem  Reset Button");  break;
        case 4: Serial.print("the Start/Stop Button");  break;
        case 6: Serial.print("the Debounce slider");    break;
        case 8: Serial.print("the Sensitivity slider"); break;
        case 10:Serial.print("the Refresh button");     break;
        case 11:Serial.print("the Admin Code field");   break;
        case 12:Serial.print("the LED on-off switch");  break;
        case 21:Serial.print("the Update button");      break;
        default:
            Serial.print(" element ");
            Serial.print(event.id);
            break;
    }
    switch (event.type) {
        case 0: Serial.print(" was pressed with a value of ");  Serial.println(event.value);    break;
        case 1: Serial.println(" was pressed"); break;
        case 2: Serial.println(" was released");break;
        default:
            Serial.print(" had an event ");
            Serial.println(event.type);
            break;
    }
}

time_t findMidnight(time_t unixT) // Need to break at midnight
{
    tmElements_t timeElement;
    breakTime(unixT, timeElement);
    unsigned long workingNumber;
    workingNumber = unixT - 3600*timeElement.Hour -60*timeElement.Minute - timeElement.Second;
    return workingNumber;
}


void toArduinoTime(time_t unixT)   // Converts to date time for the UI
{
    char dateTimeArray[18]="mm/dd/yy hh:mm:ss";
    char *dateTimePointer;
    tmElements_t timeElement;
    breakTime(unixT, timeElement);
    dateTimePointer = dateTimeArray;

    if(timeElement.Month < 10) {
        dateTimeArray[0] = '0';
        dateTimeArray[1] = timeElement.Month+48;  // Stupid but the +48 gives the right ASCII code
    }
    else {
        dateTimeArray[0] = int(timeElement.Month/10);
        dateTimeArray[1] = (timeElement.Month%10);
    }
    dateTimeArray[2] =  '/';
    if(timeElement.Day < 10) {
        dateTimeArray[3] = '0';
        dateTimeArray[4] = timeElement.Day+48;  // Stupid but the +48 gives the right ASCII code
    }
    else {
        dateTimeArray[3] = int(timeElement.Day/10)+48;
        dateTimeArray[4] = (timeElement.Day%10)+48;
    }
    dateTimeArray[5] =  '/';
    int currentYear = int(timeElement.Year)-30; // Year is displayed as 2016 not 16
                   
    if(currentYear < 10) {
        dateTimeArray[6] = '0';
        dateTimeArray[7] = currentYear+48;  // Stupid but the +48 gives the right ASCII code
    }
    else {
        dateTimeArray[6] = int(currentYear/10)+48;
        dateTimeArray[7] = (currentYear%10)+48;
    }
    dateTimeArray[8] =  ' ';
    if(timeElement.Hour < 10) {
        dateTimeArray[9] = '0';
        dateTimeArray[10] = timeElement.Hour+48;  // Stupid but the +48 gives the right ASCII code
    }
    else {
        dateTimeArray[9] = int(timeElement.Hour/10)+48;
        dateTimeArray[10] = (timeElement.Hour%10)+48;
    }
    dateTimeArray[11] =  ':';
    if(timeElement.Minute < 10) {
        dateTimeArray[12] = '0';
        dateTimeArray[13] = timeElement.Minute+48;  // Stupid but the +48 gives the right ASCII code
    }
    else {
        dateTimeArray[12] = int(timeElement.Minute/10)+48;
        dateTimeArray[13] = (timeElement.Minute%10)+48;
    }
    dateTimeArray[14] =  ':';
    if(timeElement.Second < 10) {
        dateTimeArray[15] = '0';
        dateTimeArray[16] = timeElement.Second+48;  // Stupid but the +48 gives the right ASCII code
    }
    else {
        dateTimeArray[15] = int(timeElement.Second/10)+48;
        dateTimeArray[16] = (timeElement.Second%10)+48;
    }
    Serial.print("Current Date and Time: ");
    Serial.println(dateTimePointer);
    SimbleeForMobile.updateText(dateTimeField, dateTimePointer);
}



int AlarmInteruptHandler(uint32_t dummyPin) // This is where we will take action when we detect an alarm
{
    alarmInterrupt = true;
    return 0;
}


void BlinkForever()
{
    Serial.println(F("Error - Reboot"));
    while(1) { }
}


