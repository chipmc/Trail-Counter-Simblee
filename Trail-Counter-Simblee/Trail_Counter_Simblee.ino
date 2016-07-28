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
#define VERSIONNUMBER 6       // Increment this number each time the memory map is changed
#define WORDSIZE 8            // For the Word size
#define PAGESIZE 4096         // Memory size in bytes / word size - 256kb FRAM
// First Word - 8 bytes for setting global values
#define DAILYOFFSET 2        // First word of daily counts
#define HOURLYOFFSET 32        // First word of hourly counts (remember we start counts at 1)
#define DAILYCOUNTNUMBER 30    // used in modulo calculations - sets the # of days stored
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


// Prototypes
// Prototypes From the included libraries
MAX17043 batteryMonitor;                      // Init the Fuel Gauge



// Prototypes for General Functions
void BlinkForever(); // Ends execution if there is an error
int sprintf ( char * str, const char * format, ... );



// Prototypes for Date and Time Functions
void toArduinoHour(time_t unixT, int xAxis, int yAxis);  // Just gets the hour
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

// Variables for Simblee Display
uint8_t ui_RefreshButton;   // Refrsh button ID on Current Tab
uint8_t ui_StartStopSwitch; // Start stop button ID on Admin Tab
uint8_t ui_StartStopStatus; // Test field ID for start stop status on Admin Tab
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

// Accelerometer Values
int debounce;               // This is a minimum debounce value - additional debounce set using pot or remote terminal
int accelInputValue;            // Raw sensitivity input (0-9);
byte accelSensitivity;               // Hex variable for sensitivity

// Variables for the control byte
// Control Register  (8 - 5 Reserved, 4- LEDs, 3-Start / Stop Test, 2-Set Sensitivity, 1-Set Delay)
byte signalDebounceChange = B0000001;  // Mask for accessing the debounce bit
byte signalSentitivityChange = B00000010;   // Mask for accessing the sensitivity bit
byte toggleStartStop = B00000100;   // Mask for accessing the start / stop bit
byte toggleLEDs = B00001000;        // Mask for accessing the LED on off bit
byte controlRegisterValue;  // Current value of the control register

// include newlib printf float support (%f used in sprintf below)
asm(".global _printf_float");

// Add setup code
void setup()
{
    Wire.beginOnPins(SCLpin,SDApin);  // Not sure if this talks to the i2c bus or not - just to be safe...
    Serial.begin(9600);
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
            case 'N':
                Serial.println(F("Cannot proceed"));
                BlinkForever();
                break;
            default:
                BlinkForever();
        }
    }
    
    // Set up the Simblee Mobile App
    SimbleeForMobile.deviceName = "Umstead";          // Device name
    SimbleeForMobile.advertisementData = "Rte 70";  // Name of data service
    SimbleeForMobile.begin();
    
    Serial.println("Ready to go....");
}

// Add loop code
void loop()
{
    SimbleeForMobile.process(); // process must be called in the loop for SimbleeForMobile
    if (alarmInterrupt)
    {
        Simblee_resetPinWake(20); // reset state of pin that caused wakeup
        alarmInterrupt = false;
        TakeTheBus();
            t = RTC.get();
        GiveUpTheBus();
        if (hour(t) == 22) {
            detachPinInterrupt(20);
            Serial.println("Sleeping at 10om");
            Simblee_pinWake(20, LOW); // configures pin 20 to wake up device on a Low signal
            Simblee_ULPDelay(INFINITE);; // Stay in ultra low power mode until interrupt from the BLE or pin
        }
        else if (hour(t) == 6) {
            attachPinInterrupt(20,AlarmInteruptHandler,LOW);
            Serial.println("Waking up at 6am");
        }
        
    }
}

void SimbleeForMobile_onConnect()   // Actions to take once we get connected.
{
    currentScreen = -1;     // Reset the current screen to non being displayed
}


void SimbleeForMobile_onDisconnect()    // Can clean up resources once we disconnect
{
    // Not sure how best to use this but will leave in case I come up with something
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
    if (event.id == ui_RefreshButton && event.type == EVENT_RELEASE)       // Releasing the Refresh button on the Current screen
    {                                                                      // Updates all the fields
        updateCurrentScreen();
    }
    else if (event.id == menuBar)                                          // This is the event handler for the menu bar
    {
        switch(event.value)
        {
            case 0:
                SimbleeForMobile.showScreen(1);
                break;
                
            case 1:
                SimbleeForMobile.showScreen(2);
                break;
                
            case 2:
                SimbleeForMobile.showScreen(3);
                break;
                
            case 3:
                SimbleeForMobile.showScreen(4);
                break;
        }
    }
    else if (event.id == ui_LEDswitch) // LED on off switch on the Current Status Screen
    {
        controlRegisterValue = FRAMread8(CONTROLREGISTER);
        FRAMwrite8(CONTROLREGISTER, toggleLEDs ^ controlRegisterValue); // Toggle the LED bit
        Serial.println("Toggled the LED bit");
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
    menuBar = SimbleeForMobile.drawSegment(20, 60, 280, titles, countof(titles));
    SimbleeForMobile.updateValue(menuBar, 0);

    // all we are doing here is laying out the screen - updates are in a separate function
    dateTimeField = SimbleeForMobile.drawText(50, 140, " ");
    SimbleeForMobile.drawText(50, 160, "Hourly Count:");
    hourlyField = SimbleeForMobile.drawText(210,160," ");
    SimbleeForMobile.drawText(50, 180, "Daily Count:");
    dailyField =   SimbleeForMobile.drawText(210,180," ");
    SimbleeForMobile.drawText(50, 200, "State of Charge:");
    chargeField = SimbleeForMobile.drawText(210,200," ");
    SimbleeForMobile.drawText(50, 220, "Counter Status:");
    ui_StartStopStatus = SimbleeForMobile.drawText(210, 220, " ");
    ui_RefreshButton = SimbleeForMobile.drawButton(110, 260, 90, "Refresh");    // we need a Refresh Button for subsequent updates
    SimbleeForMobile.setEvents(ui_RefreshButton, EVENT_RELEASE);
    SimbleeForMobile.drawText(50,355, "Indicators: Off-On");
    ui_LEDswitch = SimbleeForMobile.drawSwitch(210,350);
    SimbleeForMobile.setEvents(ui_LEDswitch,EVENT_PRESS);
    SimbleeForMobile.endScreen();
}

void updateCurrentScreen() // Since we have to update this screen three ways: create, menu bar and refresh button
{
    TakeTheBus();
        t = RTC.get();
        stateOfCharge = batteryMonitor.getSoC();
    GiveUpTheBus();
    
    toArduinoTime(t);  // Update Time Field on screen and in the console
    Serial.println("");
    
    if (stateOfCharge >= 105) {         // Update the value of the state of charge field
        SimbleeForMobile.drawText(210,200,"Error");
    }
    else {
        SimbleeForMobile.updateValue(chargeField, stateOfCharge);
        SimbleeForMobile.drawText(230,200," %");
    }
    SimbleeForMobile.updateValue(hourlyField, FRAMread16(CURRENTHOURLYCOUNTADDR)); // Populate the hourly and daily fields with values
    SimbleeForMobile.updateValue(dailyField, FRAMread16(CURRENTDAILYCOUNTADDR));
    
    controlRegisterValue = FRAMread8(CONTROLREGISTER);          // Update the Start and Stop fields with the latest status
    if ((controlRegisterValue & toggleStartStop) >> 2) {
        SimbleeForMobile.updateText(ui_StartStopStatus, "Running");
    }
    else SimbleeForMobile.updateText(ui_StartStopStatus, "Stopped");
    
    if ((controlRegisterValue & toggleLEDs) >> 3) {    // Show the state of the LED switch
        SimbleeForMobile.updateValue(ui_LEDswitch,1);
    }
    else SimbleeForMobile.updateValue(ui_LEDswitch,0);
}

void createDailyScreen() // This is the screen that displays current status information
{
    int xAxis = 30;
    int yAxis = 90;
    int rowHeight = 15;
    int columnWidth = 5;
    int row = 1;

    SimbleeForMobile.beginScreen(WHITE, PORTRAIT); // Sets orientation
    menuBar = SimbleeForMobile.drawSegment(20, 60, 280, titles, countof(titles));
    SimbleeForMobile.updateValue(menuBar, 1);
    
    SimbleeForMobile.drawText(xAxis, yAxis,"Date");
    SimbleeForMobile.drawText(xAxis+21*columnWidth, yAxis,"Count");
    SimbleeForMobile.drawText(xAxis+41*columnWidth, yAxis,"Batt %");
    SimbleeForMobile.endScreen();       // So, everything below this is not cached
    
    for (int i=0; i < DAILYCOUNTNUMBER; i++) {
        int pointer = (DAILYOFFSET + (i+FRAMread8(DAILYPOINTERADDR)) % DAILYCOUNTNUMBER)*WORDSIZE;
        yAxis = yAxis + rowHeight;
        SimbleeForMobile.drawText(xAxis, yAxis,FRAMread8(pointer));
        SimbleeForMobile.drawText(xAxis+3*columnWidth, yAxis,"/");
        SimbleeForMobile.drawText(xAxis+5*columnWidth, yAxis,FRAMread8(pointer+DAILYDATEOFFSET));
        SimbleeForMobile.drawText(xAxis+15*columnWidth, yAxis," - ");
        SimbleeForMobile.drawText(xAxis+22*columnWidth, yAxis,FRAMread16(pointer+DAILYCOUNTOFFSET));
        SimbleeForMobile.drawText(xAxis+34*columnWidth, yAxis,"  -  ");
        SimbleeForMobile.drawText(xAxis+42*columnWidth, yAxis,FRAMread8(pointer+DAILYBATTOFFSET));
        SimbleeForMobile.drawText(xAxis+46*columnWidth, yAxis,"%");
    }

}

void createHourlyScreen() // This is the screen that displays the past 24 hourly counts
{
    int xAxis = 30;
    int yAxis = 90;
    int rowHeight = 15;
    int columnWidth = 5;
    int row = 1;
    int hoursReported = 14;
    int numberHourlyDataPoints = FRAMread16(HOURLYPOINTERADDR);

    SimbleeForMobile.beginScreen(WHITE, PORTRAIT); // Sets orientation
    menuBar = SimbleeForMobile.drawSegment(20, 60, 280, titles, countof(titles));
    SimbleeForMobile.updateValue(menuBar, 2);
    
    SimbleeForMobile.drawText(xAxis, yAxis,"Date");
    SimbleeForMobile.drawText(xAxis+26*columnWidth, yAxis,"Count");
    SimbleeForMobile.drawText(xAxis+39*columnWidth, yAxis,"Batt %");
    
    SimbleeForMobile.endScreen();       // So, everything below this is not cached
    
    if (numberHourlyDataPoints < hoursReported) {
        hoursReported = numberHourlyDataPoints;
    }
    
    for (int i=1; i <= hoursReported; i++) {
        int address = (HOURLYOFFSET + (numberHourlyDataPoints - i) % HOURLYCOUNTNUMBER)*WORDSIZE;
        yAxis = yAxis + rowHeight;
        time_t unixTime = FRAMread32(address);
        
        toArduinoHour(unixTime,xAxis,yAxis);
        SimbleeForMobile.drawText(xAxis+22*columnWidth, yAxis," - ");
        SimbleeForMobile.drawText(xAxis+29*columnWidth, yAxis,FRAMread16(address+HOURLYCOUNTOFFSET));
        SimbleeForMobile.drawText(xAxis+32*columnWidth, yAxis,"  -  ");
        SimbleeForMobile.drawText(xAxis+41*columnWidth, yAxis,FRAMread8(address+HOURLYBATTOFFSET));
        SimbleeForMobile.drawText(xAxis+46*columnWidth, yAxis,"%");
    }
    SimbleeForMobile.drawText(xAxis,yAxis,"Done");

    
}


void createAdminScreen() // This is the screen that displays current status information
{
    SimbleeForMobile.beginScreen(WHITE, PORTRAIT); // Sets orientation
    menuBar = SimbleeForMobile.drawSegment(20, 60, 280, titles, countof(titles));
    SimbleeForMobile.updateValue(menuBar, 3);

    // The first control will be a switch
    controlRegisterValue = FRAMread8(CONTROLREGISTER);

    ui_StartStopStatus = SimbleeForMobile.drawText(190, 160, " ");

    ui_StartStopSwitch = SimbleeForMobile.drawButton(60,150,120, "Start/Stop");
    SimbleeForMobile.setEvents(ui_StartStopSwitch, EVENT_RELEASE);
    
    SimbleeForMobile.drawText(20,220,"0 sec          Debounce           1 sec");
    ui_DebounceSlider = SimbleeForMobile.drawSlider(20,240,270,0,1000);
 
    SimbleeForMobile.drawText(20,280,"Max            Sensitivity           Min");
    ui_SensitivitySlider = SimbleeForMobile.drawSlider(20,300,270,0,16);

    //SimbleeForMobile.drawText(20,340,"Set Date");
    ui_setYear = SimbleeForMobile.drawTextField(35,350,60,tm.Year);
    ui_setMonth = SimbleeForMobile.drawTextField(135,350,45,tm.Month);
    ui_setDay = SimbleeForMobile.drawTextField(240,350,40,tm.Day);

    //SimbleeForMobile.drawText(20,400,"Set Time");
    ui_setHour = SimbleeForMobile.drawTextField(40,410,40,tm.Hour);
    ui_setMinute = SimbleeForMobile.drawTextField(135,410,45,tm.Minute);
    ui_setSecond = SimbleeForMobile.drawTextField(230,410,40,tm.Second);

    ui_hourStepper = SimbleeForMobile.drawStepper(15,440,20,0,24);
    ui_minStepper = SimbleeForMobile.drawStepper(110,440,20,0,60);
    ui_secStepper = SimbleeForMobile.drawStepper(205,440,20,0,60);
    ui_yearStepper = SimbleeForMobile.drawStepper(15,380,20,2016,2017);
    ui_monthStepper = SimbleeForMobile.drawStepper(110,380,20,1,12);
    ui_dayStepper = SimbleeForMobile.drawStepper(205,380,20,1,31);
    
    SimbleeForMobile.updateText(ui_setHour,"HH");
    SimbleeForMobile.updateText(ui_setMinute,"MM");
    SimbleeForMobile.updateText(ui_setSecond,"SS");
    SimbleeForMobile.updateText(ui_setYear,"YYYY");
    SimbleeForMobile.updateText(ui_setMonth,"MM");
    SimbleeForMobile.updateText(ui_setDay,"DD");
    
    ui_UpdateButton = SimbleeForMobile.drawButton(100,500,120, "Update");
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
        case 2: Serial.print("the Start/Stop Button");  break;
        case 4: Serial.print("the Debounce slider");    break;
        case 6: Serial.print("the Sensitivity slider"); break;
        case 10:Serial.print("the Refresh button");     break;
        case 12:Serial.print("the LED on-off switch");  break;
        case 19:Serial.print("the Update button");      break;
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


void toArduinoHour(time_t unixT, int xAxis, int yAxis)  // Just gets the hour
{
    tmElements_t timeElement;
    int columnWidth = 5;
    breakTime(unixT, timeElement);
    if(timeElement.Month < 10) {
        SimbleeForMobile.drawText(xAxis, yAxis,"0");
        SimbleeForMobile.drawText(xAxis+2*columnWidth, yAxis,timeElement.Month);
    }
    else SimbleeForMobile.drawText(xAxis, yAxis,timeElement.Month);
    SimbleeForMobile.drawText(xAxis+5*columnWidth, yAxis,"/");
    if(timeElement.Day < 10) {
        SimbleeForMobile.drawText(xAxis+6*columnWidth, yAxis,"0");
        SimbleeForMobile.drawText(xAxis+7*columnWidth, yAxis,timeElement.Day);
    }
    else SimbleeForMobile.drawText(xAxis+6*columnWidth, yAxis,timeElement.Day);
    SimbleeForMobile.drawText(xAxis+10*columnWidth, yAxis," ");
    SimbleeForMobile.drawText(xAxis+11*columnWidth, yAxis,timeElement.Hour);
    SimbleeForMobile.drawText(xAxis+15*columnWidth, yAxis,":");
    SimbleeForMobile.drawText(xAxis+16*columnWidth, yAxis,"00 ");
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


