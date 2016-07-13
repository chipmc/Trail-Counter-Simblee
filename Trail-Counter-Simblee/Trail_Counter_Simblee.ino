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
//Time Period Deinifinitions - used for debugging
#define HOURLYPERIOD t.hour()   // Normally t.hour but can use t.min for debugging
#define DAILYPERIOD t.day() // Normally t.date but can use t.min or t.hour for debugging

//These defines let me change the memory map without hunting through the whole program
#define VERSIONNUMBER 5       // Increment this number each time the memory map is changed
#define WORDSIZE 8            // For the Word size
#define PAGESIZE 4096         // Memory size in bytes / word size - 256kb FRAM
// First Word - 8 bytes for setting global values
#define DAILYOFFSET 2        // First word of daily counts
#define HOURLYOFFSET 16        // First word of hourly counts (remember we start counts at 1)
#define DAILYCOUNTNUMBER 14    // used in modulo calculations - sets the # of days stored
#define HOURLYCOUNTNUMBER 4078 // used in modulo calculations - sets the # of hours stored - 256k (4096-14-2)
#define VERSIONADDR 0x0       // Memory Locations By Name not Number
#define SENSITIVITYADDR 0x1   // For the 1st Word locations
#define DEBOUNCEADDR 0x2        // Two bytes for debounce
#define DAILYPOINTERADDR 0x4
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
#include "Wire.h"
#include "TimeLib.h"               // This library brings Unix Time capabilities
#include "RTClib.h"
#include "MAX17043.h"           // Drives the LiPo Fuel Gauge
#include "Adafruit_FRAM_I2C.h"   // Note - had to comment out the Wire.begin() in this library
#include "SimbleeForMobile.h"
#include <stdlib.h>
#include <stdio.h>


// Prototypes
// Prototypes From the included libraries
MAX17043 batteryMonitor;                      // Init the Fuel Gauge
Adafruit_FRAM_I2C fram = Adafruit_FRAM_I2C(); // Init the FRAM
RTC_DS3231 rtc;                               // Init the DS3231


// Prototypes From my functions
// Prototypes for FRAM Functions
unsigned long FRAMread32(unsigned long address); // Reads a 32 bit word
void FRAMwrite32(int address, unsigned long value);  // Write 32 bits to FRAM
int FRAMread16(unsigned int address); // Reads a 16 bit word
void FRAMwrite16(unsigned int address, int value);   // Write 16 bits to FRAM
uint8_t FRAMread8(unsigned int address);  // Reads a 8 bit word
void FRAMwrite8(unsigned int address, uint8_t value);    // Write 8 bits to FRAM
void FRAMwrite8(unsigned int address, uint8_t value); //Writes a 32-bit word
void ResetFRAM();  // This will reset the FRAM - set the version and preserve delay and sensitivity

// Prototypes for i2c functions
boolean GiveUpTheBus(); // Give up the i2c bus
boolean TakeTheBus(); // Take the 12c bus
void enable32Khz(uint8_t enable);  // Need to turn on the 32k square wave for bus moderation


// Prototypes for General Functions
void BlinkForever(); // Ends execution if there is an error
int sprintf ( char * str, const char * format, ... );
void NonBlockingDelay(int millisDelay);  // Used for a non-blocking delay


// Prototypes for Date and Time Functions
void toArduinoHour(unsigned long timeElement, int xAxis, int yAxis);  // Just gets the hour
void toArduinoDateTime(unsigned long unixT); // Puts time in format for reporting
unsigned long toUnixTime(DateTime ut);  // For efficiently storing time in memory
void toDateTimeValues(unsigned long unixT);   // Converts to date time for the Values on the Settings Tab


// Prototypes for the Simblee
void SimbleeForMobile_onConnect();   // Actions to take once we get connected
void SimbleeForMobile_onDisconnect();    // Can clean up resources once we disconnect
void ui();   // The function that defines the iPhone UI
void ui_event(event_t &event);   // This is where we define the actions to occur on UI events
void createCurrentScreen(); // This is the screen that displays current status information
void createDailyScreen(); // This is the screen that displays current status information
void createHourlyScreen(); // This is the screen that displays current status information
void createAdminScreen(); // This is the screen that you use to administer the device
void printEvent(event_t &event);     // Utility method to print information regarding the given event

// Define variables and constants
// Pin Value Variables
int SCLpin = 13;    // Simblee i2c Clock pin
int SDApin = 14;    // Simblee i2c Data pin
int TalkPin = 23;  // This is the open-drain line for signaling i2c mastery
int The32kPin = 24;  // This is a 32k squarewave from the DS3231

// Battery monitor
float stateOfCharge = 0;    // Initialize state of charge

// FRAM and Unix time variables
//unsigned int  framAddr;
unsigned long unixTime;     // This is time / date encoded as a 32-bit word
tmElements_t currentTime;   // Here we have a data structure for date and time
tmElements_t t;         // Variable use for passing date / time
int lastHour = 0;  // For recording the startup values
int lastDate = 0;   // For providing dat break counts
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
unsigned int hourlyPersonCount = 0;  // hourly counter
unsigned int dailyPersonCount = 0;   //  daily counter
//byte currentHourlyPeriod;    // This is where we will know if the period changed
//byte currentDailyPeriod;     // We will keep daily counts as well as period counts
int setYear,setMonth,setDay,setHour,setMinute,setSecond;


// Variables for Simblee Display
uint8_t ui_RefreshButton;   // Refrsh button ID on Current Tab
// uint8_t ui_ReturnButton;
uint8_t ui_StartStopSwitch; // Start stop button ID on Admin Tab
uint8_t ui_StartStopStatus; // Test field ID for start stop status on Admin Tab
uint8_t ui_DebounceSlider;  // Slider ID for adjusting debounce on Admin Tab
uint8_t ui_SensitivitySlider;   // Slider ID for adjusting sensitivity on Admin Tab
uint8_t ui_UpdateButton;  // Update button ID on Admin Tab
uint8_t dateTimeField;  // Text field on Current Tab
uint8_t hourlyField;    // Hour field ID for Houly Tab
uint8_t dailyField;     // Date feild ID for Hourly Tab
uint8_t chargeField;    // State of charge field ID on Current Tab
uint8_t menuBar;        // ID for the tabbed Menu Bar
const char *titles[] = { "Current", "Daily", "Hourly", "Admin" };
const char *fields[] = { "Hr","Min","Sec","Yr","Mon","Sec"};
// uint8_t hourlyTitle;
// uint8_t dailyTitle;
int currentScreen; // The ID of the current screen being displayed
uint8_t ui_setYear, ui_setMonth,ui_setDay,ui_setHour,ui_setMinute,ui_setSecond; // Element which displays date and time values on Admin Tab
uint8_t ui_hourStepper, ui_minStepper, ui_secStepper, ui_yearStepper, ui_monthStepper, ui_dayStepper;   // Stepper IDs for adjusting on Admin Tab

// Accelerometer Values
int debounce;               // This is a minimum debounce value - additional debounce set using pot or remote terminal
int accelInputValue;            // Raw sensitivity input (0-9);
byte accelSensitivity;               // Hex variable for sensitivity

// Variables for the control byte
// Control Register  (8 - 4 Reserved, 3-Start / Stop Test, 2-Set Sensitivity, 1-Set Delay)
byte signalDebounceChange = B0000001;  // Mask for accessing the debounce bit
byte signalSentitivityChange = B00000010;   // Mask for accessing the sensitivity bit
byte toggleStartStop = B00000100;   // Mask for accessing the start / stop bit
// byte signalTimeChange = B00001000;  // Mask for accessing the time change bit
byte controlRegisterValue;  // Current value of the control register


// include newlib printf float support (%f used in sprintf below)
asm(".global _printf_float");


// Add setup code
void setup()
{
    Wire.beginOnPins(SCLpin,SDApin);
    Serial.begin(9600);
    Serial.println("Startup delay...");
    delay(1000); // This is to make sure that the Arduino boots first as it initializes the various devices
    
    // Unlike Arduino Simblee does not pre-define inputs
    pinMode(TalkPin, INPUT);  // Shared Talk line
    pinMode(The32kPin, INPUT);   // Shared 32kHz line from clock
    
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

    FRAMwrite8(CONTROLREGISTER, 0x0);       // Reset the control values
    
    // Set up the Simblee Mobile App
    SimbleeForMobile.deviceName = "Ulmstead";          // Device name
    SimbleeForMobile.advertisementData = "counts";  // Name of data service
    SimbleeForMobile.domain = "ulmstead.simblee.com";    // use a shared cache
    SimbleeForMobile.begin();
    
    Serial.println("Ready to go....");
}

// Add loop code
void loop()
{
    SimbleeForMobile.process(); // process must be called in the loop for SimbleeForMobile
}

void SimbleeForMobile_onConnect()   // Actions to take once we get connected.
{
    currentScreen = -1;     // Reset the current screen to non being displayed
    Serial.print("The dimensions (H/W) of the screen are:");    // Will start to use this information once we start accessing other phones
    Serial.print(SimbleeForMobile.screenHeight);
    Serial.print(" / ");
    Serial.println(SimbleeForMobile.screenWidth);
}


void SimbleeForMobile_onDisconnect()    // Can clean up resources once we disconnect
{
    /*
     * Not used in this sketch. Could clean up any resources
     * no longer required.
    */
}

void ui()   // The function that defines the iPhone UI
{

    if(SimbleeForMobile.screen == currentScreen) return;    // If we are on the right screen then we are all set
    currentScreen = SimbleeForMobile.screen;    // If not, let's capture the current screen number
    switch(SimbleeForMobile.screen)
    {
        case 1:
            createCurrentScreen();  // Create the current screen
            SimbleeForMobile.updateValue(hourlyField, FRAMread16(CURRENTHOURLYCOUNTADDR)); // Populate the fields with values
            SimbleeForMobile.updateValue(dailyField, FRAMread16(CURRENTDAILYCOUNTADDR));
            controlRegisterValue = FRAMread8(CONTROLREGISTER);
            if ((controlRegisterValue & toggleStartStop) >> 2) {
                SimbleeForMobile.updateText(ui_StartStopStatus, "Running");
            }
            else SimbleeForMobile.updateText(ui_StartStopStatus, "Stopped");
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
        toArduinoDateTime(FRAMread32(CURRENTCOUNTSTIME));
        Serial.println("");
        SimbleeForMobile.updateValue(hourlyField, FRAMread16(CURRENTHOURLYCOUNTADDR));
        SimbleeForMobile.updateValue(dailyField, FRAMread16(CURRENTDAILYCOUNTADDR));
        if (batteryMonitor.getSoC() <= 100) {
            SimbleeForMobile.updateValue(chargeField, batteryMonitor.getSoC());
        }
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
    else if (event.id == ui_StartStopSwitch && event.type == EVENT_RELEASE) // Here is were we handle button release from the Admin screen
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
        accelSensitivity = map(accelInputValue,0,10,0,16);  // Map slider values - today only using sensitivity up to 16 (out of 256 possible)

        if (FRAMread16(DEBOUNCEADDR) != debounce)    // Check to see if debounce value needs to be updated
        {
            FRAMwrite16(DEBOUNCEADDR, debounce);    // If so, write to FRAM
            FRAMwrite8(CONTROLREGISTER,signalDebounceChange ^ controlRegisterValue);    // Then set the flag so Arduino will apply new setting
            Serial.println("Updating debounce");    // Let the console know
        }
        if (FRAMread8(SENSITIVITYADDR) != accelSensitivity) // Same as debounce above - but now for sensitivity
        {
            FRAMwrite8(SENSITIVITYADDR, accelSensitivity);
            FRAMwrite8(CONTROLREGISTER,signalSentitivityChange ^ controlRegisterValue);
            Serial.println("Updating sensitivity");
        }
        if (setYear >> 0 && setMonth >> 0 && setDay >> 0) // Will only update the update if these values are all non-zero
        {
            TakeTheBus();  // Clock is an i2c device
            rtc.adjust(DateTime(setYear,setMonth,setDay,setHour, setMinute, setSecond));  // Set the clock using values from the screen
            GiveUpTheBus();
            Serial.println("Updating time");
        }
        controlRegisterValue = FRAMread8(CONTROLREGISTER);  // Read and display the control register after these updates
        Serial.print("Control Register Value =");
        Serial.println(controlRegisterValue);
    }
    else if (event.id == ui_hourStepper)    // Used to increment / decrement values for setting clock on the Admin Screen
    {
        setHour = event.value;
        SimbleeForMobile.updateValue(ui_setHour, setHour);
    }
    else if (event.id == ui_minStepper)
    {
        setMinute = event.value;
        SimbleeForMobile.updateValue(ui_setMinute, setMinute);
    }
    else if (event.id == ui_secStepper)
    {
        setSecond = event.value;
        SimbleeForMobile.updateValue(ui_setSecond, setSecond);
    }
    else if (event.id == ui_yearStepper)
    {
        setYear = event.value;
        SimbleeForMobile.updateValue(ui_setYear, setYear);
    }
    else if (event.id == ui_monthStepper)
    {
        setMonth = event.value;
        SimbleeForMobile.updateValue(ui_setMonth, setMonth);
    }
    else if (event.id == ui_dayStepper)
    {
        setDay = event.value;
        SimbleeForMobile.updateValue(ui_setDay, setDay);
    }
}


void createCurrentScreen() // This is the screen that displays current status information
{

    SimbleeForMobile.beginScreen(WHITE, PORTRAIT); // Sets orientation
    menuBar = SimbleeForMobile.drawSegment(10, 90, 280, titles, countof(titles));
    SimbleeForMobile.updateValue(menuBar, 0);

    dateTimeField = SimbleeForMobile.drawText(50, 140, " ");
    toArduinoDateTime(FRAMread32(CURRENTCOUNTSTIME));
    Serial.println("");
    SimbleeForMobile.drawText(50, 160, "Hourly Count:");
    hourlyField = SimbleeForMobile.drawText(210,160," ");
    SimbleeForMobile.drawText(50, 180, "Daily Count:");
    dailyField =   SimbleeForMobile.drawText(210,180," ");
    SimbleeForMobile.drawText(50, 200, "State of Charge:");
    if (batteryMonitor.getSoC() >= 105) {
        chargeField =   SimbleeForMobile.drawText(210,200,"Error");
    }
    else {
        chargeField =   SimbleeForMobile.drawText(210,200,batteryMonitor.getSoC());
        SimbleeForMobile.drawText(230,200," %");
    }
    controlRegisterValue = FRAMread8(CONTROLREGISTER);
    SimbleeForMobile.drawText(50, 220, "Counter Status:");
    ui_StartStopStatus = SimbleeForMobile.drawText(210, 220, " ");


    // we need a momentary button (the default is a push button)
    ui_RefreshButton = SimbleeForMobile.drawButton(110, 260, 90, "Refresh");
    SimbleeForMobile.setEvents(ui_RefreshButton, EVENT_RELEASE);
    

    SimbleeForMobile.endScreen();
}

void createDailyScreen() // This is the screen that displays current status information
{
    int xAxis = 30;
    int yAxis = 140;
    int rowHeight = 15;
    int columnWidth = 5;
    int row = 1;

    SimbleeForMobile.beginScreen(WHITE, PORTRAIT); // Sets orientation
    menuBar = SimbleeForMobile.drawSegment(10, 90, 280, titles, countof(titles));
    SimbleeForMobile.updateValue(menuBar, 1);
    
    SimbleeForMobile.drawText(100, 120, "Daily Screen");
    SimbleeForMobile.drawText(xAxis, yAxis,"Date");
    SimbleeForMobile.drawText(xAxis+21*columnWidth, yAxis,"Count");
    SimbleeForMobile.drawText(xAxis+41*columnWidth, yAxis,"Batt %");
    for (int i=0; i < DAILYCOUNTNUMBER; i++) {
        if (FRAMread8((DAILYOFFSET + (i+FRAMread8(DAILYPOINTERADDR)) % DAILYCOUNTNUMBER)*WORDSIZE+DAILYCOUNTOFFSET) != 0) {
            yAxis = yAxis + rowHeight;
            if (yAxis > SimbleeForMobile.screenHeight) yAxis = 140 + rowHeight;
            SimbleeForMobile.drawText(xAxis, yAxis,FRAMread8((DAILYOFFSET + (i+FRAMread8(DAILYPOINTERADDR)) % DAILYCOUNTNUMBER)*WORDSIZE));
            SimbleeForMobile.drawText(xAxis+3*columnWidth, yAxis,"/");
            SimbleeForMobile.drawText(xAxis+5*columnWidth, yAxis,FRAMread8((DAILYOFFSET + (i+FRAMread8(DAILYPOINTERADDR)) % DAILYCOUNTNUMBER)*WORDSIZE+DAILYDATEOFFSET));
            SimbleeForMobile.drawText(xAxis+15*columnWidth, yAxis," - ");
            SimbleeForMobile.drawText(xAxis+22*columnWidth, yAxis,FRAMread16((DAILYOFFSET + (i+FRAMread8(DAILYPOINTERADDR)) % DAILYCOUNTNUMBER)*WORDSIZE+DAILYCOUNTOFFSET));
            SimbleeForMobile.drawText(xAxis+34*columnWidth, yAxis,"  -  ");
            SimbleeForMobile.drawText(xAxis+42*columnWidth, yAxis,FRAMread8((DAILYOFFSET + (i+FRAMread8(DAILYPOINTERADDR)) % DAILYCOUNTNUMBER)*WORDSIZE+DAILYBATTOFFSET));
            SimbleeForMobile.drawText(xAxis+46*columnWidth, yAxis,"%");
        }
    }
    SimbleeForMobile.drawText(xAxis, yAxis+rowHeight,"Done");
    SimbleeForMobile.endScreen();
}

void createHourlyScreen() // This is the screen that displays current status information
{
    int xAxis = 20;
    int yAxis = 140;
    int rowHeight = 15;
    int columnWidth = 5;
    int row = 1;
    int hoursReported = 24;

    SimbleeForMobile.beginScreen(WHITE, PORTRAIT); // Sets orientation
    menuBar = SimbleeForMobile.drawSegment(10, 90, 280, titles, countof(titles));
    SimbleeForMobile.updateValue(menuBar, 2);
    
    SimbleeForMobile.drawText(100, 120, "Hourly Screen");
    SimbleeForMobile.drawText(xAxis, yAxis,"Date");
    SimbleeForMobile.drawText(xAxis+26*columnWidth, yAxis,"Count");
    SimbleeForMobile.drawText(xAxis+39*columnWidth, yAxis,"Batt %");
    for (int i=HOURLYCOUNTNUMBER; i>=1; i--) {
        int address = (HOURLYOFFSET + (i+FRAMread16(HOURLYPOINTERADDR)) % HOURLYCOUNTNUMBER)*WORDSIZE;
        if (FRAMread8((HOURLYOFFSET + (i+FRAMread16(HOURLYPOINTERADDR)) % HOURLYCOUNTNUMBER)*WORDSIZE) != 0) {
            yAxis = yAxis + rowHeight;
            if (yAxis > SimbleeForMobile.screenHeight) yAxis = 140 + rowHeight;
            if (hoursReported > 0 ) hoursReported--;
            else break;
            unsigned long unixTime = FRAMread32((HOURLYOFFSET + (i+FRAMread16(HOURLYPOINTERADDR)) % HOURLYCOUNTNUMBER)*WORDSIZE);
            toArduinoHour(unixTime,xAxis,yAxis);
            SimbleeForMobile.drawText(xAxis+25*columnWidth, yAxis," - ");
            SimbleeForMobile.drawText(xAxis+28*columnWidth, yAxis,FRAMread16(((HOURLYOFFSET + (i+FRAMread16(HOURLYPOINTERADDR)) % HOURLYCOUNTNUMBER)*WORDSIZE)+HOURLYCOUNTOFFSET));
            SimbleeForMobile.drawText(xAxis+36*columnWidth, yAxis,"  -  ");
            SimbleeForMobile.drawText(xAxis+40*columnWidth, yAxis,FRAMread8(((HOURLYOFFSET + (i+FRAMread16(HOURLYPOINTERADDR)) % HOURLYCOUNTNUMBER)*WORDSIZE)+HOURLYBATTOFFSET));
            SimbleeForMobile.drawText(xAxis+44*columnWidth, yAxis,"%");
        }
    }
    SimbleeForMobile.endScreen();
}

void createAdminScreen() // This is the screen that displays current status information
{
    SimbleeForMobile.beginScreen(WHITE, PORTRAIT); // Sets orientation
    menuBar = SimbleeForMobile.drawSegment(10, 90, 280, titles, countof(titles));
    SimbleeForMobile.updateValue(menuBar, 3);

    const char hourly[] = "Hr";

    
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
    ui_setYear = SimbleeForMobile.drawTextField(35,350,60,setYear);
    ui_setMonth = SimbleeForMobile.drawTextField(135,350,45,setMonth);
    ui_setDay = SimbleeForMobile.drawTextField(240,350,40,setDay);

    //SimbleeForMobile.drawText(20,400,"Set Time");
    ui_setHour = SimbleeForMobile.drawTextField(40,410,40,setHour);
    ui_setMinute = SimbleeForMobile.drawTextField(135,410,45,setMinute);
    ui_setSecond = SimbleeForMobile.drawTextField(230,410,40,setSecond);

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
        case 1:
            Serial.print("On Current screen ");
            break;
        case 2:
            Serial.print("On Daily screen ");
            break;
        case 3:
            Serial.print("On Hourly screen ");
            break;
        case 4:
            Serial.print("On Admin screen ");
            break;
        default:
            break;
    }
    switch (event.id) {
        case 0:
            Serial.print("the Menu bar");
            break;
        case 2:
            Serial.print("the Start / Stop Button");
            break;
        case 4:
            Serial.print("the Debounce slider");
            break;
        case 6:
            Serial.print("the Sensitivity slider");
            break;
        case 11:
            Serial.print("the Refresh button");
            break;
        case 19:
            Serial.print("the Update button");
            break;
        default:
            Serial.print(" element ");
            Serial.print(event.id);
            break;
    }
    switch (event.type) {
        case 0:
            Serial.print(" was pressed with a value of ");
            Serial.println(event.value);
            break;
        case 1:
            Serial.println(" was pressed");
            break;
        case 2:
            Serial.println(" was released");
            break;
        default:
            Serial.print(" had an event ");
            Serial.println(event.type);
            break;
    }
    
  /*
   
    Serial.print("    Text: ");
    Serial.println(event.text);
    
    Serial.print("    Coords: ");
    Serial.print(event.x);
    Serial.print(",");
    Serial.println(event.y); 
   */
}


uint8_t FRAMread8(unsigned int address)
{
    uint8_t result;
    //Serial.println("In FRAMread8");
    if (TakeTheBus()) {   // Request exclusive access to the bus
        //Serial.println("got the bus");
        result = fram.read8(address);
    }
    GiveUpTheBus();       // Release exclusive access to the bus
    return result;
}
                       
void FRAMwrite8(unsigned int address, uint8_t value)    // Write 8 bits to FRAM
{
    if (TakeTheBus()) {  // Request exclusive access to the bus
        fram.write8(address,value);
    }
    GiveUpTheBus();// Release exclusive access to the bus
}

int FRAMread16(unsigned int address)
{
    long two;
    long one;
    if(TakeTheBus()) {  // Request exclusive access to the bus
        //Read the 2 bytes from  memory.
        two = fram.read8(address);
        one = fram.read8(address + 1);
    }
    GiveUpTheBus();// Release exclusive access to the bus
    //Return the recomposed long by using bitshift.
    return ((two << 0) & 0xFF) + ((one << 8) & 0xFFFF);
}

void FRAMwrite16(unsigned int address, int value)   // Write 16 bits to FRAM
{
    //This function will write a 2 byte (16bit) long to the eeprom at
    //the specified address to address + 1.
    //Decomposition from a long to 2 bytes by using bitshift.
    //One = Most significant -> Four = Least significant byte
    byte two = (value & 0xFF);
    byte one = ((value >> 8) & 0xFF);
    
    //Write the 2 bytes into the eeprom memory.
    if (TakeTheBus()) {  // Request exclusive access to the bus
        fram.write8(address, two);
        fram.write8(address + 1, one);
    }
    GiveUpTheBus();// Release exclusive access to the bus
}

unsigned long FRAMread32(unsigned long address)
{
    long four;
    long three;
    long two;
    long one;
    if(TakeTheBus()) {  // Request exclusive access to the bus
        //Read the 4 bytes from memory.
        four = fram.read8(address);
        three = fram.read8(address + 1);
        two = fram.read8(address + 2);
        one = fram.read8(address + 3);
    }
    GiveUpTheBus();// Release exclusive access to the bus
    //Return the recomposed long by using bitshift.
    return ((four << 0) & 0xFF) + ((three << 8) & 0xFFFF) + ((two << 16) & 0xFFFFFF) + ((one << 24) & 0xFFFFFFFF);
}

void FRAMwrite32(int address, unsigned long value)  // Write 32 bits to FRAM
{
    //This function will write a 4 byte (32bit) long to the eeprom at
    //the specified address to address + 3.
    //Decomposition from a long to 4 bytes by using bitshift.
    //One = Most significant -> Four = Least significant byte
    
    byte four = (value & 0xFF);
    byte three = ((value >> 8) & 0xFF);
    byte two = ((value >> 16) & 0xFF);
    byte one = ((value >> 24) & 0xFF);
    
    //Write the 4 bytes into the eeprom memory.
    if (TakeTheBus()) {  // Request exclusive access to the bus
        fram.write8(address, four);
        fram.write8(address + 1, three);
        fram.write8(address + 2, two);
        fram.write8(address + 3, one);
    }
    GiveUpTheBus();// Release exclusive access to the bus
}


void ResetFRAM()  // This will reset the FRAM - set the version and preserve delay and sensitivity
{
    // Note - have to hard code the size here due to this issue - http://www.microchip.com/forums/m501193.aspx
    Serial.println("Resetting Memory");
    for (unsigned long i=3; i < 32768; i++) {  // Start at 3 to not overwrite debounce and sensitivity
        FRAMwrite8(i,0x0);
        //Serial.println(i);
        if (i==8192) Serial.println(F("25% done"));
        if (i==16384) Serial.println(F("50% done"));
        if (i==(24576)) Serial.println(F("75% done"));
        if (i==32767) Serial.println(F("Done"));
    }
    FRAMwrite8(VERSIONADDR,VERSIONNUMBER);  // Reset version to match #define value for sketch
}

void toDateTimeValues(unsigned long unixT)   // Converts to date time for the Values on the Settings Tab
{
    TimeElements timeElement;
    breakTime(unixT, timeElement);

    SimbleeForMobile.updateValue(ui_setYear, int(timeElement.Year)-30);
    SimbleeForMobile.updateValue(ui_setMonth, int(timeElement.Month));
    SimbleeForMobile.updateValue(ui_setDay, int(timeElement.Day));
    SimbleeForMobile.updateValue(ui_setHour, int(timeElement.Hour));
    SimbleeForMobile.updateValue(ui_setMinute, int(timeElement.Minute));
    SimbleeForMobile.updateValue(ui_setSecond, int(timeElement.Second));
    
}


void toArduinoDateTime(unsigned long unixT)   // Converts to date time for the UI
{
    char dateTimeArray[18]="mm/dd/yy hh:mm:ss";
    char *dateTimePointer;
    TimeElements timeElement;
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

void toArduinoHour(unsigned long unixT, int xAxis, int yAxis)  // Just gets the hour
{
    tmElements_t timeElement;
    int columnWidth = 6;
    breakTime(unixT, timeElement);
    if(timeElement.Month < 10) {
        SimbleeForMobile.drawText(xAxis, yAxis,"0");
        SimbleeForMobile.drawText(xAxis+1.5*columnWidth, yAxis,timeElement.Month);
    }
    else SimbleeForMobile.drawText(xAxis, yAxis,timeElement.Month);
    SimbleeForMobile.drawText(xAxis+3*columnWidth, yAxis,"/");
    if(timeElement.Day < 10) {
        SimbleeForMobile.drawText(xAxis+4*columnWidth, yAxis,"0");
        SimbleeForMobile.drawText(xAxis+5.5*columnWidth, yAxis,timeElement.Day);
    }
    else SimbleeForMobile.drawText(xAxis+4*columnWidth, yAxis,timeElement.Day);
    SimbleeForMobile.drawText(xAxis+8*columnWidth, yAxis," ");
    SimbleeForMobile.drawText(xAxis+8*columnWidth, yAxis,timeElement.Hour);
    SimbleeForMobile.drawText(xAxis+11*columnWidth, yAxis,":");
    SimbleeForMobile.drawText(xAxis+12*columnWidth, yAxis,"00 ");
}

unsigned long toUnixTime(DateTime ut)   // For efficiently storing time in memory
{
    TimeElements timeElement;
    timeElement.Month = ut.month();
    timeElement.Day = ut.day();
    timeElement.Year = (ut.year()-1970);
    timeElement.Hour = ut.hour();
    timeElement.Minute = ut.minute();
    timeElement.Second = ut.second();
    return makeTime(timeElement);
}

void BlinkForever()
{
    Serial.println(F("Error - Reboot"));
    while(1) { }
}

boolean TakeTheBus()
{
    //Serial.print("Simblee: Asking for the bus...");
    while(digitalRead(The32kPin)) {} // The Simblee will only read the Talk line when SQW pin goes low
    //Serial.print("..Tick..TalkPin=");
    //Serial.print(digitalRead(TalkPin));
    while (!digitalRead(TalkPin)) { // Only proceed once the TalkPin is high
        NonBlockingDelay(50);
    }
    pinMode(TalkPin,OUTPUT);        // Change to output
    digitalWrite(TalkPin,LOW);      // Claim the bus by bringing the TalkPin LOW
    //Serial.println("..We have the bus");
    return 1;                       // We have it
}

boolean GiveUpTheBus()
{
    digitalWrite(TalkPin,HIGH); // Not sure if this is needed - still for completeness.
    pinMode(TalkPin,INPUT_PULLUP);  // Start listening again
    //Serial.println("Simblee: We gave up the Bus");
    return 1;
}

void NonBlockingDelay(int millisDelay)  // Used for a non-blocking delay
{
    unsigned long commandTime = millis();
    while (millis() <= millisDelay + commandTime) { }
    return;
}
