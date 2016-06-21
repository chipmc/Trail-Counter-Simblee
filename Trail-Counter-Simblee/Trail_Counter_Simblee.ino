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
void MemoryMapReport(); // Creates a memory map report on start

// Prototypes for i2c functions
boolean GiveUpTheBus(); // Give up the i2c bus
boolean TakeTheBus(); // Take the 12c bus
void enable32Khz(uint8_t enable);  // Need to turn on the 32k square wave for bus moderation


// Prototypes for General Functions
void BlinkForever(); // Ends execution
int sprintf ( char * str, const char * format, ... );


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
int SCLpin = 13;
int SDApin = 14;
int TalkPin = 23;  // This is the open-drain line for signaling i2c mastery
int The32kPin = 24;  // This is a 32k squarewave from the DS3231


// Battery monitor
float stateOfCharge = 0;

// FRAM and Unix time variables
unsigned int  framAddr;
unsigned long unixTime;
tmElements_t currentTime;
tmElements_t t;
int lastHour = 0;  // For recording the startup values
int lastDate = 0;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
unsigned int hourlyPersonCount = 0;  // hourly counter
unsigned int dailyPersonCount = 0;   //  daily counter
byte currentHourlyPeriod;    // This is where we will know if the period changed
byte currentDailyPeriod;     // We will keep daily counts as well as period counts
int setYear,setMonth,setDay,setHour,setMinute,setSecond;


// Variables for Simblee Display
uint8_t ui_RefreshButton;
uint8_t ui_ReturnButton;
uint8_t ui_StartStopSwitch;
uint8_t ui_StartStopStatus;
uint8_t ui_DebounceSlider;
uint8_t ui_SensitivitySlider;
uint8_t ui_UpdateButton;
uint8_t dateTimeField;
uint8_t hourlyField;
uint8_t dailyField;
uint8_t chargeField;
uint8_t menuBar;
char *titles[] = { "Current", "Daily", "Hourly", "Admin" };
uint8_t hourlyTitle;
uint8_t dailyTitle;
int currentScreen; // The ID of the current screen being displayed
uint8_t ui_setYear, ui_setMonth,ui_setDay,ui_setHour,ui_setMinute,ui_setSecond;

// Accelerometer Values
int debounce;               // This is a minimum debounce value - additional debounce set using pot or remote terminal
int accelInputValue;            // Raw sensitivity input (0-9);
byte accelSensitivity;               // Hex variable for sensitivity

// Variables for the control byte
// Control Register  (8 - 4 Reserved, 3-Start / Stop Test, 2-Set Sensitivity, 1-Set Delay)
byte signalDebounceChange = B0000001;
byte signalSentitivityChange = B00000010;
byte toggleStartStop = B00000100;
byte signalTimeChange = B00001000;
byte controlRegisterValue;


// Map Values
int numberDays = 10;
int numberHoursPerDay = 10;
int seedDayValue = 1000;
int seedHourValue = 100;
boolean firstTime = 1;

// include newlib printf float support (%f used in sprintf below)
asm(".global _printf_float");


// Add setup code
void setup()
{
    Wire.beginOnPins(SCLpin,SDApin);
    Serial.begin(9600);
    
    // Unlike Arduino Simblee does not pre-define inputs
    pinMode(TalkPin, INPUT);  // Shared Talk line
    pinMode(The32kPin, INPUT);   // Shared 32kHz line from clock


    if (fram.begin()) {  // you can stick the new i2c addr in here, e.g. begin(0x51);
        Serial.println("Found I2C FRAM");
    }
    else {
        Serial.println("No I2C FRAM found ... check your connections\r\n");
        BlinkForever();
    }

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
    
    
    // Reset the control values
    FRAMwrite8(CONTROLREGISTER, 0x0);
    
    // Set up the Simblee Mobile App
    SimbleeForMobile.deviceName = "Ulmstead";          // Device name
    SimbleeForMobile.advertisementData = "counts";  // Name of data service
    SimbleeForMobile.domain = "ulmstead.simblee.com";    // use a shared cache
    SimbleeForMobile.begin();
}

// Add loop code
void loop()
{
    // process must be called in the loop for SimbleeForMobile
    SimbleeForMobile.process();
    if (firstTime) {
        //MemoryMapReport();
        Serial.println("Ready to go....");
        firstTime = 0;
    }

}

void MemoryMapReport()  // Creates a memory map report on start
{
    Serial.println(" ");
    Serial.println("The first word - System Settings");
    Serial.print("  Version number: "); Serial.println(FRAMread8(VERSIONADDR));
    Serial.print("  Sensitivity: "); Serial.println(FRAMread8(SENSITIVITYADDR));
    Serial.print("  Debounce: "); Serial.println(FRAMread16(DEBOUNCEADDR));
    Serial.print("  Daily Pointer: "); Serial.println(FRAMread8(DAILYPOINTERADDR));
    Serial.print("  Hourly Pointer: "); Serial.println(FRAMread16(HOURLYPOINTERADDR));
    Serial.println(" ");
    Serial.println("The second word - Current Settings");
    Serial.print("  Current Hourly Count: "); Serial.println(FRAMread16(CURRENTHOURLYCOUNTADDR));
    Serial.print("  Current Daily Count: "); Serial.println(FRAMread16(CURRENTDAILYCOUNTADDR));
    Serial.print("  Current Count Time: "); Serial.println(FRAMread32(CURRENTCOUNTSTIME));
    Serial.print("  Current Time Converted: "); toArduinoDateTime(FRAMread32(CURRENTCOUNTSTIME));
    Serial.println(" ");
    //Serial.print("  Current Time Midnight:  "); toArduinoDateTime(findMidnight(FRAMread32(CURRENTCOUNTSTIME)));
    Serial.println(" ");
    Serial.println("The daily count words");
    for (int i=0; i<= numberDays-1; i++ )
    {
        Serial.print("  Day: "); Serial.print(i+1); Serial.print("  \t");
        Serial.print(FRAMread8((DAILYOFFSET+i)*WORDSIZE)); Serial.print("/");
        Serial.print(FRAMread8((DAILYOFFSET+i)*WORDSIZE+DAILYDATEOFFSET)); Serial.print("\t");
        Serial.print("Count: "); Serial.print(FRAMread16((DAILYOFFSET+i)*WORDSIZE+DAILYCOUNTOFFSET)); Serial.print("\t");
        Serial.print("Charge: "); Serial.print(FRAMread8((DAILYOFFSET+i)*WORDSIZE+DAILYBATTOFFSET)); Serial.println("%");
    }
    for (int i=0; i<= numberHoursPerDay*numberDays-1; i++ )
    {
        Serial.print("  TimeStamp: "); Serial.print(i+1); Serial.print("  \t");
        toArduinoDateTime(FRAMread32((HOURLYOFFSET+i)*WORDSIZE)); Serial.print("  \t");
        Serial.print("Count: "); Serial.print(FRAMread16((HOURLYOFFSET+i)*WORDSIZE+HOURLYCOUNTOFFSET)); Serial.print("\t");
        Serial.print("Charge: "); Serial.print(FRAMread8((HOURLYOFFSET+i)*WORDSIZE+HOURLYBATTOFFSET)); Serial.println("%");
    }
}


void SimbleeForMobile_onConnect()   // Actions to take once we get connected.
{
    /*
     * Callback when a Central connects to this device
     *
     * Reset the current screen to non being displayed
     */
    currentScreen = -1;
}


void SimbleeForMobile_onDisconnect()    // Can clean up resources once we disconnect
{
    /*
     * Callback when a Central disconnects from this device
     *
     * Not used in this sketch. Could clean up any resources
     * no longer required.
     */
}

void ui()   // The function that defines the iPhone UI
{
    Serial.print("The dimensions (H/W) of the screen are:");
    Serial.print(SimbleeForMobile.screenHeight);
    Serial.print(" / ");
    Serial.println(SimbleeForMobile.screenWidth);

    if(SimbleeForMobile.screen == currentScreen) return;

    currentScreen = SimbleeForMobile.screen;
    switch(SimbleeForMobile.screen)
    {
        case 1:
            createCurrentScreen();
            SimbleeForMobile.updateValue(hourlyField, FRAMread16(CURRENTHOURLYCOUNTADDR));
            SimbleeForMobile.updateValue(dailyField, FRAMread16(CURRENTDAILYCOUNTADDR));
            if ((controlRegisterValue & toggleStartStop) >> 2) {
                SimbleeForMobile.updateText(ui_StartStopStatus, "Running");
            }
            else SimbleeForMobile.updateText(ui_StartStopStatus, "Stopped");
            break;

        case 2:
            createDailyScreen();
            break;

        case 3:
            createHourlyScreen();
            break;
            
        case 4:
            createAdminScreen();
            if ((controlRegisterValue & toggleStartStop) >> 2) {
                SimbleeForMobile.updateText(ui_StartStopStatus, "Running");
            }
            else SimbleeForMobile.updateText(ui_StartStopStatus, "Stopped");
            debounce = FRAMread16(DEBOUNCEADDR);
            SimbleeForMobile.updateValue(ui_DebounceSlider, debounce);
            accelInputValue = map(FRAMread8(SENSITIVITYADDR),0,64,0,10);
            SimbleeForMobile.updateValue(ui_SensitivitySlider, accelInputValue);
            toDateTimeValues(FRAMread32(CURRENTCOUNTSTIME));
            
            break;
            
        default:
            Serial.print("ui: Uknown screen requested: ");
            Serial.println(SimbleeForMobile.screen);
    }
}

void ui_event(event_t &event)   // This is where we define the actions to occur on UI events
{
    if (event.id == ui_RefreshButton && event.type == EVENT_RELEASE)
    {
        printEvent(event);
        toArduinoDateTime(FRAMread32(CURRENTCOUNTSTIME));
        Serial.println("");
        SimbleeForMobile.updateValue(hourlyField, FRAMread16(CURRENTHOURLYCOUNTADDR));
        SimbleeForMobile.updateValue(dailyField, FRAMread16(CURRENTDAILYCOUNTADDR));
        if (batteryMonitor.getSoC() <= 100) {
            SimbleeForMobile.updateValue(chargeField, batteryMonitor.getSoC());
        }
    }
    else if (event.id == menuBar)
    {
        printEvent(event);
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
    else if (event.id == ui_StartStopSwitch && event.type == EVENT_RELEASE)
    {
        printEvent(event);
        FRAMwrite8(CONTROLREGISTER,toggleStartStop ^ controlRegisterValue);
        controlRegisterValue = FRAMread8(CONTROLREGISTER);
        Serial.println((controlRegisterValue & toggleStartStop) >> 2);
        if ((controlRegisterValue & toggleStartStop) >> 2) {
            SimbleeForMobile.updateText(ui_StartStopStatus, "Running");
        }
        else SimbleeForMobile.updateText(ui_StartStopStatus, "Stopped");
    }
    else if (event.id == ui_DebounceSlider)
    {
        debounce = event.value;
        Serial.print("debounce value =");
        Serial.println(debounce);
    }
    else if (event.id == ui_SensitivitySlider)
    {
        accelInputValue = event.value;
        Serial.print("sensitivty value =");
        Serial.println(accelInputValue);
    }
    else if (event.id == ui_UpdateButton && event.type == EVENT_RELEASE)
    {
        accelSensitivity = map(accelInputValue,0,10,0,64);
        DateTime t = DateTime(setYear,setMonth,setDay,setHour,setMinute,setSecond); // Convert to Unit Timx
        unsigned long unixTime = toUnixTime(t);  // Convert to UNIX Time
        if (FRAMread16(DEBOUNCEADDR) != debounce)
        {
            FRAMwrite16(DEBOUNCEADDR, debounce);
            FRAMwrite8(CONTROLREGISTER,signalDebounceChange |= controlRegisterValue);
            Serial.println("Updating debounce");
        }
        else if (FRAMread8(SENSITIVITYADDR)!=accelSensitivity)
        {
            FRAMwrite8(SENSITIVITYADDR, accelSensitivity);
            FRAMwrite8(CONTROLREGISTER,signalSentitivityChange |= controlRegisterValue);
            Serial.println("Updating sensitivity");
        }
        else if (FRAMread32(CURRENTCOUNTSTIME) != unixTime)
        {
            FRAMwrite32(CURRENTCOUNTSTIME, unixTime);   // Write to FRAM - this is so we know when the last counts were saved
            FRAMwrite8(CONTROLREGISTER,signalTimeChange |= controlRegisterValue);
            Serial.println("Updating time");
        }
        controlRegisterValue = FRAMread8(CONTROLREGISTER);
        Serial.print("Control Register Value =");
        Serial.println(controlRegisterValue);
    }
    else
    {
        Serial.print("Cound not find event.id = ");
        Serial.println(event.id);
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

    
    // The first control will be a switch
    controlRegisterValue = FRAMread8(CONTROLREGISTER);

    ui_StartStopStatus = SimbleeForMobile.drawText(200, 160, " ");

    ui_StartStopSwitch = SimbleeForMobile.drawButton(20,150,150, "Start/Stop");
    SimbleeForMobile.setEvents(ui_StartStopSwitch, EVENT_RELEASE);
    
    SimbleeForMobile.drawText(20,220,"0ms          Debounce        1000ms");
    ui_DebounceSlider = SimbleeForMobile.drawSlider(20,240,270,0,1000);
 
    SimbleeForMobile.drawText(20,280,"Max           Sensitivity         Min");
    ui_SensitivitySlider = SimbleeForMobile.drawSlider(20,300,270,0,10);

    SimbleeForMobile.drawText(20,340,"Set Date");
    ui_setYear = SimbleeForMobile.drawTextField(20,360,60,setYear,"year");
    ui_setMonth = SimbleeForMobile.drawTextField(100,360,60,setMonth,"mon");
    ui_setDay = SimbleeForMobile.drawTextField(180,360,60,setDay,"day");

    SimbleeForMobile.drawText(20,400,"Set Time");
    ui_setHour = SimbleeForMobile.drawTextField(20,420,60,setHour,"hr");
    ui_setMinute = SimbleeForMobile.drawTextField(100,420,60,setMinute,"min");
    ui_setSecond = SimbleeForMobile.drawTextField(180,420,60,setSecond,"sec");
    
    ui_UpdateButton = SimbleeForMobile.drawButton(90,500,150, "Update");
    SimbleeForMobile.setEvents(ui_UpdateButton, EVENT_RELEASE);
    
    SimbleeForMobile.endScreen();
    
}

void printEvent(event_t &event)     // Utility method to print information regarding the given event
{
    Serial.println("  Screen:");
    Serial.print("   Screen Value:");
    Serial.println(SimbleeForMobile.screen);
    Serial.print("   currentScreen = ");
    Serial.println(currentScreen);
                 
    
    Serial.println("  Event:");
    Serial.print("   ID: ");
    Serial.println(event.id);
    
    Serial.print("    Type: ");
    Serial.println(event.type);
    
    Serial.print("    Value: ");
    Serial.println(event.value);
    
    Serial.print("    Text: ");
    Serial.println(event.text);
    
    Serial.print("    Coords: ");
    Serial.print(event.x);
    Serial.print(",");
    Serial.println(event.y);  
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
    
    //setYear = timeElement.Year;
    //setMonth = timeElement.Month;
    //setDay = timeElement.Day;
    //setHour = timeElement.Hour;
    //setMinute = timeElement.Minute;
    //setSecond = timeElement.Second;

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
    Serial.print(dateTimePointer);
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
    int timeout = 10000;  // We will wait ten seconds then give up
    unsigned long startListening = millis();
    while(digitalRead(The32kPin)) {} // The Simblee will wait until the SQW pin goes low
    //Serial.println("Simblee a low SQW Pin");
    while (!digitalRead(TalkPin))  { // Only proceed once the TalkPin is high or we timeout
        //Serial.println("Simblee a Low Talk Pin");
        if (millis() >= timeout + startListening) return 0;  // timed out
    }
    //Serial.println("Simblee a High Talk Pin");
    pinMode(TalkPin,OUTPUT);  // Change to output
    digitalWrite(TalkPin,LOW);  // Claim the bus
    return 1;           // We have it
}

boolean GiveUpTheBus()
{

    pinMode(TalkPin,INPUT);  // Start listening again
    //Serial.println("Simblee gave up the Bus");

    return 1;
}
