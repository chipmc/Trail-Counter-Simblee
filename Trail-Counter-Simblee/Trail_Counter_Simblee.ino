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
#define VERSIONNUMBER 4       // Increment this number each time the memory map is changed
#define WORDSIZE 8            // For the Word size
#define PAGESIZE 4096         // Memory size in bytes / word size - 256kb FRAM
// First Word - 8 bytes for setting global values
#define DAILYOFFSET 2        // First word of daily counts
#define HOURLYOFFSET 16        // First word of hourly counts (remember we start counts at 1)
#define DAILYCOUNTNUMBER 14    // used in modulo calculations - sets the # of days stored
#define HOURLYCOUNTNUMBER 4078 // used in modulo calculations - sets the # of hours stored - 256k (4096-14-2)
#define VERSIONADDR 0x0       // Memory Locations By Name not Number
#define SENSITIVITYADDR 0x1   // For the 1st Word locations
#define DEBOUNCEADDR 0x2
#define DAILYPOINTERADDR 0x3
#define HOURLYPOINTERADDR 0x4
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
#include <Wire.h>
#include <Time.h>               // This library brings Unix Time capabilities
#include "RTClib.h"             // Adafruit's library which includes the DS3231
#include "MAX17043.h"           // Drives the LiPo Fuel Gauge
#include "Adafruit_FRAM_I2C.h"   // Note - had to comment out the Wire.begin() in this library
#include "SimbleeForMobile.h"
#include <stdlib.h>


// Prototypes
// Prototypes From the included libraries
RTC_DS3231 rtc;                               // Init the DS3231
MAX17043 batteryMonitor;                      // Init the Fuel Gauge
Adafruit_FRAM_I2C fram = Adafruit_FRAM_I2C(); // Init the FRAM

// Prototypes From my functions
// Prototypes for FRAM Functions
unsigned long FRAMread32(unsigned long address); // Reads a 32 bit word
int FRAMread16(unsigned int address); // Reads a 16 bit word
uint8_t FRAMread8(unsigned int address);  // Reads a 8 bit word
void FRAMwrite8(unsigned int address, uint8_t value); //Writes a 32-bit word
void ResetFRAM();  // This will reset the FRAM - set the version and preserve delay and sensitivity
void MemoryMapReport(); // Creates a memory map report on start

// Prototypes for i2c functions
boolean GiveUpTheBus(); // Give up the i2c bus
boolean TakeTheBus(); // Take the 12c bus
void enable32Khz(uint8_t enable);  // Need to turn on the 32k square wave for bus moderation


// Prototypes for General Functions
void BlinkForever(); // Ends execution


// Prototypes for Date and Time Functions
void toArduinoDateTime(unsigned long unixT, int xAxis, int yAxis);   // Converts to date time for the UI
void toArduinoHour(unsigned long unixT, int xAxis, int yAxis);  // Just gets the hour



// Prototypes for the Simblee
void SimbleeForMobile_onConnect();   // Actions to take once we get connected
void SimbleeForMobile_onDisconnect();    // Can clean up resources once we disconnect
void ui();   // The function that defines the iPhone UI
void ui_event(event_t &event);   // This is where we define the actions to occur on UI events
void createCurrentScreen(); // This is the screen that displays current status information
void createDailyScreen(); // This is the screen that displays current status information
void createHourlyScreen(); // This is the screen that displays current status information


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
DateTime currentTime;
DateTime t;
int lastHour = 0;  // For recording the startup values
int lastDate = 0;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
unsigned int hourlyPersonCount = 0;  // hourly counter
unsigned int dailyPersonCount = 0;   //  daily counter
byte currentHourlyPeriod;    // This is where we will know if the period changed
byte currentDailyPeriod;     // We will keep daily counts as well as period counts

// Variables for Simblee Display
uint8_t ui_button;
uint8_t dateTimeField;
char dateTimeArray[19] = "Test Array";
char *dateTimePointer;
uint8_t hourlyField;
uint8_t dailyField;
uint8_t chargeField;
uint8_t menuBar;
char *titles[] = { "Current", "Daily", "Hourly" };
uint8_t hourlyTitle;
uint8_t dailyTitle;
int currentScreen; // The ID of the current screen being displayed

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
        MemoryMapReport();
        firstTime = 0;
    }

}

void MemoryMapReport()  // Creates a memory map report on start
{
    Serial.println(" ");
    Serial.println("The first word - System Settings");
    Serial.print("  Version number: "); Serial.println(FRAMread8(VERSIONADDR));
    Serial.print("  Sensitivity: "); Serial.println(FRAMread8(SENSITIVITYADDR));
    Serial.print("  Debounce: "); Serial.println(FRAMread8(DEBOUNCEADDR));
    Serial.print("  Daily Pointer: "); Serial.println(FRAMread8(DAILYPOINTERADDR));
    Serial.print("  Hourly Pointer: "); Serial.println(FRAMread16(HOURLYPOINTERADDR));
    Serial.println(" ");
    Serial.println("The second word - Current Settings");
    Serial.print("  Current Hourly Count: "); Serial.println(FRAMread16(CURRENTHOURLYCOUNTADDR));
    Serial.print("  Current Daily Count: "); Serial.println(FRAMread16(CURRENTDAILYCOUNTADDR));
    Serial.print("  Current Count Time: "); Serial.println(FRAMread32(CURRENTCOUNTSTIME));
    Serial.print("  Current Time Converted: "); toArduinoDateTime(FRAMread32(CURRENTCOUNTSTIME), 0, 0);
    Serial.println(" ");
    //Serial.print("  Current Time Midnight:  "); toArduinoDateTime(findMidnight(FRAMread32(CURRENTCOUNTSTIME)), 0, 0);
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
        toArduinoDateTime(FRAMread32((HOURLYOFFSET+i)*WORDSIZE),0,0); Serial.print("  \t");
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
            break;

        case 2:
            createDailyScreen();
            break;

        case 3:
            //workingSplash();
            createHourlyScreen();
            break;

        default:
            Serial.print("ui: Uknown screen requested: ");
            Serial.println(SimbleeForMobile.screen);
    }
}

void ui_event(event_t &event)   // This is where we define the actions to occur on UI events
{
    if (event.id == ui_button)
    {
        if (event.type == EVENT_PRESS) {
            //TakeTheBus();
            //unsigned long unixTime = rtc.now();
            //GiveUpTheBus();
            toArduinoDateTime(now(), 50, 140);
            SimbleeForMobile.updateValue(hourlyField, FRAMread16(CURRENTHOURLYCOUNTADDR));
            SimbleeForMobile.updateValue(dailyField, FRAMread16(CURRENTDAILYCOUNTADDR));
            if (batteryMonitor.getSoC() <= 100) {
                SimbleeForMobile.updateValue(chargeField, batteryMonitor.getSoC());
            }
        }
        else if (event.type == EVENT_RELEASE) {
        }

    }

    if(event.id == menuBar) {
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
        }
    }
}


void createCurrentScreen() // This is the screen that displays current status information
{
    //TakeTheBus();
    //unsigned long unixTime = rtc.now();
    //GiveUpTheBus();
    SimbleeForMobile.beginScreen(WHITE, PORTRAIT); // Sets orientation
    menuBar = SimbleeForMobile.drawSegment(30, 90, 240, titles, countof(titles));
    SimbleeForMobile.updateValue(menuBar, 0);
    dateTimeField = SimbleeForMobile.drawText(50, 140, " ");
    toArduinoDateTime(now(), 50, 140);
    SimbleeForMobile.drawText(50, 160, "Hourly Count:");
    hourlyField = SimbleeForMobile.drawText(210,160,FRAMread16(CURRENTHOURLYCOUNTADDR));
    SimbleeForMobile.drawText(50, 180, "Daily Count:");
    dailyField =   SimbleeForMobile.drawText(210,180,FRAMread16(CURRENTDAILYCOUNTADDR));
    SimbleeForMobile.drawText(50, 200, "State of Charge:");
    if (batteryMonitor.getSoC() >= 105) {
        chargeField =   SimbleeForMobile.drawText(210,200,"Error");
    }
    else {
        chargeField =   SimbleeForMobile.drawText(210,200,batteryMonitor.getSoC());
        SimbleeForMobile.drawText(230,200," %");
    }

    // we need a momentary button (the default is a push button)
    ui_button = SimbleeForMobile.drawButton(110, 260, 90, "Refresh");
    SimbleeForMobile.setEvents(ui_button, EVENT_PRESS | EVENT_RELEASE);

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
    menuBar = SimbleeForMobile.drawSegment(30, 90, 240, titles, countof(titles));
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
    menuBar = SimbleeForMobile.drawSegment(30, 90, 240, titles, countof(titles));
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


// Note - have to hard code the size here due to this issue - http://www.microchip.com/forums/m501193.aspx
void ResetFRAM()  // This will reset the FRAM - set the version and preserve delay and sensitivity
{
    Serial.println("Resetting Memory");
    for (unsigned long i=3; i < 32768; i++) {  // Start at 3 to not overwrite debounce and sensitivity
        fram.write8(i,0x0);
        //Serial.println(i);
        if (i==8192) Serial.println(F("25% done"));
        if (i==16384) Serial.println(F("50% done"));
        if (i==(24576)) Serial.println(F("75% done"));
        if (i==32767) Serial.println(F("Done"));
    }
    fram.write8(VERSIONADDR,VERSIONNUMBER);  // Reset version to match #define value for sketch
}

void toArduinoDateTime(unsigned long unixT, int xAxis, int yAxis)   // Converts to date time for the UI
{
    int columnWidth = 6;
    dateTimePointer = dateTimeArray;
    DateTime timeElement(unixT);

    if(timeElement.month() < 10) {
        dateTimeArray[0] = '0';
        dateTimeArray[1] = timeElement.month()+48;  // Stupid but the +48 gives the right ASCII code
    }
    else {
        dateTimeArray[0] = int(timeElement.month()/10)+48;
        dateTimeArray[1] = (timeElement.month()%10)+48;
    }
    dateTimeArray[2] =  '/';
    if(timeElement.day() < 10) {
        dateTimeArray[3] = '0';
        dateTimeArray[4] = timeElement.day()+48;  // Stupid but the +48 gives the right ASCII code
    }
    else {
        dateTimeArray[3] = int(timeElement.day()/10)+48;
        dateTimeArray[4] = (timeElement.day()%10)+48;
    }
    dateTimeArray[5] =  '/';
    int currentYear = timeElement.year();
    if(currentYear < 10) {
        dateTimeArray[6] = '0';
        dateTimeArray[7] = currentYear+48;  // Stupid but the +48 gives the right ASCII code
    }
    else {
        dateTimeArray[6] = int(currentYear/10)+48;
        dateTimeArray[7] = (currentYear%10)+48;
    }
    dateTimeArray[8] =  ' ';
    if(timeElement.hour() < 10) {
        dateTimeArray[9] = '0';
        dateTimeArray[10] = timeElement.hour()+48;  // Stupid but the +48 gives the right ASCII code
    }
    else {
        dateTimeArray[9] = int(timeElement.hour()/10)+48;
        dateTimeArray[10] = (timeElement.hour()%10)+48;
    }
    dateTimeArray[11] =  ':';
    if(timeElement.minute() < 10) {
        dateTimeArray[12] = '0';
        dateTimeArray[13] = timeElement.minute()+48;  // Stupid but the +48 gives the right ASCII code
    }
    else {
        dateTimeArray[12] = int(timeElement.minute()/10)+48;
        dateTimeArray[13] = (timeElement.minute()%10)+48;
    }
    dateTimeArray[14] =  ':';
    if(timeElement.second() < 10) {
        dateTimeArray[15] = '0';
        dateTimeArray[16] = timeElement.second()+48;  // Stupid but the +48 gives the right ASCII code
    }
    else {
        dateTimeArray[15] = int(timeElement.second()/10)+48;
        dateTimeArray[16] = (timeElement.second()%10)+48;
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
