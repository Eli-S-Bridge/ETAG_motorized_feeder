
/*
  Data logging sketch for the ETAG RFID Reader
  Version 1.1
  Code by:
   Alexander Moreno
   David Mitchell
   Eli Bridge
   Jay Wilhelm
   May-2018

  Licenced in the public domain

  REQUIREMENTS:
  Power supply for the board should come from the USB cable or a 5V battery or DV power source.
  A 3.3V CR1025 coin battery should be installed on the back side of the board to maintain the date
      and time when the board is disconnected from a primary power source.

  FLASH MEMORY STRUCTURE:

  The onboard flash memory is divided into pages of 528 bytes each. There are probably several thousand pages.
  Page 0 is reserved for RFID tag codes
  Page 1 is reserved for parameters and counters
    bytes 0-2 - first four bytes are the current backup memory address
    byte 3 - Set to 0xAA once the memory address counter is intialized. (important mainly for the very first initialization process with a completely blank Flash memory)
    bytes 4-7 - next four bytes are for the reader ID charaters
    byte 8 - Set to 0xAA once the Reader ID is established - if this byte is anything other than 0xAA then the Reader ID will be set to the default
    byte 9 = tagCount (the number of tags stored on page 0)
    byte 10 = feeder mode
  The rest is for backup memory.

  Change log
  5-4-18 - Added interrupt driven RFID reader (jaywilhelm)
*/

//***********INITIALIZE INCLUDE FILES AND I/O PINS*******************
//#include "ManchesterDecoder.h" //Interrupt driven RFID decoder
#include "SparkFun_RV1805.h"
#//include "ETAGLowPower.h"
#include <Wire.h>        //include the standard wire library - used for I2C communication with the clock
#include <SD.h>          //include the standard SD card library
#include <SPI.h>

#define serial SerialUSB     //Designate the USB connection as the primary serial comm port
#define DEMOD_OUT_PIN   30   //(PB03) this is the target pin for the raw RFID data
#define SHD_PINA         8   //(PA06) Setting this pin high activates the primary RFID circuit (only one can be active at a time)
#define SHD_PINB         9    //(PA07) Setting this pin high activates the seconday RFID circuit (only one can be active at a time)
#define MOD_PIN          0    //not used - defined as zero
#define READY_CLOCK_PIN  0    //not used - defined as zero
#define SDselect         7    //Chip select for SD card - make this pin low to activate the SD card, also the clock interupt pin
#define csFlash         2   //Chip select for flash memory
#define LED_RFID        31    //Pin to control the LED indicator.  
#define interruptPin    7
#define MOTPWM          0  //motor pulse width
#define MOTL            1  //motor left
#define MOTR            2  //motor right
#define mStby           3  //motor controller standby
#define mSwitch         13  //motor switch
#define INT1            7

//ManchesterDecoder gManDecoder1(DEMOD_OUT_PIN, SHD_PINA, ManchesterDecoder::EM4095, CollectedBitMinCount);
//ManchesterDecoder gManDecoder2(DEMOD_OUT_PIN, SHD_PINB, ManchesterDecoder::EM4095, CollectedBitMinCount);

//RTC_RV1805 rtc;
RV1805 rtc;

//************************* initialize variables******************************
uint32_t tagNo;                    //four least significant hexidecimals in 5 number tag code
uint32_t tagNo2;                   //four least significant hexidecimals in 5 number tag code
uint32_t tagNoA;                   //four least significant hexidecimals in 5 number tag code - used for comparisons associated with RFID circuit 1 and delayTime
unsigned long hun;                  //for the millisecond counter and use with delayTime
unsigned long pastHun;              //for the millisecond counter and use with delayTime

byte flashArray[528];              //Large array of 528 bytes for writing a full page to onboard flash memory
byte tagCount;                     //keeps track of number of stored tags
byte byte0;                        //general purpose reusable variable
long long0;                        //general purpose reusable variable
unsigned int pageAddress;          //page address for flash memory
unsigned int byteAddress;          //byte address for flash memory
union flashMem {                   //Union variable for constructing instructions to flash memory
  unsigned long flashAddress;
  byte flashAddrByte[4];
};

byte match;                       // used to determine if tags match.
byte RFcircuit = 1;               //Used to determine which RFID circuit is active. 1 = primary circuit, 2 = secondary circuit.
byte ss, mm, hh, da, mo, yr;          //Byte variables for storing date/time elements
String sss, mms, hhs, das, mos, yrs;  //String variable for storing date/time text elements
String currentDate; //Get the current date in mm/dd/yyyy format (we're weird in the US)
String currentTime; //Get the time
String currentDateTime;
String timeString;                    //String for storing the whole date/time line of data
char incomingByte = 0;                 //Used for incoming serial data
unsigned int timeIn[12];              //Used for incoming serial data during clock setting
byte menu;
char feedMode;
char feederID[4];
String feederNameStr;
String fileNameSD;

//tag reading state variables
byte longPulseDetected = 0;
byte pastPulseLong = 0;
byte RFIDbitCounter;
byte RFIDbyteCounter;
byte RFIDbytes[11];
byte paritybits[6];
byte OneCounter;      //count number of consecutive 1s
unsigned int pulseCount;
unsigned int parityFail;
byte rParity;

//Global variable for tag codes
String RFIDstring;                 //Stores the TagID as a 10 character string
byte RFIDtagUser = 0;              //Stores the first (most significant) byte of a tag ID (user number)
unsigned long RFIDtagNumber = 0;   //Stores bytes 1 through 4 of a tag ID (user number)
byte RFIDtagArray[5];              //Stores the five individual bytes of a tag ID.

//********************CONSTANTS (SET UP LOGGING PARAMETERS HERE!!)*******************************
const byte checkTime = 30;                //How long in milliseconds to check to see if a tag is present
const unsigned int pollTime1 = 200;       //How long in milliseconds to try to read a tag on circuit 1 if a tag was initially detected
const unsigned int delayTime = 5000;      //Minimim time in milliseconds between recording the same tag twice in a row (only applies to data logging--other operations are unaffected)
const unsigned long pauseTime = 250;      //How long in milliseconds to wait between polling intervals

const byte slpH = 21;                            //When to go to sleep at night - hour
const byte slpM = 00;                            //When to go to sleep at night - minute
const byte wakH = 07;                            //When to wake up in the morning - hour
const byte wakM = 00;                            //When to wake up in the morning - minute
const byte slpInterval = 25;                     //How many seconds to sleep at a time, must be less than 1 min.
const unsigned int onTime = 1000;                //how man MILLISECONDS to stay on between sleep intervals
const unsigned int slpTime = slpH * 100 + slpM;  //Combined hours and minutes for sleep time
const unsigned int wakTime = wakH * 100 + wakM;  //Combined hours and minutes for wake time

unsigned int cycleCount = 0;                     //counts read cycles
unsigned int stopCycleCount = 100;               //How many read cycles to maintain serial comminications
byte doorState = 1;                              //Status of the motorized door. 0 = open, 1 = closed.
byte readFlashByte(unsigned long fAddress);
bool writeFlashByte(unsigned long fAddress, byte fByte);
unsigned long getFlashAddr();
void writeFlashAddr(unsigned long fAddress);
void setClk();
void dumpMem();
void printDirectory(File dir, int numTabs);

//*******************************SETUP**************************************
void setup() {  // This function sets everything up for logging.
  //Establish startup settings and I/O pins

  Wire.begin();  //Enable I2C communication

  pinMode(LED_RFID, OUTPUT);     // pin for controlling the on-board LED
  digitalWrite(LED_RFID, HIGH);  // turn the LED off (LOW turns it on)

  //SD card and flash memory chip select pins - these pins enable SPI communication with these peripherals
  pinMode(SDselect, OUTPUT);     // Chip select pin for SD card must be an output
  pinMode(csFlash, OUTPUT);      // Chip select pin for Flash memory
  digitalWrite(SDselect, HIGH);  //Make both chip selects high (not selected)
  digitalWrite(csFlash, HIGH);
  //gManDecoder1.DisableMonitoring();
  //gManDecoder2.DisableMonitoring();

  //Pins for controlling the RFID circuits
  pinMode(SHD_PINA, OUTPUT);      // Make the primary RFID shutdown pin an output.
  digitalWrite(SHD_PINA, HIGH);   // turn the primary RFID circuit off (LOW turns on the EM4095)
  pinMode(SHD_PINB, OUTPUT);      // Make the secondary RFID shutdown pin an output.
  digitalWrite(SHD_PINB, HIGH);   // turn the secondary RFID circuit off (LOW turns on the EM4095)

  //Motor control pins
  pinMode(mStby, OUTPUT);         // pin that can put the motor controller in low-power standby mode
  digitalWrite(mStby, LOW);       // motor in standby mode
  pinMode(MOTR, OUTPUT);          // Right motor control pin
  digitalWrite(MOTR, LOW);        // turn motor off
  pinMode(MOTL, OUTPUT);          // Left motor control pin
  digitalWrite(MOTL, LOW);        // turn motor off
  pinMode(mSwitch, INPUT_PULLUP); // motor switch enabled as input with internal pullup resistor

  //establish a a couple of parameters for low power sleep mode
  SYSCTRL->XOSC32K.bit.RUNSTDBY = 1;                 // Set the XOSC32K to run in standby (not sure why this is needed)
  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;                 //Set sleep mode

  //Try to initiate a serial connection
  serial.begin(9600);

  //Slow flashing LED to create a time window for serial connection.
  blinkLED(3);

  //Start real time clock interfcae and get the current time
  if (rtc.begin() == false) serial.println("Something wrong with clock"); //Try initiation and report if something is wrong
  if (rtc.updateTime() == false) serial.print("RTC failed"); //Updates the time variables from RTC, report if there's a failure
  serial.println("Clock initialized and set to....");                        //Serial message for clock output
  showTime();                                                                //Function that displays the clock time

  //Set up SD card communication
  serial.println("Initializing SD card...");                                 //message to user
  if (!SD.begin(SDselect)) {
    serial.println("SD card failed, or not present"); //Initiate the SD card function with the pin that activates the card SD card error message
    blipLED(10000);
  } else {
    serial.println("SD card online.");
    //      serial.println("SD Contents");   //Show entire contents of SD card Root directory (optional)
    //      File root = SD.open("/");        //get the contents of the root folder
    //      printDirectory(root, 0);         //display all of it.
  }
  digitalWrite(SDselect, HIGH); //SD card turned off for now

  //Set up communication with the flash memory
  SPI.begin();                                                       //Enable SPI communication for Flash Memory
  unsigned long fAddressEnd2 = getFlashAddr();                       // get flash address
  serial.print("The flash address is ");
  serial.println(fAddressEnd2, BIN);
  serial.print("checking if Flash Memory is initialized: ");         //It is neccessary to initialize the flash memory on the first startup
  byte0 = readFlashByte(0x00000403);                                 //Read a particular byte from the flash memory
  serial.println(byte0, HEX);                                        //Display the byte
  if (byte0 != 0xAA) {                                               //If the byte is 0xFF then the flash memory needs to be initialized
    writeFlashByte(0x00000403, 0xAA);                                //Write a different byte to this memory location
    serial.print("Initializing Flash Memory: ");                     //Message
    writeFlashAddr(0x00000800);                                      //Set initial flash memory address for logging data to page 2, byte address 0
  }
  serial.println("Flash Memory IS initialized.");                    //Confirm initialization
  if (getFlashAddr() < 0x00000800) writeFlashAddr(0x00000800);       //Fixes flash memory address if it get corrupted.

  //checkSwitch();


  //Initialize motorized door
  serial.print("Initializing motor....");  //Message to user
  motInit();                               //Function that allows the door system to recognize its current position and move to the closed position
  //serial.print("Current feed mode: ");     //Display feeder mode
  //serial.println(feedMode);

  //Display options menu to user
  menu = 1;
  while (menu == 1) {                                             //Keep displaying menu until an exit condition happens (i.e. menu is set to 0)
    serial.println();                                             //Skip a line
    if (rtc.updateTime() == false) {
      serial.print("RTC failed"); //Updates the time variables from RTC
    }
    tagCount = readFlashByte(0x00000409);
    showTime();                                                   //Show the current time
    getFeederID();
    feederNameStr = String(feederID);
    fileNameSD = String(feederNameStr + "DATA.TXT");
    serial.print("Feeder ID: ");
    for (int n = 0; n < 4; n++) {           //loop to print out ID
      serial.print(feederID[n]);
    }
    serial.println();
    feedMode = getMode();                    //Get and show the feeder mode stored in the flash memory.
    serial.print(tagCount);
    serial.println(" tags in memory");
    serial.println();
    serial.println("What to do? (input capital letters)");        //Ask the user for instruction and display the options
    serial.println("    C = set clock");
    serial.println("    I = set feeder ID");
    serial.println("    B = Display backup memory (lowercase to see entire flash memory)");
    serial.println("    W = Write backup memory to SD card (lowercase to write entire flash memory)");
    serial.println("    E = Erase (reset) backup memory");
    serial.println("    O = set feeder mode to 'OPEN'");
    serial.println("    A = set feeder mode to 'ALL TAGGED'");
    serial.println("    T = set feeder mode to 'TARGETED'");
    serial.println("    L = Load Tag IDs");                     //Transfer tag IDs from a file on the SD card to the flash memory.
    serial.println("    S = Show current Tag IDs");             //Read and display Tag IDs from the flash memory
    serial.println("    D = test Door");
    serial.println("    Anything else = start logging");
    unsigned int serDelay = 0;                                         //If there's no response then eventually move on and just start logging
    while (serial.available() == 0 && serDelay < 10000) { //wait about 10 seconds for a user response
      delay(1);                                           //delay 1 ms
      serDelay++;                                         //add to the delay count
    }
    if (serial.available()) {                                             //If there is a response then perform the corresponding operation
      incomingByte = serial.read();                                       //get the entry from the user
      switch (incomingByte) {                                             //execute whatever option the user selected
        case 'C': {                                                         //option to set clock
            inputTime();                                                    //  calls function to get time values from user
            if (rtc.setTime(0, ss, mm, hh, da, mo, yr, 1) == false) {       //  attempt to set clock with input values
              Serial.println("Something went wrong setting the time");
            }   //  show error message if something went wrong
            break;                                                        //  break out of this option, menu variable still equals 1 so the menu will display again
          }
        case 'I': {
            inputID();   //  calls function to get a new feeder ID
            break;       //  break out of this option, menu variable still equals 1 so the menu will display again
          }
        case 'B': {
            dumpMem(0);  //display backup memory; calls function dumpmem(); break and show menu again
            break;
          }
        case 'b': {
            dumpMem(1);  //display backup memory; calls function dumpmem(); break and show menu again
            break;
          }
        case 'W': {
            writeMem(0);  //display backup memory; calls function dumpmem(); break and show menu again
            break;
          }
        case 'w': {
            writeMem(1);  //display backup memory; calls function dumpmem(); break and show menu again
            break;
          }
        case 'T': {
            feedMode = setMode('T');  //set feeder mode to T: feed only targeted birds; break and show menu again
            break;
          }
        case 'A': {
            feedMode = setMode('A');  //set feeder mode to A: feed all tagged birds; break and show menu again
            break;
          }
        case 'O': {
            feedMode = setMode('O');  //set feeder mode to O: feeder remains open at all times; break and show menu again
            break;
          }
        case 'E': {
            writeFlashAddr(0x0800);                                 //reinitialize flash memory by setting memory address to byte 0 of page 2
            getFlashAddr();                                         //  read and show new flash address to confirm
            break;
          }                                                 //  break out of this option, menu variable still equals 1 so the menu will display again
        case 'L': {
            transferTags();                                          //Transfer tag data from the SD card to the flash memory
            showTags();
            break;
          }
        case 'S': {
            showTags();
            break;
          }
        case 'D': {                                                         //test motor (not implemented yet)
            doorOpen();
            delay(1000);
            doorClose();
            break;
          }
          break;
        default:
          menu = 0;            //Any other entries - set menu to 0 and break from switch function. Move on to logging data
          break;
      }
    } else {                 //if there is no response to the request for input then set menu to 1 to exit the loop and start logging
      menu = 0;
    }
  }

  //Prepare for data logging
  checkDataFile(fileNameSD);    //Check if the proper data file exists on the SD card

  RFcircuit = 1;                              //Indicates that the reader should start with the primary RFID circuit
  //RFcircuit = 2;                            //Indicates that the reader should start with the secondary RFID circuit
  if (feedMode == 'O') doorOpen();            //open door if feedmode is "O"

  pastHun = 0;
  serial.println("Scanning for tags...\n");   //message to user

}   //end setup




//******************************MAIN PROGRAM*******************************

void loop() {  //This is the main function. It loops (repeats) forever.

  if (rtc.updateTime() == false) {
    serial.print("RTC failed at beginning of loop "); //Updates the time variables from RTC
  }
  showTime();
  unsigned int curTime = rtc.getHours() * 100 + rtc.getMinutes(); //combine hours and minutes into one variable
  if (curTime == slpTime) {                 //Designated go to sleep time
    digitalWrite(SHD_PINB, HIGH);           //shut down both RFID readers
    digitalWrite(SHD_PINA, HIGH);
    //serial.println("seting alarm");

    rtc.setAlarm(00, wakM, wakH, 1, 1);   // second, minute, hour, date, month
    rtc.enableInterrupt(INTERRUPT_AIE);
    rtc.setAlarmMode(4);

    //rtc.setRptTimer(slpInterval, 2);      //set timer for repeat alarm every x seconds
    //rtc.startTimer();                       //start the timer
    saveLogSD("SLEEP STARTED");             //note for the log
    pinMode(SDselect, INPUT);               //make the SD card pin an input so it can serve as interrupt pin
    //while (curTime != wakTime) {            //loop through periodic checks until the wake up time rolls around
    lpSleep();                            //call sleep funciton
    //............//                      //sleep happens here
    blipLED(30);                          //blink indicator - processor reawakened
    //      if (rtc.updateTime() == false) {
    //        serial.print("RTC failed on wake from long sleep "); //Updates the time variables from RTC
    //      }
    //      curTime = rtc.getHours() * 100 + rtc.getMinutes();            //calculate curTime for comparison with wakTime
    //    }
    rtc.stopTimer();
    pinMode(SDselect, OUTPUT);     // Chip select pin for SD card must be an output for writing to SD card
    SD.begin(SDselect);
    saveLogSD("SLEEP ENDED");
  }
  //if(cycleCount < stopCycleCount){
  serial.print("Scanning RFID circuit "); //Message part 1: Tell the user which circuit is active
  serial.println(RFcircuit);              //Message part 2: show the active RF circuit
  //}
  tagNo2 = 0xFFFFFFFF;                    //Load a dummy (impossible) code into the TagNo2 variable to indicate that no tag has yet been read on this attempt
  byte goneCount = 0;

  digitalWrite(SHD_PINA, LOW);                            //Turn on primary RFID circuit
  digitalWrite(SHD_PINB, HIGH);                           //Turn off Secondary RFID circuit

  if (FastRead(DEMOD_OUT_PIN, SHD_PINA, checkTime, pollTime1) == 1) {

    //The following is executed if a tag is detected
    goneCount = 0;                                                //Reset goneCount to zero - a tag is present
    processTag();
    serial.print("tag read. old tag is ");                        //Optional display -- this will display multiple times as long a tag is present
    serial.println(tagNo2, HEX);                                  //Disply previously read tag ID
    if (rtc.updateTime() == false) {
      serial.print("RTC failed after tag read "); //Updates the time variables from Real Time Clock
    }
    tagNo = RFIDtagNumber;                                //Stores the 4 least significant tag ID numbers - good for all tag comparisons I think.
    if (feedMode != 'O' && tagNo != tagNo2) {                     //If the feed mode is A or T and we have not just read the same tag again, then we may need to open the feeder door
      if (feedMode == 'A') {                                    //Feed all tagged birds -- open the feeder regardless of tag number and stay open until no tags are present.
        if (doorState == 1) {
          doorOpen(); //If the door is closed call subroutine to open the door
        }
      }
      if (feedMode == 'T') {                                    //Feed only selected birds -- check the tag number agains the list and open the door if called for
        //serial.println("feedmode T: check tag.");
        if (checkTag(tagNo) | checkTag(tagNo)) {                //Call function to check the tag against a list. If it returns true then the door should be opened - call function twice because it does not work sometimes when low power mode is enabled
          if (doorState == 1) {
            doorOpen(); //If the door is closed call subroutine to open the door
          }
        }
        getFlashAddr();                           //For some reason the checkTag function derails the getFlashAddr function. The work around is to just run the getFlashAddr funciton once here
      }
    }

    if (tagNo != tagNo2) {                    //Execute the following data logging steps each time a NEW tag is read--That is, don't repeatedly log data while a bird continues to stand on the perch
      //if(cycleCount < stopCycleCount){
      serial.print(RFIDstring);                        //Call a subroutine to display the tag data via serial USB
      serial.print(" detected on antenna ");  //Message part 1: add a note about which atenna was used
      serial.print(RFcircuit);                //Message part 2: Which antenna
      serial.print(" at ");                   //Message part 3
      //}
      showTime();                             //Message part 4: display the time
      flashLED();                             //Flash the LED briefly to indicate a tag read
      hun =  (8640000 * rtc.getDate()) + (360000 * rtc.getHours()) +  (6000 * rtc.getMinutes()) + (100 * rtc.getSeconds()) + rtc.getHundredths();
      serial.println(hun, DEC);
      serial.println(pastHun, DEC);
      serial.println(hun - pastHun, DEC);
      serial.println(delayTime / 10, DEC);
      if (((hun - pastHun) > (delayTime / 10)) | (tagNoA != tagNo)) {
        serial.print("Logging data to ");
        serial.println(fileNameSD);
        logRFID_To_SD(RFIDstring, currentDateTime, fileNameSD);        //Call function to log data to the SD card
        writeRFID_To_FlashLine();              //Call function to log to backup memory
        tagNoA = tagNo;                        //for keeping track of delaytime.
        pastHun = hun;                  //for keeping track of delaytime.
      }
      tagNo2 = tagNo;                          //set tagNo2 to current tag number for future comparisons

    }
    //Wait for tag to leave
    while (tagNo2 == tagNo) {
      if (FastRead(DEMOD_OUT_PIN, SHD_PINA, 50, 300) == 0) {  //Use slightly longer check and poll times...
        tagNo = 0xFFFFFFFF;                                   //clear tagNo if no tag is read - this will exit the while loop
      } else {
        processTag();
        tagNo = RFIDtagNumber;
        //if(cycleCount < stopCycleCount){
        serial.print("tag still present ");
        serial.println(tagNo, HEX);
        //}
      }
      delay(100);
    }
    if (feedMode != 'O') doorClose();
  }

  else {
    //Do the following if a tag is not detected       //What to do if a tag is NOT read
    //serial.println("No tag.......");
    if (doorState == 0 && feedMode != 'O') {
      doorClose(); //   Check the state of the door and whether it should remain open - if it is open and should not remain open, then close it
    }
    tagNo2 = 0xFFFFFFFF;                            //   Set dummy tagID to FFFFFFFFFF indicate no bird is currently present
  }                                                 //   End of if statement (What to do if a tag is NOT read)


  //  delay(pauseTime);               //pause between polling attempts

  if (cycleCount < stopCycleCount) {
    cycleCount++;
    delay(pauseTime);               //pause between polling attempts
  } else {
    byte pauseInterval = (pauseTime * 64) / 1000;
    //serial.print("pause interval: ");
    //serial.println(pauseInterval, DEC);
    rtc.setRptTimer(pauseInterval, 1);      //set timer and use frequency of 64 Hz
    pinMode(SDselect, INPUT);               //make the SD card pin an input so it can serve as interrupt pin
    rtc.startTimer();                       //start the timer
    lpSleep();                            //call sleep funciton
    //blipLED(30);                          //blink indicator - processor reawakened
    rtc.stopTimer();
    pinMode(SDselect, OUTPUT);     // Chip select pin for SD card must be an output for writing to SD card
    SD.begin(SDselect);
  }

}// end void loop


//*********************SUBROUTINES*************************//
////The Following are all subroutines called by the code above.//////////////

//byte FastRead(int demodOutPin, int shutDownPin, byte checkDelay, unsigned int readTime) {
//  byte randNumber = random(1,40);
//  serial.println(randNumber, DEC);
//  //BlinkLED(randNumber);
//  if(randNumber == 3){
//    RFIDbytes[0] = 0x00;
//    RFIDbytes[1] = 0x01 << 1;
//    RFIDbytes[2] = 0x01 << 1;
//    RFIDbytes[3] = 0x00;
//    RFIDbytes[4] = 0x01 << 1;
//    RFIDbytes[5] = 0x06 << 1;
//    RFIDbytes[6] = 0x0D << 1;
//    RFIDbytes[7] = 0x0B << 1;
//    RFIDbytes[8] = 0x04 << 1;
//    RFIDbytes[9] = 0x0F << 1;
//    serial.println("tag detected... ");
//    return(1);
//  } else {
//    return(0);
//  }
//}


byte FastRead(int demodOutPin, int shutDownPin, byte checkDelay, unsigned int readTime) {
  digitalWrite(SHD_PINA, LOW);                            //Turn on primary RFID circuit
  digitalWrite(SHD_PINB, HIGH);                           //Turn off Secondary RFID circuit
  //serial.println("fast read activated...");
  pinMode(demodOutPin, INPUT);      //set up the input pin as an input
  rParity = 0;
  parityFail = 0x07FF;  //start with 11 bits set and clear one for every line-parity check that passes, and clear the last for the column parity check
  pulseCount = 0;
  OneCounter = 0;
  longPulseDetected = 0;
  pastPulseLong = 0;
  RFIDbyteCounter = 0;
  RFIDbitCounter = 4;    //counts backwards from 4 to zero
  memset(RFIDbytes, 0, sizeof(RFIDbytes));  //Clear RFID memory space
  unsigned long currentMillis = millis();                               //To determine how long to poll for tags, first get the current value of the built in millisecond clock on the processor
  unsigned long stopMillis = currentMillis + readTime;
  attachInterrupt(digitalPinToInterrupt(demodOutPin), INT_demodOut, CHANGE);

  //delay(checkTime);
  delay(checkDelay);
  //serial.print("pulses detected... ");
  //serial.println(pulseCount, DEC);
  if (pulseCount > (checkDelay - 25)) {                          //May want a separate variable for threshold pulse count.
    while (millis() < stopMillis & parityFail != 0) {
      delay(1);
    }
  } else {
    detachInterrupt(digitalPinToInterrupt(demodOutPin));
    digitalWrite(SHD_PINA, HIGH);                           //Turn off primary RFID circuit
    digitalWrite(SHD_PINB, HIGH);                           //Turn off Secondary RFID circuit
    //serial.print("nothing read... ");
    return (0);
  }

  detachInterrupt(digitalPinToInterrupt(demodOutPin));
  digitalWrite(SHD_PINA, HIGH);                           //Turn off primary RFID circuit
  digitalWrite(SHD_PINB, HIGH);                           //Turn off Secondary RFID circuit
  if (parityFail == 0) {
    serial.print("parityOK... ");
    return (1);
  } else {
    serial.print("parity fail... ");
    return (0);
  }

}

void processTag(void)
{
  //process each byte (could do a loop but.....)
  RFIDtagArray[0] = ((RFIDbytes[0] << 3) & 0xF0) + ((RFIDbytes[1] >> 1) & 0x0F);
  String StringOne = String(RFIDtagArray[0], HEX);
  if (RFIDtagArray[0] < 0x10) {
    StringOne = String("0" + StringOne);
  }
  RFIDtagUser = RFIDtagArray[0];

  RFIDtagArray[1] = ((RFIDbytes[2] << 3) & 0xF0) + ((RFIDbytes[3] >> 1) & 0x0F);
  String StringTwo = String(RFIDtagArray[1], HEX);
  if (RFIDtagArray[1] < 0x10) {
    StringTwo = String("0" + StringTwo);
  }
  RFIDtagNumber = RFIDtagArray[1] << 24;

  RFIDtagArray[2] = ((RFIDbytes[4] << 3) & 0xF0) + ((RFIDbytes[5] >> 1) & 0x0F);
  String StringThree = String(RFIDtagArray[2], HEX);
  if (RFIDtagArray[2] < 0x10) {
    StringThree = String("0" + StringThree);
  }
  RFIDtagNumber = RFIDtagNumber + (RFIDtagArray[2] << 16);

  RFIDtagArray[3] = ((RFIDbytes[6] << 3) & 0xF0) + ((RFIDbytes[7] >> 1) & 0x0F);
  String StringFour = String(RFIDtagArray[3], HEX);
  if (RFIDtagArray[3] < 0x10) {
    StringFour = String("0" + StringFour);
  }
  RFIDtagNumber = RFIDtagNumber + (RFIDtagArray[3] << 8);

  RFIDtagArray[4] = ((RFIDbytes[8] << 3) & 0xF0) + ((RFIDbytes[9] >> 1) & 0x0F);
  String StringFive = String(RFIDtagArray[4], HEX);
  if (RFIDtagArray[4] < 0x10) {
    StringFive = String("0" + StringFive);
  }
  RFIDtagNumber = RFIDtagNumber + RFIDtagArray[4];
  RFIDstring = String(StringOne + StringTwo + StringThree + StringFour + StringFive);
  RFIDstring.toUpperCase();
}


void INT_demodOut(void)
{
  volatile uint32_t timeNow = micros();              //Store the current microsecond timer value in timeNow
  volatile static uint32_t lastTime = 0;             //Clear this variable
  volatile static uint8_t  lastRead = 0;             //Clear this variable
  uint16_t fDiff = timeNow - lastTime;               //Calculate time elapsed since the last execution of this function
  lastTime = timeNow;                                //Establish a new value for lastTime
  //int8_t fTimeClass = ManchesterDecoder::tUnknown;   //??????
  int16_t fVal = digitalRead(DEMOD_OUT_PIN);           //set fVal to the opposite (!) of the value on the RFID data pin (default is pin 30).
  byte RFbit = 255;                                    //set to default, 255, (no bit read)

  if (fDiff > 395 & fDiff < 600) {
    pulseCount++;
    longPulseDetected = 1;
    pastPulseLong = 1;
    RFbit = 200;                //Indicate that successful reading is still going on
    if (OneCounter < 9) {
      fVal == 1 ? OneCounter++ : OneCounter = 0;         //If we have read a 1 add to the one counter. if not clear the one counter
    } else {
      RFbit = fVal;
    }
  }
  if (fDiff < 395 & fDiff > 170) {
    pulseCount++;
    RFbit = 200;                                         //Indicate that successful reading is still going on
    if (longPulseDetected == 1 && pastPulseLong == 1) {  //Before this input means anything we must have registered one long bit and the last pulse must have been long (or a transition bit)
      if (OneCounter < 9) {                              //Only write tag bits when we have read 9 ones.
        fVal == 1 ? OneCounter++ : OneCounter = 0;       //If we have read a 1 add to the one counter. if not clear the one counter
      } else {
        RFbit = fVal;
      }
      pastPulseLong = 0;          //Indicate that the last pulse was short
    } else {
      pastPulseLong = 1;          //Indicate that the last pulse was long. This is not really true, but the second of two consecutive short pulses means the next pulse should indicate a read bit.
    }
  }

  //Now check if RFbit was changed from 255 and if so add to the data compiled in RFIDbytes
  if (RFbit < 100) {
    RFbit == 1 ? bitSet(RFIDbytes[RFIDbyteCounter], RFIDbitCounter) : bitClear(RFIDbytes[RFIDbyteCounter], RFIDbitCounter); //Set or clear the RFID data bit
    if (RFIDbitCounter > 0) {
      rParity = rParity ^ RFbit;   //Calculate running parity bit -- Exclusive or between row parity variable and current RF bit
      RFIDbitCounter--;
    } else {

      if ((RFIDbitCounter == 0) & (RFIDbyteCounter < 10)) {  //Indicates we are at the end of a line - Do a line parity check
        byte tb = RFIDbytes[RFIDbyteCounter];
        rParity = ((tb >> 4) & 1) ^ ((tb >> 3) & 1) ^ ((tb >> 2) & 1) ^ ((tb >> 1) & 1);
        rParity == (tb & 1) ? bitClear(parityFail, RFIDbyteCounter) : bitSet(parityFail, RFIDbyteCounter); //Check parity match and adjust parityFail
        rParity = 0;
        RFIDbyteCounter++;
        RFIDbitCounter = 4;
      }

      if ((RFIDbitCounter == 0) & (RFIDbyteCounter == 10)) { //Indicates we are on the last bit of an ID code
        //test all column parity
        byte xorByte = (RFIDbytes[10] & B00011111) >> 1;
        for (byte i = 0; i <= 9; i++) { //loop through bytes 1 though 9 (parity row included on first interation - should Xor out to zero
          xorByte = xorByte ^  (RFIDbytes[i] >> 1);
        }
        if (xorByte == 0) {
          bitClear(parityFail, RFIDbyteCounter) ;          //If parity checks out clear the last bit
        }
      }
    }
  }

  if ((RFbit == 255) & (pulseCount != 0)) {             // no pulse detected, clear everything except pulseCount
    rParity = 0;
    parityFail = 0x07FF;
    OneCounter = 0;
    longPulseDetected = 0;
    pastPulseLong = 0;
    RFIDbyteCounter = 0;
    RFIDbitCounter = 4;    //counts backwards from 4 to zero
    memset(RFIDbytes, 0, sizeof(RFIDbytes));  //Clear RFID memory space
  }
}



void showTime() {
  currentDate = rtc.stringDateUSA(); //Get the current date in mm/dd/yyyy format (we're weird in the US)
  //String currentDate = rtc.stringDate()); //Get the current date in dd/mm/yyyy format (Rest-of-the-world format)
  currentTime = rtc.stringTime(); //Get the time
  currentDateTime = currentDate + " " + currentTime;
  serial.println(currentDateTime);
}

void writeI2C(byte dev, byte addr, byte val) {  //Set alarm registers
  Wire.begin();                 //Start I2C functions
  Wire.beginTransmission(dev);  //Begin I2C communication using the I2C address for the clock
  Wire.write(addr);             //set register
  Wire.write(val);              //write to register
  Wire.endTransmission();
}

//void displayTag() {
//  serial.println();
//  char tbuf[32];
//  serial.print("Card Number (hex): ");
//  serial.println(RFIDstring);
//  uint32_t cardNum = RFIDtagNumber;
//  serial.print("Card Number (dec): ");
//  serial.println(cardNum);
//}

void flashLED() {
  for (unsigned int n = 0; n < 100 ; n = n + 30) {   //loop to Flash LED and delay reading after getting a tag
    digitalWrite(LED_RFID, LOW);                          //The approximate duration of this loop is determined by the readFreq value
    delay(5);                                             //It determines how long to wait after a successful read before polling for tags again
    digitalWrite(LED_RFID, HIGH);
    delay(25);
  }
}

void blinkLED(byte repeats) {
  for (int i = 0; i < repeats; i++) //flash LED 5 times while waiting for serial connection to come online
  {
    delay(500);                     //pause for 0.5 seconds
    digitalWrite(LED_RFID, LOW);    //turn the LED on (LOW turns it on)
    delay(500);                     //pause again
    digitalWrite(LED_RFID, HIGH);   //turn the LED off (HIGH turns it off)
    //ss = ss + 1;                    //add to counting variable
  }//end while
  digitalWrite(LED_RFID, HIGH);     // make sure LED is off
}

void blipLED(unsigned int blip) {
  digitalWrite(LED_RFID, LOW);    //turn the LED on (LOW turns it on)
  delay(blip);                    //leave LED on for a quick flash
  digitalWrite(LED_RFID, HIGH);   //turn the LED off (HIGH turns it off)
  if (blip == 200) {
    delay(blip);
  }
}

void checkDataFile(String fName) {
  digitalWrite(SDselect, LOW);
  digitalWrite(csFlash, HIGH); //deactivate flash chip
  SD.begin();
  byte result = SD.exists(fName.c_str());
  //  serial.print(fName);
  //  serial.print(": file exists? ");
  //  serial.println(result, DEC);
  if (result == 0) {
    serial.print("Making new file: ");
    serial.println(fName);
    File dataFile = SD.open(fName, FILE_WRITE);
    dataFile.println(fName);
    dataFile.close();
  }
}

void logRFID_To_SD(String tagID, String timeString, String fName) {
  digitalWrite(csFlash, HIGH); //deactivate flash chip
  digitalWrite(SDselect, LOW); //Activate SD card
  File dataFile = SD.open(fName, FILE_WRITE);        //Initialize the SD card and open the file "datalog.txt" or create it if it is not there.
  if (dataFile) {
    digitalWrite(SDselect, LOW);
    digitalWrite(csFlash, HIGH); //deactivate flash chip
    SD.begin();
    File dataFile = SD.open(fName, FILE_WRITE);        //Initialize the SD card and open the file "datalog.txt" or create it if it is not there.
    if (dataFile) {
      /*for (int n = 0; n < 5; n++) {               //loop to print out the RFID code to the SD card
        if (tagData[n] < 10) dataFile.print("0"); //add a leading zero if necessary
        dataFile.print(tagData[n], HEX);          //print to the SD card
        }*/
      char tbuf[48];

      dataFile.print(tagID);
      //    dataFile.print(",");                        //comma for data delineation
      //    dataFile.print(RFcircuit);                  //log which antenna was active
      //    dataFile.print(",");                        //comma for data delineation
      dataFile.print(" ");
      dataFile.println(timeString);               //log the time
      dataFile.close();                           //close the file
      //serial.println("saved to SD card.");        //serial output message to user
    } // check dataFile is present
    else {
      serial.print("error opening ");  //error message if the "datafile.txt" is not present or cannot be created
      serial.println(fName);
    }
  }// end function
}


bool checkTag(unsigned long tag) {
  //serial.print("looking to match ");
  //serial.println(tag, HEX);
  //serial.print("tag count ");
  //serial.println(tagCount, DEC);
  unsigned long tag2 = 0;
  byte tagLoop = 1;
  bool tFound = false;

  unsigned long fAddr = 0x00000000;
  digitalWrite(csFlash, HIGH);  // initially deactivate flash chip
  digitalWrite(SDselect, HIGH); // deactivate SD card
  SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
  digitalWrite(csFlash, LOW);   //activate flash chip
  SPI.transfer(0x03);           //opcode for low freq read
  SPI.transfer(fAddr >> 16);           //first of three address bytes
  SPI.transfer((fAddr >> 8) & 0xFF);   // second address byte
  SPI.transfer(fAddr & 0xFF);          // third address byte
  while (tagLoop <= tagCount) {
    tag2 = SPI.transfer(0);               // ignore the first (most significant) byte
    tag2 = SPI.transfer(0);               // pile the next four bytes into a long variable
    tag2 = (tag2 << 8) + SPI.transfer(0);
    tag2 = (tag2 << 8) + SPI.transfer(0);
    tag2 = (tag2 << 8) + SPI.transfer(0);
    //serial.print("Tag retrieved ");
    //serial.println(tag2, HEX);
    if (tag2 == tag) {
      tFound = true;
      //serial.println("match found...");
      return (true);
    }
    tagLoop = tagLoop + 1;
  }
  digitalWrite(csFlash, HIGH);   //deactivate flash chip
  SPI.endTransaction();
  serial.println("match not found.");
  return (false); //No match found, return false
}

byte asciiToHex(byte x) {
  if (x > 0x39) x -= 7; // adjust for hex letters upper or lower case
  x -= 48;
  return x;
}

void transferTags() {
  digitalWrite(SDselect, LOW);
  byte tagNib;
  byte tagData[500];
  int n = 0;
  int tagCount = 0;
  unsigned long memAddress = 0;

  File myfile = SD.open("TAGS.TXT");  // attempt to open the file with a list of tag numbers
  if (!myfile) {
    serial.println("TAGS.TXT file not found");
    return;
  }

  if (myfile) {                      // if the file is available, read the file
    serial.println("TAGS.TXT file found");
    while (myfile.available()) {
      byte read1 = myfile.read();     //each myfile.read() operation brings in one byte (one character) - get the first and combine it with the second
      if ((read1 > 47 & read1 < 58) | (read1 > 64 & read1 < 71)) {  //Filter out anything that is not part of a string of hexidecimal numbers (only keep numerals 0-9 and captial letters A-F)
        tagData[n] = asciiToHex(read1);   //if statement to make sure the characters make sense for hex
        n = n + 1;
      }
    }
    myfile.close();

    if (n < 10) {
      serial.println("No tags read from TAGS.TXT");
      return;
    } else {  //do the following if there are enough chracters (i.e. 10) for at least one tag ID

      if (n % 10 != 0) {
        serial.println("Tag data misaligned. Check TAGS.TXT file");
        return;
      }

      byte t = 0;
      while (t < n / 2) {
        tagData[t] = (tagData[t * 2] << 4) + tagData[(t * 2) + 1];
        t = t + 1;
      }
      
      tagCount = t / 5;

      digitalWrite(csFlash, HIGH); //activate flash chip
      digitalWrite(SDselect, HIGH);
      SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
      digitalWrite(csFlash, LOW); //activate flash chip

      SPI.transfer(0x58); //opcode for read modify write
      SPI.transfer(0x00);                                  // first of three address bytes
      SPI.transfer(0x00);                                  // second address byte (always use first page or page 0)
      SPI.transfer(0x00);                     // third address byte
      for (int c = 0; c <= n; c++) {
        SPI.transfer(tagData[c]);
      }
      digitalWrite(csFlash, HIGH); //deactivate flash chip
      SPI.endTransaction();
      delay(30);

      //Now write the tag count to page 1 byte 9
      if (!writeFlashByte(0x00000409, tagCount)) {
        //serial.print("write failed try again");
        writeFlashByte(0x00000409, tagCount);        //Try again on failure??
      }
    }
    delay(10);
  }
  return;
} // end function transferTags

void showTags() {
  tagCount = readFlashByte(0x00000409);
  serial.println();
  serial.print(tagCount, DEC);
  serial.println(" tags currently in memory.");

  digitalWrite(csFlash, HIGH); //activate flash chip
  digitalWrite(SDselect, HIGH);
  SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
  digitalWrite(csFlash, LOW); //activate flash chip
  SPI.transfer(0x03);           //opcode for low freq read
  SPI.transfer(0x00);           //first of three address bytes
  SPI.transfer(0x00);          // second address byte
  SPI.transfer(0x00);          // third address byte
  for (int n = 0; n < tagCount; n++) {
    for (int p = 0; p < 5; p++) {
      byte RFbyte = SPI.transfer(0);
      if (RFbyte < 0x10) serial.print("0");
      serial.print(RFbyte, HEX);
    }
    serial.println();
  }
  digitalWrite(csFlash, HIGH);   //deactivate flash chip
  SPI.endTransaction();
}



void erasePage0() {
  digitalWrite(csFlash, HIGH);  // initially deactivate flash
  digitalWrite(SDselect, HIGH); // make sure the SD card select is off
  SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
  digitalWrite(csFlash, LOW); //activate flash chip
  SPI.transfer(0x81); //opcode for page erase
  SPI.transfer(0x00); // first of three address bytes
  SPI.transfer(0x00);  // second address byte
  SPI.transfer(0x00);        // third address byte
  digitalWrite(csFlash, HIGH);                // turn off flash
  SPI.endTransaction();
}

void dumpMem(byte doAll) {
  byte tagData[5];
  serial.println("Transmitting data from backup memory.");
  unsigned long fAddressEnd;
  doAll == 0 ? fAddressEnd = getFlashAddr() : fAddressEnd = ((528 << 10) + 512) ;   // get flash address
  serial.print("last flash memory address: ");
  serial.println(fAddressEnd, BIN);
  //fAddressEnd = 0x00001000;
  unsigned long fAddress = 0x00000800;          // first address for stored data

  while (fAddress < fAddressEnd) {
    if (serial.available())
    {
      serial.println("User exit");
      break;
    }

    digitalWrite(csFlash, HIGH);  // initially deactivate flash
    digitalWrite(SDselect, HIGH); // make sure the SD card select is off
    SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
    digitalWrite(csFlash, LOW); //activate flash chip
    SPI.transfer(0x03);                           // opcode for low freq read
    SPI.transfer(fAddress >> 16);                 // write most significant byte of Flash address
    SPI.transfer((fAddress >> 8) & 0xFF);         // second address byte
    SPI.transfer(fAddress & 0xFF);                // third address byte
    while ((fAddress & 0x000003FF) < 500) {       // repeat while the byte address is less than 500
      //serial.print("from flash memory address: ");
      //serial.println(fAddress, BIN);
      for (int n = 0; n < 5; n++) {             // loop to read in an RFID code from the flash and send it out via serial comm
        tagData[n] = SPI.transfer(0);
        //if (tagData[n] < 10) serial.print("0"); // add a leading zero if necessary
        //serial.print(tagData[n], HEX);         // send out tag datum
      }
      RFcircuit = SPI.transfer(0);               // read which antenna was active
      mo = SPI.transfer(0);                      // read in date and time
      da = SPI.transfer(0);
      yr = SPI.transfer(0);
      hh = SPI.transfer(0);
      mm = SPI.transfer(0);
      ss = SPI.transfer(0);

      static char charRFID[10];
      sprintf(charRFID, "%02X%02X%02X%02X%02X", tagData[0], tagData[1], tagData[2], tagData[3], tagData[4]);
      static char dateAndTime[17];
      sprintf(dateAndTime, "%02d/%02d/%02d %02d:%02d:%02d", mo, da, yr, hh, mm, ss);
      serial.print(charRFID);
      serial.print(" ");
      serial.println(dateAndTime);

      fAddress = fAddress + 12;                 // update flash address
      if (fAddress >= fAddressEnd) break;       // break if we are at the end of the backup data stream
    }
    // When the byte address exceeds 500 the page address needs to be incremented
    fAddress = (fAddress & 0xFFFFC00) + 0x0400;   //set byte address to zero and add 1 to the page address
    if (fAddress >= fAddressEnd) {                //Stop when we are at the end of the backup data stream
      digitalWrite(csFlash, HIGH);                // turn off flash
      SPI.endTransaction();
      break;                                      // break if we are at the end of the backup data stream
    }
  }
}

void writeMem(byte doAll) {
  byte tagData[5];
  serial.println("Transmitting data from backup memory.");
  unsigned long fAddressEnd;
  doAll == 0 ? fAddressEnd = getFlashAddr() : fAddressEnd = ((528 << 10) + 512) ;   // get flash address
  serial.print("last flash memory address: ");
  serial.println(fAddressEnd, BIN);
  //fAddressEnd = 0x00001000;
  unsigned long fAddress = 0x00000800;          // first address for stored data

  while (fAddress < fAddressEnd) {
    if (serial.available())
    {
      serial.println("User exit");
      break;
    }

    while ((fAddress & 0x000003FF) < 500) {       // repeat while the byte address is less than 500
      digitalWrite(csFlash, HIGH);  // initially deactivate flash
      digitalWrite(SDselect, HIGH); // make sure the SD card select is off
      SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
      digitalWrite(csFlash, LOW); //activate flash chip
      SPI.transfer(0x03);                           // opcode for low freq read
      SPI.transfer(fAddress >> 16);                 // write most significant byte of Flash address
      SPI.transfer((fAddress >> 8) & 0xFF);         // second address byte
      SPI.transfer(fAddress & 0xFF);                // third address byte


      //serial.print("from flash memory address: ");
      //serial.println(fAddress, BIN);
      for (int n = 0; n < 5; n++) {             // loop to read in an RFID code from the flash and send it out via serial comm
        tagData[n] = SPI.transfer(0);
        //if (tagData[n] < 10) serial.print("0"); // add a leading zero if necessary
        //serial.print(tagData[n], HEX);         // send out tag datum
      }
      RFcircuit = SPI.transfer(0);               // read which antenna was active
      mo = SPI.transfer(0);                      // read in date and time
      da = SPI.transfer(0);
      yr = SPI.transfer(0);
      hh = SPI.transfer(0);
      mm = SPI.transfer(0);
      ss = SPI.transfer(0);

      digitalWrite(csFlash, HIGH);  // deactivate flash
      SPI.endTransaction();

      static char charRFID[10];
      sprintf(charRFID, "%02X%02X%02X%02X%02X", tagData[0], tagData[1], tagData[2], tagData[3], tagData[4]);
      static char dateAndTime[17];
      sprintf(dateAndTime, "%02d/%02d/%02d %02d:%02d:%02d", mo, da, yr, hh, mm, ss);
      //      serial.print(charRFID);
      //      serial.print(" ");
      //      serial.println(dateAndTime);

      String BakupFileName = String(feederNameStr + "BKUP.TXT");

      logRFID_To_SD(charRFID, dateAndTime, BakupFileName);

      fAddress = fAddress + 12;                 // update flash address
      if (fAddress >= fAddressEnd) break;       // break if we are at the end of the backup data stream
    }
    // When the byte address exceeds 500 the page address needs to be incremented
    fAddress = (fAddress & 0xFFFFC00) + 0x0400;   //set byte address to zero and add 1 to the page address
    serial.print("Writing from next page: memory address");
    serial.println(fAddress, DEC);
    if (fAddress >= fAddressEnd) {                //Stop when we are at the end of the backup data stream
      digitalWrite(csFlash, HIGH);                // turn off flash
      SPI.endTransaction();
      break;                                      // break if we are at the end of the backup data stream
    }
  }
}




unsigned long getFlashAddr() {    //get the address counter for the flash memory from page 1 address 0
  digitalWrite(csFlash, HIGH);    //initially deactivate flash chip
  digitalWrite(SDselect, HIGH);   // make sure the SD card select is off
  SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
  digitalWrite(csFlash, LOW);
  unsigned long fAddress = 0x00000400;   //Page 2, byte 0
  SPI.transfer(0x03);           //opcode for low freq read
  SPI.transfer(0x00);           //first of three address bytes
  SPI.transfer(0x04);           // 00000100  second address byte - selects page 1
  SPI.transfer(0x00);           // third address byte selects byte address 0
  fAddress = SPI.transfer(0) & 0xFF;               //Shift in the address value
  fAddress = (fAddress << 8) + SPI.transfer(0);   //Shift in the address value
  fAddress = (fAddress << 8) + SPI.transfer(0);   //Shift in the address value
  digitalWrite(csFlash, HIGH); //deactivate flash chip
  SPI.endTransaction();
  //serial.print("The flash address is ");
  //serial.println(fAddress, BIN);
  return fAddress;
}

void getFeederID() {    //get the feeder ID from the flash memory: page 1 address 4-7
  digitalWrite(csFlash, HIGH);    //initially deactivate flash chip
  digitalWrite(SDselect, HIGH);   // make sure the SD card select is off
  SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
  digitalWrite(csFlash, LOW);
  SPI.transfer(0x03);           //opcode for low freq read
  SPI.transfer(0x00);           //first of three address bytes
  SPI.transfer(0x04);           // 00000100  second address byte - selects page 1
  SPI.transfer(0x04);           // third address byte selects byte address 4
  //serial.println("Getting feeder ID: ");
  for (int n = 0; n < 4; n++) {           //loop to log the data to the flash
    feederID[n] = SPI.transfer(0);
    //serial.print(feederID[n]);
  }
  byte checkFeederID = SPI.transfer(0);
  //serial.println(checkFeederID, HEX);
  SPI.endTransaction();
  digitalWrite(csFlash, HIGH); //deactivate flash chip
  if (checkFeederID != 0xAA) {
    feederID[0] = 'G';
    feederID[1] = 'P';
    feederID[2] = 'R';
    feederID[3] = '0';
    writeFeederID();
  }
}

void writeFeederID() {
  digitalWrite(csFlash, HIGH);   //initially deactivate flash chip
  digitalWrite(SDselect, HIGH); // make sure the SD card select is off
  SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
  digitalWrite(csFlash, LOW);   //activate flash chip
  SPI.transfer(0x58);           //opcode for read modify write
  SPI.transfer(0x00);           //first of three address bytes
  SPI.transfer(0x04);           // 000001oo  second address byte - selects page 1
  SPI.transfer(0x04);           // third address byte selects byte address 4
  for (int n = 0; n < 4; n++) {          //loop to read all the data from the serial buffer once it is ready
    SPI.transfer(feederID[n]);           //write characters to Flash
  }
  SPI.transfer(0xAA);              //Initialization indicator - indicates that a valid feederID is present.
  SPI.endTransaction();
  digitalWrite(csFlash, HIGH); //deactivate flash chip
  delay(20);
}

void writeFlashAddr(unsigned long fAddress) {           //write the address counter for the flash memory on page 1 byte address 0
  digitalWrite(csFlash, HIGH);   //initially deactivate flash chip
  digitalWrite(SDselect, HIGH); // make sure the SD card select is off
  SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
  digitalWrite(csFlash, LOW);   //activate flash chip
  SPI.transfer(0x58);           //opcode for read modify write
  SPI.transfer(0x00);           //first of three address bytes
  SPI.transfer(0x04);           // 000001oo  second address byte - selects page 1
  SPI.transfer(0x00);           // third address byte selects byte address 0
  SPI.transfer(fAddress >> 16);           // write most significant byte of Flash address
  SPI.transfer((fAddress >> 8) & 0xFF);   // second address byte
  SPI.transfer(fAddress & 0xFF);          // third address byte
  digitalWrite(csFlash, HIGH); //deactivate flash chip
  SPI.endTransaction();
  delay(20);                  //Writing may take some time - the dely here provides some time in case another command follows immediately.
}

void writeRFID_To_FlashLine() {
  unsigned long fAddress = getFlashAddr(); //Get the current flash memory address
  serial.print("transferring to address: ");
  serial.println(fAddress, BIN);
  //serial.print(" ");
  //displayTag();
  digitalWrite(csFlash, HIGH);   //initially deactivate flash chip
  digitalWrite(SDselect, HIGH); // make sure the SD card select is off
  SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
  digitalWrite(csFlash, LOW);   //activate flash chip
  SPI.transfer(0x58);           //opcode for read modify write
  SPI.transfer((fAddress >> 16) & 0xFF);           // write most significant byte of Flash address
  SPI.transfer((fAddress >> 8) & 0xFF);   // second address byte
  SPI.transfer(fAddress & 0xFF);          // third address byte
  for (int n = 0; n < 5; n++) {           //loop to log the RFID code to the flash
    SPI.transfer(RFIDtagArray[n]);
  }
  SPI.transfer(RFcircuit);                  //log which antenna was active
  SPI.transfer(rtc.getMonth());
  SPI.transfer(rtc.getDate());
  SPI.transfer(rtc.getYear());
  SPI.transfer(rtc.getHours());
  SPI.transfer(rtc.getMinutes());
  SPI.transfer(rtc.getSeconds());
  digitalWrite(csFlash, HIGH); //deactivate flash chip
  SPI.endTransaction();
  unsigned int bAddress = fAddress & 0x03FF;      //and with 00000011 11111111 to isolate byte address
  bAddress = bAddress + 12;                       //add 12 to accound for new bytes (5 for RFID, 1 for RFcircuit, and 6 for date/time)
  if (bAddress > 500) {                           //stop writing if beyond byte address 500 (this is kind of wasteful)
    fAddress = (fAddress & 0xFFFFC00) + 0x0400;   //set byte address to zero and add 1 to the page address
  } else {
    fAddress = (fAddress & 0xFFFFC00) + bAddress; //just add to the byte address
  }
  //digitalWrite(csFlash, HIGH); //deactivate flash chip
  delay(20);
  writeFlashAddr(fAddress);  //Write the updated address to flash.
  serial.println("saved to flash.");        //serial output message to user
}

char getMode() {
  char fMode = readFlashByte(0x0000040A);        //feeder mode is stored in page 1, byte 10
  if ((fMode != 'O') && (fMode != 'T')) {
    fMode = 'A';
    writeFlashByte(0x0000040A, 'A');
  }
  serial.print("Feeder mode: ");
  serial.println(fMode);
  return fMode;
}

char setMode(char sMode) {
  writeFlashByte(0x0000040A, sMode);
  serial.print("Feeder mode set to: ");
  serial.println(sMode);
  delay(20);
  return sMode;
}

byte readFlashByte(unsigned long fAddress) {
  digitalWrite(csFlash, HIGH);   //initially deactivate flash chip
  digitalWrite(SDselect, HIGH); // make sure the SD card select is off
  SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
  digitalWrite(csFlash, LOW);   //activate flash chip
  SPI.transfer(0x03);           //opcode for low freq read
  SPI.transfer((fAddress >> 16) & 0xFF);           //first of three address bytes
  SPI.transfer((fAddress >> 8) & 0xFF);   // second address byte
  SPI.transfer(fAddress & 0xFF);          // third address byte
  byte fByte = SPI.transfer(0);
  digitalWrite(csFlash, HIGH); //deactivate flash chip
  SPI.endTransaction();
  return fByte;
}

bool writeFlashByte(unsigned long fAddress, byte fByte) {
  digitalWrite(csFlash, HIGH);   //initially deactivate flash chip
  digitalWrite(SDselect, HIGH); // make sure the SD card select is off
  SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
  digitalWrite(csFlash, LOW);   //activate flash chip
  SPI.transfer(0x58);           //opcode for read modify write
  SPI.transfer((fAddress >> 16) & 0xFF);  // first of three address bytes
  SPI.transfer((fAddress >> 8) & 0xFF);   // second address byte
  SPI.transfer(fAddress & 0xFF);          // third address byte
  SPI.transfer(fByte);
  delay(20);
  digitalWrite(csFlash, HIGH); //deactivate flash chip
  SPI.endTransaction();

  byte checkByte = readFlashByte(fAddress);
  if (checkByte == fByte) {
    return (true);
  } else {
    return (false);
  }
  delay(20);
}


void inputTime() {                          //Function to set the clock
  serial.println("Enter mmddyyhhmmss");  //Ask for user input
  while (serial.available() == 0) {}    //wait for 12 characters to accumulate
  for (int n = 0; n < 13; n++) {        //loop to read all the data from the serial buffer once it is ready
    timeIn[n] = serial.read();         //Read the characters from the buffer into an array of 12 bytes one at a time
  }
  while (serial.available())         //Clear the buffer, in case there were extra characters
  {
    serial.read();                  //Read in any extra characters (and then ignore them)
  }
  //Transform the input into decimal numbers
  mo = ((timeIn[0] - 48) * 10 + (timeIn[1] - 48)); //Convert two ascii characters into a single decimal number
  da = ((timeIn[2] - 48) * 10 + (timeIn[3] - 48)); //Convert two ascii characters into a single decimal number
  yr = ((timeIn[4] - 48) * 10 + (timeIn[5] - 48)); //Convert two ascii characters into a single decimal number
  hh = ((timeIn[6] - 48) * 10 + (timeIn[7] - 48)); //Convert two ascii characters into a single decimal number
  mm = ((timeIn[8] - 48) * 10 + (timeIn[9] - 48)); //Convert two ascii characters into a single decimal number
  ss = ((timeIn[10] - 48) * 10 + (timeIn[11] - 48)); //Convert two ascii characters into a single decimal number
}

void inputID() {                                         //Function to input and set feeder ID
  serial.println("Enter four alphanumeric characters");  //Ask for user input
  while (serial.available() == 0) {}                     //wait for input
  for (int n = 0; n < 4; n++) {                          //loop to read all the data from the serial buffer once it is ready
    feederID[n] = serial.read();         //Read the characters from the buffer into an array one at a time
  }
  while (serial.available())         //Clear the buffer, in case there were extra characters
  {
    serial.read(); //Read in any extra characters (and then ignore them)
  }
  writeFeederID();
}

void printDirectory(File dir, int numTabs) {
  while (true) {

    File entry =  dir.openNextFile();
    if (! entry) {
      // no more files
      break;
    }
    for (uint8_t i = 0; i < numTabs; i++) {
      serial.print('\t');
    }
    serial.print(entry.name());
    if (entry.isDirectory()) {
      serial.println("/");
      printDirectory(entry, numTabs + 1);
    } else {
      // files have sizes, directories do not
      serial.print("\t\t");
      serial.println(entry.size(), DEC);
    }
    entry.close();
  }
}


void motInit() {  //determine initial motor position and move to closed position
  serial.println("Initializing motor...");
  digitalWrite(mStby, HIGH);
  bool motDir = false;              //false for down, true for up
  unsigned int motTime = 100;
  while (digitalRead(mSwitch)) {                //mSwitch high (switch open) means door is fully open or fully closed
    if (motDir == 0) {
      digitalWrite(MOTR, HIGH);   //Set motor to move door down
      digitalWrite(MOTL, LOW);
      serial.print("move down...");
      serial.println(motTime, DEC);
    } else  {
      digitalWrite(MOTR, LOW);   //Set motor to move door up
      digitalWrite(MOTL, HIGH);
      serial.print("move up...");
      serial.println(motTime, DEC);
    }
    nudgeMot(motTime);           //Move motor a little
    motTime = motTime + 100;     //add to the run time
    motDir = !motDir;            //try the other direction
  }
  doorState = 0;
  doorClose();    //if door is in the partially open position, then close it.
}

void doorOpen() {
  //serial.print("door state = "); //for debugging
  //serial.println(doorState); //for debugging
  if (doorState == 1) {
    serial.println("open"); //for debugging
    digitalWrite(mStby, HIGH);
    digitalWrite(MOTR, HIGH);
    digitalWrite(MOTL, LOW);
    runMot();
    stopMot(true);
  }
  doorState = 0;
  serial.print("door state = "); //for debugging
  serial.println(doorState); //for debugging
}

void doorClose() {
  serial.print("door state = "); //for debugging
  serial.println(doorState); //for debugging
  serial.println("close"); //for debugging
  if (doorState == 0) {
    digitalWrite(mStby, HIGH);
    digitalWrite(MOTR, LOW);  //powers up sleep....
    digitalWrite(MOTL, HIGH);
    runMot();
    stopMot(true);
  }
  doorState = 1;
  serial.print("door state = "); //for debugging
  serial.println(doorState); //for debugging
}

void stopMot(bool off) {
  digitalWrite(MOTPWM, HIGH);
  digitalWrite(MOTR, HIGH);
  digitalWrite(MOTL, HIGH);
  delay(50);
  digitalWrite(MOTPWM, LOW);
  digitalWrite(MOTR, LOW);
  digitalWrite(MOTL, LOW);
  if (off == true) {
    digitalWrite(mStby, LOW);
    serial.print("door state stop = "); //for debugging
    serial.println(doorState); //for debugging
  }
}

void pulseMot(byte pTime) {
  unsigned long currentMil = millis();          //Determine how long to activate the motor - first note the current value of the millisecond counter
  unsigned long stopMil = currentMil + pTime;   //   next add the desired movement time
  while (stopMil > millis()) {                  //   As long as the stoptime is less than the current millisecond counter, then keep the motor moving
    digitalWrite(MOTPWM, HIGH);
    //analogWrite(MOTPWM, motSpeed);
    //digitalWrite(MOTPWM, HIGH);               //Speed is determined by the ratio of the high and low pulse
    //delay(3);                                   //This delay determines the duration of the high pulse
    //digitalWrite(MOTPWM, LOW);                //Comment this out to go full speed
    delay(5);                                   //This delay determines the duration of the low pulse.
  }
}

void runMot() {
  pinMode(mSwitch, INPUT_PULLUP); // motor switch enabled as input with internal pullup resistor
  //three stage motor routine. First run the motor until the switch is closed
  //serial.println("stage 1");
  byte sw = digitalRead(mSwitch);
  pulseMot(100);
  while (sw == 1) {             //mSwitch high (switch open) means door is fully open or fully closed
    sw = digitalRead(mSwitch);
    //serial.print(sw);
    pulseMot(5);
  }
  pulseMot(200);                   //pulses to get through any switch bounce
  //serial.println("stage 2");
  //Now run the motor until the switch opens again
  //serial.print("switch ");
  while (sw == 0) {         //mSwitch low (switch closed) means door is between open and closed
    sw = digitalRead(mSwitch);
    serial.print(sw);
    pulseMot(5);
  }
  digitalWrite(MOTR, HIGH); //Apply brakes
  digitalWrite(MOTL, HIGH);
  delay(20);
  //serial.println();
  //Now nudge the motor just a little farther if desired
  //nudgeMot(100);
}

void nudgeMot(unsigned int motTime) {
  serial.print("nudging...");
  serial.println(motTime, DEC);
  unsigned long currentMs = millis();                //To determine how long to poll for tags, first get the current value of the built in millisecond clock on the processor
  unsigned long stopMs = currentMs + motTime;   //next add the value of polltime to the current clock time to determine the desired stop time.
  while (stopMs > millis()) {
    pulseMot(10);
  }
  stopMot(false);
}


void saveLogSD(String event) { //Save to log file in SD card
  File dataFile = SD.open("log.txt", FILE_WRITE);      //Initialize the SD card and open the file "datalog.txt" or create it if it is not there.
  if (dataFile) {
    dataFile.print(event);
    dataFile.print(": "); //Space for data delineation
    if (rtc.updateTime() == false) {
      serial.print("RTC failed"); //Updates the time variables from RTC
    }
    String currentDate = rtc.stringDateUSA(); //Get the current date in mm/dd/yyyy format (we're weird in the US)
    //String currentDate = rtc.stringDate()); //Get the current date in dd/mm/yyyy format (Rest-of-the-world format)
    String currentTime = rtc.stringTime(); //Get the time
    String currentDateTime = currentDate + " " + currentTime ;
    dataFile.println(currentDateTime);               //log the time
    dataFile.close();                          //close the file
    serial.println("saved log to SD card.");        //serial output message to user
  } // check dataFile is present
  else {
    serial.println("error opening log.txt"); //error message if the "datafile.txt" is not present or cannot be created
  }// end check for file
}

void lpSleep() {
  digitalWrite(MOTR, HIGH) ;              //Must be set high to get low power working - don't know why
  digitalWrite(SHD_PINB, HIGH);           //shut down both RFID readers
  digitalWrite(SHD_PINA, HIGH);
  //  pinMode(DEMOD_OUT_PIN, OUTPUT);digitalWrite(DEMOD_OUT_PIN,LOW);  //Does not help - pin high or low does not matter.
  //  pinMode(21,OUTPUT); //THIS ONE CAUSES HIGH POWER IF INPUT
  //  digitalWrite(21,LOW);
  //  pinMode(20,OUTPUT);digitalWrite(20,HIGH);//THIS ONE CAUSES HIGH POWER IF INPUT

  //pinMode(DEMOD_OUT_PIN, INPUT);      //Does not help.
  //ETAGLowPower::LowPower_SetGPIO();   //Does not work - no wake.
  //LowPower_SetUSBMode();              //Does not help.
  attachInterrupt(INT1, ISR, FALLING);
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCM_EIC) |     // Configure EIC to use GCLK1 which uses XOSC32K
                      GCLK_CLKCTRL_GEN_GCLK1 |       // This has to be done after the first call to attachInterrupt()
                      GCLK_CLKCTRL_CLKEN;
  USB->DEVICE.CTRLA.reg &= ~USB_CTRLA_ENABLE; //disable USB
  SysTick->CTRL  &= ~SysTick_CTRL_ENABLE_Msk;
  __WFI();    //Enter sleep mode
  //...Sleep...wait for interrupt
  USB->DEVICE.CTRLA.reg |= USB_CTRLA_ENABLE;  //enable USB
  detachInterrupt(INT1);
  SysTick->CTRL  |= SysTick_CTRL_ENABLE_Msk;
}

void ISR() {     // dummy routine - not really needed??
  //SLEEP_FLAG ^= true;  // toggle SLEEP_FLAG by XORing it against true
  byte SLEEP_FLAG = false;
  //serial.print("EIC_ISR SLEEP_FLAG = ");
  //serial.println(SLEEP_FLAG);
}

void checkSwitch() {
  while (1) {
    if (digitalRead(mSwitch) == 0) {
      serial.println("zero");
    } else {
      serial.println("one");
      delay(200);
    }
  }
}
