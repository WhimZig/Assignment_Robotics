//Header file for the EDMO study
//////////////DO NOT MODIFY THIS FILE!!!/////////////////////////////////////////
#include <SD.h> //sd card library
#include "RTClib.h" // Date and time functions using a PCF8523 RTC connected via I2C and Wire lib
RTC_PCF8523 rtc; //real time clock library

///////////SD-Card variables/////////////////////////////////////
// file name to use for writing
String boardID ="CPG_Study"; //experimental id
const char filename[] = "CPG01.txt"; //name of text file on sd card
// File object to represent file
File txtFile;
// string to buffer output
String buffer;

///////////SD-Card variables/////////////////////////////////////////
unsigned long startTime = 0; //start time = time of last upload
int timeStamp = 0; //time stamp counting upwards from the startTime
int spaceCount = 0; //variable to store number of spaces in the received input string
boolean recFlag = 0; //flag to tag if serial input was received
boolean firstIter = 1; //flag to tag if it is the first loop iteration since the last uplaod

///////Store current date and time on SD-Card/////////////////////////////////////////////////
void writeTime(){    
    DateTime now = rtc.now();
    txtFile.println();    
    txtFile.print(now.day());
    txtFile.print('/');
    txtFile.print(now.month());
    txtFile.print('/');
    txtFile.println(now.year());

    txtFile.print(now.hour());
    txtFile.print(':');
    txtFile.print(now.minute());
    txtFile.print(':');
    txtFile.println(now.second());
    
    txtFile.println(boardID);
    startTime = millis(); //save current millis as start time 
}
///////Initialize Real time clock //////////////////////////////////////
//Hint: all RTC functions need to be in the header file. Otherwise rtc.now() does not work and blocks the code from further execution (without any errors)
void initRTC() 
{
    Serial.println("Initialzing RTC");
    if (! rtc.begin()) {
      Serial.println("Couldn't find RTC");
      Serial.flush();
      abort();
    }
    if (! rtc.initialized() || rtc.lostPower()) {
      Serial.println("RTC is NOT initialized, let's set the time!");
      // When time needs to be set on a new device, or after a power loss, the
      // following line sets the RTC to the date & time this sketch was compiled
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
      // This line sets the RTC with an explicit date & time, for example to set
      // January 21, 2014 at 3am you would call:
    }
    //start RTC
    rtc.start();
    writeTime();
}
///////Initialize SD-Card/////////////////////////////////////////////////
void initSD(){
  Serial.println("Initialzing Sd-Card");
    // reserve 1kB for String used as a buffer
    buffer.reserve(1024);
    pinMode(LED_BUILTIN, OUTPUT);
    // init the SD card
    if (!SD.begin(10)) {
      Serial.println("Card failed, or not present");
      // don't do anything more:
      while (1);
    }
    // If you want to start from an empty file after each upload,
    // uncomment the next line:
    //SD.remove(filename);
  
    // try to open the file for writing
    txtFile = SD.open(filename, FILE_WRITE);
    if (!txtFile) {
      Serial.print("error opening ");
      Serial.println(filename);
      while (1);
    }
}
/////////////////////////Save Time Stamp//////////////////////////
void saveTime(int delta){
  if(firstIter==1){
    timeStamp += (delta)-startTime;
    firstIter=0;
    }else{
      timeStamp += (delta);
    }
    buffer += timeStamp;
    buffer += " "; 
}
/////////////////////////Save Buffer//////////////////////////
void saveBuffer(){
    buffer += "\r\n";//newline in txt file on sd card    
    // check if the SD card is available to write data without blocking
    // and if the buffered data is enough for the full chunk size
    unsigned int chunkSize = txtFile.availableForWrite();
    if (chunkSize && buffer.length() >= chunkSize) {
      //Serial.println ("Writing to SD:");
      // write to file and blink LED
      digitalWrite(LED_BUILTIN, HIGH);
      txtFile.write(buffer.c_str(), chunkSize);
      digitalWrite(LED_BUILTIN, LOW);
  
      // remove written data from buffer
      buffer.remove(0, chunkSize);
    }
}
//////////////////////////EDMO/CPG Positions////////////////////////////
//saves the angle that was send to the EDMO motors and the angle that was
//measured to the sd card buffer
void savePose(int sendPose, int readPose){
  buffer += " ";
  buffer += sendPose;
  buffer += " ";
  buffer += readPose;
}
