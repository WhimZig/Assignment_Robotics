//Tutorial07 CPG Control Template

//////////////////////////////////////////////////////////////////////////////////////////////////////////
//Calibration of Servo Limits
//Please enter our specific servo limits for servo 1,2, (and 3) here
//You can find the corresponding limits in your EDMO box
int SERVOMIN[]  {100, 104, 106, 100}; //PLEASE ENTER: The lower motor PPM limit (Servo Min) as noted in your EDMO box
int SERVOMAX[]  {456, 450, 454, 442}; //PLEASE ENTER: The upper motor PPM limit (Servo Max) as noted in your EDMO box

int POTIMIN[]  {118, 145, 142, 130}; //PLEASE ENTER: The lower potentiometer limit (Poti Low) as noted in your EDMO box
int POTIMAX[]  {694, 747, 720, 709}; //PLEASE ENTER: The upper potentiometer limit (Poti High) as noted in your EDMO box

//////////////////////////////////////////////////////////////////////////////////////////////////////// 
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
//#include "study.h"

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
<<<<<<< Updated upstream
const unsigned int NUM_OSCILLATORS = 3; // this number has to match entries in array osc[] (do NOT modify!!)
=======
const int NUM_OSCILLATORS = 4; // this number has to match entries in array osc[] (do NOT modify!!)
// third motor wires are not connected, so we are limited to two oscilators in reality
>>>>>>> Stashed changes

//Poti variables
int POTIPINS[] = {14, 15, 16, 17, 18, 19}; //Do NOT modify!! Analog pins to which the potis' of the servos are connected (A0=14, A1=15, A2=16, ...)
float poti_value[] = {0, 0, 0, 0}; //Do NOT modify! Array to store raw poti values
int servo_angle[] = { 0, 0, 0, 0}; //Do NOT modify! Array to store mapped poti values

//Timing variables
unsigned long previousMillis = 0;
unsigned long currentMillis = 0;
unsigned long timeStep = 10; // period used to update CPG state variables and servo motor control (do NOT modify!!)
int interval = 0; //variable to store actual measured update time of the PID

//CPG parameter
<<<<<<< Updated upstream
double frequency = 0.5; // oscillator frequency
double rateOfFrequency = 0;
double targetFrequency = 0.5;
=======
double frequency = 0; // oscillator frequency; the current frequency, which slowly goes to target frequency when that changes
double gain_frequency = 1; // Adaptation rate for frequency, we replaced c by this variable and gain_offset
double rateOfFrequency = 0; // derivative of the frequency (can be used for plotting)
double targetFrequency = 0;
>>>>>>> Stashed changes

double w = 0.025; // we assume that all oscillators use same coupling weight
double a = 1; // we assume that all oscillators use same adaptation rate for amplitude
double c = 0.5; // adaptation rate for frequency and offset

//float calib[NUM_OSCILLATORS];

typedef struct 
{
    double phase;                       // phase of the oscillation
    double amplitude;                   // amplitude of the oscillation 
    double targetAmplitude;             // amplitude to gradually change to
    double offset;                      // offset for the oscillation (in range of servo 0-180)
    double targetOffset;                // added parameter to offset smoothly
    double rateOfPhase;                 // current rate of change of the phase parameter
    double rateOfAmplitude;             // current rate of change of the amplitude parameter
    double rateOfOffset;                // current rate of change of the offset parameter
    double pos;                         // oscillator output = servos angular position
    uint16_t angle_motor;               // mapped motor value
    double phaseBias[NUM_OSCILLATORS];  // controls pairwise coupling phase bias
    double coupling[NUM_OSCILLATORS];   // controls topology of the network
} oscillator;

// initalisation with offset 90 for all motors (since servos operate in the range 0-180)
oscillator osc[NUM_OSCILLATORS] = 
{
<<<<<<< Updated upstream
    {0,30,30,90,90,0,0,0,0,0,{0,PI,0},{0,1,0}},
    {0,30,30,90,90,0,0,0,0,0,{-PI,0,PI},{1,0,1}},
    {0,30,30,90,90,0,0,0,0,0,{0,-PI,0},{0,1,0}}
=======
    {0,30,30,90,90,0,0,0,0,0,{0,PI,0,0},{0,1,0,0}},
    {0,30,30,90,30,0,0,0,0,0,{-PI,0,0,0},{1,0,1,0}},
    {0,30,30,90,90,0,0,0,0,0,{0,0,0,PI},{0,1,0,1}},
    {0,30,30,90,90,0,0,0,0,0,{0,0,-PI,0},{0,0,1,0}}
>>>>>>> Stashed changes
};

// strings for reading input
String  valueString, indexString;
String commandString = "Start 0 0";
////////////////////////////// setup //////////////////////////////////////
//(do NOT modify!!)
void setup() 
{
    //////////////Initialize Serial Communication//////////////////
    Serial.begin(9600);
    Serial.flush();//(do NOT modify!!)
    while (!Serial); ////(do NOT modify!!) wait for the USB to serial chip to get ready (really quick!!)Does NOT! wait for the serial monitor to open!!!!
    ///////Initialize SD-Card /////////////////////
//    initSD();//(do NOT modify!!)
    ///////Initialize Real time clock ////////////   
//    initRTC(); //(do NOT modify!!)         
    ///////Motor control/////////////////////////////////
    pwm.begin();
    pwm.setPWMFreq(50);  // Analog servos run at ~60 Hz update
    delay(4);
//    zeroCalib();
//    setCalib(0,0);
//    setCalib(1,0);
//    setCalib(2,0);
}

////////////////////////////// loop //////////////////////////////////////
void loop(){   
   
    currentMillis = millis(); //update the current time (do NOT modify!!)
    if (currentMillis - previousMillis >= timeStep){//CPG update interval (10ms)(do NOT modify!!)
      interval=currentMillis - previousMillis; //calculate actual interval time (do NOT modify!!)
//      saveTime(interval); //saves current interval time (do NOT modify!!)
      previousMillis = currentMillis; //update the previous time step (do NOT modify!!)
      readInput(); //read input command from serial monitor (do NOT modify!!)
      
      //+++++++++++++++IMPLEMENT your CPG control code here BELOW !!!++++++++++++++++++++++++++++++++++++
                          
      

      for (int i = 0; i < NUM_OSCILLATORS; i++) {

        // Calculate CPG here
        //double deriv_phase = compute_phase_derivative(i);
        //double deriv_r = equation_2(i);
        
        osc[i].pos = equation_3(i);

        // set motor to new position (do NOT modify!!)
        osc[i].angle_motor = map(osc[i].pos,0,180,SERVOMIN[i],SERVOMAX[i]);//(do NOT modify!!)
        osc[i].angle_motor = constrain(osc[i].angle_motor,SERVOMIN[i],SERVOMAX[i]);  //(do NOT modify!!)       
        pwm.setPWM(i, 0, osc[i].angle_motor); //(do NOT modify!!)         
        poti_value[i] = analogRead(POTIPINS[i]);//(do NOT modify!!)
        servo_angle[i] = map(poti_value[i], POTIMIN[i] , POTIMAX[i], 0, 180);//(do NOT modify!!)

      
        Serial.print( osc[i].pos);
        Serial.print (" ");        
      }
      //+++++++++++++++IMPLEMENT your CPG control code here ABOVE!!!++++++++++++++++++++++++++++++++++++
      Serial.println();
      for(int t=0;t<NUM_OSCILLATORS;t++){//(do NOT modify!!)
//        savePose(osc[t].pos,servo_angle[t]);//saves positions to sd-buffer (do NOT modify!!)
      }
//      saveBuffer();//upload buffer to sd (do NOT modify!!)
    }
}

double compute_phase_derivative(int i) {
  double sum = 0;
  for (int j=0; j < sizeof(osc)/sizeof(oscillator); j++) {
    if (i != j) {
      sum += w * osc[i].coupling[j] * osc[j].amplitude * 
        sin(osc[j].phase - osc[i].phase - osc[i].phaseBias[j]);
    }
  }
  return 2*PI*frequency + sum;
}

/////////////////Function for Reading Inputs via the Serial Monitor//////////////////////////////
///////////////////////////- PLEASE DO NOT MODIFY!/////////////////////////////////
void readInput() 
{
    //if there is an input via the serial monitor read and parse it
    if (Serial.available()) 
    {
<<<<<<< Updated upstream
      spaceCount = 0;
      recFlag =1;
      commandString = Serial.readStringUntil('\n'); //read received string 
=======
      int spaceCount = 0;
      int recFlag =1;
      commandString = Serial.readStringUntil('\n'); //read received string
>>>>>>> Stashed changes
      /////////////////PARSE INPUT//////////////////////////////////////////
      //count the number of spaces in the string 
      for(int i=0; i<=commandString.length();i++){
        if(commandString.charAt(i)== ' '){
          spaceCount=spaceCount+1;
        }         
      }
      //Check for the different input commands, extract the input paramters, convert them to integers and save the results in the sd-card buffer
      if (recFlag ==1 && spaceCount == 2 && (commandString.startsWith("amp") || commandString.startsWith("off"))){
          // extract and save the input command
          indexString = commandString.substring(4, 5);
          valueString = commandString.substring(6, commandString.length());
          //convert input parameters to integers
          if(commandString.startsWith("amp")){
            osc[(int) indexString.toInt()].targetAmplitude = (int) valueString.toInt();
          }else if(commandString.startsWith("off")){
            osc[(int) indexString.toInt()].targetOffset = (int) valueString.toInt();
<<<<<<< Updated upstream
          } 
          //save received command string to buffer    
          buffer += commandString;
          buffer += " ";
          buffer += "-";
=======
          }
          //save received command string to buffer
//          buffer += commandString;
          //buffer += " ";
          //buffer += "-";
>>>>>>> Stashed changes
          recFlag =0;
      }else if (recFlag ==1 && spaceCount == 1 && (commandString.startsWith("freq") || (commandString.startsWith("weight")))){ 
          // change the target frequency for all oscillators
          if(commandString.startsWith("freq")){
            valueString = commandString.substring(5, commandString.length());
            targetFrequency = (float) valueString.toFloat();
          }else if(commandString.startsWith("weight")){
            valueString = commandString.substring(7, commandString.length());
            w = (float) valueString.toFloat();
          }
<<<<<<< Updated upstream
          //save received command string to buffer  
          buffer += commandString;
          buffer += " ";  
          buffer += "-";
          buffer += " "; 
          buffer += "-";
          recFlag =0;        
      } else if (recFlag ==1 && spaceCount == 3 && commandString.startsWith("phb")){ 
=======
          //save received command string to buffer
          /*buffer += commandString;
          buffer += " ";
          buffer += "-";
          buffer += " ";
          buffer += "-";*/
          recFlag =0;
      } else if (recFlag ==1 && spaceCount == 3 && commandString.startsWith("phb")){
>>>>>>> Stashed changes
          // change the phase bias between the two specified oscillators
          indexString = commandString.substring(4, 5);
          int index1 = (int) indexString.toInt();
          indexString = commandString.substring(6, 7);
          int index2 = (int) indexString.toInt();
          valueString = commandString.substring(8, commandString.length());
          osc[index1].phaseBias[index2] =  ((PI * valueString.toFloat())/180);//deg2rad!!! (float)
          osc[index2].phaseBias[index1] = -osc[index1].phaseBias[index2];
<<<<<<< Updated upstream
          //save received command string to buffer  
          buffer += commandString;
          recFlag =0;      
      } else if (recFlag ==1 && spaceCount == 0 && commandString.startsWith("print")){ 
          //save received command string to buffer  
          buffer += commandString;
          buffer += " ";  
          buffer += "-";
          buffer += " "; 
          buffer += "-"; 
          buffer += " "; 
          buffer += "-";
=======
          //save received command string to buffer
          //buffer += commandString;
          recFlag =0;
      } else if (recFlag ==1 && spaceCount == 0 && commandString.startsWith("print")){
          //save received command string to buffer
//          buffer += commandString;
//          buffer += " ";
          /*buffer += "-";
          buffer += " ";
          buffer += "-";
          buffer += " ";
          buffer += "-";*/
>>>>>>> Stashed changes
          recFlag =0;
          // print information about the current state of the oscillators
          Serial.print("Frequency: ");
          //Serial.println(frequency);
          for (int i = 0; i < NUM_OSCILLATORS; i++) 
          {
              Serial.print(i);
              Serial.print(": ");
              Serial.print("[");
              Serial.print(osc[i].phase);
              Serial.print(", ");
              Serial.print(osc[i].amplitude);
              Serial.print(", ");
              Serial.print(osc[i].targetAmplitude);
              Serial.print(", ");
              Serial.print(osc[i].offset);
              Serial.print(", ");
              Serial.print(osc[i].targetOffset);
              Serial.print(", ");
              Serial.print(osc[i].rateOfPhase);
              Serial.print(", ");
              Serial.print(osc[i].rateOfAmplitude);
              Serial.print(", ");
              Serial.print(osc[i].rateOfOffset);
              Serial.print(", ");
              Serial.print(osc[i].pos);
              Serial.print(", [");
              Serial.print(osc[i].phaseBias[0]);
              Serial.print(", ");
              Serial.print(osc[i].phaseBias[1]);
              Serial.print(", ");
              Serial.print(osc[i].phaseBias[2]);
              Serial.print("]]");
              Serial.println();
          }
        //in case none of the above input variations is correct it is declared as an invalid input
        }else if (recFlag ==1){
<<<<<<< Updated upstream
          //save received command string to buffer  
          buffer += "Invalid Input";
          buffer += " "; 
          buffer += "-"; 
          buffer += " "; 
          buffer += "-";
=======
          //save received command string to buffer
          /*buffer += "Invalid Input";
          buffer += " ";
          buffer += "-";
          buffer += " ";
          buffer += "-";*/
>>>>>>> Stashed changes
          recFlag =0;
        }
    //if there is NO new input that has been received via the serial monitor, fill the empty spots in the .txt file with dashes
    }else{
      /*buffer += "-";
      buffer += " ";
      buffer += "-";
      buffer += " ";  
      buffer += "-";
<<<<<<< Updated upstream
      buffer += " "; 
      buffer += "-"; 
      buffer += " "; 
=======
      buffer += " ";*/
>>>>>>> Stashed changes
    }
}

double equation_2(int osc_num){
  double R = osc[osc_num].targetAmplitude;
  double r = osc[osc_num].amplitude;
  return a*(R-r);
}

double equation_3(int osc_num){
  // This method assumes that the phase is in radians already
  double r = osc[osc_num].amplitude;
  double fi = osc[osc_num].phase;
  double x = osc[osc_num].targetOffset;

  double res = r*sin(fi) + x;
  return res;
}
///////////////////////////////////////////////////////////////////////////////////
//void zeroCalib()
//{
//    for (byte j = 0 ; j < NUM_OSCILLATORS ; j++)
//      calib[j] = 0;
//}
//
//void setCalib(int motor,int val)
//{
//    if(motor < NUM_OSCILLATORS)
//        calib[motor] = val;
//    else
//       Serial.println("Enter a valid motor number"); 
//}
