//Tutorial07 CPG Control Code
// Nicolas Perez & Pascal Anema
// See BELOW for extensive comments and theoretical explanation of observations
//////////////////////////////////////////////////////////////////////////////////////////////////////////
//Calibration of Servo Limits
//Please enter our specific servo limits for servo 1,2, (and 3) here
//You can find the corresponding limits in your EDMO box
int SERVOMIN[]  {100, 104, 105}; //PLEASE ENTER: The lower motor PPM limit (Servo Min) as noted in your EDMO box
int SERVOMAX[]  {456, 450, 465}; //PLEASE ENTER: The upper motor PPM limit (Servo Max) as noted in your EDMO box

int POTIMIN[]  {118, 145, 130}; //PLEASE ENTER: The lower potentiometer limit (Poti Low) as noted in your EDMO box
int POTIMAX[]  {694, 747, 710}; //PLEASE ENTER: The upper potentiometer limit (Poti High) as noted in your EDMO box

//////////////////////////////////////////////////////////////////////////////////////////////////////// 
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "study.h"

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
const int NUM_OSCILLATORS = 3; // this number has to match entries in array osc[] (do NOT modify!!)
// third motor wires are not connected, so we are limited to two oscilators in reality

//Poti variables
int POTIPINS[] = {14, 15, 16, 17, 18, 19}; //Do NOT modify!! Analog pins to which the potis' of the servos are connected (A0=14, A1=15, A2=16, ...)
float poti_value[] = {0, 0, 0}; //Do NOT modify! Array to store raw poti values
int servo_angle[] = { 0, 0, 0}; //Do NOT modify! Array to store mapped poti values

//Timing variables
unsigned long previousMillis = 0;
unsigned long currentMillis = 0;
unsigned long timeStep = 10; // period used to update CPG state variables and servo motor control (do NOT modify!!)
int interval = 0; //variable to store actual measured update time of the PID (in ms)

//********************* COMMENTS AS EXPLANATION AND FOR HIGHLIGHTING OF THEORY *********************//
/*
 * We added gain parameters for frequency and offset seperately, as we felt it was asapting too rapidly or too slowly on either one otherwise
 * These gain parameters are weights to the proportional control of the offset and frequency of the oscillators (frequency is the same for all oscillators)
 * The same thing was implemented for the amplitude (gain parameter a),
 *  whereas the control for the phase is done through a central pattern generator (coupling-weight parameter w)
 * All of these also have a corresponding "rateOf" parameter which holds their derivative.
 * 
 * Whenever a gain parameter is *increased*, theory dictates that we should see a faster rate of convergence for changes to the corresponding control parameter (frequency, amplitude, etc.)
 * In figures, this means that we can see the derivative go towards 0 over time (if we don't change the control parameter),
 *   this happens *faster* if the gain is *higher*
 * In practice, we should see the motor-pair 'settle' on a consistent pattern *faster* with *higher* gain, which is **exactly what we observed**
 * For w, this particularly means that the phase difference between two oscillators will converge (in practice: become visibly constant) faster
 * IE: "it takes less time for two in-phase oscillators to become anti-phase if w is higher"
 */
//CPG parameter
double frequency = 0.5; // oscillator frequency; the current frequency, which slowly goes to target frequency when that changes
double gain_frequency = 1; // Adaptation rate for frequency, we replaced c by this variable and gain_offset
double rateOfFrequency = 0; // derivative of the frequency (can be used for plotting)
double targetFrequency = 0.5;

double w = 0.025; // we assume that all oscillators use same coupling weight
double a = 1; // we assume that all oscillators use same adaptation rate for amplitude
double gain_offset = 0.1; // we assume that all oscillators use same adaptation rate for offset

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
    {0,30,30,90,90,0,0,0,0,0,{0,PI,0},{0,1,0}},
    {0,30,30,90,90,0,0,0,0,0,{-PI,0,PI/2},{1,0,1}},
    {0,30,30,90,90,0,0,0,0,0,{0,-PI/2,0},{0,1,0}}
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
    initSD();//(do NOT modify!!)
    ///////Initialize Real time clock ////////////   
    initRTC(); //(do NOT modify!!)         
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
      saveTime(interval); //saves current interval time (do NOT modify!!)
      previousMillis = currentMillis; //update the previous time step (do NOT modify!!)
      readInput(); //read input command from serial monitor (do NOT modify!!)
      
      //+++++++++++++++IMPLEMENT your CPG control code here BELOW !!!++++++++++++++++++++++++++++++++++++

      // We update the amplitude, offset and frequency BEFORE updating the phase and position,
      // We do this because the coupling sum depends on these variables, and if they change in the same loop
      //  as the phase and position, then unexpected results may occur
      
      // This behavior is slightly intended for phase-changes however, because if all the phases are changed at the same
      //  time, then we might get stuck in an infinite loop (and never reach desired phase bias) in specific situations
      //  thus, they are still changed in order of index, but they will at least all have their equally updated control variables (amplitude, frequency, etc.)
      updateVariables(interval);

      for (int i = 0; i < NUM_OSCILLATORS; i++) {

        // Calculate CPG here
        /* This calculation makes sure phase bias is kept for coupled oscillators.
         * This is done through the drivative of the phase to make sure all transitions are smooth.
         * The phase is then put through an output_function to get the target position for the motor (fed to the PID controller)
         * We can change this output_function to get nice interesting alterations of the sinusoid movement, we implemented one
         *   example of this in the "holding_sin" function (see comments at definition for instructions).
         */
        osc[i].rateOfPhase = compute_phase_derivative(i);
        osc[i].phase = osc[i].phase + osc[i].rateOfPhase * interval /1000.0;
        osc[i].pos = output_function(i);
        
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
        savePose(osc[t].pos,servo_angle[t]);//saves positions to sd-buffer (do NOT modify!!)
      }
      saveBuffer();//upload buffer to sd (do NOT modify!!)
    }
}

//**************************** OUR METHODS ****************************//

/*
 * Implemented with default implementation;
 * w_{ij} is replaced by w * coupling[i][j] as we assume a constant weight for all couplings
 * However, different values for coupling weights can still be used as 'coupling' is a double array
 * This may be useful for generating advanced patterns, although we did not do a lot of experimenting with this
 */
double compute_phase_derivative(int osc_num) {
  int i = osc_num;
  double sum = 0;
  for (int j=0; j < NUM_OSCILLATORS; j++) {
    if (i != j) {
      sum += w * osc[i].coupling[j] * osc[j].amplitude * 
        sin(osc[j].phase - osc[i].phase - osc[i].phaseBias[j]);
    }
  }
  return 2*PI*frequency + sum;
}

/*
 * A sin function as output function.
 * To use the holding_sin function, replace "sin" in this method
 *  with "holding_sin" and see the change in behavior :)
 */
double output_function(int osc_num){
  // This method assumes that the phase is in radians already
  double r = osc[osc_num].amplitude;
  double fi = osc[osc_num].phase;
  double x = osc[osc_num].offset;

  double res = r*sin(fi) + x;
  return res;
}

// alternative output function (used for walk-like behavior during competition, not shown in assignment)
// to use, replace "sin(fi)" in the above "output_function" with "holding_sin(fi)" 
// we hope this might help with more walking-like behavior for interlocking legs
double holding_sin(double fi) {
  double phase = fi / (2*PI);
  if (phase < 0) { phase -= 1; }
  phase -= trunc(phase);
  phase *= 2; // phase in terms of pi
  if (phase < 0.25 || (phase >= 1.75 && phase < 2)) {
    return sin(phase*PI * 2);       // for 0.5PI, we return sin at double frequency (UP)
  } if (phase < 0.75) {
    return 1;                       // for 0.5PI, we hold at maximum amplitude
  } if (phase < 1.25) { 
    return sin((phase-0.5)*PI * 2); // for 0.5PI, we return sin at double frequency (DOWN)
  } if (phase < 1.75) {
    return -1;                      // for 0.5PI, we hold at minimum amplitude
  }
  return 0; // this return will not be reached, but is here as failsafe
}

/*
 * This method computes and stores the derivatives of the amplitude and offset; d-frequency is computed in "updateVariables"
 * We assume only P control is enough to manage the change in amplitude, offset and frequency
 * Phase derivatives are computed and stored seperately in the "compute_phase_derivative" method
 */
void compute_derivates(int osc_num) {
  double R = osc[osc_num].targetAmplitude;
  double r = osc[osc_num].amplitude;
  osc[osc_num].rateOfAmplitude = a*(R-r);
  
  double X = osc[osc_num].targetOffset;
  double x = osc[osc_num].offset;
  osc[osc_num].rateOfOffset = gain_offset*(X-x);
}

/*
 * We use Euler's method for simple integration of the computed derivatives
 * The interval is used as time-step; Rates of change are assumed to be in units/second
 * This method is executed *before* computing the phase derivatives of all oscillators,
 *   this is done to make sure no unexpected behavior occurs
 */
void updateVariables(double interval) {
  rateOfFrequency = gain_frequency * (targetFrequency - frequency);
  frequency = frequency + rateOfFrequency * interval / 1000.0;

  for (int i=0; i < NUM_OSCILLATORS; i++) {
    compute_derivates(i);
    osc[i].amplitude = osc[i].amplitude + osc[i].rateOfAmplitude * interval / 1000.0;
    osc[i].offset = osc[i].offset + osc[i].rateOfOffset * interval/1000.0;
  }
}

//**************************** END OF OUR METHODS ****************************//

/////////////////Function for Reading Inputs via the Serial Monitor//////////////////////////////
///////////////////////////- PLEASE DO NOT MODIFY!/////////////////////////////////
void readInput()
{
    //if there is an input via the serial monitor read and parse it
    if (Serial.available())
    {
      spaceCount = 0;
      recFlag =1;
      commandString = Serial.readStringUntil('\n'); //read received string
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
          }
          //save received command string to buffer
          buffer += commandString;
          buffer += " ";
          buffer += "-";
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
          //save received command string to buffer
          buffer += commandString;
          buffer += " ";
          buffer += "-";
          buffer += " ";
          buffer += "-";
          recFlag =0;
      } else if (recFlag ==1 && spaceCount == 3 && commandString.startsWith("phb")){
          // change the phase bias between the two specified oscillators
          indexString = commandString.substring(4, 5);
          int index1 = (int) indexString.toInt();
          indexString = commandString.substring(6, 7);
          int index2 = (int) indexString.toInt();
          valueString = commandString.substring(8, commandString.length());
          osc[index1].phaseBias[index2] =  ((PI * valueString.toFloat())/180);//deg2rad!!! (float)
          osc[index2].phaseBias[index1] = -osc[index1].phaseBias[index2];
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
          //save received command string to buffer
          buffer += "Invalid Input";
          buffer += " ";
          buffer += "-";
          buffer += " ";
          buffer += "-";
          recFlag =0;
        }
    //if there is NO new input that has been received via the serial monitor, fill the empty spots in the .txt file with dashes
    }else{
      buffer += "-";
      buffer += " ";
      buffer += "-";
      buffer += " ";
      buffer += "-";
      buffer += " ";
      buffer += "-";
      buffer += " ";
    }
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
