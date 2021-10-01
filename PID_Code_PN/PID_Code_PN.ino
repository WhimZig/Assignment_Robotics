//PID Control Template

// *********************************************************************************************************************** //
// Variables                                                                                                               //
// *********************************************************************************************************************** //

//pin definition
#define ENA 9
#define IN1 21
#define IN2 20
#define SENSOR_PIN A0

// ******** <TODO> **********************
// ******** define interval between recomputing error and adjusting feedback (in milliseconds) ********************** 
const int INTERVAL = 10; 
unsigned long previousTime = 0;
unsigned long previousMicro = 0;

int motorSpeed = 0; // speed of the motor, values between 0 and 255
int target = 512; // position (as read by potentiometer) to move the motor to, default value 512

// ******** <TODO> **********************
// ******** define the different gains **********************
float kp = 3.0; // proportional gain
float ki = 0.0; // integral gain
float kd = 0.0; // derivative gain

float error = 0;
float error_record[1000];
float timestamp_record[1000];
int record_count = 0;
// with INTERVAL=10, record_count ~= 530 at time step

int pos = 0; // current position for plotting
//serial communication variables
float PID_values[4];
byte i = 0;
char record[100];
char recvchar;
byte indx = 0;

// setup code, setting pin modes and initialising the serial connection
void setup() 
{
    Serial.begin(115200);

    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);    
    pinMode(SENSOR_PIN, INPUT);

    previousMicro = micros();
}

void loop() 
{
        //  ******** <TODO> **********************
        //  ******** implement your code  here **********************
        unsigned long ct = millis();
        
        readInput();
        pos = analogRead(SENSOR_PIN);
        error = target - pos;

        error_record[record_count] = error;
        timestamp_record[record_count] = (micros() - previousMicro) / 1000.0;
        ++record_count;

        if(ct-previousTime > INTERVAL){
          float uP = kp * error;
          float integral = integrate();
          float uI = -ki * integral;
          float uD = 0;
          float U = uP + uI + uD;
          if (abs(error) >= 5) {
            setMovement(-U, U/4.0);
          } else {
            setMovement(-U, 0);
          }
          previousTime = millis();
          previousMicro = micros();
          record_count = 0;

          // MORE GARBAGE CODE BY NICK
          if(use_zn_tuning && !zn_tuning_done){
            update_ziegler_nichols_tuning(error, targer);
          }
          // GARBAGE CODE ENDS HERE
          
        }
        
        //print actual motor position and target value to serial-monitor/plotter
        Serial.print(pos);
        Serial.print(" ");   
        Serial.println(target);
    
}

float integrate() {
  float integral = 0;
  for (int i=0; i < record_count-1; i++) {
    float t0 = timestamp_record[i];
    float t1 = timestamp_record[i+1];
    float f0 = error_record[i];
    float f1 = error_record[i+1];
    integral += (t1 - t0) * 0.5 * (f1 + f0);
  }
  return integral;
}

// method to set direction and speed of the motor
void setMovement(int dir, int speed1) 
{
        analogWrite(ENA, speed1);
        // dir > 0 is clockwise
        // otherwise, go counter-clockwise
        if (speed1==0) {
          digitalWrite(IN1, LOW);
          digitalWrite(IN2, LOW);
        }
        if (dir>0) {
          digitalWrite(IN1, HIGH);
          digitalWrite(IN2, LOW);
        } else {
          digitalWrite(IN1, LOW);
          digitalWrite(IN2, HIGH);
        }
}
// *********************************************************************************************************************** //
// method for receiving commands over the serial port
void readInput() 
{
      if (Serial.available())
    {
        recvchar = Serial.read();
        if (recvchar != '\n')
        { 
            record[indx++] = recvchar;
        }
        else if (recvchar == '\n')
        {
          record[indx] = '\0';
          indx = 0;
          
          convertData(record);
          if(i==4){
          target = PID_values[0];
          kp = PID_values[1];
          ki = PID_values[2];
          kd = PID_values[3];
          //Serial.print("Entered Values:");
          printData(PID_values);
          }
          else
          {
          //Serial.println("Enter correct number of values separated by commas!!");            
          }
        }
    }
}
// *********************************************************************************************************************** //
//method for reading/interpreting serial input 
void convertData(char record[])
{
    i = 0;
    char *index = strtok(record, ",");
    while(index != NULL)
    {
       PID_values[i++] = atof(index); 
        index = strtok(NULL, ",");
    }
}
// *********************************************************************************************************************** //
//method for printing values entered via the serial monitor
void printData(float data[])
{
    for (byte j = 0 ; j < 4 ; j++)
    {
      Serial.print(data[j]);
      Serial.print('\t');
    }
    Serial.println(); 
}

// CODE WRITTEN BY NICK AFTER HOURS THAT MIGHT SUCK DICK

// Arbitrary error bounds when checking for difference in error
int error_bounds = 1;

// Bool to decide on whether to use this tuning or not
bool use_zn_tuning = true;

// Bool to check whether it's been done already and doesn't need to be repeated
bool zn_tuning_done = false;

float get_derivative(int err_count){
  // So, top = numerator
  float top = error_record[err_count-1] - error_record[err_count];

  // bot = denominator
  // Reason it's this value it because this is the only way of getting h
  float bot = timestamp_error[err_count-1] - timestamp_error[err_count];
  return top/bot;
}

/*
 * Method to implement the given tuning automatically
 * 
 * Updates the tuning values, so it assumes that ki, kd = 0 at start
 * 
 * cur_error is the location of the last error value, not the actual value
 * objective is what the system should actually be
 */

void update_ziegler_nichols_tuning(int cur_error, int objective){

  int period = determine_if_error_is_period(cur_error, objective);

  if(period < 0){
    // So it's not periodic, meaning we increase kp again
    kp = kp*1.1;
  } else {
    // So the function is currently periodic, so we update the values nearby
    kp = 0.6*kp;

    ki = kp*period/4.0;
    kd = kp*period/8.0;
    zn_tuning_done = true;

    // And yeah, I think that's it
  }
  
}

/*
 * Assume that oscillating means that a previous error value is similar to current one
 *  Within bounds of delta
 *  Also assumes that error must reach around the desired value
 * Will avoid straight lines by making sure each consecutive value is greater than error_bounds
 * 
 * Returns:
 *  Size of period of the oscillation if there's one
 *    Period is measured as number of values saved per loop, not actual time
 *  -1 if it's not periodic
 *  
 *  Method uses error bounds as I don't remember how error was measured
 *  And this is good enough to avoid major issues
 *  
 *  IMPORTANT: Doesn't take into account cases where it never tends towards something
 *    So this would say that the endless loop towards 0 or 1024 are oscillating
 *    Can't think of a way of dealing with that as of now
 *    For now just assume it's not a problem?
 *    Or never tune towards those numbers?
 */


int determine_if_error_is_period(int cur_error, int desired_value){
  
  // First checks if there's an error that is near desired_value
  int closest_error_location = -1;
  for(int i=cur_error-1; i>0; --i){
    int var = desired_value - error_record[i];

    if (abs(var) < error_bounds){
      closest_error_location = i;
      break;
    }
    
  }

  if(closest_error_location == -1){
    return -1;
  }

  // Now it checks if there's an actual oscillation
  // Requires for each consecutive value to vary by more than var, if not it'll read it as a straight line

  for(int i=closest_error_location-1; i>0; --i){
    int var = error_record[i] - error_record[i+1];

    // This means there's not enough variation at each time step
    // So I take it to mean there's no period
    if (abs(var) < error_bounds){
      return -1;
    }

    // So if it's a repeat value within error bounds, then it's oscillating
    if(abs(error_record[i] - error_record[closest_error_location]) < error_bounds){
      return (closest_error_location - i);
    }
    
  }

  // Means no oscillation was found in the bounds, so repeat
  return -1;
  
}


// HERE IS WHERE MY CODE ENDS
