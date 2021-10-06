//PID Control Template

// *********************************************************************************************************************** //
// Variables                                                                                                               //
// *********************************************************************************************************************** //

//pin definition
#define ENA 9
#define IN1 21
#define IN2 20
#define SENSOR_PIN A0

// *********** Defining important constants and variables *********** //
// interval between recomputing error and adjusting feedback (in milliseconds)
const int INTERVAL = 10; 
unsigned long previousTime = 0;
unsigned long previousMicro = 0;

int motorSpeed = 0; // speed of the motor, values between 0 and 255
int target = 512; // position (as read by potentiometer) to move the motor to, default value 512
int prev_target = target; // previously set target, this is used to delete all records when a new target is selected

// These are the tuned parameters we found using Ziegler-Nichols' method and adjusting Kd to reduce overshoot
float kp = 12.0; // proportional gain
float ki = 0.18172; // integral gain
float kd = 220; // derivative gain

// We keep a long record to allow more flexibility in testing alternative implementations;
// The record-size seems large, but if 'record_interval' is set to a lower value, the second-interval of the record decreases, thus 2000 was a good sweet-spot
float error = 0;
float error_record[2000];
float timestamp_record[2000];

// record_interval was set to a value smaller than INTERVAL to allow the integral and derivative to have values to work with on the first step of a new target 
//  (after we delete all records to prevent wind-up), but the implementation difference with the PID-control Ziegler-Nichols is tuned to was too large,
//  thus we reverted this back to be the same interval.
const float record_interval = INTERVAL; // ms to wait between recording data
float last_record_time = 0;
// record_count keeps track of where we are in the record, if set to 0, we assume there are no records stored (stored values are not deleted for efficiency reasons)
int record_count = 0;

// Arbitrary error bounds when checking for difference in error
int error_bounds = 1;

// Bool to decide on whether to use this tuning or not
bool use_zn_tuning = false;

// Bool to check whether it's been done already and doesn't need to be repeated
bool zn_tuning_done = false;

// ********************** Code from the template ********************** //
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
        // ********************** Our code implemented here ********************** //
        unsigned long ct = millis();
    
        // This deletes all records when the target has changed from the previous timestep
        // We do this to reduce integral wind-up after target-shifts
        if (prev_target != target) {
          record_count = 0;
          prev_target = target;
        }
        
        readInput();
        pos = analogRead(SENSOR_PIN);
        error = target - pos;

        // Records are on a seperate loop; this loop currently uses the same interval as the control-loop;
        //  That is done due to implementation differences between ours and Ziegler-Nichols' PID controller (for tuning)
        // Otherwise, the records could be recorded more frequently than the control-loop, to give more information to integral and derivative calculations
        // This would require manual tuning however
        float ct_micro = micros()/1000.0;
        if (ct_micro - last_record_time > record_interval) {
          storeError();
          last_record_time = ct_micro;

          //print actual motor position and target value to serial-monitor/plotter
          Serial.print(pos);
          Serial.print(" ");   
          Serial.println(target);
        }

        if(ct-previousTime > INTERVAL){
          // *********** Control Loop *********** //
          float uP = kp * error; // proportional control
          float integral = integrate();
          float uI = -ki * integral; // integral control
          float uD = -kd * get_derivative(record_count, 1); // derivative control
          float U = uP + uI + uD; // total control output
          
          // If the error is below 7 degrees, we turn off the motor (LOW/LOW)
          if (abs(error) >= 7) {
            // as U is calculated from an input signal with range 0-1024, we scale it down to the range 0-255 that the motor expects
            setMovement(-U, abs(U)/4.0);
          } else {
            setMovement(-U, 0);
          }
          previousTime = millis();
          previousMicro = micros();
            
          // This was intended for use in Ziegler-Nichols tuning, but we ended up going for more a manual approach
          //  Code is untested but might work, although it does not accurately reflect our used tuning method
          if(use_zn_tuning && !zn_tuning_done){
            update_ziegler_nichols_tuning(error, target);
          }
          
        }
    
}

// Code responsible for storing records and timestamps
// When the record is full, the oldest 50 records are removed to make room for new records
void storeError() {
  int move_count = 50;
  if (record_count >= sizeof(error_record)/sizeof(float)-1) {
    for (int i=0; i < record_count-1; i++) {
      error_record[i] = error_record[i+move_count];
      timestamp_record[i] = timestamp_record[i+move_count];
    }
    record_count -= move_count;
  }
  error_record[record_count] = error;
  // timestamp is recorded in milliseconds but with microsecond precision
  timestamp_record[record_count] = (micros() - previousMicro) / 1000.0; 
  ++record_count;
}

// method to set direction and speed of the motor
// speed1 is expected in range 0-255
void setMovement(int dir, int speed1) 
{
        analogWrite(ENA, speed1);
        // dir > 0 is clockwise
        // dir = 0 is OFF (LOW/LOW)
        // dir < 0 is counter-clockwise
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

// author: Pascal
float integrate() {
  // Our integration uses the trapezoid rule on all available records and their timestamps
  float integral = 0;
  for (int i=0; i < record_count-1; i++) {
    float t0 = timestamp_record[i];
    float t1 = timestamp_record[i+1];
    float f0 = error_record[i];
    float f1 = error_record[i+1];
    integral += (t1 - t0) * 0.5 * (f1 + f0);
  }
  // Integral is capped between -20 and 20 to eliminate integral wind-up
  return max(min(integral,20),-20);
}

// author: Nicolas
float get_derivative(int err_count, int delta){
  if (record_count < delta) {
      return 0;
  }
  float top = error_record[err_count-delta] - error_record[err_count];
  // top = numerator
  // bot = denominator = h; h is the difference in milliseconds (microsecond precision) between the two records
  float bot = timestamp_record[err_count-delta] - timestamp_record[err_count];
  return top/bot;
}

// *********************************************** CODE UNCHANGED *********************************************** //
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
          } else if(i==1){
            target = PID_values[0];
          }
        }
    }
}

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

// ********************** DEPRECATED CODE ********************** //
/*
 * Method to implement the given tuning automatically
 * 
 * Updates the tuning values, so it assumes that ki, kd = 0 at start
 * 
 * cur_error is the location of the last error value, not the actual value
 * objective is what the system should actually be
 */
// author: Nicolas
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
// author: Nicolas
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
