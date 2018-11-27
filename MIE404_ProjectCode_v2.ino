/* 

MIE404 Control Systems 1 
Propeller Pendulum Project

Code Written by Lilly Yu, Francis Cruz, and Amy Bilton  

Last Updated: 2018-10-30


*/ 

//---------------------LIBRARIES---------------------------// 
/*
  Download the ESC library here: https://github.com/RB-ENantel/RC_ESC/ 
*/ 

#include "ESC.h"

//---------------------ESC SETTINGS-----------------------// 
#define SPEED_MIN (1000)                                  // Set the Minimum Speed Setting of the ESC
#define SPEED_MAX (2000)                                  // Set the Maximum Speed Setting of the ESC
                      
ESC myESC (9, SPEED_MIN, SPEED_MAX, 500);                 // Initialize ESC (ESC PIN, Minimum Value, Maximum Value, Default Speed)
int oESC;                                                 // Variable for the speed sent to the ESC

/*
  The variables and ports defined below are to guide you when writing the code, but feel free to change or add/delete as nessecary. 
*/ 

//---------------------ENCODER PORTS----------------------// 
#define outputA 2                                         // Define Encoder output A to Digital Pin 2 on Arduino Uno 
#define outputB 3                                         // Define Encoder output B to Digital Pin 3 on Arduino Uno
#define NUMPULSES 1200                                    // Define the number of pulses on Encoder resolution (enter twice the amount of pulses on datasheet)

//---------------------ENCODER VARIABLES------------------//
int counter = 0;                                          // Counter variable for the number of pulses from the encoder
int aState = LOW;                                         // Digital state of the encoder output
int aLastState = LOW;                                     // Past state of the encoder output

//---------------------TIMER VARIABLES---------------------//
long t_now = 0;
long t_last_print = 0;
long t_last_PID = 0;
int T_sample = 5;  //20;                                        // sample time in milliseconds (ms)
int T_print = 10;  //1000;                                       // sample print to monitor time in milliseconds (ms)

//---------------------PID VARIABLES----------------------//
// Define variables for the output of the encoder(sensed_output), angle output of the encoder(sensed_output converted to angle), and error w.r.t the setpoint
double sensed_output, error, sensed_angle;   
             
// ==================INSERT DESIRED SETPOINT ANGLE HERE================== //
double setpoint_angle = 30; 
// ====================================================================== //


// Define variables for total error over time, previous error in the last sampling time interval, control signal, and limitations to the controller
double setpoint = setpoint_angle * NUMPULSES/360;
long total_error = 0;
double last_error = 0;
long control_signal; 
int max_control = 1700;
int min_control = 1000; 

                      
// ==================INSERT CONTROL GAINS HERE=========== 
double Kp = 0.8;                                          // proportional gain
double Ki = 0.003;                                        // integral gain in [ms^-1]
double Kd = 120;                                          // derivative gain in [ms]


void setup() {
  /* 
    Setup function to initialize serial plotter/monitor, initialize origin of encoder, 
    arm ESC driver, and to initialize propeller by ramping up and down
  */ 

  Serial.begin(9600);
  aLastState = digitalRead(outputA);                        // Reads the initial state of the outputA 

  myESC.arm();                                              // Send the Arm value so the ESC will be ready to take commands
  delay(7500);                                              // Wait for a while
  rampUpDown();                                             // Propeller speed to ramp up and down to check functionality
  
  pinMode (outputA,INPUT);
  pinMode (outputB,INPUT);
  
}


void loop() {
  /* 
    Main loop of the project. 
    1. Read Encoder
    2. Implement PID control
    3. Send control signal to motor
    4. Print sensed angle with respect to the setpoint

  */
  
 // Read the encoder 
  encoder_read();

 // Take the magnitude of the encoder output (accounting for CW or CCW rotation)
  sensed_output = abs(counter);

  // Implement PID control
  PID_Control();
 
 // Write control signal to motor
  myESC.speed(control_signal);   
     
 // Print sensed angle and setpoint angle
  print_results();

}


void rampUpDown() {
  /* 
    Function written to test functionality of brushless motor by accelerating and decelerating the rotation 
    of the motor. Speed of the motor does not accelerate to rated speed. 

  */

  for (oESC = SPEED_MIN; oESC <= 1150; oESC += 1) {        // iterate from minimum speed to a speed setting of 1150
    myESC.speed(oESC);                                     // write speed setting to motor
    delay(10);                                             // waits 10ms for the ESC to reach speed
  }
  delay(2000);                                             // wait a while
  
  for (oESC = 1150; oESC >= SPEED_MIN; oESC -= 1) {        // iterate from speed setting of 1150 to minimum speed
    myESC.speed(oESC);                                     // write speed setting to motor
    delay(10);                                             // waits 10ms for the ESC to reach speed  
  }
  delay(2000);                                             // Wait for a while before going into control loop
}


void encoder_read() {
  /* 
    STUDENTS TO ANSWER
    Function to count the number of pulses from the encoder output. 
    Must count at least 2 count per pulse, which would give you 1200 count per rotation.  
    
    *****DO NOT USE INTERUPTS TO READ ENCODER, IT COULD INTERFERE WITH THE CONTROLLER CALCULATIONS, USE IT AT YOUR OWN RISK*****  

  */ 
  
  // ==================INSERT ENCODER ALGORITHM HERE ==================== //
   aState = digitalRead(outputA); // Reads the "current" state of the outputA
   // If the previous and the current state of the outputA are different, that means a Pulse has occured
   if (aState != aLastState){     
     // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
     if (digitalRead(outputB) != aState) { 
       counter++;
     } 
	 else {
       counter--;
     }
   } 
   aLastState = aState; // Updates the previous state of the outputA with the current state
  // ==================INSERT ENCODER ALGORITHM HERE ==================== //

   
}


void PID_Control() {
  /* 
    STUDENTS TO ANSWER
    Function to implement PID control. 
    Input: encoder position
    Output: motor speed
    
    Steps:
      1. Determine amount of time that has passed
      2. Determine 'if statement' to implement the use of a sampling time to implement control signal
        (i.e. create if statement to represent sampling time and to calculate the control signal every interval)
      3. Calculate the current error.
      4. Calculate control output assuming all gains of the PID are defined; a min and max limit must be placed on the output ESC speed signal
        (HINT: use finite difference approx for derivative, use rectangular approximation method to estimate area 
        under curve for the integral term)

  */

  t_now = millis();                  // returns the number of milliseconds passed since the Arduino started running the program
 
  if (t_now - t_last_PID >= T_sample){      // if the elapsed time is greater than the sampling time, time to send control signal
    t_last_PID = t_now;

    // ==================INSERT CONTROL ALGORITHM HERE ==================== //
  // Calculate current error
  //sensed_output = abs(counter);
  sensed_angle = sensed_output/(NUMPULSES/360);
  error = setpoint_angle - abs(sensed_angle);

  // Calculate control output
  
	// integral_term += ((last_error + error)/2)*T_sample;
	// total_error = Kp*error + Ki*(((last_error + error)/2)*T_sample) + Kd*((error - last_error)/T_sample);
  
  total_error += ((last_error + error)/2)*T_sample;
  control_signal = Kp*error + Ki*total_error + Kd*((error - last_error)/T_sample);
  
  if (control_signal > max_control) {
	control_signal = max_control;
  }
  else if (control_signal < min_control) {
    control_signal = min_control;
  }
  //else {
	//control_signal = ;
  //}
  
  last_error = error;

    // ==================INSERT CONTROL ALGORITHM HERE ====================== //
    
  }  
}


void print_results() {
  /* 
    STUDENTS TO ANSWER
    Function to print the sensed output/angle to the setpoint every 50 ms. Use Serial plotter on Arduino to graphically plot the 
    sensed angle with respect to the setpoint angle. 

    HINT: You might want to print sensed_output to verify that the encoder is reading correctly
    HINT: You might also want to print control_signal to ensure the PID is working properly before writing it to the motor 
    
  */

  t_now = millis();
  
  if (t_now - t_last_print >= T_print){
    t_last_print = t_now;
    
    // ==================INSERT PRINT ALGORITHM HERE ==================== //
	Serial.print(sensed_output);
	Serial.print(" ");
	//Serial.println(sensed_angle);
	Serial.println(control_signal);
	
	delay(50);
    // ==================INSERT PRINT ALGORITHM HERE ==================== //

    
  }
}
