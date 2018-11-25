#include <PID_v1.h>
#include <Wire.h>

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
#define outputA 6                                         // Define Encoder output A to Digital Pin 6 on Arduino Uno 
#define outputB 7                                         // Define Encoder output B to Digital Pin 7 on Arduino Uno
#define NUMPULSES 1200                                    // Define the number of pulses on Encoder resolution (enter twice the amount of pulses on datasheet)

//---------------------ENCODER VARIABLES------------------//
float counter = 0;                                          // Counter variable for the number of pulses from the encoder
int aState;                                               // Digital state of the encoder output
int aLastState;                                           // Past state of the encoder output
int angle;

//---------------------TIMER VARIABLES---------------------//
long t_now = 0;
long t_last_print = 0;
long t_last_PID = 0;
int T_sample = 20;                                        // sample time in milliseconds (ms)
int T_print = 1000;                                       // sample print to monitor time in milliseconds (ms)

//---------------------PID VARIABLES----------------------//
// Define variables for the output of the encoder(sensed_output), angle output of the encoder(sensed_output converted to angle), and error w.r.t the setpoint
double sensed_output, error, sensed_angle;                

// Define variables for total error over time, previous error in the last sampling time interval, control signal, and limitations to the controller
double setpoint_angle = 26;  
//double setpoint = setpoint_angle * NUMPULSES/360;
long total_error =0;
double last_error;
long control_signal; 
int max_control = 1700;
int min_control = 1000; 

// ==================INSERT DESIRED SETPOINT ANGLE HERE================== //
                      
   
// ==================INSERT CONTROL GAINS HERE=========== 
double Kp = 0.8;                                          // proportional gain
double Ki = 0.003;                                        // integral gain in [ms^-1]
double Kd = 120;                                          // derivative gain in [ms]

int tl = 0;
int num = 0;

double Setpoint= 20;
double Input; 
double Output;

void setup() {
  /* 
    Setup function to initialize serial plotter/monitor, initialize origin of encoder, 
    arm ESC driver, and to initialize propeller by ramping up and down
  */ 
  pinMode(outputA, INPUT);
  pinMode(outputB, INPUT);
  pinMode(9, OUTPUT);
  
  encoder_read();
  delay(100);
  
  Wire.begin(); //initialize as a master
  Serial.begin(9600);
  myESC.arm();                                              // Send the Arm value so the ESC will be ready to take commands
  Serial.println("Wait");
  delay(7500);                                              // Wait for a while
  Serial.println("Begin");
//  rampUpDown();                                            // Propeller speed to ramp up and down to check functionality
tl = millis(); 
}


void loop() {
  encoder_read();
  if(millis() - tl > 20000){
    myESC.speed(0);
    encoder_read();
  } 
  else{
    if(num == 0){
      encoder_read();
      int revpermin = 1200;
      rampUpTo(revpermin);
      myESC.speed(revpermin);                                     // write speed setting to motor        
      encoder_read();
    }
    num = 1;
    encoder_read();
  }

//Reading from other arduino

//  Wire.requestFrom(8, 6);
//  while (Wire.available()) {  // slave may send less than requested
//    char c = Wire.read();     // receive a byte as character
//    Serial.print(c);          // print the character
//  }
//
//  delay(500);
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
  aState = digitalRead(outputA);

  if(aState != aLastState){
    if(digitalRead(outputB) != aState){
      counter++;
    } else{
      counter--;
    }
//    Reset counter after 1 full rotation occurs
  if(counter>1200 || counter < -1200){
    counter = 0;
  }
    angle = (counter/1200)*360;
    Serial.println(angle); 
//    Serial.println("deg");
  }

  aLastState = aState;
}

void rampUpTo(int rpm) {
  /* 
    Function written to test functionality of brushless motor by accelerating and decelerating the rotation 
    of the motor. Speed of the motor does not accelerate to rated speed. 

  */
  encoder_read();
  for (oESC = SPEED_MIN; oESC <= rpm; oESC += 1) {        // iterate from minimum speed to a speed setting of 1150
    myESC.speed(oESC);                                     // write speed setting to motor
    encoder_read();                                             // waits 10ms for the ESC to reach speed
  }
  encoder_read();
}
