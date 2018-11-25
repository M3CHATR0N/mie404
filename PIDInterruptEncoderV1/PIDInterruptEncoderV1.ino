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


//---------------------ENCODER PORTS----------------------// 
#define outputA 2                                         // Define Encoder output A to Digital Pin 6 on Arduino Uno 
#define outputB 3                                         // Define Encoder output B to Digital Pin 7 on Arduino Uno
#define NUMPULSES 1200                                    // Define the number of pulses on Encoder resolution (enter twice the amount of pulses on datasheet)

//---------------------ENCODER VARIABLES------------------//
float counter = 0;                                          // Counter variable for the number of pulses from the encoder
int aState;                                               // Digital state of the encoder output
int aLastState;                                           // Past state of the encoder output

//---------------------TIMER VARIABLES---------------------//
long t_now; 
//long t_last_print = 0;
long t_last_PID = 0;
int T_sample = 10;                                        // sample time in milliseconds (ms)
//int T_print = 1000;                                       // sample print to monitor time in milliseconds (ms)

//---------------------PID VARIABLES----------------------//
// Define variables for the output of the encoder(sensed_output), angle output of the encoder(sensed_output converted to angle), and error w.r.t the setpoint
//double sensed_output, error, sensed_angle;                

// Define variables for total error over time, previous error in the last sampling time interval, control signal, and limitations to the controller
//double setpoint_angle = 26;  
//long total_error =0;
//double last_error;
int max_control = 1700;
int min_control = 1000; 

// ==================INSERT DESIRED SETPOINT ANGLE HERE================== //
                      
   
// ==================INSERT CONTROL GAINS HERE=========== 
double Kp = 0.3;                                          // proportional gain
double Ki = 0.003;                                        // integral gain in [ms^-1]
double Kd = 120;                                          // derivative gain in [ms]

int tl = 0;
int num = 0;

// ==================INTERRUPT STUFF=========== //
volatile byte reading = 0;
volatile byte aFlag = 0;
volatile byte bFlag = 0;
volatile float encoderPos = 0;
volatile float oldEncPos = 0;


double setpoint_angle = 15; 
double setpoint = setpoint_angle * 1200/360;
double error = 0;
double control_signal=1000; 
double angle= 0;
int nowSpeed = SPEED_MIN;
int lastSpeed = SPEED_MIN;
double errSum = 0;
double dErr = 0;
double lastErr = 0;
double stepSize = 0;


void setup() {
  /* 
    Setup function to initialize serial plotter/monitor, initialize origin of encoder, 
    arm ESC driver, and to initialize propeller by ramping up and down
  */ 
  pinMode(outputA, INPUT_PULLUP);
  pinMode(outputB, INPUT_PULLUP);
  pinMode(9, OUTPUT);
  
  Serial.begin(9600);

  attachInterrupt(digitalPinToInterrupt(outputA), PinA, RISING);
  attachInterrupt(digitalPinToInterrupt(outputB) ,PinB ,RISING);
  
  myESC.arm();                                              // Send the Arm value so the ESC will be ready to take commands
  Serial.println("Wait");
  delay(7500);                                              // Wait for a while
  Serial.println("Begin");
}


void loop() {
  angle = (encoderPos/NUMPULSES)*720;
  cli();
  PID_control();                          // Changes the global variable control_signal
  myESC.speed(control_signal);
  sei();
  Serial.print("CS: ");  
  Serial.print(control_signal); 
  Serial.print("  E:");
  Serial.print(error);
  Serial.print("  A:");
  Serial.print(angle);
  Serial.print(" S:");
  Serial.print(setpoint_angle);
  Serial.print(" ESum:");
  Serial.print(errSum);
  Serial.print(" dErr:");
  Serial.println(120*dErr);
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

void rampUpTo(double target) {
  /*  Function ramps the output to the target rpm starting from the current rpm and reading encoder along the way
  */
  for (oESC = lastSpeed; oESC <= target; oESC += 1) {     // iterate from minimum speed to a speed setting of 1150
    myESC.speed(oESC);                                    // write speed setting to motor                                       
  }
  lastSpeed = target;
}

void PID_control(){
  //  Currently just doing P-Control
  // time var for derivative control
  t_now = millis();
  if(t_now - t_last_PID > T_sample){
    error = setpoint_angle - abs(angle);                 // Calculate error

    // integral control
    errSum += error*T_sample;
    
    // derivative control
    dErr = (error - lastErr)/T_sample;
    
    int threshold = abs(error);           //If error is below this threshold then don't update the control_signal

//# control_signal Method 1 #  Code that limits the stepSize calculated to 50, use either this or the 
//    stepSize = Kp*error + Ki*errSum + Kd*dErr;
//    if(threshold > 2){                  // if the absolute error is less than 2 then don't change the control_signal
//      if(stepSize > 50){                //Limiting step size to 50
//        control_signal += 50;
//        } else{
//            control_signal += Kp*error + Ki*errSum + Kd*dErr;
//        }
//    }

//# control_signal Method 2 #  No stepSize Limiting -both have seemed to work the same
    if(threshold > 2){
        control_signal += Kp*error + Ki*errSum + Kd*dErr;
    }    
    
//Limit control_signal within bounds of 1000 and 1800
    if(control_signal > 1800){
      control_signal = 1800;            //If the calculated control signal from above is greater than 1800
                                        //then set the control_signal to 1800
    } else if(control_signal < 1100) {
      control_signal = 1150;
    }
    
 // Update values for derivative control
    lastErr = error;
    t_last_PID = t_now;
    }
}

void PinA() {
  cli();
  reading = PIND & 0xC;
  if(reading == B00001100 && aFlag) { 
    encoderPos --;
    bFlag = 0;
    aFlag = 0;
  }
  else if (reading == B00000100){
    bFlag = 1;
  }
  sei();
}

void PinB() {
  cli();
  reading = PIND & 0xC;
  if(reading == B00001100 && bFlag) { 
    encoderPos ++;
    bFlag = 0;
    aFlag = 0;
  }
  else if (reading == B00001000){
    aFlag = 1;
  }
  sei();
}
