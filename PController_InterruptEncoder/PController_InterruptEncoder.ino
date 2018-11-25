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
#define outputA 2                                         // Define Encoder output A to Digital Pin 6 on Arduino Uno -- need to attached to interrupt pins
#define outputB 3                                         // Define Encoder output B to Digital Pin 7 on Arduino Uno -- only pins 2 & 3 are interrupt pins on UNO
#define NUMPULSES 1200                                    // Define the number of pulses on Encoder resolution (enter twice the amount of pulses on datasheet)

//---------------------ENCODER VARIABLES------------------//
float counter = 0;                                          // Counter variable for the number of pulses from the encoder
int aState;                                               // Digital state of the encoder output
int aLastState;                                           // Past state of the encoder output

//---------------------TIMER VARIABLES---------------------//
long t_now; 
//long t_last_print = 0;
long t_last_PID = 0;
int T_sample = 20;                                        // sample time in milliseconds (ms)
//int T_print = 1000;                                       // sample print to monitor time in milliseconds (ms)

//---------------------PID VARIABLES----------------------//
// Define variables for the output of the encoder(sensed_output), angle output of the encoder(sensed_output converted to angle), and error w.r.t the setpoint
//double sensed_output, error, sensed_angle;                

// Define variables for total error over time, previous error in the last sampling time interval, control signal, and limitations to the controller
double setpoint_angle = 26;  
double setpointClicks = setpoint_angle * NUMPULSES/360;
double error = 0;
double control_signal;                                     // output of controller
double angle;

double nowSpeed = SPEED_MIN;                               // control loop output is a double
double lastSpeed = SPEED_MIN;

double pErr = 0;                                           // Kp control attempt -- can likely delete
double errSum = 0;                                         // Ki control
double dErr = 0;                                           // Kd control
double lastErr = 0;                                        // Kd control

// clicks control attempt
int max_control = 1700;
int min_control = 1000;   
   
// ================== CONTROL GAINS HERE=========== //
double Kp = 1; //recommended start point = 0.8;            // proportional gain
double Ki = 0.003;                                        // integral gain in [ms^-1]
double Kd = 120;                                          // derivative gain in [ms]

// ==================INTERRUPT STUFF=========== //
volatile byte reading = 0;
volatile byte aFlag = 0;
volatile byte bFlag = 0;
volatile float encoderPos = 0;

void setup() {
  /* 
    Setup function to initialize serial plotter/monitor, initialize origin of encoder, 
    arm ESC driver, and to initialize propeller by ramping up and down
  */ 
  pinMode(outputA, INPUT_PULLUP);
  pinMode(outputB, INPUT_PULLUP);
  pinMode(9, OUTPUT);
  
  Serial.begin(9600);

  attachInterrupt(digitalPinToInterrupt(outputA),PinA, RISING);
  attachInterrupt(digitalPinToInterrupt(outputB) ,PinB ,RISING);
  
  myESC.arm();                                              // Send the Arm value so the ESC will be ready to take commands
  Serial.println("Wait");
  delay(7500);                                              // Wait for a while
  Serial.println("Begin");
}


void loop() {
  angle = (encoderPos/NUMPULSES)*720;                       // angle doesn't seem to print
  cli();                                                    // clears the global interrup flag -- turns it off so it doesn't fuck with the PID calculation
//  PID_control();                                            // Changes the global variable control_signal, determines a % relative to lastSpeed to adjust by
  nowSpeed = control_signal*lastSpeed + lastSpeed;        // original & approach 1 - % adjustment
//  nowSpeed = (control_signal + 1100);                     // approach 2 - clicks: offset approach
  nowSpeed = control_signal;                                // approach 3 - % adjustment with mapping; rotor speed 2000 gives 30 deg angle
//  nowSpeed //approach 4 - % adjustment with offset
  myESC.speed(nowSpeed);
  sei();                                                    // sets the bit and switches interrupts on

  // debugging AF
  /*
   * IMPORTANT NOTE: max speed of 2000 gives us an angle of 30 deg lol; debugged that
   */
  Serial.print("CS: ");  
  Serial.print(control_signal);
  Serial.print("  NS: ");  
  Serial.print(nowSpeed); 
  Serial.print("  E:");
  Serial.print(error);
  Serial.print("  A:");
  Serial.print(abs(angle));                                  // CCW is negative for our encoder readings
  Serial.print("  S:");
  Serial.println(setpoint_angle);

  nowSpeed = lastSpeed;
}

void PID_control(){
  // Using sampling time; Currently just doing P-Control
  // Outputs the percent adjust needed (is later multipled by rotor speed in loop)

  // time var for derivative control
  t_now = millis();
  if(t_now - t_last_PID > T_sample){
    error = setpoint_angle - abs(angle);                                   // Calculate error

//    pErr = 1-(error/setpoint_angle);                                  // 1-% error calculates how much we need to compensate by
//    error = setpointClicks + encoderPos;                                // the encoderPos is default negative angle
    
    // integral control
    //errSum += error*T_sample;
    
    // derivative control
    //double dErr = (error - lastErr)/T_sample;
    
    // calculate the output needed
//    control_signal = Kp*(error/setpoint_angle);                       // Original: Calculate p-component of control signal                           
//    control_signal = Kp*(pErr);                                       // approach 1 - %change: Calculate p-component of control signal                           
//      control_signal = Kp*(error/150)*900;                            // approach 2 - clicks: 150 experimentally determined to be 90 deg
    double control_signal_temp = Kp*error;                                          // approach 3: do mapping in loop
    control_signal = map(control_signal_temp,0,30,SPEED_MIN,SPEED_MAX);              // want deg > 0 only, and can't get negative speeds

//    // approach 2 - clicks: account for saturation -- redundant
//    if(control_signal > 1000) {
//      control_signal = max_control;
//    } 
//    else if(control_signal<0){                                          // if we have it spinning CW or backwards, then set that shit back to min speed to get a positive angle          
//      control_signal = 0;                                               // which sets nowSpeed to 1100 (just above min speed)
//    }
      
    // Update values for derivative control
    //lastErr = error;
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
