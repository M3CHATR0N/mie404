/* This code is supposed to Step up the motor speeds incrementally up to 1600 and print out the encoder 
 *  values as this stepping is happening. Make sure the neccessary libraries are installed. 
 */

#include <PID_v1.h>
#include "ESC.h"

//------------------Encoder Settings---------------------//
#define outputA 7
#define outputB 6
float counter;
int aState;
int aLastState;


//---------------------ESC SETTINGS-----------------------// 
#define SPEED_MIN 1000                                
#define SPEED_MAX 1600                                  
#define ESC_Pin 11                                              //ESC Control pin, AKA white wire
int speedStep = 10;       
ESC myESC (ESC_Pin, SPEED_MIN, SPEED_MAX, 500);                 // Initialize ESC (ESC PIN, Minimum Value, Maximum Value, Default Speed)
int stepSpeed = 20; 
int oESC = 1000;                                     

void setup() {
  pinMode(outputA, INPUT);
  pinMode(outputB, INPUT);
  pinMode(ESC_Pin, OUTPUT);
  
  myESC.arm(); 
  delay(7500);                                                  // I think this delay is for the ARM command above
  Serial.begin(115200);
  aLastState = digitalRead(outputA);
}

void loop() {

// Read Encoder, step up motor speed by 10, declared above^. Set motor speed to that stepped up value
  encoder_read();
  oESC += speedStep;
  myESC.speed(oESC); 
  
}

void encoder_read(){
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
    int angle = (counter/1200)*360;
    Serial.println(angle); 
//    Serial.println("deg");
  }

  aLastState = aState;
}

void rampUpDown() {
  /* 
    Function written to test functionality of brushless motor by accelerating and decelerating the rotation 
    of the motor. Speed of the motor does not accelerate to rated speed. 

  */
  Serial.println("rampUpFunc");

  for (oESC = SPEED_MIN; oESC <= 1150; oESC += 1) {        // iterate from minimum speed to a speed setting of 1150
    myESC.speed(oESC);                                     // write speed setting to motor
    encoder_read();
    delay(10);                                             // waits 10ms for the ESC to reach speed
  }
  delay(2000);                                             // wait a while
  
  for (oESC = 1150; oESC >= SPEED_MIN; oESC -= 1) {        // iterate from speed setting of 1150 to minimum speed
    myESC.speed(oESC);                                     // write speed setting to motor
    encoder_read();
    delay(10);                                             // waits 10ms for the ESC to reach speed  
   }
  delay(2000);                                             // Wait for a while before going into control loop
}
