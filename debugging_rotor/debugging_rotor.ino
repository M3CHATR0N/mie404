#include "ESC.h"

//---------------------ESC SETTINGS-----------------------// 
#define SPEED_MIN (1000)                                  // Set the Minimum Speed Setting of the ESC
#define SPEED_MAX (2000)                                  // Set the Maximum Speed Setting of the ESC
                      
ESC myESC (9, SPEED_MIN, SPEED_MAX, 500);                 // Initialize ESC (ESC PIN, Minimum Value, Maximum Value, Default Speed)
int oESC;                                                 // Variable for the speed sent to the ESC


void setup() {
  // put your setup code here, to run once:
  pinMode(9, OUTPUT);
  Serial.begin(9600);
  myESC.arm();                                              // Send the Arm value so the ESC will be ready to take commands
  Serial.println("Wait");
  delay(7500);                                              // Wait for a while
  Serial.println("Begin");
//  myESC.speed(SPEED_MIN);                                   // start the fan initially at minimum speed so we can add to it during control
}

void loop() {
  // put your main code here, to run repeatedly:
  myESC.speed(1200);
}

void rampUpTo(double target) {
  /*  Function ramps the output to the target rpm starting from the current rpm and reading encoder along the way
  */
  for (oESC = SPEED_MIN; oESC <= target; oESC += 1) {     // iterate from minimum speed to a speed setting of 1150
    myESC.speed(oESC);                                    // write speed setting to motor                                       
    delay(100);
  }
}
