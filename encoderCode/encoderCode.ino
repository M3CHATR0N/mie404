//---------------------ENCODER PORTS----------------------// 
#define outputA 6                                         // Define Encoder output A to Digital Pin 6 on Arduino Uno 
#define outputB 7                                         // Define Encoder output B to Digital Pin 7 on Arduino Uno
#define NUMPULSES 1200                                    // Define the number of pulses on Encoder resolution (enter twice the amount of pulses on datasheet)

//---------------------ENCODER VARIABLES------------------//
int counter = 0;                                          // Counter variable for the number of pulses from the encoder
int angle = 0;
int aState;                                               // Digital state of the encoder output
int aLastState;                                           // Past state of the encoder output
int bState;                                               // inserted var for digital state of the B channel that helps track direction
int degPerRevPulse = 360/NUMPULSES;                            // 360deg/1200pulses for each revolution; we are only couting pulses on A channel thus 1200

void setup() {
  // sets the two pins to right pinMode to read outputs
  pinMode (outputA,INPUT);
  pinMode (outputB,INPUT);
  
  Serial.begin(9600);
  aLastState = digitalRead(outputA);
  Serial.println("Started");

}

void loop() {
  // Read the encoder 
  encoder_read();
  
}

void encoder_read(){
  aState = digitalRead(outputA);
  bState = digitalRead(outputB);

  if (aState != aLastState){                            // we're on the falling edge or rising edge of A
     Serial.println("====================================");                    // for debugging
     Serial.println("A: " + String(aState) + "  B: " + String(bState));         // for debugging
     
     if (bState != aState) {                            // CW spin
       counter ++;
       Serial.println("Counter: " + String(counter));   // for debugging
       Serial.println("Angle: " + String(angle));       // for debugging
       Serial.println("===================================="); // for debugging
     }
     else {                                             // counter CCW spin
       counter--;
       Serial.println("====================================");                  // for debugging
     } 
     if (counter >=360 ) {                              // assume that the arm can rotate 360 deg; reset once it does
      counter = 0;
     }

     // print the angle or position
     angle = counter*degPerRevPulse;
     Serial.print("Position: " + String(angle) + "deg");
     
   }
  aLastState = aState;
}
