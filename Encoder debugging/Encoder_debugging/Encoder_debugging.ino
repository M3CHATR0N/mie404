//---------------------ENCODER PORTS----------------------// 
#define outputA 6                                         // Define Encoder output A to Digital Pin 6 on Arduino Uno 
#define outputB 7                                         // Define Encoder output B to Digital Pin 7 on Arduino Uno
#define NUMPULSES 1200                                    // Define the number of pulses on Encoder resolution (enter twice the amount of pulses on datasheet)

//---------------------ENCODER VARIABLES------------------//
int counter = 0;                                          // Counter variable for the number of pulses from the encoder
int aState;                                               // Digital state of the encoder output
int aLastState;                                           // Past state of the encoder output
int bState;                                               // inserted var for digital state of the B channel that helps track direction


void setup() {
  /* 
    Took out ESC commands. Will turn the encoder shaft with hand for now.
  */ 

  pinMode(outputA, INPUT);                                 // setup pins to read values
  pinMode(outputB, INPUT);
  
  Serial.begin(9600);
  aLastState = digitalRead(outputA);                        // Reads the initial state of the outputA 

  Serial.println("Setup complete");
}


void loop() {
  
 // Read the encoder 
  encoder_read();

 // Take the magnitude of the encoder output (accounting for CW or CCW rotation)
//  sensed_output = abs(counter);
}

void encoder_read() {
  /* 
    1. Function to count the number of pulses from the encoder output. 
    2. Must count at least 2 count per pulse, which would give you 1200 count per rotation.  

    QUESTION RE 2.: are the numbers above for each channel? yes.
    QUESTION RE 2.: why must >= 2 counts be done for each pulse? is it minimum 2 for the rise and fall of the wave?
    
    //---------------------ENCODER PORTS----------------------// 
#define outputA 6                                         // Define Encoder output A to Digital Pin 6 on Arduino Uno 
#define outputB 7                                         // Define Encoder output B to Digital Pin 7 on Arduino Uno
#define NUMPULSES 1200                                    // Define the number of pulses on Encoder resolution (enter twice the amount of pulses on datasheet)

    QUESTION: why is NUMPULSES 1200???? isn't it 2400 pulses per revolution with 2 channels?
    
//---------------------ENCODER VARIABLES------------------//
int counter = 0;                                          // Counter variable for the number of pulses from the encoder
int aState;                                               // Digital state of the encoder output
int aLastState;                                           // Past state of the encoder output

  */ 

   aState = digitalRead(outputA);                         // want to know the current state of A channel and B channel
   bState = digitalRead(outputB);

   

   //---------------------DEBUGGING CODE----------------------// 
   Serial.println("A: " + String(aState) + "   B: " + String(bState)); //get initial values when nothing is moving
   // initial states for BOTH A and B seem to be outputting 1

   if (aState != bState){
    Serial.println("not equal");
   }
   else{
    Serial.println("equal");
   }
   
   /* RESULT: 
    * states don't change with spinning of shaft....
    * encoder does not seem to respond to spinning of shaft -- is this thing even working??
   */
}
