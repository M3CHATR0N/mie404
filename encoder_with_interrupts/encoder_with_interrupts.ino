/*
 * SOURCE: https://www.youtube.com/watch?v=QE4IQlwOgiA
 * 
 * GOAL: see if encoder works
 * 
 * RESULT: encoder not working??
 * serial monitor was blank with the exception of "serial be werrrrking!!!"
 * so, when the shaft was turned:
 * - the counter did not print
 * - the A and B channel outputs did not print 
 * what the heck. Encoder was powered by 6V from Arduino
 * 
 * When encoder was powered by 6V, 8V from power supply, same result
 */
volatile unsigned int temp, counter = 0; //This variable will increase or decrease depending on the rotation of encoder
    
void setup() {
  Serial.begin (9600);
  Serial.println("serial be werrrrking!!!");                             // debug

  pinMode(2, INPUT_PULLUP); // internal pullup input pin 2 
  
  pinMode(3, INPUT_PULLUP); // internalเป็น pullup input pin 3
  
  //Setting up interrupt
  //A rising pulse from encodenren activated ai0(). AttachInterrupt 0 is DigitalPin 2
  attachInterrupt(0, ai0, RISING);
   
  //B rising pulse from encodenren activated ai1(). AttachInterrupt 1 is DigitalPin 3
  attachInterrupt(1, ai1, RISING);
  }
   
  void loop() {
    // Send the value of counter
    if( counter != temp ){
      Serial.println (counter);
      temp = counter;
      }
    }
   
  void ai0() {
    // ai0 is activated if DigitalPin nr 2 is going from LOW to HIGH
    // Check pin 3 to determine the direction
    if(digitalRead(3)==LOW) {
      Serial.println("A: " + String(digitalRead(3)));                       // debug
      counter++;
      }
    else{
      counter--;
      }
    }
   
  void ai1() {
    // ai0 is activated if DigitalPin nr 3 is going from LOW to HIGH
    // Check with pin 2 to determine the direction
    if(digitalRead(2)==LOW) {
      Serial.println("A: " + String(digitalRead(2)));                       // debug
      counter--;
      }
    else{
      counter++;
      }
    }
