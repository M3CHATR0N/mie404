#define outputA 7
#define outputB 6
//volatile unsigned int counter = 0;
float counter;
int aState;
int aLastState;

void setup() {
  pinMode(outputA, INPUT);
  pinMode(outputB, INPUT);

  Serial.begin(115200);
  aLastState = digitalRead(outputA);
}

void loop() {
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
