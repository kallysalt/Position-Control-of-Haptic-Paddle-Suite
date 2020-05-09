// L293D chip

#define ENABLE 6
#define CW 3
#define CCW 5
//pins 3,5,6,9,10,11 have PWM capability

void setup() {
  //---set pin direction
  pinMode(ENABLE,OUTPUT);
  pinMode(CW,OUTPUT);
  pinMode(CCW,OUTPUT);
  Serial.begin(9600);
}

void loop() {
  Serial.print("1 more pulling cycle");
  digitalWrite(CCW,HIGH); //one way
  digitalWrite(CW,LOW);
  analogWrite(ENABLE,180); 
  
  //digitalWrite(CW,HIGH); //one way
  //digitalWrite(CCW,LOW);
  //analogWrite(ENABLE,130);
   
  //delay(100);

}
   
