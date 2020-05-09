int anglePin = A0;
int theta = 0;

void setup() {
  Serial.begin(9600);
  pinMode(anglePin,INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  theta = analogRead(anglePin);
  Serial.println(theta);
  delay(10);
  

}
