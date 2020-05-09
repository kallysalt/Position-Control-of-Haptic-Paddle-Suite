int anglePin = A0;
//int anglePin = A1;
const int size = 5;
volatile float theta[size] = {};
volatile float theta_total[size] = {};
volatile float angle_track;
volatile float sum;
int n = 0;

int minadc = 50;
int maxadc = 973;
int adc0;
float k = 180.00000000 / (maxadc - minadc);
float diff;

void setup() {
  Serial.begin(9600);
  pinMode(anglePin, INPUT);  
  adc0 = analogRead(anglePin);

  //initialize theta total array
  for (int i = 0; i < size; i++) {
    theta[i] = (analogRead(anglePin)) * k;
    theta_total[i] = theta[i] ;
  }
}

void loop() {
  //store original data in theta array
  for (int i = 0; i < (size - 1); i++) {
    theta[i] = theta[i + 1];
  }
  theta[size - 1] = (analogRead(anglePin) ) * k;

  //determine when to +/- cycle number
  diff = theta[size - 1] - theta[size - 2];
  if(diff < -115) {
    n = n + 1;
    //Serial.println("One More!");
  }
  else if  (diff > 115) {
    n = n - 1;
    //Serial.println("One Less!");
  }
  else {
    n = n;
  }

  //store processed data in theta total array
  for (int i = 0; i < (size - 1); i++) {
    theta_total[i] = theta_total[i + 1];
  }
  theta_total[size - 1] = n * 180 + theta[size - 1] - adc0 * k;

  // calc average
  sum = 0;
  for (int i = 0; i < size; i++) {
    sum += theta_total[i];
    //Serial.println(theta_total[i]);
  }

  angle_track = sum / size;
  Serial.println(angle_track);
  //Serial.println(" is accumulated angle tracking in degree \n");
}
