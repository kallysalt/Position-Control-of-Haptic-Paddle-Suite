// L293D chip
#define ENABLE 2
#define CW 3
#define CCW 5
#define CW_m 6
#define CCW_m 11
//pins 3,5,6,9,10,11 have PWM capability
//use the better device (white) as slave

#define anglePin_slave A0
#define anglePin_master A1

// angles in degree
const int size = 5;
float thetaS[size] = {};
float theta_totalS[size] = {};
float angle_trackS;
float sumS;
int n_s = 0;
int adc0S;
float diffS;

float thetaM[size] = {};
float theta_totalM[size] = {};
float angle_trackM;
float sumM;
int n_m = 0;
int minadc = 50;
int maxadc = 973;
int adc0M;
float diffM;

float k = 180.00000000 / (maxadc - minadc);

// for PID & motor
volatile double r ; //setpoint
volatile double y; // angle sensor
volatile double u ; //
volatile int pwm;
double Kp = 0.09, Ki = 0.00, Kd = 0.005;  //vin=5v
volatile float  error, error_integral, error_derivative;
int minpwm = 120;  //vin=pololu
int maxpwm = 190;  //vin=pololu

//float dt = 0.099;  // pwm loop time interval
float dt = 0.0099;  // pwm loop time interval
volatile float previous_error = 0;

void setup()
{
  // for angle sensor
  cli();//stop interrupts
  Serial.begin(9600);
  pinMode(anglePin_slave, INPUT);
  pinMode(ENABLE, OUTPUT);
  pinMode(CW, OUTPUT);
  pinMode(CCW, OUTPUT);

  // for angle sensor
  adc0S = analogRead(anglePin_slave);
  for (int i = 0; i < size; i++) {
    thetaS[i] = (analogRead(anglePin_slave)) * k;
    theta_totalS[i] = thetaS[i] ;
  }
  adc0M = analogRead(anglePin_master);
  for (int i = 0; i < size; i++) {
    thetaM[i] = (analogRead(anglePin_master)) * k;
    theta_totalM[i] = thetaM[i] ;
  }

  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 1549;// = (16*10^6) / (10.08*1024) - 1 (must be <65536)
  //OCR1A = 154;// = (16*10^6) / (100.8*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei();//allow interrupts
}

ISR(TIMER1_COMPA_vect) { //timer1 interrupt for contorller update
  if (u < 0) {
    digitalWrite(CCW, HIGH);
    digitalWrite(CW, LOW);
    pwm = minpwm - u * maxpwm / 2;
  }
  else {
    digitalWrite(CW, HIGH);
    digitalWrite(CCW, LOW);
    pwm = minpwm + u * maxpwm / 2;
  }
  // Set threshold
  if (abs(error) < 90) {
    digitalWrite(CW, LOW);
    digitalWrite(CCW, LOW);
    pwm = 0;
    // error_integral = 0;
  }
  if (pwm > maxpwm) {
    pwm = maxpwm;
  }
}

void loop()
{
  r=readAngleM(anglePin_master);
  y = readAngleS(anglePin_slave);
  u = controlUpdate(y, r);
  Serial.println(pwm);
  //Serial.print(" is error \n");
  //Serial.print(y);
  //Serial.println(" is y \n");
  //Serial.print(r);
  //Serial.print(" is r \n");
  //Serial.println();
  //Serial.print(y);
  //Serial.print(" ");
  //Serial.println(r);
  delay(10);
  analogWrite(ENABLE, pwm);
}

float readAngleS(int anglePin) {
  for (int i = 0; i < (size - 1); i++) {
    thetaS[i] = thetaS[i + 1];
  }
  thetaS[size - 1] = (analogRead(anglePin) ) * k;

  diffS = thetaS[size - 1] - thetaS[size - 2];
  if (diffS < -100) n_s = n_s + 1;
  else if  (diffS > 100) n_s = n_s - 1;
  else n_s = n_s;
  for (int i = 0; i < (size - 1); i++) {
    theta_totalS[i] = theta_totalS[i + 1];
  }
  theta_totalS[size - 1] = n_s * 180 + thetaS[size - 1] - adc0S * k;

  sumS = 0;
  for (int i = 0; i < size; i++) {
    sumS += theta_totalS[i];
  }
  angle_trackS = sumS / size;
  return angle_trackS;
}

float readAngleM(int anglePin) {
  for (int i = 0; i < (size - 1); i++) {
    thetaM[i] = thetaM[i + 1];
  }
  thetaM[size - 1] = (analogRead(anglePin) ) * k;

  diffM = thetaM[size - 1] - thetaM[size - 2];
  if (diffM < -115) n_m = n_m + 1;
  else if  (diffM > 115) n_m = n_m - 1;
  else n_m = n_m;
  for (int i = 0; i < (size - 1); i++) {
    theta_totalM[i] = theta_totalM[i + 1];
  }
  theta_totalM[size - 1] = n_m * 180 + thetaM[size - 1] - adc0M * k;

  sumM = 0;
  for (int i = 0; i < size; i++) {
    sumM += theta_totalM[i];
  }
  angle_trackM = sumM / size;
  return angle_trackM;
}

float controlUpdate(float y, float r) {
  error = r - y;
  error_integral += error * dt;
  error_derivative = (error - previous_error) / dt;
  u = Kp * error + Ki * error_integral + Kd * error_derivative;
  if (u > 1) {
    u = 1;
  }
  else if (u < -1) {
    u = -1;
  }
  previous_error = error;
  return u;
}
