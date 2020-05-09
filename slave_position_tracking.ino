// L293D chip
#define ENABLE 2
#define CW 3
#define CCW 5
//pins 3,5,6,9,10,11 have PWM capability

#define anglePin_slave A0

// angles in degree
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

// for PID & motor
volatile double r ; //setpoint
volatile double y; // angle sensor
volatile double u ; //
volatile int pwm;
double Kp = 0.09, Ki = 0.00, Kd = 0.005;  //vin=5v
volatile float  error, error_integral, error_derivative;
int minpwm = 120;  //vin=5v
int maxpwm = 190;  //vin=5v

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

  // hard code setpoint for now
  r = 200;
  // for angle sensor
  adc0 = analogRead(anglePin_slave);
  for (int i = 0; i < size; i++) {
    theta[i] = (analogRead(anglePin_slave)) * k;
    theta_total[i] = theta[i] ;
  }
  //set timer1 interrupt at every 0.099s
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 1549;// = (16*10^6) / (10.08*1024) - 1 (must be <65536)
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
  if (abs(error) < 60) {
    digitalWrite(CW, LOW);
    digitalWrite(CCW, LOW);
    pwm = 0;
  }
  if (pwm > maxpwm) {
    pwm = maxpwm;
  }
}

void loop()
{
  y = readAngle(anglePin_slave);
  u = controlUpdate(y, r);
  //Serial.println(error);
  //Serial.print(" is error \n");
  Serial.print(y);
  Serial.print(" ");
  Serial.println(r);
  //Serial.print(" is y \n");
  //Serial.print(u);
  //Serial.print(" is u \n");
  //Serial.println(pwm);
  //Serial.print(" is pwm \n");
  //Serial.println();
  delay(10);
  analogWrite(ENABLE, pwm);
}

float readAngle(int anglePin) {
  for (int i = 0; i < (size - 1); i++) {
    theta[i] = theta[i + 1];
  }
  theta[size - 1] = (analogRead(anglePin) ) * k;

  diff = theta[size - 1] - theta[size - 2];
  if (diff < -100) n = n + 1;
  else if  (diff > 100) n = n - 1;
  else n = n;
  for (int i = 0; i < (size - 1); i++) {
    theta_total[i] = theta_total[i + 1];
  }
  theta_total[size - 1] = n * 180 + theta[size - 1] - adc0 * k;
  sum = 0;
  for (int i = 0; i < size; i++) {
    sum += theta_total[i];
  }
  angle_track = sum / size;
  return angle_track;
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
