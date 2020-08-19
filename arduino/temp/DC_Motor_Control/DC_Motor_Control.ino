const int encoderPinA = 2;
const int encoderPinB = 3;

int encoderPos = 0;

double angle = 0;

double temp_angle = 0;
double angular_velocity = 0;
double frequency=0;
double t=0;


double input_v = 80;
double error; 

double Ke = 0.0382;
double Kp = 0.083206;
double Ki = 0.096457;

double PControl=0;
double IControl=0;
double PIControl=0;
double PIControl_prev = 0;
int pwm_in=0, pwm_in_prev=0;


void setup()
{
  pinMode(encoderPinA, INPUT_PULLUP);
  attachInterrupt(0, doEncoderA, CHANGE);
 
  pinMode(encoderPinB, INPUT_PULLUP);
  attachInterrupt(1, doEncoderB, CHANGE);
  
  Serial.begin(9600);
  TCCR3A = (0<<WGM31) | (0<<WGM30); // CTC Mode
  TCCR3B = (1<<WGM33) |(1<<WGM32) | (1<<CS32) | (0<<CS31) | (1<<CS30) ; //1024분주  16000000/1024 = 15625

  ICR3 = 15625/10; //주파수 정하기 (15625일 때 10Hz)
  frequency = 15625/ICR3;
  t = 1/frequency;

  
  TIMSK3 |= (1<<OCIE3A); // Compare 인터럽트 사용.
  sei();
  
  TCNT3 = 0; // 카운터 초기화

  pinMode(11, OUTPUT);
  pinMode(9, OUTPUT);

  digitalWrite(9, LOW);
}

void loop()
{
//  Serial.println(angle);
//  delay(500);
//    angle = (double)encoderPos * 360/52/120;
//    Serial.println(angle);
//    delay(100);
}


ISR (TIMER3_COMPA_vect)  // Compare 인터럽트
{
  Serial.println(angular_velocity);
  //angle = (double)encoderPos * 360/52/120;
  angular_velocity = (angle-temp_angle)/t;
  
  temp_angle = angle;
  
  
  error=input_v-angular_velocity;
  PControl=Kp*error;
  
  if(IControl < 100)
    IControl+=Ki*error*t; 

  PIControl = PIControl_prev + PControl + IControl;
  
  pwm_in = 255/24 * PIControl;

  if(pwm_in < 0)
  {
    pwm_in = 0;
  }
  else if(pwm_in > 255)
  {
    pwm_in = 255;
  }


  analogWrite(11, pwm_in);


}


void doEncoderA(){ 
//  encoderPos += (digitalRead(encoderPinA)==digitalRead(encoderPinB))?1:-1;
  angle += (digitalRead(encoderPinA)==digitalRead(encoderPinB))?0.0577:-0.0577;
//  Serial.print("A   ");
//  angle = (double)encoderPos * 90;
//  Serial.println(encoderPos);
}

void doEncoderB(){ 
//  encoderPos += (digitalRead(encoderPinA)==digitalRead(encoderPinB))?-1:1;
  angle += (digitalRead(encoderPinA)==digitalRead(encoderPinB))?-0.0577:0.0577;
//  Serial.print("B   ");
//  angle = (double)encoderPos * 90;
//  Serial.println(encoderPos);
}
