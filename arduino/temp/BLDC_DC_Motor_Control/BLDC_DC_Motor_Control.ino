#define A  7
#define B  30

#define C  31
#define D  32

#define SPEED_OUT 20


double angle_BLDC = 0;
double temp_angle_BLDC = 0;
double angular_velocity_BLDC = 0;

double input_v_BLDC = 15;
double error_BLDC;
double Ke_BLDC = 0.0688;
double Kp_BLDC = 0.20824;
double Ki_BLDC = 1.7062;

double PControl_BLDC;
double IControl_BLDC=0;
double PIControl_BLDC;
double PIControl_prev_BLDC = 0;
int pwm_in_BLDC, pwm_in_prev_BLDC;

const int encoderPinA = 2;
const int encoderPinB = 3;

const int encoderPinC = 18;
const int encoderPinD = 19;

double angle_L = 0;
double temp_angle_L = 0;
double angular_velocity_L = 0;

double angle_R = 0;
double temp_angle_R = 0;
double angular_velocity_R = 0;

double frequency=0;
double t=0;

double input_v_L = 80;
double error_L; 

double input_v_R = 80;
double error_R; 

double Ke_DC = 0.0382;
double Kp_DC = 0.083206;
double Ki_DC = 0.096457;

double PControl_L=0;
double IControl_L=0;
double PIControl_L=0;
int pwm_in_L=0;

double PControl_R=0;
double IControl_R=0;
double PIControl_R=0;
int pwm_in_R=0;

void setup()
{
  pinMode(encoderPinA, INPUT_PULLUP);
  attachInterrupt(0, doEncoderA, CHANGE);
 
  pinMode(encoderPinB, INPUT_PULLUP);
  attachInterrupt(1, doEncoderB, CHANGE);

  pinMode(encoderPinC, INPUT_PULLUP);
  attachInterrupt(5, doEncoderC, CHANGE);
 
  pinMode(encoderPinD, INPUT_PULLUP);
  attachInterrupt(4, doEncoderD, CHANGE);
  
  Serial.begin(9600);
  TCCR3A = (0<<WGM31) | (0<<WGM30); // CTC Mode
  TCCR3B = (1<<WGM33) |(1<<WGM32) | (1<<CS32) | (0<<CS31) | (1<<CS30) ; //1024분주  16,000,000/1,024 = 15,625

  ICR3 = 15625/10; //주파수 정하기 (15625/10일 때 10Hz)
  
  frequency = 15625/ICR3;
  t = 1/frequency; //제어주기
  
  TIMSK3 |= (1<<OCIE3A); // Compare 인터럽트 사용.
  sei();
  
  TCNT3 = 0; // 카운터 초기화

  pinMode(11, OUTPUT);
  pinMode(10, OUTPUT); //PWM
  
  pinMode(9, OUTPUT); //LEFT ST
  pinMode(8, OUTPUT); //RIGHT ST 

  pinMode(22, OUTPUT);
  pinMode(23, OUTPUT); //LEFT_MOTOR 시계 반시계
  
  pinMode(24, OUTPUT);
  pinMode(25, OUTPUT); //RIGHT_MOTOR 시계 반시계

  digitalWrite(9, LOW); //LOW : ST, HIGH : SP
  digitalWrite(8, LOW); //LOW : ST, HIGH : SP
    
  digitalWrite(22, HIGH); //22 HIGH, 23 LOW : 시계방향
  digitalWrite(23, LOW); //22 LOW, 23 HIGH : 반시계방향

  digitalWrite(24, HIGH); //24 HIGH, 25 LOW : 시계방향
  digitalWrite(25, LOW); //24 LOW, 25 HIGH : 반시계방향

//BLDC MOTOR 초기 설정
  pinMode(A, OUTPUT);
  pinMode(B, OUTPUT);
  pinMode(C, OUTPUT);
  pinMode(D, OUTPUT);
  pinMode(SPEED_OUT, INPUT_PULLUP);  
  attachInterrupt(3, encoder_interrupt, FALLING);
  // initialize
  digitalWrite(B, LOW);
  digitalWrite(C, LOW);
  digitalWrite(D, HIGH);
  
}

void loop()
{
}

ISR (TIMER3_COMPA_vect)  // Compare 인터럽트
{
  //DC모터 Left 제어
 //Serial.println(pwm_in_R);

 angular_velocity_L = (angle_L-temp_angle_L)/t;  //각속도 = 각속도변화/제어주기
 
   if(angular_velocity_L < 0)
     angular_velocity_L = -angular_velocity_L; //역방향 회전시 절댓값 변화
 
   temp_angle_L = angle_L; //이전 각도 저장
   
   error_L=input_v_L-angular_velocity_L; //PID 에러값
 
   PControl_L=Kp_DC*error_L;
   
   if(IControl_L < 100)
     IControl_L+=Ki_DC*error_L*t; //와인드업 방지
 
   PIControl_L = PControl_L + IControl_L;
   
   pwm_in_L = 255/24 * PIControl_L; //PI값을 PWM으로 변환
 
   if(pwm_in_L < 0)
   {
     pwm_in_L = 0;
   } //PWM 하한선 
   else if(pwm_in_L > 255)
   {
     pwm_in_L = 255;
   } //PWM 상한선
 
   analogWrite(11, pwm_in_L); //PWM출력


    //DC모터 Right 제어
   angular_velocity_R = (angle_R-temp_angle_R)/t; //각속도 = 각속도변화/제어주기
 
   if(angular_velocity_R < 0)
     angular_velocity_R = -angular_velocity_R; //역방향 회전시 절댓값 변화
 
   temp_angle_R = angle_R; //이전 각도 저장
   
   error_R=input_v_R-angular_velocity_R; //PID 에러값
 
   PControl_R=Kp_DC*error_R;
   
   if(IControl_R < 100)
     IControl_R+=Ki_DC*error_R*t; //와인드업 방지
 
   PIControl_R = PControl_R + IControl_R;
   
   pwm_in_R = 255/24 * PIControl_R; //PI값을 PWM으로 변환
 
   if(pwm_in_R < 0)
   {
     pwm_in_R = 0;
   } //PWM 하한선 
   else if(pwm_in_R > 255)
   {
     pwm_in_R = 255;
  } //PWM 상한선

  analogWrite(10, pwm_in_R); 

  //BLDC모터 제어

  angular_velocity_BLDC = (angle_BLDC-temp_angle_BLDC)/t;
  
  temp_angle_BLDC = angle_BLDC;
  
  
  error_BLDC=input_v_BLDC-angular_velocity_BLDC;
  PControl_BLDC=Kp_BLDC*error_BLDC;
  
  if(IControl_BLDC < 100)
    IControl_BLDC+=Ki_BLDC*error_BLDC*t;


  
  PIControl_BLDC = PControl_BLDC + IControl_BLDC;
  
  pwm_in_BLDC = 255/24 * PIControl_BLDC;

  if(pwm_in_BLDC < 0)
  {
    pwm_in_BLDC = 0;
  }
  else if(pwm_in_BLDC > 255)
  {
    pwm_in_BLDC = 255;
  }


  analogWrite(A, pwm_in_BLDC);
//  Serial.println(angular_velocity_BLDC);

}


void doEncoderA(){ 
  
  angle_L += (digitalRead(encoderPinA)==digitalRead(encoderPinB))?0.0577:-0.0577; // 360/52/120 = 0.0577 (52 : 1회전시 52펄스, 120 : 감속비)
  
}

void doEncoderB(){ 

  angle_L += (digitalRead(encoderPinA)==digitalRead(encoderPinB))?-0.0577:0.0577; // 360/52/120 = 0.0577 (52 : 1회전시 52펄스, 120 : 감속비)

}

void doEncoderC(){ 
  
  angle_R += (digitalRead(encoderPinC)==digitalRead(encoderPinD))?0.0577:-0.0577; // 360/52/120 = 0.0577 (52 : 1회전시 52펄스, 120 : 감속비)
  
}

void doEncoderD(){ 

  angle_R += (digitalRead(encoderPinC)==digitalRead(encoderPinD))?-0.0577:0.0577; // 360/52/120 = 0.0577 (52 : 1회전시 52펄스, 120 : 감속비)

}


void encoder_interrupt()
{

  angle_BLDC += 0.12;

}
