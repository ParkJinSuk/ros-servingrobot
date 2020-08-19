#include <avr/io.h>
#include <avr/interrupt.h>


#define A  11
#define B  10

#define C  6
#define D  5

#define SPEED_OUT 2



double angle = 0;
double temp_angle = 0;
double angular_velocity = 0;
unsigned long time_global = 0;
double frequency;
double t;

double total_angle = 0;
double temp_total_angle = 0;
double angle_per_sec = 0;
double input_v = 15;
double error; 

double Ke = 0.0688;
double Kp = 0.20824;
double Ki = 1.7062;

double PControl;
double IControl=0;
double PIControl;
double PIControl_prev = 0;
int pwm_in, pwm_in_prev;

void setup()
{
  Serial.begin(9600);
  TCCR1A = (0<<WGM11) | (0<<WGM10); // CTC Mode
  TCCR1B = (1<<WGM13) |(1<<WGM12) | (1<<CS12) | (0<<CS11) | (1<<CS10) ; //1024분주

  ICR1 = 1562/10; //주파수 정하기 (1562일 때 10Hz)
  frequency = 15625/ICR1;
  t = 1/frequency;
  
  TIMSK1 |= (1<<OCIE1A); // Compare 인터럽트 사용.
  sei();
  
  TCNT0 = 0; // 카운터 초기화



  pinMode(A, OUTPUT);
  pinMode(B, OUTPUT);
  pinMode(C, OUTPUT);
  pinMode(D, OUTPUT);
  pinMode(SPEED_OUT, INPUT_PULLUP);  
  attachInterrupt(0, encoder_interrupt, FALLING);
  // initialize
  digitalWrite(B, LOW);
  digitalWrite(C, LOW);
  digitalWrite(D, HIGH);

}

void loop()
{

}


ISR (TIMER1_COMPA_vect)  // Compare 인터럽트
{
  TCNT0 = 0;

//  Serial.print("각속도:"); Serial.println(angular_velocity);
//  Serial.print("\tError:"); Serial.print(error);
//  Serial.print("\tP값  :"); Serial.print(PControl);
//  Serial.print("\tI값  :"); Serial.print(IControl);
//  Serial.print("\tPI값 :"); Serial.print(PIControl);
//  Serial.print("\t각도 :"); Serial.print(angle);
//  Serial.print("\tpwm값 :"); Serial.println(pwm_in);
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


  analogWrite(A, pwm_in);

}

void encoder_interrupt()
{

  angle += 0.12;

}
