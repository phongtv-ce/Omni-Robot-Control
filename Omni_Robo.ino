#include "math.h"
#include <PID_v1.h>
//#include "PinChangeInt.h"

#define MAX_SPEED 255
#define MIN_SPEED -255

#define MAX_SPEED_R 100 //max speed with pwm = 250(about 1.7 pwm for 1 speed unit)
// update time (1s = 1M)
#define UPDATE_TIME 10000


//Motor controll pin
struct Motor
{
  int out1;
  int out2;
  int en;
};

const Motor 
  MOTOR1 = {6, 7, 5}, 
  MOTOR3 = {11, 12, 8}, 
  MOTOR2 = {9, 10, 13};

//Encoder pin
struct Encoder
{
  int pinA, pinB;
};

const Encoder 
  ENC1 = {2, 3}, 
  ENC2 = {18, 19}, 
  ENC3 = {20, 21};

//read from controller
int xAxis=0;
int yAxis=0;
double angle = 0;
int moveSpeed=0;

//motor count
volatile long motor1Count = 0;
volatile long motor2Count = 0;
volatile long motor3Count = 0;

//PID variables

//Kp,Ki,Kd values
double Kp=0.1, Ki=2, Kd=0.01;
//double Kp=00.987, Ki=00.00, Kd=00.01;

//PID variable
double Setpoint1, Input1, Output1,SetpointMove1;
PID pid1(&Input1, &Output1, &Setpoint1, Kp, Ki, Kd, DIRECT);

double Setpoint2, Input2, Output2,SetpointMove2;
PID pid2(&Input2, &Output2, &Setpoint2, Kp, Ki, Kd, DIRECT);

double Setpoint3, Input3, Output3,SetpointMove3;
PID pid3(&Input3, &Output3, &Setpoint3, Kp, Ki, Kd, DIRECT);

//previous time;
unsigned long previousMicros;

//for calc time
long timestamp = 0;
long ms=0;
long msCnt=0;

//process for moving follow shape

long countSquare=0;
long countCircle=0;
long countTriangle=0;
bool special = false;

//update time
long motorUpdateTime = 0;
double angl=0;

void setup()
{
  // put your setup code here, to run once:
  //config pinmode
  pinMode(MOTOR1.out1, OUTPUT);
  pinMode(MOTOR1.out2, OUTPUT);
  pinMode(MOTOR1.en, OUTPUT);
  
  pinMode(MOTOR2.out1, OUTPUT);
  pinMode(MOTOR2.out2, OUTPUT);
  pinMode(MOTOR2.en, OUTPUT);
  
  pinMode(MOTOR3.out1, OUTPUT);
  pinMode(MOTOR3.out2, OUTPUT);
  pinMode(MOTOR3.en, OUTPUT);
  
  pinMode(ENC1.pinA, INPUT);
  pinMode(ENC1.pinB, INPUT);
  
  pinMode(ENC2.pinA, INPUT);
  pinMode(ENC2.pinB, INPUT);
  
  pinMode(ENC3.pinA, INPUT);
  pinMode(ENC3.pinB, INPUT);

  // ATMega2560 Interupt
  attachInterrupt(1, motor1interuptA, CHANGE);
  attachInterrupt(0, motor1interuptB, CHANGE);
  
  attachInterrupt(4, motor2interuptA, CHANGE);
  attachInterrupt(5, motor2interuptB, CHANGE);
  
  attachInterrupt(2, motor3interuptA, CHANGE);
  attachInterrupt(3, motor3interuptB, CHANGE);

  //init
  previousMicros =0;
  
  // config PID
  pid1.SetMode(AUTOMATIC);
  pid1.SetOutputLimits(-255,255);
  //pid1.SetSampleTime(200);
   
  pid2.SetMode(AUTOMATIC);
  pid2.SetOutputLimits(-255,255);
  //pid2.SetSampleTime(200);
  
  pid3.SetMode(AUTOMATIC);
  pid3.SetOutputLimits(-255,255);
  //pid3.SetSampleTime(200);

  //init setpoint
  Setpoint1 = 0;
  SetpointMove1 = 0;
  Setpoint2 = 0;
  SetpointMove2 = 0;
  Setpoint3 = 0;
  SetpointMove3 = 0;
  
  //TCCR1B = TCCR1B & 0b11111000 | 1;
  Serial.begin(115200); 
//init time
  motorUpdateTime=0;
  timestamp = 0;
  ms=0;
  msCnt=0;
//init others
  special=false; // true: in draw shape processing 
  countSquare=0;
  countCircle=0;
  countTriangle=0;
  
  moveSpeed=0;
  xAxis=0;
  yAxis=0; 
  
}

void loop()
{
  unsigned long currentMicros = micros();
  
  // calc time passed
  long timeElapsed = currentMicros- previousMicros;
  previousMicros = currentMicros;
  timestamp = (timestamp + timeElapsed) % 1000000000;
  motorUpdateTime += timeElapsed;
  //count ms
  ms +=  timeElapsed;
  if(ms>1000){
    ms-=1000;
    msCnt+=1;  
  }
  //delay(100);
  //Serial.println(msCnt);
  //xAxis=102;
 
  if(motorUpdateTime>UPDATE_TIME)
  {
    //ti le thoi gian da troi qua trong 1 s
    double secEllpased = motorUpdateTime/1000000.f;
    
    //he so toc do
    double speedFactor = (secEllpased * 46.75);
    
    //Serial.println(motorUpdateTime);
    motorUpdateTime = motorUpdateTime % UPDATE_TIME;
    //Serial.println(xAxis);
    if(abs(xAxis)<=100 && abs(yAxis)<=100 && special == false){
      angl = calcAngle(xAxis,yAxis);
      moveSpeed= sqrt(sq(yAxis)+sq(xAxis));
      moveSpeed=constrain(moveSpeed,-100,100);
      angl = angl+PI/6;
      if(xAxis==0 && yAxis ==0)
      {
        SetpointMove1 = 0;
        SetpointMove2 = 0;
        SetpointMove3 = 0;
      }else{
        SetpointMove1 = v1(angl)*moveSpeed;
        SetpointMove2 = v2(angl)*moveSpeed;
        SetpointMove3 = v3(angl)*moveSpeed;
      }
    }else if( xAxis == 102){ //square
      if(special == false){
        special = true;
        countSquare=8000;
        msCnt=0;
         //set state for draw
      }else if(countSquare>0){
        //Serial.println(countSquare);
        //draw square
        moveSpeed=60;
        if(countSquare>6000){
          // edge 1
          angl = 0;
          SetpointMove1 = v1(angl)*moveSpeed;
          SetpointMove2 = v2(angl)*moveSpeed;
          SetpointMove3 = v3(angl)*moveSpeed;
        }else if(countSquare>4000){
          //edge 2
          angl = PI/2;
          SetpointMove1 = v1(angl)*moveSpeed;
          SetpointMove2 = v2(angl)*moveSpeed;
          SetpointMove3 = v3(angl)*moveSpeed;
        }else if(countSquare>2000){
          //edge 3
          angl = PI;
          SetpointMove1 = v1(angl)*moveSpeed;
          SetpointMove2 = v2(angl)*moveSpeed;
          SetpointMove3 = v3(angl)*moveSpeed;
        }else{
          //edge 4
          angl = 3*PI/2;
          SetpointMove1 = v1(angl)*moveSpeed;
          SetpointMove2 = v2(angl)*moveSpeed;
          SetpointMove3 = v3(angl)*moveSpeed;
        }
        countSquare-=msCnt;
        if(countSquare<=0)
        {
          xAxis=0;
          yAxis=0;
          countSquare=0;
          special=false;
          SetpointMove1 = 0;
          SetpointMove2 = 0;
          SetpointMove3 = 0;
        }
        msCnt=0;
      }
    }else if( xAxis == 103){ //triangle
      if(special == false){
        special = true;
        countTriangle=6000;
        msCnt=0;
        //set state for draw
      }else if(countTriangle>0){
        //draw triangle
        moveSpeed=60;
        if(countTriangle>4000){
          angl = 0;
          SetpointMove1 = v1(angl)*moveSpeed;
          SetpointMove2 = v2(angl)*moveSpeed;
          SetpointMove3 = v3(angl)*moveSpeed;
        }else if(countTriangle>2000){
          angl = 2*PI/3;
          SetpointMove1 = v1(angl)*moveSpeed;
          SetpointMove2 = v2(angl)*moveSpeed;
          SetpointMove3 = v3(angl)*moveSpeed;
        }else{
          angl = PI/3;
          SetpointMove1 = v1(angl)*moveSpeed;
          SetpointMove2 = v2(angl)*moveSpeed;
          SetpointMove3 = v3(angl)*moveSpeed;
        }
        countTriangle-=msCnt;
         if(countTriangle<=0)
        {
          xAxis=0;
          yAxis=0;
          countTriangle=0;
          special=false;
          SetpointMove1 = 0;
          SetpointMove2 = 0;
          SetpointMove3 = 0;
        }
        msCnt=0;
      }
    }else if( xAxis == 104 && special == false){ // left
      SetpointMove1 = 50;
      SetpointMove2 = 50;
      SetpointMove3 = 50;
    }else if( xAxis == 105&& special == false){ // right
      SetpointMove1 = -50;
      SetpointMove2 = -50;
      SetpointMove3 = -50;
    }else if( xAxis == 106){ // circle
      if(special == false){
        special = true;
        countCircle=3600;
        msCnt=0;
        //set state for draw
      }else if(countCircle>0){
        //draw circle
        moveSpeed=50;
        for(long i=0; i<36; ++i){
          if(countCircle>i*100){
            angl = (PI/9)*(double)(i);
            SetpointMove1 = v1(angl)*moveSpeed;
            SetpointMove2 = v2(angl)*moveSpeed;
            SetpointMove3 = v3(angl)*moveSpeed;
          }
        }
        countCircle-=msCnt;
        if(countCircle<=0)
        {
          xAxis=0;
          yAxis=0;
          countTriangle=0;
          special=false;
          SetpointMove1 = 0;
          SetpointMove2 = 0;
          SetpointMove3 = 0;
        }
        msCnt=0;
      }
    }
    
    // toc do mong muon
    Setpoint1 = SetpointMove1;
    Setpoint2 = SetpointMove2;
    Setpoint3 = SetpointMove3;
    
    Input1  = (double)motor1Count/   speedFactor;
    Input2  = (double)motor2Count/   speedFactor; //(vong/s)
    Input3  = (double)motor3Count/   speedFactor; 
    
    motor1Count=0;
    motor2Count=0;
    motor3Count=0;
    
//    Serial.print("SPEED 1: ");
//    Serial.println(Input1);
//    Serial.print("SPEED 2: ");
//    Serial.println(Input2);
//    Serial.print("SPEED 3: ");
//    Serial.println(Input3);
//  
    //motor 1  
    pid1.Compute();
    int currentPwm1 = -Output1*1.7;
    currentPwm1 = constrain(currentPwm1,-255,255);
    runMotor(MOTOR1,currentPwm1);
    //motor2
    pid2.Compute();
    int currentPwm2 = -Output2*1.7;
    currentPwm2 = constrain(currentPwm2,-255,255);
    runMotor(MOTOR2,currentPwm2);

    //motor3
    pid3.Compute();
    int currentPwm3 = -Output3*1.7;
    currentPwm3 = constrain(currentPwm3,-255,255);
    runMotor(MOTOR3,currentPwm3);
  }
  if(special==false)
    handleSerialInput();
  else if(Serial.available() >=2){
    Serial.read();
    Serial.read();
  } 
}

void handleSerialInput(){
  //Receive x,y from android app
  if(Serial.available() >= 2){
    xAxis = Serial.read()-100;
    yAxis = Serial.read()-100; 
  } 
}

void runMotor(Motor motor, int pwm)
{ 
  pwm = constrain(pwm,-255,255);
  if(pwm == 0){
    digitalWrite(motor.out2, LOW);
    analogWrite(motor.out1, LOW);
    digitalWrite(motor.en, LOW);
    return;
  }
  if(pwm>0){
    digitalWrite(motor.en, HIGH);
    digitalWrite(motor.out2, HIGH); 
    analogWrite(motor.out1, 255 - pwm);
    return;
  }
  if(pwm<0){
    digitalWrite(motor.en, HIGH);
    digitalWrite(motor.out2, LOW);
    analogWrite(motor.out1, -pwm);
    return;
  }
}


double calcAngle(int x, int y){
  if(x==0 && y==0) 
    return -10;
  if(x<0 && y>=0)
    return atan(y/(double)x)+PI;
  if(x<0 && y<0)
    return atan(y/(double)x)+PI;
  return atan(y/(double)x);
}

double adjustAngle(double angle){
  return angle = angle+PI/6;
}

double v1(double a)
{
  return -sqrt(3) / 2.0 * cos(a) - 0.5 * sin(a);
}
double v2(double a)
{
  return sqrt(3) / 2.0 * cos(a) - 0.5 * sin(a);
}
double v3(double a)
{
  return sin(a);
}

////
////interupts
//// 
void motor1interuptA()
{
  if (digitalRead(ENC1.pinA) != digitalRead(ENC1.pinB))
  {
    motor1Count--;
  }
  else  
  {
    motor1Count++;
  }
}
void motor1interuptB()
{
  if (digitalRead(ENC1.pinA) != digitalRead(ENC1.pinB))
  {
    motor1Count++;
  }
  else  
  {
     motor1Count--;
  }
}

void motor2interuptA()
{
  if (digitalRead(ENC2.pinA) != digitalRead(ENC2.pinB))
  {
    motor2Count--;
  }
  else  
  {
    motor2Count++;
  }
}
void motor2interuptB()
{
  if (digitalRead(ENC2.pinA) != digitalRead(ENC2.pinB))
  {
    motor2Count++;
  }
  else  
  {
     motor2Count--;
  }
}
void motor3interuptA()
{
  if (digitalRead(ENC3.pinA) != digitalRead(ENC3.pinB))
  {
    motor3Count--;
  }
  else  
  {
    motor3Count++;
  }
}
void motor3interuptB()
{
  if (digitalRead(ENC3.pinA) != digitalRead(ENC3.pinB))
  {
    motor3Count++;
  }
  else  
  {
    motor3Count--;
  }
}
