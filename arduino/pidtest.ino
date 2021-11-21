#include <AFMotor.h>

AF_DCMotor FRONT_LEFT_MOTOR(1);
AF_DCMotor BACK_LEFT_MOTOR(2);
AF_DCMotor FRONT_RIGHT_MOTOR(4);
AF_DCMotor BACK_RIGHT_MOTOR(3);

#define fspeed 250   //this's not worable yet
#define corspeed 150 //this is base/initial motor speed
#define turnspeed 160

#define Kp 0
#define Ki 0
#define Kd 0

#define leftCenterSensor 19  //analog pin A5
#define leftNearSensor 18    //analog pin A4
#define leftFarSensor 17     //analog pin A3
#define rightCenterSensor 20 //analog pin A6
#define rightNearSensor 21   //analog pin A7
#define rightFarSensor 2     //digital pin D2
#define Lleapsensor 16       //analog pin A2
#define Rleapsensor 14       //analog pin A0
#define Vin 15               //analog pin A1

//Motor PWM pins
#define leftMotor1 3 //forward pin
#define leftMotor2 9
#define rightMotor1 10 //forward pin
#define rightMotor2 11

short interval = 0; //For generelized Blink without delay function
bool psensor[7];    //The Sensor Array
short phase = 0;
unsigned long previousMillis = 0; //For blink without delay function
unsigned char mode;
char path[100];
short pathlen = 0;
short Speed;

short error;      //line following error, pid will always try to minimize this error
float p;          //proportional term
float i;          //integral term
float d;          //differential term
float pid;        //pid term
float prev_error; //previous error term
float prev_i;     //previous integral term
short initial_motor_speed = 0;

//List of all function (Not Prototype... (prototypes not required/mandatory in Arduino))

//Tone Functions

//Main functions
void readSensor();
void Straight(short);
void correct(); //Correct the normal line following
void Stop();    //Stop all motors
void Leap();
void Left();
void Right();
void Yaw(char, short); //Generalized Yaw function

void setup()
{
  //Motor pins Mode config
  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);
  pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);

  Serial.begin(115200); //For debugging only
}

//Setting up the Sensor Array
void readSensor(int inp)
{
  switch (inp)
  {

  case 1:
  {
    mode = 'O';
    error = 3;
    break;
  }

  case 2:
  {
    mode = 'O';
    error = 2;
    break;
  }

  case 3:
  {
    mode = 'O';
    error = 1;
    break;
  }

  case 4:
  {
    mode = 'O';
    error = 0;
    break;
  }

  case 5:
  {
    mode = 'O';
    error = -1;
    break;
  }

  case 6:
  {
    mode = 'O';
    error = -2;
  }

  case 7:
  {
    mode = 'O';
    error = -3;
  }
  }
  //Sensor array config done

  void loop()
  {
    unsigned long cur_mil = millis();
    readSensor();
    initial_motor_speed = corspeed;

    switch (mode)
    {
    case 'O': //On-Line
    {
      //Serial.println("on-line");
      cal_pid();
      forward_correct();
      //Go straight and on the same time correct position
      //Should not consist of any delay
      readSensor();
      break;
    }

  void cal_pid()
  {
    p = error;
    i = i + prev_i;
    d = error - (prev_error);
    pid = (Kp * p) + (Ki * i) + (Kd * d);
    prev_i = i;
    prev_error = error;
  }

  void forward_correct()
  {
    // Calculating the effective motor speed:
    int left_motor_speed = initial_motor_speed - pid;
    int right_motor_speed = initial_motor_speed + pid;

    // The motor speed should not exceed the max PWM value
    left_motor_speed = constrain(left_motor_speed, 0, 255);
    right_motor_speed = constrain(right_motor_speed, 0, 255);

    analogWrite(leftMotor1, left_motor_speed);   //Left Motor Speed
    analogWrite(rightMotor1, right_motor_speed); //Right Motor Speed
  }

  void Left()
  {
    Stop();
    unsigned long cur_mil = millis();
    /*do
    {
    Yaw('L', turnspeed/Bat_V);  //High Speed Left Yaw
    Serial.println("LEFT TURN EXECUTED");
    readSensor();
    }while(psensor[0]!=0); */
    Yaw('L', turnspeed);
    delay(10);
    Stop();
    do
    {
      Yaw('L', turnspeed); //High Speed Left Yaw
      Serial.println("LEFT TURN");
      readSensor();
    } while (error > 2 || error < -2);
    Stop();
  }

  void Right()
  {
    unsigned long cur_mil = millis();
    /*do
    {
    Yaw('R', turnspeed/Bat_V);  //High Speed Right Yaw
    Serial.println("RIGHT TURN EXECUTED");
    readSensor();
    }while(psensor[5]!=0); */
    Yaw('R', turnspeed);
    delay(10);
    Stop();
    do
    {
      Yaw('R', turnspeed); //High Speed Right Yaw
      Serial.println("RIGHT TURN");
      readSensor();
    } while (error > 2 || error < -2);
    Stop();
  }

  void Yaw(char direc, short Spd)
  {
    switch (direc)
    {
    case 'L':
    {
      Serial.println("Left");
      FRONT_RIGHT_MOTOR.setSpeed(Spd);
      FRONT_RIGHT_MOTOR.run(BACKWARD);
      BACK_RIGHT_MOTOR.setSpeed(Spd);
      BACK_RIGHT_MOTOR.run(BACKWARD);
      FRONT_LEFT_MOTOR.setSpeed(Spd);
      FRONT_LEFT_MOTOR.run(FORWARD);
      BACK_LEFT_MOTOR.setSpeed(Spd);
      BACK_LEFT_MOTOR.run(FORWARD);

      //        analogWrite(leftMotor1, 0);
      //        analogWrite(leftMotor2, Spd);
      //        analogWrite(rightMotor1, Spd);
      //        analogWrite(rightMotor2, 0);

      break;
    }
    case 'R':
    {
      Serial.println("Right");
      FRONT_RIGHT_MOTOR.setSpeed(Spd);
      FRONT_RIGHT_MOTOR.run(FORWARD);
      BACK_RIGHT_MOTOR.setSpeed(Spd);
      BACK_RIGHT_MOTOR.run(FORWARD);
      FRONT_LEFT_MOTOR.setSpeed(Spd);
      FRONT_LEFT_MOTOR.run(BACKWARD);
      BACK_LEFT_MOTOR.setSpeed(Spd);
      BACK_LEFT_MOTOR.run(BACKWARD);

      //        analogWrite(leftMotor1, Spd);
      //        analogWrite(leftMotor2, 0);
      //        analogWrite(rightMotor1, 0);
      //        analogWrite(rightMotor2, Spd);
      break;
    }
    }
  }

  void Stop()
  {
    FRONT_LEFT_MOTOR.run(RELEASE);
    BACK_LEFT_MOTOR.run(RELEASE);
    FRONT_RIGHT_MOTOR.run(RELEASE);
    BACK_RIGHT_MOTOR.run(RELEASE);

    //  analogWrite(leftMotor1, 0);
    //  analogWrite(rightMotor1, 0);
    //  analogWrite(leftMotor2, 0);
    //  analogWrite(rightMotor2, 0);
  }
