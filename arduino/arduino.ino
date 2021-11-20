#include <AFMotor.h>

AF_DCMotor FRONT_LEFT_MOTOR(1);
AF_DCMotor BACK_LEFT_MOTOR(2);
AF_DCMotor FRONT_RIGHT_MOTOR(4);
AF_DCMotor BACK_RIGHT_MOTOR(3);

#define fspeed     250 //this's not worable yet
#define corspeed   150 //this is base/initial motor speed
#define turnspeed  160

#define Kp 0
#define Ki 0
#define Kd 0

#define leftCenterSensor   19  //analog pin A5
#define leftNearSensor     18  //analog pin A4
#define leftFarSensor      17  //analog pin A3
#define rightCenterSensor  20  //analog pin A6
#define rightNearSensor    21  //analog pin A7
#define rightFarSensor      2   //digital pin D2
#define Lleapsensor        16  //analog pin A2
#define Rleapsensor        14  //analog pin A0
#define Vin                15  //analog pin A1

//Motor PWM pins
#define leftMotor1    3  //forward pin
#define leftMotor2    9
#define rightMotor1  10  //forward pin
#define rightMotor2  11

short interval = 0; //For generelized Blink without delay function
bool psensor[6]; //The Sensor Array
short phase = 0;
unsigned long previousMillis = 0; //For blink without delay function
unsigned char mode;
char path[100];
short pathlen = 0;
short Speed;

short error; //line following error, pid will always try to minimize this error
float p; //proportional term
float i; //integral term
float d; //differential term
float pid; //pid term
float prev_error; //previous error term
float prev_i; //previous integral term
short initial_motor_speed = 0;


//List of all function (Not Prototype... (prototypes not required/mandatory in Arduino))

//Tone Functions

//Main functions
void readSensor();
void Straight(short);
void correct(); //Correct the normal line following
void Stop(); //Stop all motors
void Leap();
void Left();
void Right();
void Yaw(char , short);  //Generalized Yaw function

void setup()
{
  //Sensor pins Mode config
  pinMode(leftCenterSensor, INPUT);
  pinMode(leftNearSensor, INPUT);
  pinMode(leftFarSensor, INPUT);
  pinMode(rightCenterSensor, INPUT);
  pinMode(rightNearSensor, INPUT);
  pinMode(rightFarSensor, INPUT);
  pinMode(Lleapsensor, INPUT);
  pinMode(Rleapsensor, INPUT);

  //Motor pins Mode config
  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);
  pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);

  Serial.begin(115200); //For debugging only


}


//Setting up the Sensor Array
void readSensor()
{
  psensor[0] = digitalRead(leftFarSensor);
  psensor[1] = digitalRead(leftNearSensor);
  psensor[2] = digitalRead(leftCenterSensor);
  if (analogRead(rightCenterSensor) > 500)
    psensor[3] = 1;
  else
    psensor[3] = 0;
  if (analogRead(rightNearSensor) > 500)
    psensor[4] = 1;
  else
    psensor[4] = 0;
  psensor[5] = digitalRead(rightFarSensor);

  /*FIVE (5) POSIBILITIES The Robo Will encounter (including when the robo is normally following line, their Sensor Data ----
      0 0 1 1 0 0  == The Robo is on the line, perfectly alligned
      1 1 1 1 0 0  == The Robo is on an intersection => "Straight/Left" or "only Left"
      0 0 1 1 1 1  == ..................intersection => "Straight/Right" or "only Right"
      1 1 1 1 1 1  == ..................intersection => "T-intsersection" or "Cross-intersection" or "End of Maze"
      0 0 0 0 0 0  == ..................intersection => "Dead End"
  */

  if (psensor[0] == 0)
  {
    if (path[pathlen] == 'R')
    {
      ++pathlen;
      path[pathlen] = 'L';
    }
    else
    {
      path[pathlen] = 'L';
    }
  }

  else if (psensor[5] == 0)
  {
    if (path[pathlen] == 'L')
    {
      ++pathlen;
      path[pathlen] = 'R';
    }
    else
    {
      path[pathlen] = 'R';
    }
  }

  //Case: "0 0 1 1 0 0" or "0 1 1 0 0 0" or "0 0 0 1 1 0" or "0 0 1 1 1 0" or "0 1 1 1 0 0"
  //for line : "1 1 0 0 0 0" or "1 1 1 0 0 0" or "1 0 0 0 0 0" or "0 0 0 0 0 1" or "0 0 0 0 1 1" or "0 0 0 1 1 1"
  if (psensor[0] == 0 && psensor[1] == 1 && psensor[2] == 1 && psensor[3] == 1 && psensor[4] == 1 && psensor[5] == 1)
  {
    mode = 'O';
    error = 3;
  }

  else if (psensor[0] == 0 && psensor[1] == 0 && psensor[2] == 1 && psensor[3] == 1 && psensor[4] == 1 && psensor[5] == 1)
  {
    mode = 'O';
    error = 3;
    if (path[pathlen] == 'R')
    {
      ++pathlen;
      path[pathlen] = 'L';
    }
    else
    {
      path[pathlen] = 'L';
    }
  }

  else if (psensor[0] == 1 && psensor[1] == 0 && psensor[2] == 0 && psensor[3] == 1 && psensor[4] == 1 && psensor[5] == 1)
  {
    mode = 'O';
    error = 2;
  }

  else if (psensor[0] == 1 && psensor[1] == 0 && psensor[2] == 0 && psensor[3] == 0 && psensor[4] == 1 && psensor[5] == 1)
  {
    mode = 'O';
    error = 1;
  }

  else if ((psensor[0] == 1 && psensor[1] == 1 && psensor[2] == 0 && psensor[3] == 0 && psensor[4] == 1 && psensor[5] == 1) ||
           (psensor[0] == 1 && psensor[1] == 1 && psensor[2] == 1 && psensor[3] == 0 && psensor[4] == 1 && psensor[5] == 1) ||
           (psensor[0] == 1 && psensor[1] == 1 && psensor[2] == 0 && psensor[3] == 1 && psensor[4] == 1 && psensor[5] == 1))
  {
    mode = 'O';
    error = 0;
  }

  else if (psensor[0] == 1 && psensor[1] == 1 && psensor[2] == 0 && psensor[3] == 0 && psensor[4] == 0 && psensor[5] == 1)
  {
    mode = 'O';
    error = -1;
  }

  else if (psensor[0] == 1 && psensor[1] == 1 && psensor[2] == 1 && psensor[3] == 0 && psensor[4] == 0 && psensor[5] == 1)
  {
    mode = 'O';
    error = -2;
  }

  else if (psensor[0] == 1 && psensor[1] == 1 && psensor[2] == 1 && psensor[3] == 1 && psensor[4] == 0 && psensor[5] == 0)
  {
    mode = 'O';
    error = -3;
    if (path[pathlen] == 'L')
    {
      ++pathlen;
      path[pathlen] = 'R';
    }
    else
    {
      path[pathlen] = 'R';
    }
  }

  else if (psensor[0] == 1 && psensor[1] == 1 && psensor[2] == 1 && psensor[3] == 1 && psensor[4] == 1 && psensor[5] == 0)
  {
    mode = 'O';
    error = -3;
    if (path[pathlen] == 'L')
    {
      ++pathlen;
      path[pathlen] = 'R';
    }
    else
    {
      path[pathlen] = 'R';
    }
  }

  //Case: "1 1 1 1 0 0" or "1 1 1 1 1 0" second case for (in case) correction error
  else if (psensor[0] == 0 && psensor[1] == 0 && psensor[2] == 0 && psensor[3] == 0 && psensor[4] == 1 && psensor[5] == 1)
  {
    mode = 'L';
    error = 100;
    if (path[pathlen] == 'R')
    {
      ++pathlen;
      path[pathlen] = 'L';
    }
    else
    {
      path[pathlen] = 'L';
    }
  }
  //Case: "0 0 1 1 1 1" or "0 1 1 1 1 1" second case for (in case) correction error
  else if (psensor[0] == 1 && psensor[1] == 1 && psensor[2] == 0 && psensor[3] == 0 && psensor[4] == 0 && psensor[5] == 0)
  {
    mode = 'R';
    error = 101;
  }
  //Case: "1 1 1 1 1 1"
  else if (psensor[0] == 0 && psensor[1] == 0 && psensor[2] == 0 && psensor[3] == 0 && psensor[4] == 0 && psensor[5] == 0)
  {
    mode = 'X'; //checkpoint detector
    error = 102;
  }
  //Case: "0 0 0 0 0 0"
  else if (psensor[0] == 1 && psensor[1] == 1 && psensor[2] == 1 && psensor[3] == 1 && psensor[4] == 1 && psensor[5] == 1)
  {
    mode = 'D'; //Dead End
    error = 103;
  }


  Serial.print(psensor[0]);
  Serial.print(psensor[1]);
  Serial.print(psensor[2]);
  Serial.print(psensor[3]);
  Serial.print(psensor[4]);
  Serial.print(psensor[5]);
  Serial.print("  -> MODE : ");
  Serial.print(mode);
  Serial.print("  ERROR : ");
  Serial.println(error);
}
//Sensor array config done


void loop()
{
  unsigned long cur_mil = millis();
  readSensor();
  initial_motor_speed = corspeed;

  if (cur_mil - previousMillis >= 100)
  {
  }
  if (cur_mil - previousMillis >= 1000)
  {
    //idk
  }

  switch (mode)
  {
    case 'O' :  //On-Line
      {
        //Serial.println("on-line");
        cal_pid();
        forward_correct();
        //Go straight and on the same time correct position
        //Should not consist of any delay
        readSensor();
        break;
      }


    case 'L' :
      {
        Stop();
        delay(50);
        readSensor(); //new

        FRONT_RIGHT_MOTOR.setSpeed(150);
        FRONT_RIGHT_MOTOR.run(FORWARD);
        BACK_RIGHT_MOTOR.setSpeed(150);
        BACK_RIGHT_MOTOR.run(FORWARD);
        FRONT_LEFT_MOTOR.setSpeed(150);
        FRONT_LEFT_MOTOR.run(FORWARD);
        BACK_LEFT_MOTOR.setSpeed(150);
        BACK_LEFT_MOTOR.run(FORWARD);
        //        analogWrite(leftMotor1, 150);
        //        analogWrite(rightMotor1, 150);
        //        analogWrite(leftMotor2, 0);
        //        analogWrite(rightMotor2, 0);
        delay(150);
        Stop();
        //delay(50);
        Left();
        readSensor();
        break;
      }

    case 'R' :
      {
        Stop();
        delay(50);
        readSensor();

        FRONT_RIGHT_MOTOR.setSpeed(150);
        FRONT_RIGHT_MOTOR.run(FORWARD);
        BACK_RIGHT_MOTOR.setSpeed(150);
        BACK_RIGHT_MOTOR.run(FORWARD);
        FRONT_LEFT_MOTOR.setSpeed(150);
        FRONT_LEFT_MOTOR.run(FORWARD);
        BACK_LEFT_MOTOR.setSpeed(150);
        BACK_LEFT_MOTOR.run(FORWARD);

        //        analogWrite(leftMotor1, 150);
        //        analogWrite(rightMotor1, 150);
        //        analogWrite(leftMotor2, 0);
        //        analogWrite(rightMotor2, 0);
        delay(150);
        Stop();
        //delay(50);
        Right();
        readSensor();
        break;
      }


    case 'X' :  //Cross-intersection (Checkpoint detector if 1 1 1 1 1 1)
      {
        Stop();
        Serial.println("++++++++++++++++ CHECKPOINT detected +++++++++++++++++++");

        Left(); //giving particular turn the prioirity

        previousMillis = cur_mil;
        Serial.println(cur_mil);

        cal_pid();
        forward_correct();

        readSensor();
        break;
      }

    case 'D' :  //Dead-End
      {
        Stop();
        readSensor();
        Serial.println("CASE D!!!");
        Serial.print(path);
        Serial.println(" ");
        unsigned long timer = 0;
        do
        {
          timer++;
          Serial.print("Detecting Line :> ");
          Serial.println(timer);

          //          analogWrite(leftMotor1, 150);
          //          analogWrite(rightMotor1, 150);
          //          analogWrite(leftMotor2, 0);
          //          analogWrite(rightMotor2, 0);
          readSensor();
          if (timer > 30)
          {
            goto End;
          }
        } while (error > 3 || error < -3);
End:
        Stop();

        if (path[pathlen] == 'L')
        {
          Left();
        }
        else if (path[pathlen] == 'R')
        {
          Right();
        }
        pathlen = 0;
        break;
      }
  }
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

  analogWrite(leftMotor1, left_motor_speed); //Left Motor Speed
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

void Yaw(char direc , short Spd)
{
  switch (direc)
  {
    case 'L' :
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
    case 'R' :
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
