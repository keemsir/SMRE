

//#include <Stepper.h>
//#include <AccelStepper.h>
#include <SMRE.h>


// -
const int CW_x = 22;
const int CCW_x = 24;
// +
const int CW_y = 30;
const int CCW_y = 32;
// -
const int CW_z = 38;
const int CCW_z = 40;
// +
const int CW_Z = 46;
const int CCW_Z = 48;


const int motor_pin_1 = 5;
const int motor_pin_2 = 6;
const int motor_pin_3 = 7;
const int motor_pin_4 = 8;

const int motor_pin_5 = 9;
const int motor_pin_6 = 10;
const int motor_pin_7 = 11;
const int motor_pin_8 = 12;

const int motor_pin_9 = 13;
const int motor_pin_10 = 14;
const int motor_pin_11 = 15;
const int motor_pin_12 = 16;

const int motor_pin_13 = 17;
const int motor_pin_14 = 18;
const int motor_pin_15 = 19;
const int motor_pin_16 = 20;

unsigned long time_a, time_b;


/////////////////// for Rotary Encoder //////////////////
/// | int.0 | int.1 | int.2 | int.3 | int.4 | int.5 | ///
/// |   2   |   3   |  21   |  20   |  19   |  18   | ///
/////////////////////////////////////////////////////////


//int A[2][4] = {{100, -100}, {10, 10}};
int A[2] = {100, -100};

//int B[2][4] = {{200, -200}, {10, 10}};
int B[2] = {200, -200};

int S[2] = {10, 15};


//int STEPS_PER_REV = 200;

//const int step_c;

// const int stepsPerRevolution = 500; // change this to fit the number of steps per revolution

// for your motor1

// initialize the Stepper library on pins 8 through 11:

//AccelStepper stepper1(AccelStepper::FULL4WIRE, in1Pin, in2Pin, in3Pin, in4Pin);
//AccelStepper stepper2(AccelStepper::FULL4WIRE, in5Pin, in6Pin, in7Pin, in8Pin);

//StepperMotor motor1(motor_pin_1, motor_pin_3, motor_pin_2, motor_pin_4);


// for test motor
SMRE motor1(4, motor_pin_1, motor_pin_3, motor_pin_2, motor_pin_4);
SMRE motor2(4, motor_pin_5, motor_pin_7, motor_pin_6, motor_pin_8);
SMRE motor3(4, motor_pin_9, motor_pin_11, motor_pin_10, motor_pin_12);
SMRE motor4(4, motor_pin_13, motor_pin_15, motor_pin_14, motor_pin_16);


// for 4D-phantom
//SMRE motor1(2, CW_x, CCW_x);
//SMRE motor2(2, CW_y, CCW_y);
//SMRE motor3(2, CW_z, CCW_z);
//SMRE motor4(2, CW_Z, CCW_Z);


char receivedCommand;

bool deterTrue = false;
bool newData = false;

long receivedDistance_0;
long receivedDistance_1;
long receivedDistance_2;
long receivedDistance_3;

long targetEncoder_0 = 500;
long targetEncoder_1;
long targetEncoder_2;
long targetEncoder_3;

long cycletime_;


void setup()
{

  Serial.begin(115200);

  Serial.println("-----Starting Stepper Motor-----");

//  motor1.moveToEncoder(targetEncoder_0);

//  motor1.moveTo(receivedDistance_0);
//  motor2.moveTo(receivedDistance_1);
//  motor3.moveTo(receivedDistance_2);
//  motor4.moveTo(receivedDistance_3);
//
//  motor1.cycleTime(cycletime_);
//  motor2.cycleTime(cycletime_);
//  motor3.cycleTime(cycletime_);
//  motor4.cycleTime(cycletime_);

//  motor1.moveTo(300);
//  motor2.moveTo(300);
//  motor3.moveTo(300);
//  motor4.moveTo(300);
//
//  motor1.cycleTime(cycletime_);
//  motor2.cycleTime(cycletime_);
//  motor3.cycleTime(cycletime_);
//  motor4.cycleTime(cycletime_);
}



void loop()
{
  // Checking the input serial
  checkSerial();
  // Operating the Stepper motor
  operateMotor();
}



void checkSerial()
{
  if (Serial.available() > 0)
  {
    receivedCommand = Serial.read();
    newData = true;
  }

  if (newData == true)
  {
    if (receivedCommand == 'b')
    {
      deterTrue = true;

      receivedDistance_0 = Serial.parseFloat();
      receivedDistance_1 = Serial.parseFloat();
      receivedDistance_2 = Serial.parseFloat();
      receivedDistance_3 = Serial.parseFloat();

      cycletime_ = Serial.parseFloat();

      motor1.moveTo(receivedDistance_0);
      motor2.moveTo(receivedDistance_1);
      motor3.moveTo(receivedDistance_2);
      motor4.moveTo(receivedDistance_3);

      motor1.cycleTime(cycletime_);
      motor2.cycleTime(cycletime_);
      motor3.cycleTime(cycletime_);
      motor4.cycleTime(cycletime_);

      Serial.print("Distance_0: ");
      Serial.println(receivedDistance_0);
      Serial.print("Distance_1: ");
      Serial.println(receivedDistance_1);
      Serial.print("Distance_2: ");
      Serial.println(receivedDistance_2);
      Serial.print("Distance_3: ");
      Serial.println(receivedDistance_3);
      Serial.print("cycleTime_: ");
      Serial.println(cycletime_);
    }

    else if (receivedCommand == 'n') // Stop the stepper motor
    {
      Serial.println("STOP");
      deterTrue = false;
    }
  }
  newData = false;
}



void operateMotor()
{
  if (deterTrue == true) // Checking Between target position and current position difference.
  {

//    motor1.moveTo(receivedDistance_0);
//    motor2.moveTo(receivedDistance_1);
//    motor3.moveTo(receivedDistance_2);
//    motor4.moveTo(receivedDistance_3);
//
//    motor1.cycleTime(cycletime_);
//    motor2.cycleTime(cycletime_);
//    motor3.cycleTime(cycletime_);
//    motor4.cycleTime(cycletime_);
    
    motor1.run();
    motor2.run();
    motor3.run();
    motor4.run();
    
//    motor1.moveTo(receivedDistance_0);
//    motor2.moveTo(receivedDistance_1);
//    motor3.moveTo(receivedDistance_2);
//    motor4.moveTo(receivedDistance_3);
//
//    motor1.cycleTime(cycletime_);
//    motor2.cycleTime(cycletime_);
//    motor3.cycleTime(cycletime_);
//    motor4.cycleTime(cycletime_);
//
//    motor1.run();
//    motor2.run();
//    motor3.run();
//    motor4.run();

//    Serial.print("motor1 position: ");
//    Serial.println(motor1.currentPosition());
//    Serial.println(motor1.targetPosition());
  }
}



//void operateMotor()
//{
//  motor1.run();
//  motor2.run();
//  motor3.run();
//  motor4.run();
//}



