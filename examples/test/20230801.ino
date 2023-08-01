
//#include <Stepper.h>
//#include <AccelStepper.h>
#include <SMRE.h>

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

//int STEPS_PER_REV = 200;

//const int step_c;

// const int stepsPerRevolution = 500; // change this to fit the number of steps per revolution

// for your motor1

// initialize the Stepper library on pins 8 through 11:

//AccelStepper stepper1(AccelStepper::FULL4WIRE, in1Pin, in2Pin, in3Pin, in4Pin);
//AccelStepper stepper2(AccelStepper::FULL4WIRE, in5Pin, in6Pin, in7Pin, in8Pin);

//StepperMotor motor1(motor_pin_1, motor_pin_3, motor_pin_2, motor_pin_4);


SMRE motor1(4, motor_pin_1, motor_pin_3, motor_pin_2, motor_pin_4);
SMRE motor2(4, motor_pin_5, motor_pin_7, motor_pin_6, motor_pin_8);
SMRE motor3(4, motor_pin_9, motor_pin_11, motor_pin_10, motor_pin_12);
SMRE motor4(4, motor_pin_13, motor_pin_15, motor_pin_14, motor_pin_16);


//int stepCount = 0;
//int stepCount2 = 0;
//
//int cur_location1 = 0;
//
//int delay_c = 100;


int cycletime_ = 1;


void setup()
{
  Serial.begin(115200);

  Serial.println("-----Starting Stepper Motor-----");

  //  motor1.moveTo(100);
  //  motor1.cycleTime(1);
  //  motor2.moveTo(300);
  //  motor2.cycleTime(1);
  //  motor3.moveTo(0);
  //  motor3.cycleTime(1);
  //  motor4.moveTo(0);
  //  motor4.cycleTime(1);

//    motor1.moveTo(100);
//    motor2.moveTo(400);
//    motor3.moveTo(0);
//    motor4.moveTo(0);
//  
//    motor1.cycleTime(cycletime_);
//    motor2.cycleTime(cycletime_);
//    motor3.cycleTime(cycletime_);
//    motor4.cycleTime(cycletime_);
}



char receivedCommand;

bool deterTrue = false;
bool newData = false;

long receivedDistance_0, receivedDistance_1, receivedDistance_2, receivedDistance_3;

void loop()
{

  // Checking the input serial
  checkSerial();

  // Operating the Stepper motor
  operateMotor();

  motor1.run();
  motor2.run();
  motor3.run();
  motor4.run();

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
    if (receivedCommand = 'b')
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

      Serial.println();
    }
  }

  newData = false;
}



void operateMotor()
{
  if (deterTrue == true) // Checking Between target position and current position difference.
  {
    motor1.run();
    motor2.run();
    motor3.run();
    motor4.run();
  }
}


