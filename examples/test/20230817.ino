

// b3000 3000 1500 1500 100
// approx_100 -> 1000ms (1s), 1cm ?
// encoder z: 150


//#include <Stepper.h>
//#include <AccelStepper.h>
#include <SMRE.h>


//// Stepper motor info.
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


//// Rotary encoder info.
////

const int encoderx_PinA = 21;
const int encdoery_PinA = 20;

const int encoderz_PinA = 2;
const int encoderz_PinB = 3;

const int encoderZ_PinA = 19;
const int encoderZ_PinB = 18;



unsigned long time_a, time_b;


/////////////////////////////////////////////////////////
/////////////////// Interrupt pin Num. //////////////////
/////////////////// for Rotary Encoder //////////////////
/// | int.0 | int.1 | int.2 | int.3 | int.4 | int.5 | ///
/// |   2   |   3   |  21   |  20   |  19   |  18   | ///
/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////


int A[3][84] = {{-81, 82, -87, 74, -105, 82, -79, 80, -111, 80, -67, 76, -103, 72, -103, 78, -81, 92, -107, 100, -115, 74, -111, 88, -103, 82, -105, 84, -105, 84, -99, 86, -63, 74, -107, 70, -107, 128, -97, 102, -107, 108, -87, 110, -75, 106, -85, 104, -115, 118, -101, 104, -89, 98, -103, 86, -119, 104, -113, 90, -105, 88, -121, 112, -89, 94, -113, 96, -93, 94, -135, 96, -41, 70, -87, 92, -115, 92, -135, 110, -97, 96, -119, 16},
{650, 2750, 1850, 2450, 1550, 2850, 1500, 2050, 1800, 2800, 1200, 2000, 1800, 2450, 2150, 2400, 1450, 4000, 1200, 2600, 4700, 2750, 1400, 2450, 1600, 2200, 1700, 2100, 1500, 2600, 2150, 2900, 1800, 1950, 1900, 2100, 2150, 2700, 2400, 2200, 1700, 2050, 2050, 2800, 1000, 1750, 1250, 2900, 1700, 2550, 1900, 2150, 2550, 1900, 1900, 2400, 2100, 3150, 2100, 2300, 2200, 2850, 1450, 3750, 1100, 2500, 2100, 2650, 1250, 1900, 2150, 2650, 1100, 1350, 1050, 2400, 1700, 2100, 2050, 2950, 1950, 1800, 2050, 1150},
{1283, 292, 434, 329, 520, 281, 538, 392, 446, 286, 676, 402, 446, 329, 374, 336, 557, 200, 676, 309, 170, 292, 577, 329, 503, 366, 472, 383, 538, 309, 374, 276, 446, 412, 423, 383, 374, 297, 336, 366, 472, 392, 392, 286, 812, 459, 649, 276, 472, 316, 423, 374, 316, 423, 423, 336, 383, 254, 383, 350, 366, 281, 557, 214, 738, 322, 383, 303, 649, 423, 374, 303, 738, 600, 773, 336, 472, 383, 392, 271, 412, 446, 392, 706}};

//int A[2][4] = {{100, -100}, {10, 10}};
//int A[2] = {100, -100};

//int B[2][4] = {{200, -200}, {10, 10}};
int B[2] = {200, -200};

int S[2] = {10, 15};


//int STEPS_PER_REV = 200;

//const int step_c;

// const int stepsPerRevolution = 500; // change this to fit the number of steps per revolution

// for your motor0

// initialize the Stepper library on pins 8 through 11:

//AccelStepper stepper1(AccelStepper::FULL4WIRE, in1Pin, in2Pin, in3Pin, in4Pin);
//AccelStepper stepper2(AccelStepper::FULL4WIRE, in5Pin, in6Pin, in7Pin, in8Pin);

//StepperMotor motor0(motor_pin_1, motor_pin_3, motor_pin_2, motor_pin_4);


// for test motor (3,4 motor is dummy)
SMRE motor0(4, motor_pin_1, motor_pin_3, motor_pin_2, motor_pin_4);
SMRE motor1(4, motor_pin_5, motor_pin_7, motor_pin_6, motor_pin_8);
SMRE motor2(4, motor_pin_9, motor_pin_11, motor_pin_10, motor_pin_12);
SMRE motor3(4, motor_pin_13, motor_pin_15, motor_pin_14, motor_pin_16);



// for 4D-phantom
//SMRE motor0(2, CW_x, CCW_x);
//SMRE motor1(2, CW_y, CCW_y);
//SMRE motor2(2, CW_z, CCW_z);
//SMRE motor3(2, CW_Z, CCW_Z);


// Rotary encoder setting
volatile long encoderz_Pos = 0;
volatile long encoderZ_Pos = 0;


char receivedCommand;

bool deterTrue = false;
bool newData = false;

long receivedDistance_0;
long receivedDistance_1;
long receivedDistance_2;
long receivedDistance_3;

long targetEncoder_0;
long targetEncoder_1;
long targetEncoder_2;
long targetEncoder_3;


// State Roatary Encoder (position)
bool StateRotaryEncoder_0 = false;
bool StateRotaryEncoder_1 = false;
bool StateRotaryEncoder_2 = false;
bool StateRotaryEncoder_3 = false;

bool calculate_0 = false;
bool calculate_1 = false;
bool calculate_2 = false;
bool calculate_3 = false;

bool moveHome = false;
bool moveHomeCalcul = false;

long cycletime_;



void setup()
{
  Serial.begin(115200);
  
  Serial.println("-----Starting Stepper Motor-----");
  
  Serial.print("List array: ");
  Serial.print("A:");
//  Serial.print(A[0][0]);
//  Serial.print(", ");
  Serial.print(sizeof(A[0][0]));
  Serial.print(", ");
  Serial.print(sizeof(A[0]));
  Serial.print(", ");
  Serial.print(sizeof(A[1]));
  Serial.print(", ");
  Serial.println(sizeof(A[2]));

//  for (int i=0; i< sizeof(A[0])/2; i++)
//    Serial.println(A[0][i]);


  //// Rotary encoder ////
//  attachInterrupt(digitalPinToInterrupt(encoderx_PinA), Encoder_x_CW, RISING);
  
//  attachInterrupt(digitalPinToInterrupt(encodery_PinA), Encoder_y_CW, RISING);

  attachInterrupt(digitalPinToInterrupt(encoderz_PinA), Encoder_z_CW, RISING); // encoder_PinA
  attachInterrupt(digitalPinToInterrupt(encoderz_PinB), Encoder_z_CCW, RISING); // encoder_PinB

  attachInterrupt(digitalPinToInterrupt(encoderZ_PinA), Encoder_Z_CW, RISING); // encoderZ_PinA
  attachInterrupt(digitalPinToInterrupt(encoderZ_PinB), Encoder_Z_CCW, RISING); // encoderZ_PinB
}



void loop()
{
  // Checking the input serial
  checkSerial();
  // Operating the Stepper motor
  operateMotor();
  // Operating the Stepper motor (ver. list load)
  operateMotorList();
  // Homing
  goHome();
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

      // Operation motor mode
      motor0.moveTo(receivedDistance_0);
      motor1.moveTo(receivedDistance_1);
      motor2.moveTo(receivedDistance_2);
      motor3.moveTo(receivedDistance_3);

      // Operation encoder mode
//      motor0.moveToEncoder(10000);
//      motor1.moveToEncoder(10000);
//      motor2.moveToEncoder(10000);
//      motor3.moveToEncoder(10000);


      motor0.cycleTime(cycletime_);
      motor1.cycleTime(cycletime_);
      motor2.cycleTime(cycletime_);
      motor3.cycleTime(cycletime_);

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

    if (receivedCommand == 'n') // Stop the stepper motor
    {
      Serial.println("STOP");
      deterTrue = false;
    }

    if (receivedCommand == 'p') // Print the Stepper motor position
    {
      printCurrent();
    }

    if (receivedCommand == 'l')
    {
      samplePrint();
    }

    if (receivedCommand == 'h')
    {
      moveHome = true;
      moveHomeCalcul = true;
    }
  }
  newData = false;
}



void operateMotor()
{
  if (deterTrue == true) // Checking Between target position and current position difference.
  {
    if (calculate_0 == true && calculate_1 == true && calculate_2 == true && calculate_3 == true)
      {
        motor0.moveTo(receivedDistance_0);
        motor1.moveTo(receivedDistance_1);
        motor2.moveTo(receivedDistance_2);
        motor3.moveTo(receivedDistance_3);

        motor0.cycleTime(cycletime_);
        motor1.cycleTime(cycletime_);
        motor2.cycleTime(cycletime_);
        motor3.cycleTime(cycletime_);
      }

    if (motor0.distanceToGo() == 0 && motor1.distanceToGo() == 0 && motor2.distanceToGo() == 0 && motor3.distanceToGo() == 0)
    {
      Serial.print("motor0 position: ");
      Serial.println(motor0.currentPosition());
      motor0.moveTo(-motor0.currentPosition());
      
      Serial.print("motor1 position: ");
      Serial.println(motor1.currentPosition());
      motor1.moveTo(-motor1.currentPosition());
      
      Serial.print("motor2 position: ");
      Serial.println(motor2.currentPosition());
      motor2.moveTo(-motor2.currentPosition());
      
      Serial.print("motor3 position: ");
      Serial.println(motor3.currentPosition());
      motor3.moveTo(-motor3.currentPosition());

      Serial.print("encoder: ");
      Serial.println(encoderz_Pos);
      
      time_a = millis();
    }

    motor0.run();
//    motor0.runEncoder();
    motor1.run();
//    motor1.runEncoder();

//    motor2.run();
    motor2.runEncoder();
//    motor3.run();
    motor3.runEncoder();

    if (receivedDistance_0 == motor0.currentPosition() && receivedDistance_1 == motor1.currentPosition() && receivedDistance_2 == motor2.currentPosition() && receivedDistance_3 == motor3.currentPosition())
    {
      time_b = millis();
      Serial.print("Time:");
      Serial.println(time_b - time_a);
    }  
    else if (-receivedDistance_0 == motor0.currentPosition() && -receivedDistance_1 == motor1.currentPosition() && -receivedDistance_2 == motor2.currentPosition() && -receivedDistance_3 == motor3.currentPosition())
    {
      time_b = millis();
      Serial.print("Time:");
      Serial.println(time_b - time_a);
    }

//    motor0.moveTo(receivedDistance_0);
//    motor1.moveTo(receivedDistance_1);
//    motor2.moveTo(receivedDistance_2);
//    motor3.moveTo(receivedDistance_3);
//
//    motor0.cycleTime(cycletime_);
//    motor1.cycleTime(cycletime_);
//    motor2.cycleTime(cycletime_);
//    motor3.cycleTime(cycletime_);
//
//    motor0.run();
//    motor1.run();
//    motor2.run();
//    motor3.run();
//
//    Serial.print("motor0 position: ");
//    Serial.println(motor0.currentPosition());
//    Serial.println(motor0.targetPosition());
  }
}



void operateMotorList()
{
  
}



void goHome()
{
  if (moveHome == true)
  {
    if (moveHomeCalcul == true)
    {
      motor0.moveTo(0);
      motor0.cycleTime(100);
      motor1.moveTo(0);
      motor1.cycleTime(100);
      motor2.moveTo(0);
      motor2.cycleTime(100);
      motor3.moveTo(0);
      motor3.cycleTime(100);

      moveHomeCalcul = false;
    }

    motor0.run();
    motor1.run();
    motor2.run();
    motor3.run();

    if (motor0.distanceToGo() == 0 && motor1.distanceToGo() == 0 && motor2.distanceToGo() == 0 && motor3.distanceToGo() == 0)
    {
      moveHome = false;
    }
  }
}



//
// Serial input message 'p'
//
void printCurrent()
{
  Serial.println("---------Printing current stepper motor postion---------");
  Serial.print("motor0(target, current): ");
  Serial.print(motor0.currentPosition());
  Serial.print(", ");
  Serial.println(motor0.targetPosition());

  Serial.print("motor1(target, current): ");
  Serial.print(motor1.currentPosition());
  Serial.print(", ");
  Serial.println(motor1.targetPosition());

  Serial.print("motor2(target, current): ");
  Serial.print(motor2.currentPosition());
  Serial.print(", ");
  Serial.println(motor2.targetPosition());

  Serial.print("motor3(target, current): ");
  Serial.print(motor3.currentPosition());
  Serial.print(", ");
  Serial.println(motor3.targetPosition());
  Serial.println("");
}



// Serial input message 'l'
void samplePrint()
{
  Serial.println("---------Printing current encoder position---------");
  Serial.print("Encoder0(current, target): ");
  Serial.print(motor0.currentPositionEncoder());
  Serial.print(", ");
  Serial.println(motor0.targetPositionEncoder());

  Serial.print("Encoder1(current, target): ");
  Serial.print(motor1.currentPositionEncoder());
  Serial.print(", ");
  Serial.println(motor1.targetPositionEncoder());

  Serial.print("Encoder2(current, target): ");
  Serial.print(motor2.currentPositionEncoder());
  Serial.print(", ");
  Serial.println(motor2.targetPositionEncoder());

  Serial.print("Encoder3(current, target): ");
  Serial.print(motor3.currentPositionEncoder());
  Serial.print(", ");
  Serial.println(motor3.targetPositionEncoder());

  Serial.println("micros, millis");
  Serial.println(micros());
  Serial.println(millis());
}


/////////////////////////
// Rotary encoder void //
/////////////////////////


void Encoder_z_CW()
{
  int encoder_zb = digitalRead(encoderz_PinB);
  if(encoder_zb > 0)
  {
    encoderz_Pos++;
  }
}


void Encoder_z_CCW()
{
  int encoder_za = digitalRead(encoderz_PinA);
  if(encoder_za > 0)
  {
    encoderz_Pos--;
  }
}


void Encoder_Z_CW()
{
  int encoder_Zb = digitalRead(encoderZ_PinB);
  if(encoder_Zb > 0)
  {
    encoderZ_Pos++;
  }
}


void Encoder_Z_CCW()
{
  int encoder_Za = digitalRead(encoderZ_PinA);
  if(encoder_Za > 0)
  {
    encoderZ_Pos--;
  }
}


