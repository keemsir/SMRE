
//#include <Stepper.h>
//#include <AccelStepper.h>
#include <SMRE.h>


////                     ////
//// Stepper motor info. ////
////                     ////


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
const int motor_pin_2 = 7;
const int motor_pin_3 = 6;
const int motor_pin_4 = 8;

const int motor_pin_5 = 9;
const int motor_pin_6 = 11;
const int motor_pin_7 = 10;
const int motor_pin_8 = 12;

const int motor_pin_9 = 13;
const int motor_pin_10 = 15;
const int motor_pin_11 = 14;
const int motor_pin_12 = 16;

const int motor_pin_13 = 17;
const int motor_pin_14 = 19;
const int motor_pin_15 = 18;
const int motor_pin_16 = 20;

const int Encoder_output_A = 2;
const int Encoder_output_B = 3;

#define Encoder2_PinA 2
#define Encoder2_PinB 3

volatile long encoder2_Pos = 0;

// const int stepsPerRevolution = 500; // change this to fit the number of steps per revolution

// initialize the Stepper library on pins 8 through 11:
// Stepper myStepper(stepsPerRevolution, 8, 9, 10, 11);

//SMRE stepper1(4, motor_pin_1, motor_pin_2, motor_pin_3, motor_pin_4);
//SMRE stepper2(4, motor_pin_5, motor_pin_6, motor_pin_7, motor_pin_8);
//SMRE stepper3(4, motor_pin_9, motor_pin_10, motor_pin_11, motor_pin_12);
//SMRE stepper4(4, motor_pin_13, motor_pin_14, motor_pin_15, motor_pin_16);

// for 4D-phantom
SMRE motor0(2, CW_x, CCW_x);
SMRE motor1(2, CW_y, CCW_y);
SMRE motor2(2, CW_z, CCW_z);
SMRE motor3(2, CW_Z, CCW_Z);

//-------------------------------------------------------------------------//

char receivedCommand;

boolean newData = false;
boolean deterTrue = false;
boolean deterTrue_L = false;
boolean deterTrue_E = false;
boolean deterTrue_E1, deterTrue_E2, deterTrue_E3, deterTrue_E4 = false;

unsigned long time_a, time_b;
unsigned long total_time_a, total_time_b;

int step_c;

int stepCount1 = 0;
int stepCount2 = 0;

boolean stepper1_oper = false;
boolean resetPin = false;

boolean testvoid = false;

int A[2][4] = {{100, -100, 100, -100}, {1000, 1000, 1000, 1000}};

int B[2][4] = {{100, -100, 100, -100}, {1000, 1000, 1000, 1000}};

int C[] = {4, 8, 14, 22, 30, 40, 52, 64, 72, 82, 94, 108, 126, 136, 144, 150, 154, 162, 166, 168, 170, 174, 176, 178, 182, 182, 186, 188, 190, 184, 174, 164, 154, 142, 122, 116, 110, 92, 72, 66, 58, 48, 38, 28, 32, 48, 58, 62, 68, 78, 88, 108, 122, 140, 154, 158, 162, 166, 172, 174, 178, 178, 180, 186, 188, 186, 184, 174, 164, 154, 144, 128, 112, 96, 76, 66, 54, 42, 30, 20, 10, -2, 0, 8, 14, 20, 28, 36, 46, 54, 62, 74, 86, 104, 120, 134, 146, 152, 156, 160, 166, 170, 172, 174, 178, 180, 180, 182, 184, 186, 188, 184, 174, 164, 152, 132, 104, 82, 74, 64, 54, 44, 42, 50, 62, 70, 76, 98, 120, 136, 146, 152, 158, 160, 162, 164, 170, 174, 178, 178, 180, 182, 184, 180, 172, 164, 152, 140, 120, 98, 78, 70, 60, 52, 42, 36, 30, 24, 12, 6, 4, 4, 4, 16, 24, 40, 56, 64, 72, 80, 98, 118, 132, 140, 150, 156, 158, 160, 162, 166, 168, 172, 172, 176, 180, 180, 178, 176, 176, 172, 172, 168, 158, 148, 132, 102, 80, 72, 64, 58, 46, 36, 30, 22, 12, 6, 4};

int targetPos = 0;
int targetPos_C = 0;
float targetPos_1, targetPos_2, targetPos_3, targetPos_4 = 0;
float pretargetPos_1, pretargetPos_2, pretargetPos_3, pretargetPos_4 = 0;

int currentPos = 0;
int currentPos_1, currentPos_2, currentPos_3, currentPos_4 = 0;

int time_Init = 0;
int operTime = 0;
int time_current = 0;
int time_count = 0;
int time_millis = 0;
int lastStepTime = 0;
int stepInterval = 100;

int size_Count = 0;
int maxList = 0;

int time_test = 0;

int a_ = 0;
int s_ = 0;
int d_ = 0;
int f_ = 0;


void setup() {
  Serial.begin(115200);
  Serial.println("|||| Start stepper-motor ||||");
  Serial.println("Ready");

//  pinMode(Encoder2_PinA,INPUT_PULLUP); // sets the Encoder_output_A pin as the input INPUT_PULLUP
//  pinMode(Encoder2_PinB,INPUT_PULLUP); // sets the Encoder_output_B pin as the input INPUT_PULLUP

  time_Init = millis();

//  attachInterrupt(digitalPinToInterrupt(Encoder2_PinA),Encoder_2_CW,RISING);
//  attachInterrupt(digitalPinToInterrupt(Encoder2_PinB),Encoder_2_CCW,RISING);


  // 1 revolution Motor 1 CW
  //  stepper1.setMaxSpeed(1000.0);
  //  stepper1.setAcceleration(50.0);
  //  stepper1.setSpeed(100);
  //  stepper1.moveTo(512);

  // 1 revolution Motor 2 CCW
  //  stepper2.setMaxSpeed(1000.0);
  //  stepper2.setAcceleration(50.0);
  //  stepper2.setSpeed(100);
  //  stepper2.moveTo(-256);

//  stepper1.initialConfig(1000, 1000);
}



/*
  void loop() {
  if (stepper1.distanceToGo() == 0)
    stepper1.moveTo(-stepper1.currentPosition());

  stepper1.run();
  }
*/


/*
  void loop(){

  time_a = micros();
  stepper1.step(5);
  stepper2.step(1);
  stepper3.step(1);
  stepper4.step(1);
  //  Serial.print("steps:");
  //  Serial.println(stepCount1);
  stepCount1++;
  //  delay(2);
  delayMicroseconds(1800);
  time_b = micros();
  Serial.print("Time(ms): ");
  Serial.println(time_b - time_a);
  }
*/



/*
  void loop(){
  if (Serial.available())
  {
    int StepC = Serial.parseInt();

    if (StepC != 0)
    {
      stepper1.operStep(StepC);
      stepper2.operStep(StepC);

      Serial.print("Speed: ");
      Serial.println(StepC);

      stepper1_oper = true;
      stepCount1 = 0;
      time_a = micros();
    }
  }

  if (stepper1_oper == true && stepCount1 < 5000)
  {
    stepper1.moveStep();
    stepCount1++;
  //    Serial.println(stepCount1);
    stepper2.moveStep();
  }
  else if(stepper1_oper == true && stepCount1 == 5000)
  {
    time_b = micros();
    Serial.print("Time: ");
    Serial.println(time_b - time_a);
    stepper1_oper = false;
    stepCount1 = 0;
  }
  else if(stepper1_oper == false)
  {
    stepCount1 = 0;
  //    time_b = micros();
  //    Serial.print("Time: ");
  //    Serial.println(time_b - time_a);
  }
  }
*/



int x = 0;
int axis_1, axis_2, axis_3, axis_4 = 0;
int timeGap = 100; // ms

int moveArrange = 500;
int stepperDelay = 2000; // 300, 430
float stepperDelay_1, stepperDelay_2, stepperDelay_3, stepperDelay_4 = 1000;


void loop() {
//  operMotor();
//  operMotorList();
//  operMotor_E();
  
  operMotor_E1();
  operMotor_E2();
  operMotor_E3();
  operMotor_E4();
  
  checkSerial();

  resetArduino();

  testVoid();



//  timeCycle();
//
//  if (moveArrange != 0)
//  {
//    stepMotor1(x % 4);
//    stepMotor2(x % 4);
//    delayMicroseconds(stepperDelay);
//    x++;
//    moveArrange--;
//  }
//  else if (moveArrange == 0)
//  {
//    x = 0;
//  }
//  if (moveArrange == 1) {
//    time_b = millis();
//    Serial.print("time: ");
//    Serial.println(time_b - time_a);
//  }

  //  stepper1.operStep(2000);

  //  Serial.print("arrange: ");
  //  Serial.println(moveArrange);
  //  Serial.print("x: ");
  //  Serial.println(x);
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
    if (receivedCommand == 'z')
    {
      operTime = millis();
      deterTrue = true;

      targetPos = Serial.parseFloat();
      //received_0 = Serial.parseFloat();

      Serial.print("Target position: ");
      Serial.println(targetPos);

      Serial.print("Step delay: ");
      Serial.println(stepperDelay);
    }

    if (receivedCommand == 'q')
    {
      operTime = millis();
      deterTrue_E1 = true;
      
      targetPos_1 = Serial.parseFloat();

      stepperDelay_1 = 100/abs(targetPos_1 - pretargetPos_1);
    }

    if (receivedCommand == 'w')
    {
      operTime = millis();
      deterTrue_E2 = true;
      
      targetPos_2 = Serial.parseFloat();

      stepperDelay_2 = 100/abs(targetPos_2 - pretargetPos_2);
    }

    if (receivedCommand == 'e')
    {
      operTime = millis();
      deterTrue_E3 = true;
      
      targetPos_3 = Serial.parseFloat();

      stepperDelay_3 = 100/abs(targetPos_3 - pretargetPos_3);
    }

    if (receivedCommand == 'r')
    {
      operTime = millis();
      deterTrue_E4 = true;

      targetPos_4 = Serial.parseFloat();

      stepperDelay_4 = 100/abs(targetPos_4 - pretargetPos_4);
    }

    if (receivedCommand == 'x')
    {
      operTime = millis();
      deterTrue_L = true;
    }

    if (receivedCommand == 'n')
    {
      Serial.println("STOP Motor !!");
      deterTrue = false;
      deterTrue_L = false;
      
      deterTrue_E1 = false;
      deterTrue_E2 = false;
      deterTrue_E3 = false;
      deterTrue_E4 = false;
    }

    if (receivedCommand == 'p')
    {
      printCurrent();
    }

    if (receivedCommand == 'y')
    {
      resetPin = true;
    }

    if (receivedCommand == 't')
    {
      testvoid = true;
    }

    ///////////////
    // TEST code //
    ///////////////

    if (receivedCommand == 'a')
    {
      a_ = Serial.parseFloat();
      currentPos_1 = a_;
//      Serial.print("a: ");
//      Serial.println(a_);
    }
    if (receivedCommand == 's')
    {
      s_ = Serial.parseFloat();
      currentPos_2 = s_;
//      Serial.print("s: ");
//      Serial.println(s_);
    }
    if (receivedCommand == 'd')
    {
      d_ = Serial.parseFloat();
      currentPos_3 = d_;
//      Serial.print("d: ");
//      Serial.println(d_);
    }
    if (receivedCommand == 'f')
    {
      f_ = Serial.parseFloat();
      currentPos_4 = f_;
//      Serial.print("f: ");
//      Serial.println(f_);
    }
  }
  newData = false;
}



// 't'
void testVoid()
{
  if (testvoid == true)
  {
    float test__;
    float a__ = -100;
    float b__ = -26;
    time_test = millis();
    Serial.println(time_test-operTime);
    Serial.println(micros());
    Serial.println(millis());
    Serial.println(millis()/50);
    Serial.println(millis()/100);

    Serial.println(sizeof(C)/2);
    
    Serial.print("deter delay: ");
    test__ = abs(a__+b__);
    Serial.println(test__, 4);
    testvoid = false;
  }
}



//targetPos_C = C[time_millis/100]*4;



void timeCycle()
{
  time_millis = millis();
  
  if(time_millis%100 == 0)
  {
    time_count++;
    //Serial.println(time_millis);
  }
  testvoid = false;
}



void timeStepCount()
{
  time_millis = millis();
  if (stepInterval > time_millis - lastStepTime)
  {
    time_count++;
    lastStepTime = time_millis;
  }
}



// 'z' //
void operMotor()
{
  if (deterTrue == true)
  {
    time_millis = millis();
    //Serial.println(targetPos - currentPos);
    if (targetPos - currentPos > 0 & targetPos != currentPos)
    {
      stepDriver1();
      stepDriver2();
      stepDriver3();
      stepDriver4();
//      stepMotor1 (x % 4);
//      stepMotor2 (x % 4);
//      stepMotor3 (x % 4);
//      stepMotor4 (x % 4);
      delayMicroseconds(stepperDelay);
      x++;
      currentPos++;
      currentPos_1++;
    }

    else if (targetPos - currentPos < 0 & targetPos != currentPos)
    {
      stepDriver1();
      stepDriver2();
      stepDriver3();
      stepDriver4();
//      stepMotor1 (x % 4);
//      stepMotor2 (x % 4);
//      stepMotor3 (x % 4);
//      stepMotor4 (x % 4);
      delayMicroseconds(stepperDelay);
      x--;
      currentPos--;
      currentPos_1--;
    }

    else if (targetPos_C = currentPos)
    {
      Serial.print("Operation time: ");
      Serial.println(millis() - operTime);
      Serial.print("Encoder Pos if:");
      Serial.println(encoder2_Pos);
      deterTrue = false;
    }

    else
    {
      Serial.print("Operation time: ");
      Serial.println(millis() - operTime);
      Serial.print("Encoder Pos:");
      Serial.println(encoder2_Pos);
      deterTrue = false;
    }
  }
}



void operMotorList()
{
  if (deterTrue_L == true)
  {
    time_current = millis();
    time_millis = time_current - operTime;
    targetPos_C = C[time_millis/100]*30;
    maxList = sizeof(C)/2;

    //Serial.println(targetPos_C);

    if (targetPos_C - currentPos > 0)
    {
      stepMotor1 (x % 4);
      stepMotor2 (x % 4);
      stepMotor3 (x % 4);
      stepMotor4 (x % 4);
      delayMicroseconds(stepperDelay);
      x++;
      currentPos++;
      currentPos_1++;

      size_Count++;
      Serial.println(targetPos_C);
    }

    else if (targetPos_C - currentPos < 0)
    {
      stepMotor1 (x % 4);
      stepMotor2 (x % 4);
      stepMotor3 (x % 4);
      stepMotor4 (x % 4);
      delayMicroseconds(stepperDelay);
      x--;
      currentPos--;
      currentPos_1--;
      
      size_Count++;
      Serial.println(targetPos_C);
    }
    
    else if (size_Count = maxList)
    {
      
      //deterTrue_L = false;
    }
    
    else
    {
      //deterTrue_L = false;
    }
  }
}

/////////
// 'q' //
/////////

// q100 200 300 400


void operMotor_E()
{
  if (deterTrue_E == true)
  {
    time_millis = millis();
    //Serial.println(time_millis - operTime);
    //Serial.println(targetPos - currentPos);
    Serial.println(targetPos_1);
    Serial.println(currentPos_1);
    Serial.println(axis_1);
    
    if (time_millis - operTime < 1000)
    {
      if (targetPos_1 - currentPos_1 > 0 & targetPos_1 != currentPos_1)
      {
        stepMotor1 (axis_1 % 4);
        axis_1++;
        currentPos_1++;
      }
      else if (targetPos_1 - currentPos_1 < 0 & targetPos_1 != currentPos_1)
      {
        stepMotor1 (axis_1 % 4);
        axis_1--;
        currentPos_1--;
      }
      
//      if (targetPos_2 - currentPos_2 > 0 & targetPos_2 != currentPos_2)
//      {
//        stepMotor2 (axis_2 % 4);
//        axis_2++;
//        currentPos_2++;
//      }
//      else if (targetPos_2 - currentPos_2 < 0 & targetPos_2 != currentPos_2)
//      {
//        stepMotor2 (axis_2 % 4);
//        axis_2--;
//        currentPos_2--;
//      }
//      
//      if (targetPos_3 - currentPos_3 > 0 & targetPos_3 != currentPos_3)
//      {
//        stepMotor3 (axis_3 % 4);
//        axis_3++;
//        currentPos_3++;
//      }
//      else if (targetPos_3 - currentPos_3 < 0 & targetPos_3 != currentPos_3)
//      {
//        stepMotor3 (axis_3 % 4);
//        axis_3--;
//        currentPos_3--;
//      }
//      
//      if (targetPos_4 - currentPos_4 > 0 & targetPos_4 != currentPos_4)
//      {
//        stepMotor4 (axis_4 % 4);
//        axis_4++;
//        currentPos_4++;
//      }
//      else if (targetPos_4 - currentPos_4 < 0 & targetPos_4 != currentPos_4)
//      {
//        stepMotor4 (axis_4 % 4);
//        axis_4--;
//        currentPos_4--;
//      }

      delayMicroseconds(stepperDelay);
    }

//    else if (targetPos_C = currentPos)
//    {
//      Serial.print("Operation time: ");
//      Serial.println(millis() - operTime);
//      Serial.print("Encoder Pos if:");
//      Serial.println(encoder2_Pos);
//      deterTrue = false;
//    }
    
    else
    {
      Serial.print("Operation time: ");
      Serial.println(millis() - operTime);
      Serial.print("Encoder Pos:");
      Serial.println(encoder2_Pos);
      deterTrue_E = false;
    }
  }
}



void operMotor_E1()
{
  if (deterTrue_E1 == true)
  {
    time_millis = millis();
//    Serial.println(time_millis);
//    Serial.println(operTime);
//    Serial.println(targetPos - currentPos);
//    Serial.println(targetPos_1);
//    Serial.println(currentPos_1);
//    Serial.println(axis_1);

    if (time_millis - operTime < timeGap) // timeGap : time-axis
    {
      if (targetPos_1 - currentPos_1 > 0 & targetPos_1 != currentPos_1)
      {
//        Serial.println("flag2");
        stepMotor1 (currentPos_1 % 4);
//        stepDriver1();
        currentPos_1++;
      }
      else if (targetPos_1 - currentPos_1 < 0 & targetPos_1 != currentPos_1)
      {
//        Serial.println("flag3");
        stepMotor1 (currentPos_1 % 4);
//        stepDriver1();
        currentPos_1--;
      }

      delayMicroseconds(stepperDelay_1*1000); //stepperDelay: 800, stepperDelay_1*1000
    }
    else
    {
//      Serial.print("Operation time: ");
//      Serial.println(millis() - operTime);
//      Serial.print("Encoder Pos:");
//      Serial.println(encoder2_Pos);
//      Serial.println(targetPos_1);
//      Serial.println(currentPos_1);
//      Serial.println(stepperDelay_1*1000);
      pretargetPos_1 = currentPos_1;
      deterTrue_E1 = false;
    }
  }
}



void operMotor_E2()
{
  if (deterTrue_E2 == true)
  {
    time_millis = millis();
    
    if (time_millis - operTime < timeGap)
    {
      if (targetPos_2 - currentPos_2 > 0 & targetPos_2 != currentPos_2)
      {
        stepMotor2 (currentPos_2 % 4);
//        stepDriver2();
        currentPos_2++;
      }
      else if (targetPos_2 - currentPos_2 < 0 & targetPos_2 != currentPos_2)
      {
        stepMotor2 (currentPos_2 % 4);
//        stepDriver2();
        currentPos_2--;
      }

      delayMicroseconds(stepperDelay_2*1000);
    }
    
    else
    {
//      Serial.print("Operation time: ");
//      Serial.println(millis() - operTime);
//      Serial.print("Encoder Pos:");
//      Serial.println(encoder2_Pos);
      pretargetPos_2 = currentPos_2;
      deterTrue_E2 = false;
    }
  }
}


void operMotor_E3()
{
  if (deterTrue_E3 == true)
  {
    time_millis = millis();
    //Serial.println(time_millis - operTime);
    //Serial.println(targetPos - currentPos);
    
    if (time_millis - operTime < timeGap) //timeGap
    {
      if (targetPos_3 - currentPos_3 > 0 & targetPos_3 != currentPos_3)
      {
//        stepDriver3();
        stepMotor3 (currentPos_3 % 4);
        currentPos_3++;
      }
      else if (targetPos_3 - currentPos_3 < 0 & targetPos_3 != currentPos_3)
      {
//        stepDriver3();
        stepMotor3 (currentPos_3 % 4);
        currentPos_3--;
      }

      delayMicroseconds(stepperDelay_3*1000); //stepperDelay_3*1000
    }

    else
    {
//      Serial.print("Operation time: ");
//      Serial.println(millis() - operTime);
//      Serial.print("Encoder Pos:");
//      Serial.println(encoder2_Pos);
//      Serial.println(stepperDelay_3*1000);
      Serial.println(targetPos_3);
      Serial.println(currentPos_3);
      pretargetPos_3 = currentPos_3;
      deterTrue_E3 = false;
    }
  }
}


void operMotor_E4()
{
  if (deterTrue_E4 == true)
  {
    time_millis = millis();
    //Serial.println(time_millis - operTime);
    //Serial.println(targetPos - currentPos);
    
    if (time_millis - operTime < timeGap)
    {

      if (targetPos_4 - currentPos_4 > 0 & targetPos_4 != currentPos_4)
      {
//        stepDriver4();
        stepMotor4 (currentPos_4 % 4);
        currentPos_4++;
      }
      else if (targetPos_4 - currentPos_4 < 0 & targetPos_4 != currentPos_4)
      {
//        stepDriver4();
        stepMotor4 (currentPos_4 % 4);
        currentPos_4--;
      }

      delayMicroseconds(stepperDelay_4*1000);
    }

    else
    {
//      Serial.print("Operation time: ");
//      Serial.println(millis() - operTime);
//      Serial.print("Encoder Pos:");
//      Serial.println(encoder2_Pos);
      pretargetPos_4 = currentPos_4;
      deterTrue_E4 = false;
    }
  }
}



//Serial.println(sizeof(C)/2);


void stepMotor1(int thisStep) {
  switch (thisStep) {
    case 0:  // 1010
      digitalWrite(motor_pin_1, HIGH);
      digitalWrite(motor_pin_2, LOW);
      digitalWrite(motor_pin_3, HIGH);
      digitalWrite(motor_pin_4, LOW);
      break;
    case 1:  // 0110
      digitalWrite(motor_pin_1, LOW);
      digitalWrite(motor_pin_2, HIGH);
      digitalWrite(motor_pin_3, HIGH);
      digitalWrite(motor_pin_4, LOW);
      break;
    case 2:  //0101
      digitalWrite(motor_pin_1, LOW);
      digitalWrite(motor_pin_2, HIGH);
      digitalWrite(motor_pin_3, LOW);
      digitalWrite(motor_pin_4, HIGH);
      break;
    case 3:  //1001
      digitalWrite(motor_pin_1, HIGH);
      digitalWrite(motor_pin_2, LOW);
      digitalWrite(motor_pin_3, LOW);
      digitalWrite(motor_pin_4, HIGH);
      break;
  }
}



void stepMotor2(int thisStep) {
  switch (thisStep) {
    case 0:  // 1010
      digitalWrite(motor_pin_5, HIGH);
      digitalWrite(motor_pin_6, LOW);
      digitalWrite(motor_pin_7, HIGH);
      digitalWrite(motor_pin_8, LOW);
      break;
    case 1:  // 0110
      digitalWrite(motor_pin_5, LOW);
      digitalWrite(motor_pin_6, HIGH);
      digitalWrite(motor_pin_7, HIGH);
      digitalWrite(motor_pin_8, LOW);
      break;
    case 2:  //0101
      digitalWrite(motor_pin_5, LOW);
      digitalWrite(motor_pin_6, HIGH);
      digitalWrite(motor_pin_7, LOW);
      digitalWrite(motor_pin_8, HIGH);
      break;
    case 3:  //1001
      digitalWrite(motor_pin_5, HIGH);
      digitalWrite(motor_pin_6, LOW);
      digitalWrite(motor_pin_7, LOW);
      digitalWrite(motor_pin_8, HIGH);
      break;
  }
}

void stepMotor3(int thisStep) {
  switch (thisStep) {
    case 0:  // 1010
      digitalWrite(motor_pin_9, HIGH);
      digitalWrite(motor_pin_10, LOW);
      digitalWrite(motor_pin_11, HIGH);
      digitalWrite(motor_pin_12, LOW);
      break;
    case 1:  // 0110
      digitalWrite(motor_pin_9, LOW);
      digitalWrite(motor_pin_10, HIGH);
      digitalWrite(motor_pin_11, HIGH);
      digitalWrite(motor_pin_12, LOW);
      break;
    case 2:  //0101
      digitalWrite(motor_pin_9, LOW);
      digitalWrite(motor_pin_10, HIGH);
      digitalWrite(motor_pin_11, LOW);
      digitalWrite(motor_pin_12, HIGH);
      break;
    case 3:  //1001
      digitalWrite(motor_pin_9, HIGH);
      digitalWrite(motor_pin_10, LOW);
      digitalWrite(motor_pin_11, LOW);
      digitalWrite(motor_pin_12, HIGH);
      break;
  }
}

void stepMotor4(int thisStep) {
  switch (thisStep) {
    case 0:  // 1010
      digitalWrite(motor_pin_13, HIGH);
      digitalWrite(motor_pin_14, LOW);
      digitalWrite(motor_pin_15, HIGH);
      digitalWrite(motor_pin_16, LOW);
      break;
    case 1:  // 0110
      digitalWrite(motor_pin_13, LOW);
      digitalWrite(motor_pin_14, HIGH);
      digitalWrite(motor_pin_15, HIGH);
      digitalWrite(motor_pin_16, LOW);
      break;
    case 2:  //0101
      digitalWrite(motor_pin_13, LOW);
      digitalWrite(motor_pin_14, HIGH);
      digitalWrite(motor_pin_15, LOW);
      digitalWrite(motor_pin_16, HIGH);
      break;
    case 3:  //1001
      digitalWrite(motor_pin_13, HIGH);
      digitalWrite(motor_pin_14, LOW);
      digitalWrite(motor_pin_15, LOW);
      digitalWrite(motor_pin_16, HIGH);
      break;
  }
}


void stepMotor5(int thisStep) {
  switch (thisStep) {

    case 0: // 01 //
      digitalWrite(CW_x, LOW);
      digitalWrite(CCW_x, HIGH);
      break;
    case 1: // 11 //
      digitalWrite(CW_x, HIGH);
      digitalWrite(CCW_x, HIGH);
      break;
    case 2: // 10 //
      digitalWrite(CW_x, HIGH);
      digitalWrite(CCW_x, LOW);
      break;
    case 3: // 00 //
      digitalWrite(CW_x, LOW);
      digitalWrite(CCW_x, LOW);
      break;
  }
}

void stepMotor6(int thisStep) {
  switch (thisStep) {

    case 0: // 01 //
      digitalWrite(CW_y, LOW);
      digitalWrite(CCW_y, HIGH);
      break;
    case 1: // 11 //
      digitalWrite(CW_y, HIGH);
      digitalWrite(CCW_y, HIGH);
      break;
    case 2: // 10 //
      digitalWrite(CW_y, HIGH);
      digitalWrite(CCW_y, LOW);
      break;
    case 3: // 00 //
      digitalWrite(CW_y, LOW);
      digitalWrite(CCW_y, LOW);
      break;
  }
}


void stepMotor7(int thisStep) {
  switch (thisStep) {

    case 0: // 01 //
      digitalWrite(CW_z, LOW);
      digitalWrite(CCW_z, HIGH);
      break;
    case 1: // 11 //
      digitalWrite(CW_z, HIGH);
      digitalWrite(CCW_z, HIGH);
      break;
    case 2: // 10 //
      digitalWrite(CW_z, HIGH);
      digitalWrite(CCW_z, LOW);
      break;
    case 3: // 00 //
      digitalWrite(CW_z, LOW);
      digitalWrite(CCW_z, LOW);
      break;
  }
}


void stepMotor8(int thisStep) {
  switch (thisStep) {

    case 0: // 01 //
      digitalWrite(CW_Z, LOW);
      digitalWrite(CCW_Z, HIGH);
      break;
    case 1: // 11 //
      digitalWrite(CW_Z, HIGH);
      digitalWrite(CCW_Z, HIGH);
      break;
    case 2: // 10 //
      digitalWrite(CW_Z, HIGH);
      digitalWrite(CCW_Z, LOW);
      break;
    case 3: // 00 //
      digitalWrite(CW_Z, LOW);
      digitalWrite(CCW_Z, LOW);
      break;
  }
}


void stepDriver1() // pin[0] is step, pin[1] is direction
{
    digitalWrite(CCW_x, targetPos_1 - currentPos_1 > 0 ? HIGH : LOW); // Direction ? 2 : 0
    // Caution 200ns setup time 
    digitalWrite(CW_x, HIGH);
    // Caution, min Step pulse width for 3967 is 1microsec
    // Delay 1microsec
    delayMicroseconds(1);
    digitalWrite(CW_x, LOW);
}

void stepDriver2() // pin[0] is step, pin[1] is direction
{
    digitalWrite(CCW_y, targetPos_2 - currentPos_2 > 0 ? HIGH : LOW); // Direction ? 2 : 0
    // Caution 200ns setup time 
    digitalWrite(CW_y, HIGH);
    // Caution, min Step pulse width for 3967 is 1microsec
    // Delay 1microsec
    delayMicroseconds(1);
    digitalWrite(CW_y, LOW);
}

void stepDriver3() // pin[0] is step, pin[1] is direction
{
    digitalWrite(CCW_z, targetPos_3 - currentPos_3 < 0 ? HIGH : LOW); // Direction ? 2 : 0
    // Caution 200ns setup time 
    digitalWrite(CW_z, HIGH);
    // Caution, min Step pulse width for 3967 is 1microsec
    // Delay 1microsec
    delayMicroseconds(1);
    digitalWrite(CW_z, LOW);
}

void stepDriver4() // pin[0] is step, pin[1] is direction
{
    digitalWrite(CCW_Z, targetPos_4 - currentPos_4 > 0 ? HIGH : LOW); // Direction ? 2 : 0
    // Caution 200ns setup time 
    digitalWrite(CW_Z, HIGH);
    // Caution, min Step pulse width for 3967 is 1microsec
    // Delay 1microsec
    delayMicroseconds(1);
    digitalWrite(CW_Z, LOW);
}



/*

void Encoder_2_CW(){
  int b = digitalRead(Encoder2_PinB);
//  Serial.print("encoder value B: ");
//  Serial.println(digitalRead(Encoder_output_B));
  if(b > 0){
    encoder2_Pos++;
  }
//  Serial.println(Count_pulses);
//  else{
//    Count_pulses--;
//  }
}


void Encoder_2_CCW(){
  int a = digitalRead(Encoder2_PinA);
//  Serial.print("encoder value A: ");
//  Serial.println(digitalRead(Encoder_output_A));
  if(a > 0){
    encoder2_Pos--;
  }
//  Serial.println(Count_pulses);
//  else{
//    Count_pulses--;

//  }
}

*/



void printCurrent()
{
  Serial.println("---------Printing current stepper motor postion---------");
  Serial.print("motor1(target, current): ");
  Serial.print(targetPos_1);
  Serial.print(", ");
  Serial.println(currentPos_1);
  
  Serial.print("motor2(target, current): ");
  Serial.print(targetPos_2);
  Serial.print(", ");
  Serial.println(currentPos_2);
  
  Serial.print("motor3(target, current): ");
  Serial.print(targetPos_3);
  Serial.print(", ");
  Serial.println(currentPos_3);
  
  Serial.print("motor4(target, current): ");
  Serial.print(targetPos_4);
  Serial.print(", ");
  Serial.println(currentPos_4);
}



///               ///
/// Reset arduino ///
///               ///



void(* resetFunc) (void) = 0;


void resetArduino()
{
  if (resetPin == true)
  {
    resetFunc();
    Serial.println("Reseting arduino");
    resetPin = false;
  }
}


void EMERGENCY_all()
{
  deterTrue_E1 = false;
  deterTrue_E2 = false;
  deterTrue_E3 = false;
  deterTrue_E4 = false;
}

