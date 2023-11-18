
//#include <Stepper.h>
//#include <AccelStepper.h>
#include <SMRE.h>

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

// const int stepsPerRevolution = 500; // change this to fit the number of steps per revolution

// initialize the Stepper library on pins 8 through 11:
// Stepper myStepper(stepsPerRevolution, 8, 9, 10, 11);

SMRE stepper1(4, motor_pin_1, motor_pin_2, motor_pin_3, motor_pin_4);
SMRE stepper2(4, motor_pin_5, motor_pin_6, motor_pin_7, motor_pin_8);
SMRE stepper3(4, motor_pin_9, motor_pin_10, motor_pin_11, motor_pin_12);
SMRE stepper4(4, motor_pin_13, motor_pin_14, motor_pin_15, motor_pin_16);

char receivedCommand;

boolean newData = false;
boolean deterTrue = false;
boolean deterTrue_L = false;

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

int C[] = {42, 48, 50, 54, 60, 64, 68, 72, 78, 82, 86, 92, 94, 98, 102, 108, 114, 116, 124, 128, 132, 136, 138, 142, 144, 148, 150, 152, 154, 154, 156, 156, 158, 164, 166, 168, 168, 164, 160, 156, 154, 150, 146, 140, 136, 130, 124, 114, 108, 100, 94, 88, 84, 76, 68, 62, 54, 48, 42, 36, 30, 24, 18, 12, 6, 2, -2, -8, -12, -8, 0, 6, 16, 26, 30, 36, 42, 48, 50, 54, 60, 64, 68, 72, 78, 82, 86, 92, 94, 98, 102, 108, 114, 116, 124, 128, 132, 136, 138, 142, 144, 148, 150, 152, 154, 154, 156, 156, 158, 164, 166, 168, 168, 164, 160, 156, 154, 150, 146, 140, 136, 130, 124, 114, 108, 100, 94, 88, 84, 76, 68, 62, 54, 48, 42, 36, 30, 24, 18, 12, 6, 2, -2, -8, -12, -8, 0, 6, 16, 26, 30, 36, 42, 48, 50, 54, 60, 64, 68, 72, 78, 82, 86, 92, 94, 98, 102, 108, 114, 116, 124, 128, 132, 136, 138, 142, 144, 148, 150, 152, 154, 154, 156, 156, 158, 164, 166, 168, 168, 164, 160, 156, 154, 150, 146, 140, 136, 130, 124, 114, 108, 100, 94, 88, 84, 76, 68, 62, 54, 48, 42, 36, 30, 24, 18, 12, 6, 2, -2, -8, -12, -8, 0, 6, 16, 26, 30, 36, 42, 48, 50, 54, 60, 64, 68, 72, 78, 82, 86, 92, 94, 98, 102, 108, 114, 116, 124, 128, 132, 136, 138, 142, 144, 148, 150, 152, 154, 154, 156, 156, 158, 164, 166, 168, 168, 164, 160, 156, 154, 150, 146, 140, 136, 130, 124, 114, 108, 100, 94, 88, 84, 76, 68, 62, 54, 48, 42, 36, 30, 24, 18, 12, 6, 2, -2, -8, -12, -8, 0, 6, 16, 26, 30, 36, 42, 48, 50, 54, 60, 64, 68, 72, 78, 82, 86, 92, 94, 98, 102, 108, 114, 116, 124, 128, 132, 136, 138, 142, 144, 148, 150, 152, 154, 154, 156, 156, 158, 164, 166, 168, 168, 164, 160, 156, 154, 150, 146, 140, 136, 130, 124, 114, 108, 100, 94, 88, 84, 76, 68, 62, 54, 48, 42, 36, 30, 24, 18, 12, 6, 2, -2, -8, -12, -8, 0, 6, 16, 26, 30, 36, 42, 48, 50, 54, 60, 64, 68, 72, 78, 82, 86, 92, 94, 98, 102, 108, 114, 116, 124, 128, 132, 136, 138, 142, 144, 148, 150, 152, 154, 154, 156, 156, 158, 164, 166, 168, 168, 164, 160, 156, 154, 150, 146, 140, 136, 130, 124, 114, 108, 100, 94, 88, 84, 76, 68, 62, 54, 48, 42, 36, 30, 24, 18, 12, 6, 2, -2, -8, -12, -8, 0, 6, 16, 26, 30, 36, 42, 48, 50, 54, 60, 64, 68, 72, 78, 82, 86, 92, 94, 98, 102, 108, 114, 116, 124, 128, 132, 136, 138, 142, 144, 148, 150, 152, 154, 154, 156, 156, 158, 164, 166, 168, 168, 164, 160, 156, 154, 150, 146, 140, 136, 130, 124, 114, 108, 100, 94, 88, 84, 76, 68, 62, 54, 48, 42, 36, 30, 24, 18, 12, 6, 2, -2, -8, -12, -8, 0, 6, 16, 26, 30, 36, 42, 48, 50, 54, 60, 64, 68, 72, 78, 82, 86, 92, 94, 98, 102, 108, 114, 116, 124, 128, 132, 136, 138, 142, 144, 148, 150, 152, 154, 154, 156, 156, 158, 164, 166, 168, 168, 164, 160, 156, 154, 150, 146, 140, 136, 130, 124, 114, 108, 100, 94, 88, 84, 76, 68, 62, 54, 48, 42, 36, 30, 24, 18, 12, 6, 2, -2, -8, -12, -8, 0, 6, 16, 26, 30, 36, 42, 48, 50, 54, 60, 64, 68, 72, 78, 82, 86, 92, 94, 98, 102, 108, 114, 116, 124, 128, 132, 136, 138, 142, 144, 148, 150, 152, 154, 154, 156, 156, 158, 164, 166, 168, 168, 164, 160, 156, 154, 150, 146, 140, 136, 130, 124, 114, 108, 100, 94, 88, 84, 76, 68, 62, 54, 48, 42, 36, 30, 24, 18, 12, 6, 2, -2, -8, -12, -8, 0, 6, 16, 26, 30, 36};

int targetPos = 0;
int targetPos_C = 0;
int currentPos = 0;

int time_count = 0;
int time_millis = 0;
int lastStepTime = 0;
int stepInterval = 100;

int size_Count = 0;
int maxList = 0;



void setup() {
  Serial.begin(115200);
  Serial.println("|||| Start stepper-motor ||||");

  pinMode(Encoder_output_A,INPUT); // sets the Encoder_output_A pin as the input INPUT_PULLUP
  pinMode(Encoder_output_B,INPUT); // sets the Encoder_output_B pin as the input INPUT_PULLUP

//  attachInterrupt(digitalPinToInterrupt(Encoder_output_A),Encoder_CW,RISING);
//  attachInterrupt(digitalPinToInterrupt(Encoder_output_B),Encoder_CCW,RISING);

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
int moveArrange = 500;
int stepperDelay = 3000;



void loop() {
  operMotor();

  operMotorList();
  
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
      deterTrue = true;

      targetPos = Serial.parseFloat();
      //received_0 = Serial.parseFloat();

      Serial.print("target position:");
      Serial.println(targetPos);
    }

    if (receivedCommand == 'x')
    {
      deterTrue_L = true;
    }

    if (receivedCommand == 'n')
    {
      Serial.println("STOP Motor !!");
      deterTrue = false;
      deterTrue_L = false;
    }

    if (receivedCommand == 'p')
    {
      printCurrent();
    }

    if (receivedCommand == 'r')
    {
      resetPin = true;
    }

    if (receivedCommand == 't')
    {
      testvoid = true;
    }
  }
  newData = false;
} 



// 't'
void testVoid()
{
  if (testvoid == true)
  {
    Serial.println(micros());
    Serial.println(millis());
    Serial.println(millis()/50);
    Serial.println(millis()/100);
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



void operMotor()
{
  if (deterTrue == true)
  {
    time_millis = millis();
    //Serial.println(targetPos - currentPos);
    if (targetPos - currentPos > 0 & targetPos != currentPos)
    {
      stepMotor1 (x % 4);
      delayMicroseconds(stepperDelay);
      x++;
      currentPos++;
    }

    else if (targetPos - currentPos < 0 & targetPos != currentPos)
    {
      stepMotor1 (x % 4);
      delayMicroseconds(stepperDelay);
      x--;
      currentPos--;
    }

    else
    {
      deterTrue = false;
    }
  }
}



void operMotorList()
{
  if (deterTrue_L == true)
  {
    time_millis = millis();
    targetPos_C = C[time_millis/100]*4;
    maxList = sizeof(C)/2;

    //Serial.println(targetPos_C);

    if (targetPos_C - currentPos > 0)
    {
      stepMotor1 (x % 4);
      delayMicroseconds(stepperDelay);
      x++;
      currentPos++;

      size_Count++;
      Serial.println(targetPos_C);
    }

    else if (targetPos_C - currentPos < 0)
    {
      stepMotor1 (x % 4);
      delayMicroseconds(stepperDelay);
      x--;
      currentPos--;
      
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



/*
void Encoder_CW(){
  int b = digitalRead(Encoder_output_B);
//  Serial.print("encoder value B: ");
//  Serial.println(digitalRead(Encoder_output_B));
  if(b > 0){
    Count_pulses++;
  }
//  Serial.println(Count_pulses);
//  else{
//    Count_pulses--;
//  }
}


void Encoder_CCW(){
  int a = digitalRead(Encoder_output_A);
//  Serial.print("encoder value A: ");
//  Serial.println(digitalRead(Encoder_output_A));
  if(a > 0){
    Count_pulses--;
  }
//  Serial.println(Count_pulses);
//  else{
//    Count_pulses--;

//  }
}


/////////// code2 ///////////

void Encoder_CW2() {
  if(digitalRead(Encoder_output_B==LOW)) {
    Count_pulses++;
  }else{
    Count_pulses--;
  }
}

void Encoder_CCW2() {
  if(digitalRead(Encoder_output_A==HIGH)) {
    Count_pulses--;
  }else{
    Count_pulses++;
  }
}

*/

void printCurrent()
{
  Serial.println("---------Printing current stepper motor postion---------");
  Serial.print("motor0(target, current): ");
  Serial.print(targetPos_C);
  Serial.print(", ");
  Serial.println(currentPos);
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


