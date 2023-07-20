
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

SMRE stepper1(500, motor_pin_1, motor_pin_2, motor_pin_3, motor_pin_4);
SMRE stepper2(500, motor_pin_5, motor_pin_6, motor_pin_7, motor_pin_8);
SMRE stepper3(500, motor_pin_9, motor_pin_10, motor_pin_11, motor_pin_12);
SMRE stepper4(500, motor_pin_13, motor_pin_14, motor_pin_15, motor_pin_16);



unsigned long time_a, time_b;
unsigned long total_time_a, total_time_b;

int step_c;

int stepCount1 = 0;
int stepCount2 = 0;

boolean stepper1_oper = false;

int A[2][4] = {{100, -100, 100, -100}, {1000, 1000, 1000, 1000}};

int B[2][4] = {{100, -100, 100, -100}, {1000, 1000, 1000, 1000}};



void setup() {
  Serial.begin(115200);
  Serial.println("|||| Start stepper-motor ||||");

  pinMode(Encoder_output_A,INPUT); // sets the Encoder_output_A pin as the input INPUT_PULLUP
  pinMode(Encoder_output_B,INPUT); // sets the Encoder_output_B pin as the input INPUT_PULLUP

  attachInterrupt(digitalPinToInterrupt(Encoder_output_A),Encoder_CW,RISING);
  attachInterrupt(digitalPinToInterrupt(Encoder_output_B),Encoder_CCW,RISING);

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
int moveArrange = 1000;
int stepperDelay = 2000;


void loop() {
  if (moveArrange != 0) {
    stepMotor1(x % 4);
    stepMotor2(x % 4);
    delayMicroseconds(stepperDelay);
    x++;
    moveArrange--;
  }
  else if (moveArrange == 0) {
    x = 0;
  }
  if (moveArrange == 1) {
    time_b = millis();
    Serial.print("time: ");
    Serial.println(time_b - time_a);
  }
  //  stepper1.operStep(2000);

  //  Serial.print("arrange: ");
  //  Serial.println(moveArrange);
  //  Serial.print("x: ");
  //  Serial.println(x);
}



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

