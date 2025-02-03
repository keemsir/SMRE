
// 20230410 update

// example: constant code b500 500 500 500 400
// serial: b500 500 500 500 500
// a single control: z500 500, x500 500, c500 500, v500 500
// 왜 motor 2에 LED가 전부 안들어오는지 ?

#include <AccelStepper.h>
#include <MultiStepper.h>


//#include <RotaryEncoder.h>

// -
#define CW_x    22
#define CCW_x   24
// +
#define CW_y    30
#define CCW_y   32
// -
#define CW_z    38
#define CCW_z   40
// +
#define CW_Z    46
#define CCW_Z   48


/////////////////// for Rotary Encoder //////////////////
/// | int.0 | int.1 | int.2 | int.3 | int.4 | int.5 | ///
///                                                   ///
/// |   2   |   3   |  21   |  20   |  19   |  18   | ///
/////////////////////////////////////////////////////////


int A[3][84] = {{-81, 82, -87, 74, -105, 82, -79, 80, -111, 80, -67, 76, -103, 72, -103, 78, -81, 92, -107, 100, -115, 74, -111, 88, -103, 82, -105, 84, -105, 84, -99, 86, -63, 74, -107, 70, -107, 128, -97, 102, -107, 108, -87, 110, -75, 106, -85, 104, -115, 118, -101, 104, -89, 98, -103, 86, -119, 104, -113, 90, -105, 88, -121, 112, -89, 94, -113, 96, -93, 94, -135, 96, -41, 70, -87, 92, -115, 92, -135, 110, -97, 96, -119, 16},
{650, 2750, 1850, 2450, 1550, 2850, 1500, 2050, 1800, 2800, 1200, 2000, 1800, 2450, 2150, 2400, 1450, 4000, 1200, 2600, 4700, 2750, 1400, 2450, 1600, 2200, 1700, 2100, 1500, 2600, 2150, 2900, 1800, 1950, 1900, 2100, 2150, 2700, 2400, 2200, 1700, 2050, 2050, 2800, 1000, 1750, 1250, 2900, 1700, 2550, 1900, 2150, 2550, 1900, 1900, 2400, 2100, 3150, 2100, 2300, 2200, 2850, 1450, 3750, 1100, 2500, 2100, 2650, 1250, 1900, 2150, 2650, 1100, 1350, 1050, 2400, 1700, 2100, 2050, 2950, 1950, 1800, 2050, 1150},
{1283, 292, 434, 329, 520, 281, 538, 392, 446, 286, 676, 402, 446, 329, 374, 336, 557, 200, 676, 309, 170, 292, 577, 329, 503, 366, 472, 383, 538, 309, 374, 276, 446, 412, 423, 383, 374, 297, 336, 366, 472, 392, 392, 286, 812, 459, 649, 276, 472, 316, 423, 374, 316, 423, 423, 336, 383, 254, 383, 350, 366, 281, 557, 214, 738, 322, 383, 303, 649, 423, 374, 303, 738, 600, 773, 336, 472, 383, 392, 271, 412, 446, 392, 706}};

int B[3][84] = {{-81, 82, -87, 74, -105, 82, -79, 80, -111, 80, -67, 76, -103, 72, -103, 78, -81, 92, -107, 100, -115, 74, -111, 88, -103, 82, -105, 84, -105, 84, -99, 86, -63, 74, -107, 70, -107, 128, -97, 102, -107, 108, -87, 110, -75, 106, -85, 104, -115, 118, -101, 104, -89, 98, -103, 86, -119, 104, -113, 90, -105, 88, -121, 112, -89, 94, -113, 96, -93, 94, -135, 96, -41, 70, -87, 92, -115, 92, -135, 110, -97, 96, -119, 16},
{650, 2750, 1850, 2450, 1550, 2850, 1500, 2050, 1800, 2800, 1200, 2000, 1800, 2450, 2150, 2400, 1450, 4000, 1200, 2600, 4700, 2750, 1400, 2450, 1600, 2200, 1700, 2100, 1500, 2600, 2150, 2900, 1800, 1950, 1900, 2100, 2150, 2700, 2400, 2200, 1700, 2050, 2050, 2800, 1000, 1750, 1250, 2900, 1700, 2550, 1900, 2150, 2550, 1900, 1900, 2400, 2100, 3150, 2100, 2300, 2200, 2850, 1450, 3750, 1100, 2500, 2100, 2650, 1250, 1900, 2150, 2650, 1100, 1350, 1050, 2400, 1700, 2100, 2050, 2950, 1950, 1800, 2050, 1150},
{1283, 292, 434, 329, 520, 281, 538, 392, 446, 286, 676, 402, 446, 329, 374, 336, 557, 200, 676, 309, 170, 292, 577, 329, 503, 366, 472, 383, 538, 309, 374, 276, 446, 412, 423, 383, 374, 297, 336, 366, 472, 392, 392, 286, 812, 459, 649, 276, 472, 316, 423, 374, 316, 423, 423, 336, 383, 254, 383, 350, 366, 281, 557, 214, 738, 322, 383, 303, 649, 423, 374, 303, 738, 600, 773, 336, 472, 383, 392, 271, 412, 446, 392, 706}};

float Speed_weight_x = 0.1;
float Speed_weight_y = 0.1;


#define encoderx_PinA 20
#define encoderx_PinB 21
#define encoderx_PinZ 4

#define encodery_PinA 18
#define encodery_PinB 19

#define encoderz_PinA 19
#define encoderz_PinB 18

#define encoderZ_PinA 2
#define encoderZ_PinB 3


volatile long encoderx_Pos = 0;
volatile boolean encoderx_Aset = false;
volatile boolean encoderx_Bset = false;

volatile long encodery_Pos = 0;
volatile boolean encodery_Aset = false;
volatile boolean encodery_Bset = false;

volatile long encoderz_Pos = 0;
volatile boolean encoderz_Aset = false;
volatile boolean encoderz_Bset = false;

volatile long encoderZ_Pos = 0;
volatile boolean encoderZ_Aset = false;
volatile boolean encoderZ_Bset = false;



////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////


//AccelStepper stepper_x(AccelStepper::FULL4WIRE,5,6,5,7);
//AccelStepper stepper_y(AccelStepper::FULL4WIRE,8,10,9,11);
//AccelStepper stepper_z(AccelStepper::FULL4WIRE,12,14,13,15);
//AccelStepper stepper_Z(AccelStepper::FULL4WIRE,16,18,17,19);



// for 4D phantom
AccelStepper stepper_x(AccelStepper::DRIVER, CW_x, CCW_x); // CW+: PLS(pulse), CCW+: DIR
AccelStepper stepper_y(AccelStepper::DRIVER, CW_y, CCW_y); // CW+: PLS(pulse), CCW+: DIR
AccelStepper stepper_z(AccelStepper::DRIVER, CW_z, CCW_z); // CW+: PLS(pulse), CCW+: DIR
AccelStepper stepper_Z(AccelStepper::DRIVER, CW_Z, CCW_Z); // CW+: PLS(pulse), CCW+: DIR



MultiStepper steppers;

unsigned long MotorSpeed;
unsigned long MotorSpeed_Acceleration;
unsigned long time_a, time_b;


////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////

// accelator parameter setting

long receivedDistance_x = 0;
long receivedDistance_y = 0;
long receivedDistance_z = 0;
long receivedDistance_Z = 0;

long receivedDelay_x = 0;
long receivedDelay_y = 0;
long receivedDelay_z = 0;
long receivedDelay_Z = 0;

long receivedDistance_0;
long receivedDistance_1;
long receivedDistance_2;
long receivedDistance_3;

char receivedCommand;
char movedAxis = '0';

unsigned long time_x, time_x1, time_y, time_y1, time_z, time_z1, time_Z, time_Z1;
unsigned long encoder_yPOS, encoder_yPOS1;


bool newData, runallowed_x, runallowed_y, runallowed_z, runallowed_Z = false; // booleans for new data from serial, and runallowed flag
bool run_multistepper = false;
bool RepeatOperation_x, RepeatOperation_y, RepeatOperation_z, RepeatOperation_Z = false;
bool Homing_x, Homing_y, Homing_z, Homing_Z = false;



////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////



void setup() {
//  Serial.begin(2000000);
  Serial.begin(115200);

  
  Serial.println("-----Starting Stepper Motor-----");
  Serial.println("input axis: setMaxSpeed, Move"); // !!! 수정필요
  
  //MotorSpeed = 500; // step per second
  MotorSpeed_Acceleration = 800;
  
  stepper_x.setMaxSpeed(MotorSpeed);
  stepper_y.setMaxSpeed(MotorSpeed);
  stepper_z.setMaxSpeed(MotorSpeed);
  stepper_Z.setMaxSpeed(MotorSpeed);

  steppers.addStepper(stepper_x);
  steppers.addStepper(stepper_y);
  steppers.addStepper(stepper_z);
  steppers.addStepper(stepper_Z);


////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////// for acceleration /////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////


  stepper_x.setAcceleration(MotorSpeed_Acceleration);
  stepper_y.setAcceleration(MotorSpeed_Acceleration);
  stepper_z.setAcceleration(MotorSpeed_Acceleration);
  stepper_Z.setAcceleration(MotorSpeed_Acceleration);
  
  stepper_x.disableOutputs();
  stepper_y.disableOutputs();
  stepper_z.disableOutputs();
  stepper_Z.disableOutputs();


////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// for encoder ///////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////

  pinMode(encoderx_PinA, INPUT);
  pinMode(encoderx_PinB, INPUT);
//  pinMode(encoderx_PinZ, INPUT_PULLUP);
  pinMode(encodery_PinA, INPUT);
  pinMode(encodery_PinB, INPUT);
  pinMode(encoderz_PinA, INPUT_PULLUP);
  pinMode(encoderz_PinB, INPUT_PULLUP);
  pinMode(encoderZ_PinA, INPUT_PULLUP);
  pinMode(encoderZ_PinB, INPUT_PULLUP);


//  attachInterrupt(digitalPinToInterrupt(encoderx_PinA), updateEncoderx, RISING); //encoderx_PinA
//  attachInterrupt(digitalPinToInterrupt(encoderx_PinB), updateEncoderx, RISING); //encoderx_PinB

//  attachInterrupt(digitalPinToInterrupt(encoderx_PinA), doEncoderA, RISING); //encoderx_PinA
//  attachInterrupt(digitalPinToInterrupt(encoderx_PinB), doEncoderB, RISING); //encoderx_PinB

//  attachInterrupt(digitalPinToInterrupt(encoderx_PinA), updateEncoderx, RISING); //encoderx_PinA
  
//  attachInterrupt(digitalPinToInterrupt(encodery_PinA), Encoder_y_CW, RISING); //encodery_PinA
//  attachInterrupt(digitalPinToInterrupt(encodery_PinB), Encoder_y_CCW, RISING); //encodery_PinB


  attachInterrupt(digitalPinToInterrupt(encoderz_PinA), Encoder_z_CW, RISING); //encodery_PinA
  attachInterrupt(digitalPinToInterrupt(encoderz_PinB), Encoder_z_CCW, RISING); //encodery_PinB
  
  attachInterrupt(digitalPinToInterrupt(encoderZ_PinA), Encoder_Z_CW, RISING); //encoderz_PinA
  attachInterrupt(digitalPinToInterrupt(encoderZ_PinB), Encoder_Z_CCW, RISING); //encoderZ_PinA

//  PastA = (boolean)digitalRead(encoder0_PinA);
//  PastB = (boolean)digitalRead(encoder0_PinB);

}



void loop() {

// Checking the serial
  checkSerial();

// Runing the stepper motor (x,y,z,Z axis)
  continuousRun_const();
  
  continuousRun_x();
  continuousRun_y();
  continuousRun_z();
  continuousRun_Z();

// Rotary encoder
//  if (digitalRead(Encoder_x1) == HIGH)
//  {
//    Serial.println("NC");
//  }
//
//  if (digitalRead(NO))
//  {
//    Serial.println(NO);
//  }
//  delay(1000);
  
  GoHome();
  
//  Serial.print("PinA: ");
//  Serial.println(digitalRead(encoderx_PinA));
//  Serial.print("PinB: ");
//  Serial.println(digitalRead(encoderx_PinA));

//  Serial.println(digitalRead(encoderx_PinA));
//  Serial.println(digitalRead(encoderx_PinB));
//  Serial.println(encoderx_Pos);
//  Serial.print("cnt: ");
//  Serial.println(cnt0);
//  Serial.print(": ");
//  Serial.println();
//  Serial.print("cnt: ");
//  Serial.println();
//  delay(100);

}



////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////// def setting //////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////



void checkSerial()
{
  //switch-case would also work, and maybe more elegant

  if (Serial.available() > 0)
  {
    receivedCommand = Serial.read();
    newData = true;
  }

  if (newData == true)
  {
    
    //////////////////////////////////////////////////
    ///////////////// constant speed /////////////////
    //////////////////////////////////////////////////
    
    if (receivedCommand == 'b') /// constant speed ///
    {
      run_multistepper = true;

      receivedDistance_0 = Serial.parseFloat();
      receivedDistance_1 = Serial.parseFloat();
      receivedDistance_2 = Serial.parseFloat();
      receivedDistance_3 = Serial.parseFloat();

      MotorSpeed = Serial.parseFloat();
      stepper_x.setMaxSpeed(MotorSpeed);
      stepper_y.setMaxSpeed(MotorSpeed);
      stepper_z.setMaxSpeed(MotorSpeed);
      stepper_Z.setMaxSpeed(MotorSpeed);

      Serial.print("Distance_0: ");
      Serial.println(receivedDistance_0);
      Serial.print("Distance_1: ");
      Serial.println(receivedDistance_1);
      Serial.print("Distance_2: ");
      Serial.println(receivedDistance_2);
      Serial.print("Distance_3: ");
      Serial.println(receivedDistance_3);
      Serial.print("MotorSpeed: ");
      Serial.println(MotorSpeed);
    }

    if (receivedCommand == 'n') // stop
    {
      Serial.println("STOP ");
      run_multistepper = false;

//      steppers.moveTo(0);
//      steppers.runSpeedToPosition();

    }

    
    //////////////////////////////////////////////////
    //////////////// Accelator speed /////////////////
    //////////////////////////////////////////////////
    
    // motor 1 moving
    if (receivedCommand == 'z') //this is the measure part
    {
      // example a 2000 500 - 2000 steps (5 revolution with 400 step/rev microstepping) and 500 steps/s speed
      runallowed_x = true;
      RepeatOperation_x = true;

      receivedDistance_x = Serial.parseFloat();
      receivedDelay_x = Serial.parseFloat();
      
      Serial.print("Distance: ");
      Serial.print(receivedDistance_x);
      Serial.print(", Delay: ");
      Serial.println(receivedDelay_x);
      stepper_x.setCurrentPosition(0);
      stepper_x.move(-1 * receivedDistance_x);
      stepper_x.setMaxSpeed(receivedDelay_x);
    }
    
    // START - verse
    if (receivedCommand == 'a')
    {
      // example c 2000 500 - 2000 steps (5 revolution with 400 step/rev microstepping) and 500 steps/s speed; will rotate in the other direction
      runallowed_x = true;
      RepeatOperation_x = false;

      receivedDistance_x = Serial.parseFloat();
      receivedDelay_x = 500;

      Serial.print("Distance: ");
      Serial.print(receivedDistance_x);
      Serial.print(", Delay: ");
      Serial.println(receivedDelay_x);
      stepper_x.setCurrentPosition(0);
      stepper_x.move(-1 * receivedDistance_x);
      stepper_x.setMaxSpeed(receivedDelay_x);
    }

    // START - reverse
    if (receivedCommand == 'g')
    {
      // example c 2000 500 - 2000 steps (5 revolution with 400 step/rev microstepping) and 500 steps/s speed; will rotate in the other direction
      runallowed_x = true;
      RepeatOperation_x = false;

      receivedDistance_x = Serial.parseFloat();
      receivedDelay_x = 500;

      Serial.print("Distance: ");
      Serial.print(receivedDistance_x);
      Serial.print(", Delay: ");
      Serial.println(receivedDelay_x);
      stepper_x.setCurrentPosition(0);
      stepper_x.move(receivedDistance_x);
      stepper_x.setMaxSpeed(receivedDelay_x);
    }

    // motor 2 moving
    if (receivedCommand == 'x') //this is the measure part
    {
      // example a 2000 500 - 2000 steps (5 revolution with 400 step/rev microstepping) and 500 steps/s speed
      runallowed_y = true;
      RepeatOperation_y = true;

      receivedDistance_y = Serial.parseFloat();
      receivedDelay_y = Serial.parseFloat();

      Serial.print("Distance: ");
      Serial.print(receivedDistance_y);
      Serial.print(", Delay: ");
      Serial.println(receivedDelay_y);
      stepper_y.setCurrentPosition(0);
      stepper_y.move(receivedDistance_y);
      stepper_y.setMaxSpeed(receivedDelay_y);
    }

    // START - reverse
    if (receivedCommand == 's')
    {
      // example c 2000 500 - 2000 steps (5 revolution with 400 step/rev microstepping) and 500 steps/s speed; will rotate in the other direction
      runallowed_y = true;
      RepeatOperation_y = false;

      receivedDistance_y = Serial.parseFloat();
      receivedDelay_y = 500;

      Serial.print("Distance: ");
      Serial.print(receivedDistance_y);
      Serial.print(", Delay: ");
      Serial.println(receivedDelay_y);
      stepper_y.setCurrentPosition(0);
      stepper_y.move(receivedDistance_y);
      stepper_y.setMaxSpeed(receivedDelay_y);
    }

    // START - reverse
    if (receivedCommand == 'h')
    {
      // example c 2000 500 - 2000 steps (5 revolution with 400 step/rev microstepping) and 500 steps/s speed; will rotate in the other direction
      runallowed_y = true;
      RepeatOperation_y = false;

      receivedDistance_y = Serial.parseFloat();
      receivedDelay_y = 500;

      Serial.print("Distance: ");
      Serial.print(receivedDistance_y);
      Serial.print(", Delay: ");
      Serial.println(receivedDelay_y);
      stepper_y.setCurrentPosition(0);
      stepper_y.move(-1 * receivedDistance_y);
      stepper_y.setMaxSpeed(receivedDelay_y);
    }


    // motor 3 moving
    if (receivedCommand == 'c') //this is the measure part
    {
      // example a 2000 500 - 2000 steps (5 revolution with 400 step/rev microstepping) and 500 steps/s speed
      runallowed_z = true;
      RepeatOperation_z = true;

      receivedDistance_z = Serial.parseFloat();
      receivedDelay_z = Serial.parseFloat();

      Serial.print("Distance: ");
      Serial.print(receivedDistance_z);
      Serial.print(", Delay: ");
      Serial.println(receivedDelay_z);
      stepper_z.setCurrentPosition(0);
      stepper_z.move(-1 * receivedDistance_z);
      stepper_z.setMaxSpeed(receivedDelay_z);
    }

    // START - reverse
    if (receivedCommand == 'd')
    {
      // example c 2000 500 - 2000 steps (5 revolution with 400 step/rev microstepping) and 500 steps/s speed; will rotate in the other direction
      runallowed_z = true;
      RepeatOperation_z = false;
      
      receivedDistance_z = Serial.parseFloat();
      receivedDelay_z = 500;
      
      Serial.print("Distance: ");
      Serial.print(receivedDistance_z);
      Serial.print(", Delay: ");
      Serial.println(receivedDelay_z);
      stepper_z.setCurrentPosition(0);
      stepper_z.move(-1 * receivedDistance_z);
      stepper_z.setMaxSpeed(receivedDelay_z);
    }

    // START - reverse
    if (receivedCommand == 'j')
    {
      // example c 2000 500 - 2000 steps (5 revolution with 400 step/rev microstepping) and 500 steps/s speed; will rotate in the other direction
      runallowed_z = true;
      RepeatOperation_z = false;

      receivedDistance_z = Serial.parseFloat();
      receivedDelay_z = 500;

      Serial.print("Distance: ");
      Serial.print(receivedDistance_z);
      Serial.print(", Delay: ");
      Serial.println(receivedDelay_z);
      stepper_z.setCurrentPosition(0);
      stepper_z.move(receivedDistance_z);
      stepper_z.setMaxSpeed(receivedDelay_z);
    }


    // Z-axis motor moving
    if (receivedCommand == 'v') //this is the measure part
    {
      // example a 2000 500 - 2000 steps (5 revolution with 400 step/rev microstepping) and 500 steps/s speed
      runallowed_Z = true;
      RepeatOperation_Z = true;


      receivedDistance_Z = Serial.parseFloat();
      receivedDelay_Z = Serial.parseFloat();

      Serial.print("Distance: ");
      Serial.print(receivedDistance_Z);
      Serial.print(", Delay: ");
      Serial.println(receivedDelay_Z);
      stepper_Z.setCurrentPosition(0);
      stepper_Z.move(receivedDistance_Z);
      stepper_Z.setMaxSpeed(receivedDelay_Z);
    }

    // START - reverse
    if (receivedCommand == 'f')
    {
      // example c 2000 500 - 2000 steps (5 revolution with 400 step/rev microstepping) and 500 steps/s speed; will rotate in the other direction
      runallowed_Z = true;
      RepeatOperation_Z = false;

      receivedDistance_Z = Serial.parseFloat();
      receivedDelay_Z = 500;

      Serial.print("Distance: ");
      Serial.print(receivedDistance_Z);
      Serial.print(", Delay: ");
      Serial.println(receivedDelay_Z);
      stepper_Z.setCurrentPosition(0);
      stepper_Z.move(receivedDistance_Z);
      stepper_Z.setMaxSpeed(receivedDelay_Z);
    }

    // START - reverse
    if (receivedCommand == 'k')
    {
      // example c 2000 500 - 2000 steps (5 revolution with 400 step/rev microstepping) and 500 steps/s speed; will rotate in the other direction
      runallowed_Z = true;
      RepeatOperation_Z = false;

      receivedDistance_Z = Serial.parseFloat();
      receivedDelay_Z = 500;

      Serial.print("Distance: ");
      Serial.print(receivedDistance_Z);
      Serial.print(", Delay: ");
      Serial.println(receivedDelay_Z);
      stepper_Z.setCurrentPosition(0);
      stepper_Z.move(-1 * receivedDistance_Z);
      stepper_Z.setMaxSpeed(receivedDelay_Z);
    }


    // STOP the stepper motor
    if (receivedCommand == 'q') //"stop-x"
    {
      runallowed_x = false;
      RepeatOperation_x = false;

//      stepper_x.setCurrentPosition(0);
      Serial.println("STOP ");
      stepper_x.stop();
//      stepper_x.disableOutputs();
    }

    // STOP the stepper motor
    if (receivedCommand == 'w') //"stop-y"
    {
      runallowed_y = false;
      RepeatOperation_y = false;

//      stepper_y.setCurrentPosition(0);
      Serial.println("STOP ");
      stepper_y.stop();
//      stepper_y.disableOutputs();
    }

    // STOP the stepper motor
    if (receivedCommand == 'e') //"stop-z"
    {
      runallowed_z = false;
      RepeatOperation_z = false;

//      stepper_z.setCurrentPosition(0);
      Serial.println("STOP ");
      stepper_z.stop();
//      stepper_z.disableOutputs();
    }

    // STOP the stepper motor
    if (receivedCommand == 'r') //"stop-Z"
    {
      runallowed_Z = false;
      RepeatOperation_Z = false;

//      stepper_Z.setCurrentPosition(0);
      Serial.println("STOP ");
      stepper_Z.stop();
//      stepper_Z.disableOutputs();
    }


    // Homing the stepper motor x-axis
    if (receivedCommand == 'u')
    {
      Homing_x = true;
      Serial.println("Homing the x-axis motor");
      stepper_x.disableOutputs();
    }

    // Homing the stepper motor y-axis
    if (receivedCommand == 'i')
    {
      Homing_y = true;
      Serial.println("Homing the y-axis motor");
      stepper_y.disableOutputs();
    }

    // Homing the stepper motor z-axis
    if (receivedCommand == 'o')
    {
      Homing_z = true;
      Serial.println("Homing the z-axis motor");
      stepper_z.disableOutputs();
    }

    // Homing the stepper motor Z-axis
    if (receivedCommand == 'p')
    {
      Homing_Z = true;
      Serial.println("Homing the Z-axis motor");
      stepper_Z.disableOutputs();
    }

  }
  // after we went through the above tasks, newData becomes false again, so we are ready to receive new commands again.
  newData = false;
}



////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////



void continuousRun_const()
{
  if (run_multistepper == true)
  {
    long positions[4];
    
    time_a = millis();
    
    positions[0] = receivedDistance_0;
    positions[1] = -receivedDistance_1;
    positions[2] = receivedDistance_2;
    positions[3] = -receivedDistance_3;
    steppers.moveTo(positions);
    steppers.runSpeedToPosition();
    
    time_b = millis();
    Serial.print("cycle1 time:");
    Serial.println(time_b - time_a);
    
//    Serial.print("local maximum: ");
//    Serial.println(cnt0);
    
    delay(50);
    
    time_a = millis();
    
    positions[0] = -receivedDistance_0;
    positions[1] = receivedDistance_1;
    positions[2] = -receivedDistance_2;
    positions[3] = receivedDistance_3;
    steppers.moveTo(positions);
    steppers.runSpeedToPosition();

    time_b = millis();
    Serial.print("cycle2 time:");
    Serial.println(time_b - time_a);

//    Serial.print("local minimum: ");
//    Serial.println(cnt0);
    
    delay(50);
  }
  
  else if (run_multistepper == false && runallowed_x == false && runallowed_y == false && runallowed_z == false && runallowed_Z == false)
  {
    long positions[4];
    
    positions[0] = 0;
    positions[1] = 0;
    positions[2] = 0;
    positions[3] = 0;
    steppers.moveTo(positions);
    steppers.runSpeedToPosition();
  }

  else
  {
    return;
  }
}


void constance_for()
{
  for(int i=0; i<84-1; i++)
  {
    long positions[2];
    
    positions[0] = A[0][i]*8; // A[0][i-1] 8: Distance_weight_x
    positions[1] = B[0][i]*8; // 8: Distance_weight_y


    if (i == 0)
    {
      Speed_weight_x = abs(A[0][i]*0.08);
      Speed_weight_y = abs(B[0][i]*0.08);
      Serial.print(Speed_weight_x, Speed_weight_y);
    }
    else
    {
      Speed_weight_x = abs(A[0][i] - A[0][i-1])*0.08;
      Speed_weight_y = abs(B[0][i] - B[0][i-1])*0.08;
      Serial.print(Speed_weight_x, Speed_weight_y);
    }
    
    
    stepper_x.setMaxSpeed(A[2][i]*Speed_weight_x);
    stepper_y.setMaxSpeed(B[2][i]*Speed_weight_y);
    
    time_a = millis();
    
    Serial.print("x-motor current position: ");
    Serial.println(positions[0]);
    
    
    Serial.print("y-motor current position: ");
    Serial.println(positions[1]);
    
    steppers.moveTo(positions);
    
    steppers.runSpeedToPosition();
    
    time_b = millis();
    
//    delay(50);
    Serial.print("Time is(output):");
    Serial.print(time_b - time_a);
    Serial.print(", Ground Truth:");
    Serial.print(A[1][i]);
    Serial.print(", diff:");
    Serial.println(time_b - time_a - A[1][i]);
    
//    Serial.println(encoderPos);
//    Serial.print("A:");
//    Serial.println(digitalRead(encoderA));
//    Serial.print("B:");
//    Serial.println(digitalRead(encoderB));
//    Serial.print("Z:");
//    Serial.println(digitalRead(encoderZ));
    Serial.println();
    
    Serial.println("------------------------------------");
  }
}



void continuousRun_x()
{
  if (runallowed_x == true && RepeatOperation_x == true)
  {
    if (stepper_x.distanceToGo() == 0)
    {
      Serial.print("x-axis motor position: ");
      Serial.println(stepper_x.currentPosition());
      stepper_x.moveTo(-stepper_x.currentPosition());

      if (stepper_x.currentPosition() == receivedDistance_x)
      {
        time_x = millis();
        Serial.print("time(x): ");
        Serial.println(time_x-time_x1);
        Serial.print("Position encoder(x): ");
        Serial.println(encoderz_Pos);
      }
      else if (stepper_x.currentPosition() == -receivedDistance_x)
      {
        time_x1 = millis();
        Serial.print("time(x): ");
        Serial.println(time_x1-time_x);
        Serial.print("Position encoder(x): ");
        Serial.println(encoderz_Pos);
      }
    }
    stepper_x.enableOutputs();
    stepper_x.run();
  }
  
  else if (runallowed_x == true && RepeatOperation_x == false)
  {
    if (stepper_x.distanceToGo() != 0)
    {
      stepper_x.enableOutputs();
      stepper_x.run();
    }
    else
    {
      stepper_x.stop();
      stepper_x.disableOutputs();
    }
  }

  else // program enters this part if the runallowed is FALSE, we do not do anything
  {
    return;
  }
}



void continuousRun_y()
{

  if (runallowed_y == true && RepeatOperation_y == true)
  {
    if (stepper_y.distanceToGo() == 0)
    {
      Serial.print("y-axis motor position: ");
      Serial.println(stepper_y.currentPosition());
      stepper_y.moveTo(-stepper_y.currentPosition());

      if (stepper_y.currentPosition() == receivedDistance_y)
      {
        time_y = millis();
        Serial.print("time(y): ");
        Serial.println(time_y-time_y1);
        Serial.print("Encoder_position: ");
        Serial.println(encoderz_Pos);
        encoder_yPOS = encodery_Pos;
        Serial.print("Encoder Diff: ");
        Serial.println(encoder_yPOS-encoder_yPOS1);
      }
      else if (stepper_y.currentPosition() == -receivedDistance_y)
      {
        time_y1 = millis();
        Serial.print("time(y): ");
        Serial.println(time_y1-time_y);
        Serial.print("Encoder_position: ");
        Serial.println(encoderz_Pos);
        encoder_yPOS1 = encodery_Pos;
        Serial.print("Encoder Diff: ");
        Serial.println(encoder_yPOS-encoder_yPOS1);
      }
    }
    stepper_y.enableOutputs();
    stepper_y.run();
  }

  else if (runallowed_y == true && RepeatOperation_y == false)
  {
    if (stepper_y.distanceToGo() != 0)
    {
      stepper_y.enableOutputs();
      stepper_y.run();
    }
    else
    {
      stepper_y.stop();
      stepper_y.disableOutputs();
    }
  }
  
  else // program enters this part if the runallowed is FALSE, we do not do anything
  {
    return;
  }
}



void continuousRun_z()
{

  if (runallowed_z == true && RepeatOperation_z == true)
  {
    if (stepper_z.distanceToGo() == 0)
    {
      Serial.print("z-axis motor position: ");
      Serial.println(stepper_z.currentPosition());
      stepper_z.moveTo(-stepper_z.currentPosition());

      if (stepper_z.currentPosition() == receivedDistance_z)
      {
        time_z = millis();
        Serial.print("time(z): ");
        Serial.println(time_z-time_z1);
        Serial.print("Encoderz_position: ");
        Serial.println(encoderz_Pos);
      }
      else if (stepper_z.currentPosition() == -receivedDistance_z)
      {
        time_z1 = millis();
        Serial.print("time(z): ");
        Serial.println(time_z1-time_z);
        Serial.print("Encoder_position: ");
        Serial.println(encoderz_Pos);
      }
    }
    stepper_z.enableOutputs();
    stepper_z.run();
  }

  else if (runallowed_z == true && RepeatOperation_z == false)
  {
    if (stepper_z.distanceToGo() != 0)
    {
      stepper_z.enableOutputs();
      stepper_z.run();
    }
    else
    {
      stepper_z.stop();
      stepper_z.disableOutputs();
    }
  }
  
  else // program enters this part if the runallowed is FALSE, we do not do anything
  {
    return;
  }
}



void continuousRun_Z()
{

  if (runallowed_Z == true && RepeatOperation_Z == true)
  {
    if (stepper_Z.distanceToGo() == 0)
    {
      Serial.print("Z-axis motor position: ");
      Serial.println(stepper_Z.currentPosition());
      stepper_Z.moveTo(-stepper_Z.currentPosition());

      if (stepper_Z.currentPosition() == receivedDistance_Z)
      {
        time_Z = millis();
        Serial.print("time(Z): ");
        Serial.println(time_Z-time_Z1);
        Serial.print("EncoderZ_position: ");
        Serial.println(encoderZ_Pos);
      }
      else if (stepper_Z.currentPosition() == -receivedDistance_Z)
      {
        time_Z1 = millis();
        Serial.print("time(Z): ");
        Serial.println(time_Z1-time_Z);
        Serial.print("EncoderZ_position: ");
        Serial.println(encoderZ_Pos);
      }
    }
    stepper_Z.enableOutputs();
    stepper_Z.run();
  }

  else if (runallowed_Z == true && RepeatOperation_Z == false)
  {
    if (stepper_Z.distanceToGo() != 0)
    {
      stepper_Z.enableOutputs();
      stepper_Z.run();
    }
    else
    {
      stepper_Z.stop();
      stepper_Z.disableOutputs();
    }
  }

  else // program enters this part if the runallowed is FALSE, we do not do anything
  {
    return;
  }
}



void printVIEW()
{
  Serial.println();
}


void updateSelection()
{
  Serial.println();
}



void GoHome() // homing
{
  if (Homing_x == true)
  {
    if (stepper_x.currentPosition() == 0 && Homing_x == true)
    {
      Serial.println("Going at the x-axis home position.");
      stepper_x.disableOutputs();
      Homing_x = false;
    }
    else
    {
      stepper_x.enableOutputs();
      stepper_x.setMaxSpeed(400);
      stepper_x.moveTo(0);
      stepper_x.run();
    }
  }


  if (Homing_y == true)
  {
    if (stepper_y.currentPosition() == 0 && Homing_y == true)
    {
      Serial.println("Going at the y-axis home position.");
      stepper_y.disableOutputs();
      Homing_y = false;
    }
    else
    {
      stepper_y.enableOutputs();
      stepper_y.setMaxSpeed(400);
      stepper_y.moveTo(0);
      stepper_y.run();
    }
  }



  if (Homing_z == true)
  {
    if (stepper_z.currentPosition() == 0 && Homing_z == true)
    {
      Serial.println("Going at the z-axis home position.");
      stepper_z.disableOutputs();
      Homing_z = false;
    }
    else
    {
      stepper_z.enableOutputs();
      stepper_z.setMaxSpeed(400);
      stepper_z.moveTo(0);
      stepper_z.run();
    }
  }


  if (Homing_Z == true)
  {
    if (stepper_Z.currentPosition() == 0 && Homing_Z == true)
    {
      Serial.println("Going at the Z-axis home position.");
      stepper_Z.disableOutputs();
      Homing_Z = false;
    }
    else
    {
      stepper_Z.enableOutputs();
      stepper_Z.setMaxSpeed(400);
      stepper_Z.moveTo(0);
      stepper_Z.run();
    }
  }
}


void doEncoderA(){
  // look for a low-to-high on channel A
  if (digitalRead(encoderx_PinA) == HIGH){
    // check channel B to see which way encoder is turning
    if (digitalRead(encoderx_PinB) == LOW){
      encoderx_Pos = encoderx_Pos + 1;
    }
    else {
      encoderx_Pos = encoderx_Pos - 1;
    }
  }
  else   // must be a high-to-low edge on channel A
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(encoderx_PinB) == HIGH){
      encoderx_Pos = encoderx_Pos + 1;
    }
    else {
      encoderx_Pos = encoderx_Pos - 1;
    }
  }
  Serial.println (encoderx_Pos);
}



void doEncoderB(){
  // look for a low-to-high on channel B
  if (digitalRead(encoderx_PinB) == HIGH){
   // check channel A to see which way encoder is turning
    if (digitalRead(encoderx_PinA) == HIGH){
      encoderx_Pos = encoderx_Pos + 1;
    }
    else {
      encoderx_Pos = encoderx_Pos - 1;
    }
  }
  // Look for a high-to-low on channel B
  else {
    // check channel B to see which way encoder is turning
    if (digitalRead(encoderx_PinA) == LOW){
      encoderx_Pos = encoderx_Pos + 1;
    }
    else {
      encoderx_Pos = encoderx_Pos - 1;
    }
  }
  Serial.println (encoderx_Pos);
}


void Encoder_y_CW(){
  int y_b = digitalRead(encodery_PinB);
  if(y_b > 0){
    encodery_Pos++;
  }
//  else{
//    Count_pulses--;
//  }
}


void Encoder_y_CCW(){
  int y_a = digitalRead(encodery_PinA);
  if(y_a > 0){
    encodery_Pos--;
  }
//  else{
//    Count_pulses--;
//  }
}


void Encoder_z_CW(){
  int b = digitalRead(encoderz_PinB);
  if(b > 0){
    encoderz_Pos++;
  }
//  else{
//    Count_pulses--;
//  }
}


void Encoder_z_CCW(){
  int a = digitalRead(encoderz_PinA);
  if(a > 0){
    encoderz_Pos--;
  }
//  else{
//    Count_pulses--;
//  }
}


\
void Encoder_Z_CW(){
  int b = digitalRead(encoderZ_PinB);
  if(b > 0){
    encoderZ_Pos++;
  }
//  else{
//    Count_pulses--;
//  }
}


void Encoder_Z_CCW(){
  int a = digitalRead(encoderZ_PinA);
  if(a > 0){
    encoderZ_Pos--;
  }
//  else{
//    Count_pulses--;
//  }
}
