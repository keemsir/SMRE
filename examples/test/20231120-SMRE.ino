
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

int C[] = {4, 8, 14, 22, 30, 40, 52, 64, 72, 82, 94, 108, 126, 136, 144, 150, 154, 162, 166, 168, 170, 174, 176, 178, 182, 182, 186, 188, 190, 184, 174, 164, 154, 142, 122, 116, 110, 92, 72, 66, 58, 48, 38, 28, 32, 48, 58, 62, 68, 78, 88, 108, 122, 140, 154, 158, 162, 166, 172, 174, 178, 178, 180, 186, 188, 186, 184, 174, 164, 154, 144, 128, 112, 96, 76, 66, 54, 42, 30, 20, 10, -2, 0, 8, 14, 20, 28, 36, 46, 54, 62, 74, 86, 104, 120, 134, 146, 152, 156, 160, 166, 170, 172, 174, 178, 180, 180, 182, 184, 186, 188, 184, 174, 164, 152, 132, 104, 82, 74, 64, 54, 44, 42, 50, 62, 70, 76, 98, 120, 136, 146, 152, 158, 160, 162, 164, 170, 174, 178, 178, 180, 182, 184, 180, 172, 164, 152, 140, 120, 98, 78, 70, 60, 52, 42, 36, 30, 24, 12, 6, 4, 4, 4, 16, 24, 40, 56, 64, 72, 80, 98, 118, 132, 140, 150, 156, 158, 160, 162, 166, 168, 172, 172, 176, 180, 180, 178, 176, 176, 172, 172, 168, 158, 148, 132, 102, 80, 72, 64, 58, 46, 36, 30, 22, 12, 6, 4, 14, 24, 32, 48, 62, 74, 82, 104, 126, 140, 146, 152, 158, 162, 168, 170, 172, 176, 180, 180, 182, 184, 184, 186, 184, 184, 176, 166, 150, 130, 100, 78, 72, 60, 46, 38, 30, 26, 32, 42, 50, 64, 76, 82, 92, 110, 126, 138, 150, 158, 166, 170, 172, 174, 178, 180, 182, 180, 178, 182, 184, 182, 174, 168, 170, 170, 168, 166, 168, 174, 176, 182, 182, 186, 186, 190, 192, 200, 200, 186, 170, 150, 126, 88, 58, 46, 32, 18, 6, 0, 4, 10, 18, 28, 40, 52, 62, 72, 84, 106, 124, 142, 152, 160, 168, 170, 176, 182, 186, 192, 194, 198, 202, 206, 204, 208, 204, 194, 186, 186, 192, 196, 186, 180, 180, 180, 180, 186, 186, 188, 180, 170, 164, 150, 122, 98, 80, 70, 58, 52, 54, 58, 60, 60, 60, 62, 54, 46, 46, 52, 54, 54, 60, 64, 60, 52, 44, 32, 22, 16, 10, -2, -8, -6, 0, 6, 16, 26, 36, 50, 60, 64, 70, 76, 86, 98, 108, 120, 138, 150, 154, 158, 160, 162, 166, 168, 170, 172, 178, 180, 182, 180, 168, 156, 128, 104, 82, 68, 60, 48, 32, 20, 12, 2, -4, 4, 16, 28, 44, 56, 64, 76, 90, 104, 116, 130, 142, 158, 164, 166, 170, 172, 176, 178, 182, 184, 188, 192, 196, 194, 182, 172, 160, 140, 112, 92, 78, 70, 58, 46, 32, 22, 14, 10, 4, 10, 14, 22, 30, 40, 50, 62, 70, 78, 90, 110, 130, 142, 154, 164, 166, 170, 174, 176, 182, 188, 190, 190, 176, 162, 150, 138, 122, 106, 92, 78, 66, 58, 48, 34, 26, 18, 8, 2, 2, 14, 28, 38, 50, 60, 70, 78, 96, 118, 138, 150, 160, 166, 170, 174, 176, 180, 186, 190, 192, 180, 168, 154, 124, 96, 78, 68, 56, 44, 34, 26, 22, 14, 6, 2, 8, 16, 18, 26, 34, 46, 56, 66, 74, 82, 102, 116, 134, 146, 158, 164, 168, 172, 174, 178, 182, 184, 186, 188, 190, 192, 192, 182, 172, 166, 158, 142, 124, 106, 96, 82, 78, 68, 66, 60, 50, 42, 34, 26, 16, 12, 10, 8, 14, 12, 14, 24, 34, 44, 54, 64, 72, 88, 96, 106, 118, 132, 146, 154, 162, 168, 174, 176, 178, 180, 182, 184, 186, 188, 188, 192, 194, 178, 168, 156, 136, 106, 86, 78, 70, 62, 56, 50, 54, 58, 60, 60, 58, 48, 46, 48, 52, 58, 66, 72, 76, 82, 88, 102, 114, 126, 138, 150, 160, 166, 170, 174, 178, 182, 180, 176, 170, 160, 132, 118, 96, 88, 76, 66, 58, 44, 34, 24, 18, 10, 2, 2, 0, 4, 14, 26, 38, 50, 60, 70, 82, 92, 102, 114, 128, 140, 154, 164, 168, 168, 172, 176, 176, 178, 176, 178, 178, 178, 176, 172, 146, 122, 104, 88, 78, 68, 58, 48, 42, 34, 24, 16, 14, 8, 2, 0, 8, 18, 28, 36, 48, 60, 72, 84, 96, 120, 142, 158, 162, 170, 176, 180, 186, 190, 196, 208, 212, 216, 222, 226, 230, 234, 234, 222, 214, 204, 188, 174, 162, 148, 128, 102, 82, 72, 68, 66, 62, 58, 50, 42, 34, 24, 20, 16, 12, 12, 10, 18, 26, 36, 44, 56, 68, 80, 92, 110, 130, 144, 156, 162, 172, 178, 184, 186, 196, 200, 204, 208, 208, 206, 190, 180, 164, 142, 110, 86, 72, 62, 52, 42, 34, 26, 20, 10, 2, 2, 14, 24, 32, 44, 58, 66, 76, 90, 114, 136, 152, 164, 172, 182, 190, 196, 202, 208, 212, 216, 208, 196, 186, 172, 160, 136, 112, 98, 80, 70, 64, 58, 52, 50, 42, 40, 42, 38, 30, 22, 24, 22, 24, 34, 40, 36, 42, 48, 58, 70, 82, 106, 128, 146, 158, 166, 174, 182, 188, 194, 200, 206, 208, 206, 206, 210, 212, 216, 218, 216, 202, 184, 156, 114, 78, 60, 44, 34, 32, 40, 52, 60, 66, 70, 78, 90, 120, 150, 164, 178, 186, 192, 200, 204, 212, 214, 208, 194, 172, 158, 124, 96, 74, 68, 54, 42, 32, 24, 22, 22, 26, 26, 26, 28, 32, 34, 36, 46, 52, 56, 60, 66, 76, 86, 102, 118, 142, 160, 168, 174, 180, 186, 190, 196, 200, 206, 210, 212, 210, 200, 182, 168, 152, 120, 98, 80, 68, 58, 42, 32, 22, 12, 2, -6, -8, 0, 12, 20, 32, 48, 64, 76, 86, 112, 138, 158, 164, 176, 180, 188, 192, 198, 202, 210, 212, 214, 218, 222, 224, 226, 226, 220, 212, 198, 180, 168, 156, 136, 110, 86, 74, 68, 64, 50, 40, 28, 20, 14, 6, 8, 14, 16, 30, 42, 52, 62, 72, 82, 104, 128, 150, 160, 168, 176, 182, 188, 196, 200, 204, 210, 212, 208, 202, 200, 190, 172, 166, 160, 156, 140, 114, 92, 74, 66, 58, 48, 38, 34, 34, 28, 22, 22, 24, 24, 22, 18, 18, 24, 36, 48, 62, 68, 78, 98, 122, 146, 158, 164, 172, 178, 184, 188, 194, 198, 206, 198, 180, 166, 150, 128, 108, 86, 74, 68, 64, 52, 40, 32, 24, 18, 12, 8, 8, 4, 8, 14, 20, 28, 46, 56, 66, 76, 88, 98, 112, 120, 136, 146, 156, 162, 168, 174, 176, 180, 182, 186, 192, 194, 194, 192, 192, 186, 172, 162, 148, 120, 100, 78, 68, 62, 52, 38, 28, 20, 12, 4, 0, -6, -12, -8, 2, 12, 24, 36, 50, 60, 72, 86, 104, 120, 138, 154, 162, 168, 172, 180, 186, 188, 190, 194, 194, 198, 200, 204, 202, 206, 206, 204, 208, 208, 210, 210, 206, 198, 190, 178, 164, 150, 116, 94, 78, 66, 56, 48, 36, 26, 16, 10, 4, 0, -4, -6, 0, 6, 14, 28, 40, 56, 66, 76, 88, 106, 118, 126, 140, 160, 166, 172, 176, 180, 184, 188, 194, 196, 196, 184, 170, 152, 122, 90, 74, 70, 68, 68, 54, 50, 48, 54, 52, 40, 32, 26, 20, 10, 6, 4, 2, 6, 12, 16, 28, 44, 58, 68, 80, 96, 114, 136, 150, 162, 166, 168, 170, 172, 180, 182, 186, 188, 190, 190, 190, 188, 190, 194, 196, 196, 192, 180, 168, 142, 96, 72, 62, 46, 28, 16, 4, -6, -12, -14, -8, 4, 16, 26, 34, 44, 54, 64, 76, 86, 96, 114, 122, 142, 154, 156, 158, 160, 168, 178, 178, 180, 182, 184, 180, 176, 180, 186, 190, 190, 192, 200, 208, 212, 212, 218, 220, 220, 214, 186, 160, 116, 80, 64, 46, 36, 26, 18, 18, 24, 28, 32, 42, 50, 64, 74, 80, 86, 94, 106, 120, 140, 158, 168, 172, 178, 180, 182, 184, 188, 196, 198, 202, 198, 180, 168, 142, 112, 90, 80, 76, 70, 58, 46, 34, 28, 18, 12, 8, 4, 2, -2, -2, -6, -4, -2, 2, 6, 12, 20, 26, 36, 50, 58, 68, 76, 86, 94, 110, 122, 140, 154, 166, 176, 180, 186, 190, 194, 198, 202, 204, 194, 174, 146, 110, 92, 80, 72, 58, 46, 32, 20, 14, 14, 24, 36, 54, 64, 72, 82, 94, 106, 126, 148, 166, 180, 184, 186, 188, 196, 200, 202, 194, 182, 168, 152, 132, 106, 90, 80, 74, 64, 58, 50, 40, 30, 22, 12, 2, -4, -14, -22, -26, -28, -24, -22, -20, -16, -4, 6, 20, 32, 46, 62, 76, 86, 92, 102, 118, 136, 154, 168, 176, 184, 186, 190, 190, 196, 200, 204, 192, 184, 178, 170, 142, 108, 96, 88, 80, 74, 66, 66, 78, 86, 92, 94, 94, 94, 92, 98, 106, 126, 154, 172, 174, 144, 110, 90, 74, 64, 54, 42, 36, 26, 20, 22, 28, 32, 38, 56, 66, 76, 90, 100, 110, 128, 148, 162, 174, 182, 184, 186, 190, 192, 194, 192, 196, 194, 200, 198, 186, 174, 146, 110, 88, 76, 62, 48, 38, 26, 16, 6, 4, 2, -2, -8, -6, -2, 6, 16, 28, 46, 64, 78, 86, 96, 104, 120, 136, 156, 172, 184, 186, 188, 190, 194, 200, 200, 194, 186, 180, 136, 108, 92, 82, 70, 58, 48, 34, 24, 18, 10, 2, -6, -14, -18, -26, -28, -16, -2, 12, 22, 36, 46, 62, 72, 82, 90, 100, 106, 112, 126, 140, 150, 162, 174, 182, 186, 188, 190, 192, 198, 204, 206, 212, 216, 218, 218, 210, 190, 176, 132, 100, 90, 82, 78, 72, 62, 58, 58, 44, 34, 30, 22, 18, 12, 4, 8, 14, 22, 30, 40, 52, 64, 72, 82, 94, 108, 126, 136, 144, 150, 154, 162, 166, 168, 170, 174, 176, 178, 182, 182, 186, 188, 190, 184, 174, 164, 154, 142, 122, 116, 110, 92, 72, 66, 58, 48, 38, 28, 32, 48, 58, 62, 68, 78, 88, 108, 122, 140, 154, 158, 162, 166, 172, 174, 178, 178, 180, 186, 188, 186, 184, 174, 164, 154, 144, 128, 112, 96, 76, 66, 54, 42, 30, 20, 10, -2, 0, 8, 14, 20, 28, 36, 46, 54, 62, 74, 86, 104, 120, 134, 146, 152, 156, 160, 166, 170, 172, 174, 178, 180, 180, 182, 184, 186, 188, 184, 174, 164, 152, 132, 104, 82, 74, 64, 54, 44, 42, 50, 62, 70, 76, 98, 120, 136, 146, 152, 158, 160, 162, 164, 170, 174, 178, 178, 180, 182, 184, 180, 172, 164, 152, 140, 120, 98, 78, 70, 60, 52, 42, 36, 30, 24, 12, 6, 4, 4, 4, 16, 24, 40, 56, 64, 72, 80, 98, 118, 132, 140, 150, 156, 158, 160, 162, 166, 168, 172, 172, 176, 180, 180, 178, 176, 176, 172, 172, 168, 158, 148, 132, 102, 80, 72, 64, 58, 46, 36, 30, 22, 12, 6, 4, 14, 24, 32, 48, 62, 74, 82, 104, 126, 140, 146, 152, 158, 162, 168, 170, 172, 176, 180, 180, 182, 184, 184, 186, 184, 184, 176, 166, 150, 130, 100, 78, 72, 60, 46, 38, 30, 26, 32, 42, 50, 64, 76, 82, 92, 110, 126, 138, 150, 158, 166, 170, 172, 174, 178, 180, 182, 180, 178, 182, 184, 182, 174, 168, 170, 170, 168, 166, 168, 174, 176, 182, 182, 186, 186, 190, 192, 200, 200, 186, 170, 150, 126, 88, 58, 46, 32, 18, 6, 0, 4, 10, 18, 28, 40, 52, 62, 72, 84, 106, 124, 142, 152, 160, 168, 170, 176, 182, 186, 192, 194, 198, 202, 206, 204, 208, 204, 194, 186, 186, 192, 196, 186, 180, 180, 180, 180, 186, 186, 188, 180, 170, 164, 150, 122, 98, 80, 70, 58, 52, 54, 58, 60, 60, 60, 62, 54, 46, 46, 52, 54, 54, 60, 64, 60, 52, 44, 32, 22, 16, 10, -2, -8, -6, 0, 6, 16, 26, 36, 50, 60, 64, 70, 76, 86, 98, 108, 120, 138, 150, 154, 158, 160, 162, 166, 168, 170, 172, 178, 180, 182, 180, 168, 156, 128, 104, 82, 68, 60, 48, 32, 20, 12, 2, -4, 4, 16, 28, 44, 56, 64, 76, 90, 104, 116, 130, 142, 158, 164, 166, 170, 172, 176, 178, 182, 184, 188, 192, 196, 194, 182, 172, 160, 140, 112, 92, 78, 70, 58, 46, 32, 22, 14, 10, 4, 10, 14, 22, 30, 40, 50, 62, 70, 78, 90, 110, 130, 142, 154, 164, 166, 170, 174, 176, 182, 188, 190, 190, 176, 162, 150, 138, 122, 106, 92, 78, 66, 58, 48, 34, 26, 18, 8, 2, 2, 14, 28, 38, 50, 60, 70, 78, 96, 118, 138, 150, 160, 166, 170, 174, 176, 180, 186, 190, 192, 180, 168, 154, 124, 96, 78, 68, 56, 44, 34, 26, 22, 14, 6, 2, 8, 16, 18, 26, 34, 46, 56, 66, 74, 82, 102, 116, 134, 146, 158, 164, 168, 172, 174, 178, 182, 184, 186, 188, 190, 192, 192, 182, 172, 166, 158, 142, 124, 106, 96, 82, 78, 68, 66, 60, 50, 42, 34, 26, 16, 12, 10, 8, 14, 12, 14, 24, 34, 44, 54, 64, 72, 88, 96, 106, 118, 132, 146, 154, 162, 168, 174, 176, 178, 180, 182, 184, 186, 188, 188, 192, 194, 178, 168, 156, 136, 106, 86, 78, 70, 62, 56, 50, 54, 58, 60, 60, 58, 48, 46, 48, 52, 58, 66, 72, 76, 82, 88, 102, 114, 126, 138, 150, 160, 166, 170, 174, 178, 182, 180, 176, 170, 160, 132, 118, 96, 88, 76, 66, 58, 44, 34, 24, 18, 10, 2, 2, 0, 4, 14, 26, 38, 50, 60, 70, 82, 92, 102, 114, 128, 140, 154, 164, 168, 168, 172, 176, 176, 178, 176, 178, 178, 178, 176, 172, 146, 122, 104, 88, 78, 68, 58, 48, 42, 34, 24, 16, 14, 8, 2, 0, 8, 18, 28, 36, 48, 60, 72, 84, 96, 120, 142, 158, 162, 170, 176, 180, 186, 190, 196, 208, 212, 216, 222, 226, 230, 234, 234, 222, 214, 204, 188, 174, 162, 148, 128, 102, 82, 72, 68, 66, 62, 58, 50, 42, 34, 24, 20, 16, 12, 12, 10, 18, 26, 36, 44, 56, 68, 80, 92, 110, 130, 144, 156, 162, 172, 178, 184, 186, 196, 200, 204, 208, 208, 206, 190, 180, 164, 142, 110, 86, 72, 62, 52, 42, 34, 26, 20, 10, 2, 2, 14, 24, 32, 44, 58, 66, 76, 90, 114, 136, 152, 164, 172, 182, 190, 196, 202, 208, 212, 216, 208, 196, 186, 172, 160, 136, 112, 98, 80, 70, 64, 58, 52, 50, 42, 40, 42, 38, 30, 22, 24, 22, 24, 34, 40, 36, 42, 48, 58, 70, 82, 106, 128, 146, 158, 166, 174, 182, 188, 194, 200, 206, 208, 206, 206, 210, 212, 216, 218, 216, 202, 184, 156, 114, 78, 60, 44, 34, 32, 40, 52, 60, 66, 70, 78, 90, 120, 150, 164, 178, 186, 192, 200, 204, 212, 214, 208, 194, 172, 158, 124, 96, 74, 68, 54, 42, 32, 24, 22, 22, 26, 26, 26, 28, 32, 34, 36, 46, 52, 56, 60, 66, 76, 86, 102, 118, 142, 160, 168, 174, 180, 186, 190, 196, 200, 206, 210, 212, 210, 200, 182, 168, 152, 120, 98, 80, 68, 58, 42, 32, 22, 12, 2, -6, -8, 0, 12, 20, 32, 48, 64, 76, 86, 112, 138, 158, 164, 176, 180, 188, 192, 198, 202, 210, 212, 214, 218, 222, 224, 226, 226, 220, 212, 198, 180, 168, 156, 136, 110, 86, 74, 68, 64, 50, 40, 28, 20, 14, 6, 8, 14, 16, 30, 42, 52, 62, 72, 82, 104, 128, 150, 160, 168, 176, 182, 188, 196, 200, 204, 210, 212, 208, 202, 200, 190, 172, 166, 160, 156, 140, 114, 92, 74, 66, 58, 48, 38, 34, 34, 28, 22, 22, 24, 24, 22, 18, 18, 24, 36, 48, 62, 68, 78, 98, 122, 146, 158, 164, 172, 178, 184, 188, 194, 198, 206, 198, 180, 166, 150, 128, 108, 86, 74, 68, 64, 52, 40, 32, 24, 18, 12, 8, 8, 4, 8, 14, 20, 28, 46, 56, 66, 76, 88, 98, 112, 120, 136, 146, 156, 162, 168, 174, 176, 180, 182, 186, 192, 194, 194, 192, 192, 186, 172, 162, 148, 120, 100, 78, 68, 62, 52, 38, 28, 20, 12, 4, 0, -6, -12, -8, 2, 12, 24, 36, 50, 60, 72, 86, 104, 120, 138, 154, 162, 168, 172, 180, 186, 188, 190, 194, 194, 198, 200, 204, 202, 206, 206, 204, 208, 208, 210, 210, 206, 198, 190, 178, 164, 150, 116, 94, 78, 66, 56, 48, 36, 26, 16, 10, 4, 0, -4, -6, 0, 6, 14, 28, 40, 56, 66, 76, 88, 106, 118, 126, 140, 160, 166, 172, 176, 180, 184, 188, 194, 196, 196, 184, 170, 152, 122, 90, 74, 70, 68, 68, 54, 50, 48, 54, 52, 40, 32, 26, 20, 10, 6, 4, 2, 6, 12, 16, 28, 44, 58, 68, 80, 96, 114, 136, 150, 162, 166, 168, 170, 172, 180, 182, 186, 188, 190, 190, 190, 188, 190, 194, 196, 196, 192, 180, 168, 142, 96, 72, 62, 46, 28, 16, 4, -6, -12, -14, -8, 4, 16, 26, 34, 44, 54, 64, 76, 86, 96, 114, 122, 142, 154, 156, 158, 160, 168, 178, 178, 180, 182, 184, 180, 176, 180, 186, 190, 190, 192, 200, 208, 212, 212, 218, 220, 220, 214, 186, 160, 116, 80, 64, 46, 36, 26, 18, 18, 24, 28, 32, 42, 50, 64, 74, 80, 86, 94, 106, 120, 140, 158, 168, 172, 178, 180, 182, 184, 188, 196, 198, 202, 198, 180, 168, 142, 112, 90, 80, 76, 70, 58, 46, 34, 28, 18, 12, 8, 4, 2, -2, -2, -6, -4, -2, 2, 6, 12, 20, 26, 36, 50, 58, 68, 76, 86, 94, 110, 122, 140, 154, 166, 176, 180, 186, 190, 194, 198, 202, 204, 194, 174, 146, 110, 92, 80, 72, 58, 46, 32, 20, 14, 14, 24, 36, 54, 64, 72, 82, 94, 106, 126, 148, 166, 180, 184, 186, 188, 196, 200, 202, 194, 182, 168, 152, 132, 106, 90, 80, 74, 64, 58, 50, 40, 30, 22, 12, 2, -4, -14, -22, -26, -28, -24, -22, -20, -16, -4, 6, 20, 32, 46, 62, 76, 86, 92, 102, 118, 136, 154, 168, 176, 184, 186, 190, 190, 196, 200, 204, 192, 184, 178, 170, 142, 108, 96, 88, 80, 74, 66, 66, 78, 86, 92, 94, 94, 94, 92, 98, 106, 126, 154, 172, 174, 144, 110, 90, 74, 64, 54, 42, 36, 26, 20, 22, 28, 32, 38, 56, 66, 76, 90, 100, 110, 128, 148, 162, 174, 182, 184, 186, 190, 192, 194, 192, 196, 194, 200, 198, 186, 174, 146, 110, 88, 76, 62, 48, 38, 26, 16, 6, 4, 2, -2, -8, -6, -2, 6, 16, 28, 46, 64, 78, 86, 96, 104, 120, 136, 156, 172, 184, 186, 188, 190, 194, 200, 200, 194, 186, 180, 136, 108, 92, 82, 70, 58, 48, 34, 24, 18, 10, 2, -6, -14, -18, -26, -28, -16, -2, 12, 22, 36, 46, 62, 72, 82, 90, 100, 106, 112, 126, 140, 150, 162, 174, 182, 186, 188, 190, 192, 198, 204, 206, 212, 216, 218, 218, 210, 190, 176, 132, 100, 90, 82, 78, 72, 62, 58, 58, 44, 34, 30, 22, 18, 12};

int targetPos = 0;
int targetPos_C = 0;
int currentPos = 0;

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


void setup() {
  Serial.begin(115200);
  Serial.println("|||| Start stepper-motor ||||");

  pinMode(Encoder2_PinA,INPUT_PULLUP); // sets the Encoder_output_A pin as the input INPUT_PULLUP
  pinMode(Encoder2_PinB,INPUT_PULLUP); // sets the Encoder_output_B pin as the input INPUT_PULLUP

  time_Init = millis();

  attachInterrupt(digitalPinToInterrupt(Encoder2_PinA),Encoder_2_CW,RISING);
  attachInterrupt(digitalPinToInterrupt(Encoder2_PinB),Encoder_2_CCW,RISING);

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
int stepperDelay = 430; // 300



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
      operTime = millis();
      deterTrue = true;

      targetPos = Serial.parseFloat();
      //received_0 = Serial.parseFloat();

      Serial.print("Target position: ");
      Serial.println(targetPos);

      Serial.print("Step delay: ");
      Serial.println(stepperDelay);

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
    time_test = millis();
    Serial.println(time_test-operTime);
    Serial.println(micros());
    Serial.println(millis());
    Serial.println(millis()/50);
    Serial.println(millis()/100);

    Serial.println(sizeof(C)/2);
    Serial.println("");
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
      stepMotor2 (x % 4);
      delayMicroseconds(stepperDelay);
      x++;
      currentPos++;
    }

    else if (targetPos - currentPos < 0 & targetPos != currentPos)
    {
      stepMotor2 (x % 4);
      delayMicroseconds(stepperDelay);
      x--;
      currentPos--;
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
      stepMotor2 (x % 4);
      delayMicroseconds(stepperDelay);
      x++;
      currentPos++;

      size_Count++;
      Serial.println(targetPos_C);
    }

    else if (targetPos_C - currentPos < 0)
    {
      stepMotor2 (x % 4);
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

/*

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
