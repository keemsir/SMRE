// 기존 수시변경 print encoder position -> print encoder position per 50ms 로 변경 
// 

#include <AccelStepper.h>

// Config motor pin (DRIVER Mode)
const int motor1_stepPin = 38; // Step pin
const int motor1_dirPin = 40; // Direction pin

const int motor2_stepPin = 46; // Step pin
const int motor2_dirPin = 48; // Direction pin

const int motor3_stepPin = 22; // Step pin
const int motor3_dirPin = 24; // Direction pin

const int motor4_stepPin = 30; // Step pin
const int motor4_dirPin = 32; // Direction pin

// Config motor pin (FULL4WIRE Mode)
const int motor1_pin_1 = 2; // motor1_IN1
const int motor1_pin_2 = 3; // motor1_IN2
const int motor1_pin_3 = 4; // motor1_IN3
const int motor1_pin_4 = 5; // motor1_IN4

const int motor2_pin_1 = 6; // motor2_IN1
const int motor2_pin_2 = 7; // motor2_IN2
const int motor2_pin_3 = 8; // motor2_IN3
const int motor2_pin_4 = 9; // motor2_IN4

const int motor3_pin_1 = 10; // motor3_IN1
const int motor3_pin_2 = 10; // motor3_IN2
const int motor3_pin_3 = 10; // motor3_IN3
const int motor3_pin_4 = 10; // motor3_IN4

const int motor4_pin_1 = 10; // motor4_IN1
const int motor4_pin_2 = 10; // motor4_IN2
const int motor4_pin_3 = 10; // motor4_IN3
const int motor4_pin_4 = 10; // motor4_IN4

// 로터리 엔코더 핀 설정
const int encoderz_PinA = 19;
const int encoderz_PinB = 18;

const int encoderc_PinA = 21;
const int encoderc_PinB = 20;

// 엔코더 모니터링 모드 
bool encoderMonitorMode = false;

// 엔코더 카운터 변수
volatile long encoder_count = 0;
volatile long last_encoder_count = 0;

// 엔코더 상태 출력 시간 관리
unsigned long lastEncoderPrintTime = 0;
const unsigned long encoderPrintInterval = 50; // 50ms마다 엔코더 값 출력


// 모터 동작 상태 변수
bool isRunning = false;
bool emergencyStop = false;

// AccelStepper 객체 생성
//AccelStepper stepper1(AccelStepper::FULL4WIRE, motor1_pin_1, motor1_pin_3, motor1_pin_2, motor1_pin_4);
//AccelStepper stepper2(AccelStepper::FULL4WIRE, motor2_pin_1, motor2_pin_3, motor2_pin_2, motor2_pin_4);
AccelStepper stepper3(AccelStepper::FULL4WIRE, motor3_pin_1, motor3_pin_3, motor3_pin_2, motor3_pin_4);
AccelStepper stepper4(AccelStepper::FULL4WIRE, motor4_pin_1, motor4_pin_3, motor4_pin_2, motor4_pin_4);

AccelStepper stepper1(AccelStepper::DRIVER, motor1_stepPin, motor1_dirPin);
AccelStepper stepper2(AccelStepper::DRIVER, motor2_stepPin, motor2_dirPin);
//AccelStepper stepper3(AccelStepper::DRIVER, motor3_stepPin, motor3_dirPin);
//AccelStepper stepper4(AccelStepper::DRIVER, motor4_stepPin, motor4_dirPin);

// C1: {24, 26, 26, 30, 32, 36, 40, 44, 48, 50, 54, 56, 58, 60, 60, 62, 64, 70, 76, 80, 84, 86, 88, 94, 98, 102, 108, 112, 116, 120, 126, 130, 130, 134, 138, 144, 150, 150, 152, 154, 156, 158, 162, 164, 168, 168, 170, 172, 174, 172, 174, 176, 180, 184, 186, 186, 186, 186, 186, 184, 180, 176, 172, 168, 164, 160, 154, 150, 142, 138, 134, 130, 124, 114, 108, 104, 100, 98, 92, 86, 80, 74, 68, 60, 56, 52, 48, 44, 40, 38, 32, 28, 24, 22, 20, 18, 16, 16, 16, 16, 16, 18, 22, 24, 28, 34, 40, 40, 44, 46, 50, 54, 56, 58, 62, 66, 68, 70, 76, 80, 80, 82, 92, 102, 104, 108, 110, 112, 114, 120, 128, 130, 134, 134, 138, 142, 144, 146, 146, 150, 154, 156, 158, 162, 160, 164, 166, 168, 172, 174, 174, 176, 178, 178, 176, 170, 164, 160, 156, 154, 148, 142, 136, 132, 126, 122, 114, 108, 104, 98, 94, 88, 84, 76, 72, 66, 62, 60, 54, 52, 48, 44, 40, 36, 32, 28, 24, 20, 16, 14, 10, 6, 6, 8, 12, 14, 16, 18, 24, 28, 32, 38, 40, 44, 46, 48, 52, 56, 56, 56, 60, 64, 68, 74, 78, 80, 84, 88, 90, 96, 98, 102, 104, 106, 108, 110, 114, 118, 126, 134, 138, 144, 144, 148, 150, 152, 156, 158, 160, 162, 164, 166, 168, 168, 168, 170, 174, 178, 180, 182, 184, 184, 188, 184, 184, 182, 174, 170, 164, 164, 158, 154, 146, 140, 128, 122, 116, 108, 98, 92, 84, 78, 74, 68, 62, 58, 56, 54, 50, 44, 42, 38, 36, 34, 32, 30, 26, 26, 22, 20, 20, 20, 20};
// 8436(n=519): {0, 2, 6, 8, 10, 14, 14, 16, 20, 22, 24, 26, 28, 30, 30, 30, 30, 34, 38, 40, 42, 46, 48, 52, 54, 56, 62, 64, 66, 68, 68, 70, 70, 70, 74, 76, 80, 80, 82, 84, 84, 86, 88, 88, 90, 90, 90, 90, 88, 88, 90, 90, 92, 94, 94, 96, 96, 96, 98, 100, 100, 102, 102, 102, 102, 100, 100, 102, 104, 106, 104, 106, 106, 104, 104, 104, 102, 100, 100, 98, 92, 88, 82, 80, 76, 72, 70, 64, 60, 56, 50, 44, 40, 34, 26, 22, 18, 14, 10, 8, 6, -2, -2, -2, -4, -10, -4, 6, 0, 2, 8, 8, 8, 8, 16, 16, 16, 24, 24, 28, 32, 36, 38, 44, 46, 50, 52, 54, 56, 56, 60, 62, 66, 68, 70, 72, 72, 74, 78, 78, 82, 82, 84, 84, 86, 82, 86, 88, 90, 92, 92, 94, 94, 96, 96, 98, 100, 98, 100, 98, 100, 98, 100, 102, 102, 104, 104, 104, 106, 106, 108, 110, 110, 110, 112, 110, 110, 110, 112, 112, 112, 114, 116, 116, 116, 116, 116, 114, 114, 112, 110, 106, 104, 100, 98, 96, 92, 90, 84, 80, 72, 70, 62, 60, 52, 46, 40, 32, 26, 20, 16, 14, 12, 10, 10, 8, 12, 14, 16, 18, 20, 22, 26, 28, 28, 30, 34, 36, 40, 44, 44, 46, 50, 50, 52, 54, 54, 56, 56, 58, 56, 56, 60, 60, 62, 64, 66, 64, 66, 70, 66, 66, 64, 70, 68, 66, 70, 70, 68, 72, 74, 74, 74, 76, 76, 74, 76, 80, 80, 78, 80, 80, 80, 78, 80, 84, 84, 86, 90, 86, 88, 88, 90, 92, 92, 92, 92, 92, 92, 90, 92, 92, 94, 96, 96, 96, 96, 96, 96, 98, 98, 96, 96, 92, 88, 84, 80, 80, 74, 74, 68, 64, 58, 54, 46, 42, 34, 30, 24, 20, 16, 12, 8, 6, 0, -2, -8, -10, -12, -10, -8, -4, -2, 0, 6, 8, 10, 12, 18, 18, 22, 26, 28, 32, 34, 38, 42, 44, 48, 52, 54, 56, 56, 60, 64, 66, 68, 70, 70, 72, 74, 76, 78, 80, 82, 84, 84, 86, 86, 86, 90, 90, 92, 94, 94, 96, 96, 96, 98, 100, 100, 102, 102, 100, 100, 100, 104, 106, 108, 110, 110, 110, 114, 116, 114, 116, 116, 112, 106, 108, 104, 100, 102, 100, 96, 92, 88, 82, 76, 70, 68, 60, 54, 50, 42, 32, 26, 18, 14, 12, 6, 4, 2, 2, 4, 6, 8, 8, 12, 14, 16, 16, 18, 22, 28, 30, 32, 36, 38, 40, 44, 44, 48, 50, 52, 52, 56, 54, 56, 58, 62, 62, 64, 68, 68, 70, 72, 76, 76, 80, 80, 82, 82, 84, 84, 84, 86, 90, 92, 94, 94, 96, 98, 100, 100, 102, 102, 104, 104, 106, 106, 104, 108, 110, 112, 112, 116, 114, 116, 116, 118, 118, 118, 118, 116, 112, 110, 106, 104, 100, 98, 94, 92, 88, 84, 78, 72, 68, 62, 56, 50, 44, 34, 26, 22, 20, 16, 12, 10};
// 7336(n=481): {0, 2, 4, 4, 6, 8, 10, 14, 20, 28, 38, 46, 54, 62, 76, 86, 94, 104, 110, 122, 130, 134, 142, 146, 150, 154, 158, 160, 160, 164, 164, 168, 170, 172, 174, 176, 180, 182, 180, 178, 182, 182, 180, 180, 176, 170, 166, 158, 150, 142, 130, 120, 112, 102, 94, 88, 78, 72, 66, 62, 56, 52, 48, 42, 36, 32, 26, 22, 20, 16, 14, 10, 10, 10, 6, 6, 4, 2, 2, 2, -2, 0, -2, -2, -4, -6, -4, -6, -6, -6, -8, -8, -10, -12, -10, -10, -8, -6, -4, 0, 6, 14, 26, 38, 50, 60, 72, 84, 96, 102, 110, 116, 124, 128, 134, 140, 148, 152, 158, 160, 162, 162, 166, 166, 170, 170, 170, 172, 174, 174, 176, 172, 170, 168, 168, 166, 160, 154, 148, 142, 136, 130, 122, 116, 108, 98, 90, 80, 70, 60, 54, 50, 44, 38, 34, 28, 20, 16, 12, 8, 4, 2, 0, -2, -4, -6, -10, -10, -12, -10, -12, -12, -12, -12, -16, -16, -18, -16, -14, -14, -14, -12, -10, -10, -6, -6, -6, -6, -4, -2, 0, 2, 6, 12, 18, 24, 32, 40, 48, 56, 62, 72, 80, 90, 96, 104, 110, 114, 118, 124, 128, 134, 140, 146, 150, 154, 158, 158, 160, 164, 166, 168, 170, 172, 172, 176, 176, 178, 180, 182, 182, 186, 186, 186, 180, 176, 176, 174, 170, 166, 160, 152, 144, 132, 122, 112, 104, 94, 86, 74, 66, 60, 56, 54, 48, 44, 36, 30, 24, 18, 14, 12, 8, 6, 6, 6, 6, 8, 6, 6, 4, 4, 2, 2, 2, 0, -2, -2, -2, -4, -6, -8, -8, -10, -10, -10, -12, -10, -12, -12, -8, -6, -2, 2, 10, 20, 28, 40, 50, 58, 68, 76, 86, 94, 102, 110, 112, 120, 126, 130, 136, 142, 152, 152, 156, 158, 162, 164, 164, 168, 168, 170, 170, 172, 174, 176, 178, 180, 182, 184, 182, 180, 182, 184, 182, 184, 184, 184, 180, 172, 166, 160, 154, 146, 138, 126, 118, 110, 102, 96, 88, 82, 74, 64, 58, 54, 50, 44, 38, 32, 26, 22, 18, 14, 10, 8, 6, 4, 0, -2, -4, -4, -4, -4, -4, -6, -4, -4, -4, -4, -2, 0, 0, -2, 0, -2, -2, 0, 0, 2, 2, 4, 6, 12, 18, 28, 36, 46, 54, 64, 74, 82, 92, 100, 104, 110, 116, 120, 126, 132, 138, 140, 142, 148, 150, 154, 158, 160, 162, 162, 166, 166, 170, 172, 174, 176, 176, 174, 174, 174, 174, 174, 176, 176, 176, 176, 176, 176, 174, 172, 168, 164, 162, 156, 144, 132, 120, 112, 104, 96, 86, 78, 68, 58, 54, 50, 44, 38, 30, 28, 18, 12, 8, 2, -2, -4, -6, -6, -10, -10, -12, -14, -12, -14, -14, -14, -12, -10, -10, -10, -14};
// 7289(n=549): {0, 2, 4, 6, 8, 8, 10, 12, 14, 16, 20, 24, 28, 32, 36, 40, 44, 48, 54, 58, 62, 66, 72, 74, 78, 84, 90, 94, 96, 100, 106, 110, 114, 116, 120, 122, 124, 124, 128, 132, 134, 138, 138, 140, 142, 144, 148, 150, 152, 154, 156, 158, 160, 164, 164, 164, 164, 158, 148, 142, 136, 128, 122, 114, 106, 100, 92, 82, 74, 66, 60, 56, 52, 48, 42, 38, 34, 32, 26, 22, 20, 16, 12, 10, 8, 8, 8, 10, 10, 10, 8, 10, 10, 12, 14, 14, 16, 20, 20, 22, 28, 30, 32, 34, 38, 40, 44, 48, 52, 54, 56, 58, 62, 68, 72, 76, 82, 86, 92, 94, 98, 104, 110, 112, 114, 116, 120, 122, 126, 128, 134, 136, 140, 142, 146, 150, 152, 156, 158, 158, 156, 154, 152, 150, 146, 142, 138, 132, 128, 124, 118, 112, 106, 100, 92, 86, 80, 74, 68, 64, 60, 56, 54, 50, 46, 42, 40, 36, 34, 32, 30, 26, 22, 20, 16, 14, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 14, 14, 16, 18, 22, 26, 30, 34, 36, 40, 44, 48, 52, 56, 60, 64, 70, 76, 80, 84, 90, 94, 100, 104, 110, 112, 116, 120, 122, 124, 128, 132, 136, 140, 144, 144, 146, 150, 154, 156, 158, 162, 162, 164, 166, 164, 162, 158, 154, 150, 144, 138, 132, 126, 120, 114, 106, 102, 96, 90, 82, 78, 74, 68, 66, 60, 56, 52, 48, 44, 42, 38, 34, 32, 30, 28, 24, 20, 20, 16, 12, 10, 10, 8, 8, 8, 8, 8, 10, 10, 10, 12, 16, 18, 22, 24, 26, 30, 34, 38, 40, 44, 48, 50, 54, 56, 62, 66, 70, 76, 80, 84, 90, 96, 100, 104, 106, 110, 116, 120, 122, 126, 132, 138, 142, 144, 146, 150, 154, 154, 158, 158, 160, 162, 164, 164, 164, 158, 156, 154, 152, 150, 146, 144, 140, 138, 134, 126, 122, 118, 112, 106, 102, 96, 90, 84, 78, 72, 68, 66, 58, 54, 50, 48, 44, 40, 36, 32, 28, 24, 20, 16, 12, 10, 6, 2, 0, -2, -2, -2, -2, 0, 2, 4, 4, 8, 12, 12, 14, 18, 24, 26, 30, 36, 40, 44, 48, 52, 56, 62, 68, 72, 78, 84, 88, 94, 100, 104, 108, 112, 116, 120, 124, 128, 134, 138, 142, 142, 144, 146, 150, 152, 156, 158, 162, 164, 166, 168, 170, 170, 170, 168, 166, 164, 162, 160, 154, 150, 144, 140, 134, 128, 124, 120, 114, 108, 104, 100, 92, 88, 80, 76, 70, 62, 58, 56, 50, 46, 42, 40, 36, 32, 28, 24, 20, 16, 12, 10, 6, 2, 2, -2, -2, -4, -4, -4, -6, -2, -2, 4, 6, 10, 10, 14, 16, 20, 24, 28, 30, 34, 36, 40, 42, 46, 50, 54, 60, 64, 68, 70, 76, 80, 84, 88, 92, 96, 100, 104, 106, 108, 110, 112, 114, 116, 120, 122, 124, 126, 126, 126, 128, 134, 136, 138, 140, 142, 144, 144, 144, 144, 144, 140, 140, 136, 130, 124, 122, 118, 112, 104, 100, 98, 94, 86, 82, 76, 70, 66, 60, 56, 52, 48, 42, 38, 34, 30, 26, 22, 18, 14, 12, 14, 12};
// 6656(n=753): {0, 2, 4, 4, 6, 6, 8, 10, 12, 14, 18, 22, 28, 34, 42, 50, 58, 66, 72, 78, 80, 80, 82, 86, 88, 90, 94, 96, 98, 98, 98, 102, 102, 100, 98, 92, 86, 84, 82, 80, 82, 82, 84, 84, 82, 84, 86, 82, 80, 78, 70, 68, 68, 66, 64, 66, 62, 62, 62, 62, 62, 60, 62, 62, 60, 56, 56, 54, 56, 56, 58, 58, 58, 60, 60, 62, 62, 64, 64, 58, 60, 60, 60, 62, 64, 66, 70, 70, 72, 74, 76, 78, 78, 74, 68, 66, 62, 58, 54, 50, 46, 40, 36, 30, 24, 20, 14, 8, 0, -4, -6, -6, -6, -8, -10, -10, -12, -10, -10, -8, -8, -8, -8, -4, -4, -2, 4, 10, 14, 22, 34, 42, 50, 58, 68, 76, 78, 82, 88, 90, 94, 98, 102, 102, 104, 106, 108, 108, 106, 104, 100, 94, 94, 92, 92, 92, 94, 94, 94, 94, 94, 96, 94, 94, 92, 86, 84, 84, 82, 84, 84, 84, 84, 82, 82, 82, 80, 80, 78, 74, 72, 70, 68, 68, 70, 70, 70, 70, 70, 72, 72, 74, 72, 70, 68, 68, 68, 68, 68, 70, 70, 70, 72, 74, 74, 76, 76, 76, 72, 72, 72, 72, 74, 76, 76, 76, 76, 76, 76, 74, 70, 66, 56, 52, 44, 40, 36, 28, 22, 16, 10, 2, -2, -4, -8, -14, -24, -26, -32, -34, -38, -42, -44, -46, -48, -46, -44, -42, -42, -40, -42, -40, -42, -42, -40, -34, -30, -22, -10, 2, 10, 20, 28, 34, 38, 48, 54, 62, 72, 78, 84, 88, 92, 94, 96, 96, 96, 96, 90, 88, 88, 88, 88, 90, 88, 88, 88, 86, 90, 90, 88, 86, 80, 78, 78, 76, 76, 76, 74, 74, 76, 76, 78, 76, 78, 78, 76, 74, 76, 74, 74, 78, 78, 78, 78, 80, 82, 82, 80, 80, 80, 78, 80, 80, 84, 88, 86, 86, 86, 86, 88, 88, 88, 86, 82, 84, 84, 84, 84, 88, 90, 90, 92, 92, 94, 94, 96, 98, 94, 94, 94, 92, 96, 96, 98, 98, 98, 100, 100, 98, 96, 92, 86, 82, 78, 74, 70, 64, 60, 56, 50, 46, 40, 34, 26, 18, 10, 4, -2, -6, -12, -14, -20, -24, -24, -22, -20, -20, -18, -16, -14, -12, -10, -6, 0, 10, 20, 30, 42, 52, 62, 70, 76, 78, 84, 88, 92, 98, 100, 102, 104, 104, 108, 110, 108, 106, 102, 98, 96, 96, 94, 96, 98, 98, 100, 100, 102, 100, 100, 98, 98, 92, 92, 92, 92, 92, 94, 94, 94, 94, 96, 96, 96, 98, 98, 92, 92, 92, 92, 90, 92, 94, 94, 94, 94, 94, 94, 94, 94, 92, 88, 90, 90, 92, 94, 94, 98, 96, 100, 100, 102, 104, 104, 102, 100, 102, 100, 98, 96, 94, 90, 88, 84, 80, 74, 70, 64, 56, 50, 44, 38, 32, 24, 18, 12, 6, 2, 2, 0, -2, -6, -8, -6, -6, -8, -10, -10, -12, -12, -12, -12, -12, -10, -8, -10, -12, -10, -6, -2, 4, 8, 18, 30, 38, 50, 54, 60, 68, 70, 66, 72, 78, 78, 80, 82, 80, 80, 80, 82, 82, 84, 82, 80, 74, 72, 72, 70, 70, 70, 70, 72, 70, 70, 70, 68, 68, 68, 62, 62, 62, 64, 64, 64, 66, 66, 68, 66, 68, 70, 70, 68, 66, 66, 64, 64, 66, 68, 68, 70, 72, 72, 74, 74, 74, 76, 72, 70, 72, 72, 74, 78, 78, 80, 82, 86, 88, 88, 92, 92, 88, 90, 90, 88, 88, 86, 84, 80, 78, 74, 70, 64, 62, 54, 46, 40, 34, 28, 22, 16, 12, 8, 4, 4, 2, 0, 0, 2, 0, 0, 0, 2, 2, 4, 4, 6, 10, 16, 24, 34, 44, 50, 56, 66, 74, 80, 88, 94, 100, 102, 106, 108, 110, 110, 108, 104, 96, 98, 98, 100, 100, 98, 98, 98, 98, 98, 100, 98, 96, 92, 84, 82, 80, 80, 84, 84, 84, 84, 84, 84, 84, 82, 82, 80, 74, 74, 72, 70, 68, 70, 70, 70, 70, 72, 74, 74, 72, 70, 66, 66, 70, 70, 72, 76, 78, 80, 80, 80, 78, 76, 78, 78, 76, 80, 72, 78, 76, 80, 80, 80, 80, 82, 82, 80, 82, 80, 78, 82, 86, 88, 90, 90, 92, 90, 88, 86, 84, 80, 74, 64, 56, 50, 42, 36, 30, 22, 18, 12, 10, 8, 6, 6, 4, 2};
// 7624(n=350): {0, 6, 20, 36, 50, 60, 70, 76, 88, 104, 106, 112, 116, 120, 120, 122, 122, 122, 122, 122, 122, 122, 122, 120, 120, 128, 130, 128, 134, 136, 136, 138, 144, 150, 156, 162, 170, 170, 172, 170, 168, 168, 168, 170, 168, 158, 158, 154, 140, 130, 124, 108, 94, 82, 70, 56, 42, 32, 30, 22, 18, 30, 48, 62, 68, 80, 88, 92, 96, 102, 102, 104, 102, 98, 106, 112, 108, 110, 112, 112, 110, 108, 110, 108, 106, 106, 108, 118, 128, 134, 142, 150, 150, 148, 150, 146, 140, 134, 130, 124, 118, 110, 102, 94, 88, 74, 64, 58, 44, 34, 24, 12, 0, -8, -14, -14, -2, 16, 34, 46, 60, 70, 76, 94, 106, 110, 116, 122, 122, 120, 122, 122, 120, 118, 120, 118, 118, 116, 114, 116, 122, 122, 122, 128, 132, 136, 148, 156, 160, 166, 166, 164, 162, 162, 160, 152, 148, 142, 126, 120, 112, 104, 100, 92, 84, 72, 58, 48, 38, 22, 16, 4, -2, 10, 22, 40, 54, 66, 76, 82, 90, 98, 100, 106, 110, 112, 116, 114, 114, 124, 126, 126, 128, 128, 126, 126, 128, 128, 128, 128, 128, 128, 130, 132, 130, 136, 150, 154, 160, 168, 170, 170, 168, 168, 162, 156, 146, 138, 130, 124, 116, 108, 102, 92, 80, 66, 56, 44, 28, 20, 10, 0, -2, 10, 28, 46, 60, 66, 84, 96, 100, 108, 116, 116, 120, 122, 124, 124, 124, 124, 124, 124, 122, 118, 118, 124, 128, 126, 130, 134, 134, 134, 134, 138, 146, 154, 164, 166, 170, 170, 168, 166, 166, 162, 158, 152, 146, 142, 132, 122, 114, 108, 102, 94, 86, 78, 66, 58, 54, 44, 32, 30, 34, 46, 60, 74, 86, 94, 104, 112, 116, 124, 126, 124, 132, 140, 138, 142, 146, 148, 148, 150, 150, 150, 150, 150, 150, 146, 150, 148, 146, 154, 162, 166, 176, 184, 192, 192, 190, 186, 180, 178, 174, 170, 166, 160, 152, 146, 140, 130, 120, 112, 100, 86, 74, 60, 46, 34, 24, 16};


// Array C
int C1[] = {0, 6, 20, 36, 50, 60, 70, 76, 88, 104, 106, 112, 116, 120, 120, 122, 122, 122, 122, 122, 122, 122, 122, 120, 120, 128, 130, 128, 134, 136, 136, 138, 144, 150, 156, 162, 170, 170, 172, 170, 168, 168, 168, 170, 168, 158, 158, 154, 140, 130, 124, 108, 94, 82, 70, 56, 42, 32, 30, 22, 18, 30, 48, 62, 68, 80, 88, 92, 96, 102, 102, 104, 102, 98, 106, 112, 108, 110, 112, 112, 110, 108, 110, 108, 106, 106, 108, 118, 128, 134, 142, 150, 150, 148, 150, 146, 140, 134, 130, 124, 118, 110, 102, 94, 88, 74, 64, 58, 44, 34, 24, 12, 0, -8, -14, -14, -2, 16, 34, 46, 60, 70, 76, 94, 106, 110, 116, 122, 122, 120, 122, 122, 120, 118, 120, 118, 118, 116, 114, 116, 122, 122, 122, 128, 132, 136, 148, 156, 160, 166, 166, 164, 162, 162, 160, 152, 148, 142, 126, 120, 112, 104, 100, 92, 84, 72, 58, 48, 38, 22, 16, 4, -2, 10, 22, 40, 54, 66, 76, 82, 90, 98, 100, 106, 110, 112, 116, 114, 114, 124, 126, 126, 128, 128, 126, 126, 128, 128, 128, 128, 128, 128, 130, 132, 130, 136, 150, 154, 160, 168, 170, 170, 168, 168, 162, 156, 146, 138, 130, 124, 116, 108, 102, 92, 80, 66, 56, 44, 28, 20, 10, 0, -2, 10, 28, 46, 60, 66, 84, 96, 100, 108, 116, 116, 120, 122, 124, 124, 124, 124, 124, 124, 122, 118, 118, 124, 128, 126, 130, 134, 134, 134, 134, 138, 146, 154, 164, 166, 170, 170, 168, 166, 166, 162, 158, 152, 146, 142, 132, 122, 114, 108, 102, 94, 86, 78, 66, 58, 54, 44, 32, 30, 34, 46, 60, 74, 86, 94, 104, 112, 116, 124, 126, 124, 132, 140, 138, 142, 146, 148, 148, 150, 150, 150, 150, 150, 150, 146, 150, 148, 146, 154, 162, 166, 176, 184, 192, 192, 190, 186, 180, 178, 174, 170, 166, 160, 152, 146, 140, 130, 120, 112, 100, 86, 74, 60, 46, 34, 24, 16};
int C2[] = {0, 6, 20, 36, 50, 60, 70, 76, 88, 104, 106, 112, 116, 120, 120, 122, 122, 122, 122, 122, 122, 122, 122, 120, 120, 128, 130, 128, 134, 136, 136, 138, 144, 150, 156, 162, 170, 170, 172, 170, 168, 168, 168, 170, 168, 158, 158, 154, 140, 130, 124, 108, 94, 82, 70, 56, 42, 32, 30, 22, 18, 30, 48, 62, 68, 80, 88, 92, 96, 102, 102, 104, 102, 98, 106, 112, 108, 110, 112, 112, 110, 108, 110, 108, 106, 106, 108, 118, 128, 134, 142, 150, 150, 148, 150, 146, 140, 134, 130, 124, 118, 110, 102, 94, 88, 74, 64, 58, 44, 34, 24, 12, 0, -8, -14, -14, -2, 16, 34, 46, 60, 70, 76, 94, 106, 110, 116, 122, 122, 120, 122, 122, 120, 118, 120, 118, 118, 116, 114, 116, 122, 122, 122, 128, 132, 136, 148, 156, 160, 166, 166, 164, 162, 162, 160, 152, 148, 142, 126, 120, 112, 104, 100, 92, 84, 72, 58, 48, 38, 22, 16, 4, -2, 10, 22, 40, 54, 66, 76, 82, 90, 98, 100, 106, 110, 112, 116, 114, 114, 124, 126, 126, 128, 128, 126, 126, 128, 128, 128, 128, 128, 128, 130, 132, 130, 136, 150, 154, 160, 168, 170, 170, 168, 168, 162, 156, 146, 138, 130, 124, 116, 108, 102, 92, 80, 66, 56, 44, 28, 20, 10, 0, -2, 10, 28, 46, 60, 66, 84, 96, 100, 108, 116, 116, 120, 122, 124, 124, 124, 124, 124, 124, 122, 118, 118, 124, 128, 126, 130, 134, 134, 134, 134, 138, 146, 154, 164, 166, 170, 170, 168, 166, 166, 162, 158, 152, 146, 142, 132, 122, 114, 108, 102, 94, 86, 78, 66, 58, 54, 44, 32, 30, 34, 46, 60, 74, 86, 94, 104, 112, 116, 124, 126, 124, 132, 140, 138, 142, 146, 148, 148, 150, 150, 150, 150, 150, 150, 146, 150, 148, 146, 154, 162, 166, 176, 184, 192, 192, 190, 186, 180, 178, 174, 170, 166, 160, 152, 146, 140, 130, 120, 112, 100, 86, 74, 60, 46, 34, 24, 16};
int C3[] = {24, 26, 26, 30, 32, 36, 40, 44, 48, 50, 54, 56, 58, 60, 60, 62, 64, 70, 76, 80, 84, 86, 88, 94, 98, 102, 108, 112, 116, 120, 126, 130, 130, 134, 138, 144, 150, 150, 152, 154, 156, 158, 162, 164, 168, 168, 170, 172, 174, 172, 174, 176, 180, 184, 186, 186, 186, 186, 186, 184, 180, 176, 172, 168, 164, 160, 154, 150, 142, 138, 134, 130, 124, 114, 108, 104, 100, 98, 92, 86, 80, 74, 68, 60, 56, 52, 48, 44, 40, 38, 32, 28, 24, 22, 20, 18, 16, 16, 16, 16, 16, 18, 22, 24, 28, 34, 40, 40, 44, 46, 50, 54, 56, 58, 62, 66, 68, 70, 76, 80, 80, 82, 92, 102, 104, 108, 110, 112, 114, 120, 128, 130, 134, 134, 138, 142, 144, 146, 146, 150, 154, 156, 158, 162, 160, 164, 166, 168, 172, 174, 174, 176, 178, 178, 176, 170, 164, 160, 156, 154, 148, 142, 136, 132, 126, 122, 114, 108, 104, 98, 94, 88, 84, 76, 72, 66, 62, 60, 54, 52, 48, 44, 40, 36, 32, 28, 24, 20, 16, 14, 10, 6, 6, 8, 12, 14, 16, 18, 24, 28, 32, 38, 40, 44, 46, 48, 52, 56, 56, 56, 60, 64, 68, 74, 78, 80, 84, 88, 90, 96, 98, 102, 104, 106, 108, 110, 114, 118, 126, 134, 138, 144, 144, 148, 150, 152, 156, 158, 160, 162, 164, 166, 168, 168, 168, 170, 174, 178, 180, 182, 184, 184, 188, 184, 184, 182, 174, 170, 164, 164, 158, 154, 146, 140, 128, 122, 116, 108, 98, 92, 84, 78, 74, 68, 62, 58, 56, 54, 50, 44, 42, 38, 36, 34, 32, 30, 26, 26, 22, 20, 20, 20, 20};
int C4[] = {24, 26, 26, 30, 32, 36, 40, 44, 48, 50, 54, 56, 58, 60, 60, 62, 64, 70, 76, 80, 84, 86, 88, 94, 98, 102, 108, 112, 116, 120, 126, 130, 130, 134, 138, 144, 150, 150, 152, 154, 156, 158, 162, 164, 168, 168, 170, 172, 174, 172, 174, 176, 180, 184, 186, 186, 186, 186, 186, 184, 180, 176, 172, 168, 164, 160, 154, 150, 142, 138, 134, 130, 124, 114, 108, 104, 100, 98, 92, 86, 80, 74, 68, 60, 56, 52, 48, 44, 40, 38, 32, 28, 24, 22, 20, 18, 16, 16, 16, 16, 16, 18, 22, 24, 28, 34, 40, 40, 44, 46, 50, 54, 56, 58, 62, 66, 68, 70, 76, 80, 80, 82, 92, 102, 104, 108, 110, 112, 114, 120, 128, 130, 134, 134, 138, 142, 144, 146, 146, 150, 154, 156, 158, 162, 160, 164, 166, 168, 172, 174, 174, 176, 178, 178, 176, 170, 164, 160, 156, 154, 148, 142, 136, 132, 126, 122, 114, 108, 104, 98, 94, 88, 84, 76, 72, 66, 62, 60, 54, 52, 48, 44, 40, 36, 32, 28, 24, 20, 16, 14, 10, 6, 6, 8, 12, 14, 16, 18, 24, 28, 32, 38, 40, 44, 46, 48, 52, 56, 56, 56, 60, 64, 68, 74, 78, 80, 84, 88, 90, 96, 98, 102, 104, 106, 108, 110, 114, 118, 126, 134, 138, 144, 144, 148, 150, 152, 156, 158, 160, 162, 164, 166, 168, 168, 168, 170, 174, 178, 180, 182, 184, 184, 188, 184, 184, 182, 174, 170, 164, 164, 158, 154, 146, 140, 128, 122, 116, 108, 98, 92, 84, 78, 74, 68, 62, 58, 56, 54, 50, 44, 42, 38, 36, 34, 32, 30, 26, 26, 22, 20, 20, 20, 20};

// 배열 크기 계산
const int arraySize1 = sizeof(C1) / sizeof(C1[0]);
const int arraySize2 = sizeof(C2) / sizeof(C2[0]);
const int arraySize3 = sizeof(C3) / sizeof(C3[0]);
const int arraySize4 = sizeof(C4) / sizeof(C4[0]);

// 작동 시간 측정 변수
unsigned long startTime = 0;
unsigned long endTime = 0;

// unsigned long totalTime;

// 엔코더 상태 출력 시간 관리
//unsigned long lastEncoderPrintTime = 0;
//const unsigned long encoderPrintInterval = 50; // default: 500ms마다 엔코더 값 출력

void setup() {
    Serial.begin(115200);
    Serial.println("-------------------------------------");
    Serial.println("Stepper Motor and Encoder Initialized");

    // 모터 속도 및 가속 설정
    stepper1.setMaxSpeed(1600);      // 최대 속도 (steps per second)
    stepper1.setAcceleration(800);   // 가속도 (steps per second^2)

    stepper2.setMaxSpeed(1600);      // 최대 속도 (steps per second)
    stepper2.setAcceleration(800);   // 가속도 (steps per second^2)

    stepper3.setMaxSpeed(1000);      // 최대 속도 (steps per second)
    stepper3.setAcceleration(500);   // 가속도 (steps per second^2)

    stepper4.setMaxSpeed(1000);      // 최대 속도 (steps per second)
    stepper4.setAcceleration(500);   // 가속도 (steps per second^2)

    // 배열 길이 일치 여부 확인
    if (arraySize1 == arraySize2 && arraySize2 == arraySize3 && arraySize3 == arraySize4) {
    Serial.println("All arrays have the same length.");
    } else {
    Serial.println("Warning: Array lengths do not match!");

    // 불일치하는 배열 쌍 출력
    if (arraySize1 != arraySize2) Serial.println("C2 differ in arraySize.");
    if (arraySize1 != arraySize3) Serial.println("C3 differ in arraySize.");
    if (arraySize1 != arraySize4) Serial.println("C4 differ in arraySize.");
    }

    // Config a rotary encdoer pin
    pinMode(encoderz_PinA, INPUT_PULLUP);
    pinMode(encoderz_PinB, INPUT_PULLUP);
    
    // Config the interrupt
    attachInterrupt(digitalPinToInterrupt(encoderz_PinA), Encoder_z_CW, RISING);
    attachInterrupt(digitalPinToInterrupt(encoderz_PinB), Encoder_z_CCW, RISING);

    // C 배열의 이론적 작동 시간 계산 및 출력
    int theoreticalTime = arraySize1 * 50;  // 각 위치당 50ms
    Serial.print("Expected operation time: ");
    printFormattedTime(theoreticalTime);

    // Min-Max calculation
    calculateMaxMin();

    Serial.println("Available commands:");
    Serial.println("a: Operate all motors simultaneously");
    Serial.println("1: Operate only motor 1");
    Serial.println("2: Operate only motor 2");
    Serial.println("3: Operate only motor 3");
    Serial.println("4: Operate only motor 4");

    Serial.println("e: Print current encoder value");
    Serial.println("r: Reset encoder value");
    Serial.println("w: Encoder toggle on/off");
    Serial.println("s: Emergency stop !!");
    Serial.println("m: Repeat motor movements with max and min values of C1");

}


void loop() {
    
    // 비상 정지 상태 확인
    if (emergencyStop) {
        // 비상 정지 상태일 때는 모터 동작 중지
        if (isRunning) {
            stopMotors();
            isRunning = false;
            Serial.println("Emergency stop activated. Motors stopped.");
        }
        
        // 비상 정지 상태 해제
        emergencyStop = false;
        Serial.println("Ready for new commands.");
    }

    // 시리얼 입력 확인 (비상 정지 포함)
    checkSerialInput();
}


// 시리얼 입력 확인 함수
void checkSerialInput() {
    if (Serial.available() > 0) {
        char command = Serial.read();

        // 비상 정지 명령 확인
        if (command == 's') {
            emergencyStop = true;
            return;
        }
        
        // 다른 명령어들 처리
        if (!isRunning) {  // 모터가 동작 중이 아닐 때만 다른 명령 처리
            if (command == 'a') {
                Serial.println("Moving all motors simultaneously");
                
                // 시작 시간 기록
                // recordCount = 0; // 기록 초기화
                startTime = millis();
                isRunning = true;
                Serial.println("Time(ms), Encoder Value");
                moveMotorByArray();
                isRunning = false;
                // 종료 시간 기록
                endTime = millis();
                
                // 총 소요 시간 계산 및 출력
                unsigned long totalTime = endTime - startTime;
                Serial.print("Total execution time: ");
                Serial.print(totalTime);
                Serial.println(" milliseconds");
            }
            else if (command == 'e') {
                // 엔코더 값 출력 명령
                Serial.print("Current encoder count: ");
                Serial.println(encoder_count);
            }
            else if (command == 'r') {
                // 엔코더 값 리셋 명령
                encoder_count = 0;
                Serial.println("Encoder count reset to 0");
            }
            else if (command == '1') {
                // 모터 1만 작동
                Serial.println("Moving only motor 1");
                // 시작 시간 기록
                // recordCount = 0; // 기록 초기화
                startTime = millis();
                isRunning = true;
                Serial.println("Time(ms), Encoder Value"); // 헤더 출력 추가
                moveSingleMotor(1);
                isRunning = false;
                // 종료 시간 기록
                endTime = millis();

                // 총 소요 시간 계산 및 출력
                unsigned long totalTime = endTime - startTime;
                Serial.print("Total execution time: ");
                Serial.print(totalTime);
                Serial.println(" milliseconds");
            }
            else if (command == '2') {
                // 모터 2만 작동
                Serial.println("Moving only motor 2");
                // 시작 시간 기록
                // recordCount = 0; // 기록 초기화
                startTime = millis();
                isRunning = true;
                Serial.println("Time(ms), Encoder Value"); // 헤더 출력 추가
                moveSingleMotor(2);
                isRunning = false;
                // 종료 시간 기록
                endTime = millis();

                unsigned long totalTime = endTime - startTime;
                Serial.print("Total execution time: ");
                Serial.print(totalTime);
                Serial.println(" milliseconds");
            }

            else if (command == '3') {
                Serial.println("Moving only motor 3");
                startTime = millis();
                isRunning = true;
                Serial.println("Time(ms), Encoder Value");
                moveSingleMotor(3);
                isRunning = false;
                endTime = millis();

                unsigned long totalTime = endTime - startTime;
                Serial.print("Total execution time: ");
                Serial.print(totalTime);
                Serial.println(" milliseconds");
            }
            else if (command == '4') {
                Serial.println("Moving only motor 4");
                startTime = millis();
                isRunning = true;
                Serial.println("Time(ms), Encoder Value");
                moveSingleMotor(4);
                isRunning = false;
                endTime = millis();

                unsigned long totalTime = endTime - startTime;
                Serial.print("Total execution time: ");
                Serial.print(totalTime);
                Serial.println(" milliseconds");
            }

            else if (command == 'm') { // 'm' 입력 시 반복 동작 실행
                repeatMaxMin();
            }
            
            else if (command == 'w') {
                // 엔코더 값 변화 모니터링 모드 토글
                encoderMonitorMode = !encoderMonitorMode;
                if (encoderMonitorMode) {
                    Serial.println("Encoder monitoring mode: ON - All encoder changes will be printed");
                } else {
                    Serial.println("Encoder monitoring mode: OFF");
                }
            }
        }
    }
}


// 모터 정지 함수
void stopMotors() {
    stepper1.stop(); // 감속하여 정지
    stepper2.stop(); // 감속하여 정지
    stepper3.stop(); // 감속하여 정지
    stepper4.stop(); // 감속하여 정지
    
    // 모터가 완전히 멈출 때까지 run() 함수 호출
    while (stepper1.isRunning() || stepper2.isRunning() || stepper3.isRunning() || stepper4.isRunning()) {
        stepper1.run();
        stepper2.run();
        stepper3.run();
        stepper4.run();
    }
    
    // 모터 출력 비활성화
    stepper1.disableOutputs();
    stepper2.disableOutputs();
    stepper3.disableOutputs();
    stepper4.disableOutputs();
}


// 배열 C에 따라 모터 움직임 구현 함수
void moveMotorByArray() {
    for (int i = 0; i < arraySize1; i++) {
        
        long targetPosition = C1[i]*5; // 배열 C의 값 사용
        long targetPosition2 = C2[i]*5; // 배열 C의 값 사용
        // long targetPosition3 = C3[i]*4; // 배열 C의 값 사용
        // long targetPosition4 = C4[i]*4; // 배열 C의 값 사용
        
        stepper1.moveTo(-targetPosition);
        stepper2.moveTo(targetPosition2);
        // stepper3.moveTo(targetPosition);
        // stepper4.moveTo(-targetPosition2);

        unsigned long stepStartTime = millis();
        while (millis() - stepStartTime < 50) { // 비차단 방식으로 실행
            // 이 루프 안에서도 시리얼 입력 확인

            if (Serial.available() > 0) {
                char command = Serial.read();
                if (command == 's') {  // 's'키를 비상 정지로 사용
                    Serial.println("Emergency stop activated!");
                    stopMotors();
                    return;  // 함수 종료
                }
            }
            stepper1.run();
            stepper2.run();
            // stepper3.run();
            // stepper4.run();
        }
        // print encoder position per 50ms
        Serial.println(encoder_count);
    }

    Serial.println("Completed all movements based on array C.");
}



// Move single motor
void moveSingleMotor(int motorNum) {
    int* array;
    AccelStepper* stepper;
    
    switch(motorNum) {
        case 1: array = C1; stepper = &stepper1; break;
        case 2: array = C2; stepper = &stepper2; break;
        case 3: array = C3; stepper = &stepper3; break;
        case 4: array = C4; stepper = &stepper4; break;
        default: return;
    }
    
    for (int i = 0; i < arraySize1; i++) {
        long targetPosition = array[i] * 5;
        stepper->moveTo(targetPosition);

        unsigned long stepStartTime = millis();
        while (millis() - stepStartTime < 50) {
            if (Serial.available() > 0) {
                char command = Serial.read();
                if (command == 's') {
                    Serial.println("Emergency stop activated!");
                    stopMotors();
                    return;
                }
            }
            stepper->run();
        }
        Serial.println(encoder_count);
    }
    Serial.print("Completed movements for motor ");
    Serial.println(motorNum);
}


// 시간을 포맷팅하여 출력하는 함수 (밀리초를 시:분:초.밀리초 형식으로 변환)
void printFormattedTime(unsigned long timeInMs) {
    // 밀리초를 시, 분, 초로 변환
    unsigned long ms = timeInMs % 1000;
    unsigned long totalSeconds = timeInMs / 1000;
    unsigned long seconds = totalSeconds % 60;
    unsigned long minutes = (totalSeconds / 60) % 60;
    unsigned long hours = totalSeconds / 3600;
    
    // 시:분:초.밀리초 형식으로 출력
    if (hours > 0) {
        Serial.print(hours);
        Serial.print("시간 ");
    }
    
    if (minutes > 0 || hours > 0) {
        Serial.print(minutes);
        Serial.print("분 ");
    }
    
    Serial.print(seconds);
    Serial.print(".");
    
    // 밀리초는 항상 3자리로 표시
    if (ms < 10) Serial.print("00");
    else if (ms < 100) Serial.print("0");
    
    Serial.print(ms);
    Serial.println("초");
}


// C1 배열의 최대값과 최소값 계산
int maxC1 = -1000;
int minC1 = 1000;

// Min-Max calculation function
void calculateMaxMin() {
    for (int i = 0; i < arraySize1; i++) {
        if (C1[i] > maxC1) maxC1 = C1[i];
        if (C1[i] < minC1) minC1 = C1[i];
    }
    Serial.print("Max value in C1: ");
    Serial.println(maxC1);
    Serial.print("Min value in C1: ");
    Serial.println(minC1);
}


// 최대값과 최소값으로 모터를 반복 동작시키는 함수
void repeatMaxMin() {
    Serial.println("Repeating motor movements with max and min values of C1.");
    for (int repeat = 0; repeat < 10; repeat++) {
        // 최대값으로 동작
        long targetPositionMax = maxC1 * 5;
        stepper1.moveTo(targetPositionMax);
        stepper2.moveTo(-targetPositionMax);
//        stepper3.moveTo(targetPositionMax);
//        stepper4.moveTo(-targetPositionMax);

        while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0) {
            if (Serial.available() > 0) {
                char command = Serial.read();
                if (command == 's') {  // 's'키를 비상 정지로 사용
                    Serial.println("Emergency stop activated!");
                    stopMotors();
                    return;  // 함수 종료
                }
            }
            stepper1.run();
            stepper2.run();
            // stepper3.run();
            // stepper4.run();
        }
        Serial.println("Completed movement to max value.");
        Serial.println(encoder_count);

        // 최소값으로 동작
        long targetPositionMin = minC1 * 5;
        stepper1.moveTo(targetPositionMin);
        stepper2.moveTo(-targetPositionMin);
//        stepper3.moveTo(targetPositionMin);
//        stepper4.moveTo(-targetPositionMin);

        while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0) {
            if (Serial.available() > 0) {
                char command = Serial.read();
                if (command == 's') {  // 's'키를 비상 정지로 사용
                    Serial.println("Emergency stop activated!");
                    stopMotors();
                    return;  // 함수 종료
                }
            }
            stepper1.run();
            stepper2.run();
            // stepper3.run();
            // stepper4.run();
        }
        Serial.println("Completed movement to min value.");
        Serial.println(encoder_count);
    }
    stopMotors();
}


// 로터리 엔코더 시계방향(CW) 회전 감지 인터럽트 함수
void Encoder_z_CW() {
    if (digitalRead(encoderz_PinB) == LOW) {
        encoder_count++;
    }
    // else {
    //     encoder_count--;
    // }
}


// 로터리 엔코더 반시계방향(CCW) 회전 감지 인터럽트 함수
void Encoder_z_CCW() {
    if (digitalRead(encoderz_PinA) == LOW) {
        encoder_count--;
    }
    // else {
    //     encoder_count++;
    // }
}
