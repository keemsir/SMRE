/*
 * 4D Phantom Integrated Firmware (Medical Physics Researcher, Meangee Kim)
 * Integrated by Gemini
 * 
 * * Features:
 * 1. Mode Selection (DRIVER or FULL4WIRE)
 * 2. Manual Control for Zeroing (WinForm Compatible)
 * 3. Dynamic Array Loading for Respiratory Motion (WinForm Compatible)
 * 
 * Motor 1: X (LR), Motor 2: Y (SI), Motor 3: Z (AP), Motor 4: C (AP)
 */

#include <AccelStepper.h>

// ============================================================================
// [1] 하드웨어 모드 설정 (둘 중 하나만 주석 해제하여 사용)
// ============================================================================
//#define DRIVER_MODE       // A4988, DRV8825 등 드라이버 사용 시
#define FULL4WIRE_MODE    // L298N 등 4선식 모터 사용 시

// ============================================================================
// [2] 핀 맵 설정 (하드웨어에 맞게 변수값 채워넣기)
// ============================================================================
#ifdef DRIVER_MODE
  // --- DRIVER MODE PINS ---
  // Motor 1 (Lateral / X)
  const int M1_STEP = 22;  // 예시값 (실제 연결에 맞춰 수정)
  const int M1_DIR  = 24;
  
  // Motor 2 (SI / Y)
  const int M2_STEP = 30;
  const int M2_DIR  = 32;
  
  // Motor 3 (AP / Z)
  const int M3_STEP = 38;
  const int M3_DIR  = 40;
  
  // Motor 4 (Chest / C)
  const int M4_STEP = 46;
  const int M4_DIR  = 48;

#else
  // --- FULL4WIRE MODE PINS ---
  // Motor 1
  const int M1_IN1 = 2; const int M1_IN2 = 3; const int M1_IN3 = 4; const int M1_IN4 = 5;
  // Motor 2
  const int M2_IN1 = 6; const int M2_IN2 = 7; const int M2_IN3 = 8; const int M2_IN4 = 9;
  // Motor 3
  const int M3_IN1 = 0; const int M3_IN2 = 0; const int M3_IN3 = 0; const int M3_IN4 = 0;
  // Motor 4
  const int M4_IN1 = 0; const int M4_IN2 = 0; const int M4_IN3 = 0; const int M4_IN4 = 0;
#endif

// Encoder Pins
const int ENC_1_A = 19;
const int ENC_1_B = 18;

const int ENC_2_A = 19; 
const int ENC_2_B = 18;

// ============================================================================
// [3] 전역 변수 및 객체 생성
// ============================================================================

// 모터 객체 생성
#ifdef DRIVER_MODE
  AccelStepper stepper1(AccelStepper::DRIVER, M1_STEP, M1_DIR);
  AccelStepper stepper2(AccelStepper::DRIVER, M2_STEP, M2_DIR);
  AccelStepper stepper3(AccelStepper::DRIVER, M3_STEP, M3_DIR);
  AccelStepper stepper4(AccelStepper::DRIVER, M4_STEP, M4_DIR);
#else
  AccelStepper stepper1(AccelStepper::FULL4WIRE, M1_IN1, M1_IN3, M1_IN2, M1_IN4);
  AccelStepper stepper2(AccelStepper::FULL4WIRE, M2_IN1, M2_IN3, M2_IN2, M2_IN4);
  AccelStepper stepper3(AccelStepper::FULL4WIRE, M3_IN1, M3_IN3, M3_IN2, M3_IN4);
  AccelStepper stepper4(AccelStepper::FULL4WIRE, M4_IN1, M4_IN3, M4_IN2, M4_IN4);
#endif

// 호흡 데이터 버퍼
const int MAX_PATTERN_LENGTH = 200; 
int currentPatternLength = 0; // 현재 로드된 데이터 길이

// 데이터 배열
int Nx[MAX_PATTERN_LENGTH];
int Ny[MAX_PATTERN_LENGTH];
int Nz[MAX_PATTERN_LENGTH];
int Nc[MAX_PATTERN_LENGTH];

// 엔코더 변수
volatile long encoder1_count = 0; // 기존 Z축
volatile long encoder2_count = 0; // 추가된 축
unsigned long lastEncoderPrintTime = 0;
const int ENCODER_PRINT_INTERVAL = 50; // 50ms 주기

// 시스템 상태
bool isRunning = false;
bool emergencyStop = false;
bool encoderMonitorMode = false;
bool isRepeatMode = false; // 반복 재생 모드 플래그
bool isLoadingMode = false; // 260122_1 data loading flag

// ============================================================================
// [4] 함수 원형 선언
// ============================================================================
void loadDefaultData(); // 기본 호흡 데이터 로드
void checkSerial();     // 시리얼 명령어 처리
void runPattern();      // 패턴 재생 (Blocking with Interval)
void moveManual(int motorIdx, long steps); // 수동 이동
void goHome(int motorIdx); // 0점 복귀 (Homing)
void stopAllMotors();   // 비상 정지
void printEncoderData(); // 엔코더 데이터 전송
void Encoder_ISR_A();   // 엔코더 인터럽트 A
void Encoder_ISR_B();   // 엔코더 인터럽트 B

// ============================================================================
// [5] Setup
// ============================================================================
void setup() {
  Serial.begin(115200); // 115200, 250000
  
  // 모터 속도/가속도 초기 설정
  float defaultMaxSpeed = 1600.0;
  float defaultAccel = 800.0;

  stepper1.setMaxSpeed(defaultMaxSpeed); stepper1.setAcceleration(defaultAccel);
  stepper2.setMaxSpeed(defaultMaxSpeed); stepper2.setAcceleration(defaultAccel);
  stepper3.setMaxSpeed(1000.0);          stepper3.setAcceleration(500.0);
  stepper4.setMaxSpeed(1000.0);          stepper4.setAcceleration(500.0);

// Encoder 1 config
  pinMode(ENC_1_A, INPUT_PULLUP);
  pinMode(ENC_1_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_1_A), Encoder1_ISR_A, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_1_B), Encoder1_ISR_B, RISING);

  // Encoder 2 config
  pinMode(ENC_2_A, INPUT_PULLUP);
  pinMode(ENC_2_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_2_A), Encoder2_ISR_A, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_2_B), Encoder2_ISR_B, RISING);

  // 기본 호흡 데이터 메모리에 로드
  loadDefaultData();

  Serial.println("SYSTEM_READY");
  
  // 모드 출력
  Serial.print("MODE: ");
  #ifdef DRIVER_MODE
    Serial.println("DRIVER");
  #else
    Serial.println("FULL4WIRE");
  #endif
}

// ============================================================================
// [6] Main Loop
// ============================================================================
void loop() {
  // 1. 시리얼 명령어 확인
  checkSerial();

  // 2. 비상 정지 확인
  if (emergencyStop) {
    stopAllMotors();
    Serial.println("STATUS:EMERGENCY_STOP");
    emergencyStop = false; 
    isRunning = false; // 실행 상태 해제
    isLoadingMode = false;
  }

  // 3. 엔코더 데이터 주기적 전송
  if (encoderMonitorMode && !isLoadingMode) {
    printEncoderData();
  }
}

// ============================================================================
// [7] 주요 기능 함수 구현
// ============================================================================

// --- 엔코더 데이터 전송 ---
void printEncoderData() {
  if (millis() - lastEncoderPrintTime >= ENCODER_PRINT_INTERVAL) {
    Serial.print("E,");
    Serial.print(millis());
    Serial.print(",");
    
    // 축 1 (X): 엔코더 없음 -> 현재 스텝 위치(가상값) 출력
    Serial.print(stepper1.currentPosition());
    Serial.print(",");
    
    // 축 2 (Y): 엔코더 없음 -> 현재 스텝 위치(가상값) 출력
    Serial.print(stepper2.currentPosition());
    Serial.print(",");
    
    // 축 3 (Z): 실제 엔코더 1 값 출력
    Serial.print(encoder1_count);
    Serial.print(",");
    
    // 축 4 (C): 실제 엔코더 2 값 출력
    Serial.println(encoder2_count);
    
    lastEncoderPrintTime = millis();
  }
}

// --- 시리얼 통신 및 명령어 처리 (WinForm Protocol) ---
void checkSerial() {
  if (Serial.available() > 0) {
    // 버퍼 읽기 전 잠시 대기 (데이터 패킷 완성 대기)
    // delay(2); 
    char cmd = Serial.read();
    
    // S: Emergency Stop
    if (cmd == 's' || cmd == 'S') {
      emergencyStop = true;
      return;
    }

    // P: Play Pattern
    if (cmd == 'p' || cmd == 'P') {
      if (!isRunning) { // 이미 실행 중이 아닐 때만 시작
        Serial.println("STATUS:PLAYING");
        runPattern();
        Serial.println("STATUS:FINISHED");
      }
    }

    // R: Toggle Repeat Mode
    if (cmd == 'r' || cmd == 'R') {
      isRepeatMode = !isRepeatMode;
      Serial.print("REPEAT_MODE:");
      Serial.println(isRepeatMode ? "ON" : "OFF");
    }

    // M: Manual Move (형식: M1 200)
    if (cmd == 'm' || cmd == 'M') {
      int motorID = Serial.parseInt();
      long steps = Serial.parseInt();
      moveManual(motorID, steps);
    }

    // H: Homing (형식: H1)
    if (cmd == 'h' || cmd == 'H') {
      int motorID = Serial.parseInt();
      goHome(motorID);
    }
    
    // W: Encoder Monitor Toggle
    if (cmd == 'w' || cmd == 'W') {
      encoderMonitorMode = !encoderMonitorMode;
      Serial.println(encoderMonitorMode ? "MONITOR:ON" : "MONITOR:OFF");
    }

    // data loading
    // L: Load Data (Reserved)
    if (cmd == 'l' || cmd == 'L') {
       Serial.println("INFO:READY_TO_LOAD");
    }
  }
}

// --- 호흡 패턴 재생 (수정됨: 확실한 비상정지 처리) ---
void runPattern() {
  isRunning = true;
  unsigned long interval = 100; 
  
  do {
    for (int i = 0; i < currentPatternLength; i++) {
      
      // 1. 명령 확인 및 비상정지 (외부 루프)
      while (Serial.available() > 0) {
        char c = Serial.read();
        if (c == 's' || c == 'S') { emergencyStop = true; }
      }
      if (emergencyStop) break;

      stepper1.moveTo(Nx[i]);
      stepper2.moveTo(Ny[i]);
      stepper3.moveTo(Nz[i]);
      stepper4.moveTo(Nc[i]);

      unsigned long stepStart = millis();
      
      // 2. 인터벌 동안 모터 구동 (내부 루프)
      while (millis() - stepStart < interval) {
        stepper1.run();
        stepper2.run();
        stepper3.run();
        stepper4.run();

        printEncoderData(); 

        // 3. 실시간 비상 정지 체크 (버퍼를 모두 읽어서 's' 확인)
        if (Serial.available() > 0) {
           while(Serial.available() > 0){
             char c = Serial.read(); // 버퍼 비우기
             if (c == 's' || c == 'S') {
               emergencyStop = true;
             }
           }
        }
        
        if (emergencyStop) break; // while 탈출
      }
      
      if (emergencyStop) break; // for 루프 탈출
    }
    
    if (emergencyStop) break; // do-while 루프 탈출

  } while (isRepeatMode); 
  
  isRunning = false;
}

// --- 수동 이동 ---
void moveManual(int motorIdx, long steps) {
  AccelStepper* targetStepper;
  switch (motorIdx) {
    case 1: targetStepper = &stepper1; break;
    case 2: targetStepper = &stepper2; break;
    case 3: targetStepper = &stepper3; break;
    case 4: targetStepper = &stepper4; break;
    default: return;
  }
  
  Serial.print("MANUAL:M"); Serial.print(motorIdx); Serial.print(" GO "); Serial.println(steps);
  
  targetStepper->move(steps);
  while (targetStepper->distanceToGo() != 0) {
    targetStepper->run();
    
    // 수동 이동 중 비상 정지 체크
    if (Serial.available() > 0) {
       char c = Serial.read();
       if (c == 's' || c == 'S') {
         emergencyStop = true;
         return;
       }
    }
  }
}

// --- 원점 설정 ---
void goHome(int motorIdx) {
  switch (motorIdx) {
    case 1: stepper1.setCurrentPosition(0); break;
    case 2: stepper2.setCurrentPosition(0); break;
    case 3: stepper3.setCurrentPosition(0); break;
    case 4: stepper4.setCurrentPosition(0); break;
  }
  Serial.print("HOME:SET_ZERO M"); Serial.println(motorIdx);
}

// --- 비상 정지 ---
void stopAllMotors() {
  stepper1.stop(); stepper2.stop(); stepper3.stop(); stepper4.stop();
  stepper1.runToPosition();
  stepper2.runToPosition();
  stepper3.runToPosition();
  stepper4.runToPosition();
}

// [기존] 엔코더 1 (축 3) 인터럽트
void Encoder1_ISR_A() {
  if (digitalRead(ENC_1_B) == LOW) encoder1_count++; else encoder1_count--;
}
void Encoder1_ISR_B() {
  if (digitalRead(ENC_1_A) == LOW) encoder1_count--; else encoder1_count++;
}

// [추가] 엔코더 2 (축 4) 인터럽트
void Encoder2_ISR_A() {
  if (digitalRead(ENC_2_B) == LOW) encoder2_count++; else encoder2_count--;
}
void Encoder2_ISR_B() {
  if (digitalRead(ENC_2_A) == LOW) encoder2_count--; else encoder2_count++;
}

// ============================================================================
// [8] 데이터 로드 (Default)
// ============================================================================
void loadDefaultData() {
  // 기존 코드의 데이터 (예시로 일부만 넣거나, 기존 데이터를 그대로 복사)
  // WinForm이 개발되면 이 부분은 비워두고 Serial 통신으로 채워 넣게 됩니다.
  
  // WinForm에서 전송하도록 변경해야 함)
  int tempNx[] = {0, 6, 8, 32, 60, 82, 92, 100, 108, 112, 110, 72, 16, 20, 44, 56, 66, 74, 80, 88, 92, 98, 68, 16};
  int tempNy[] = {0, 6, 8, 32, 60, 82, 92, 100, 108, 112, 110, 72, 16, 20, 44, 56, 66, 74, 80, 88, 92, 98, 68, 16};
  int tempNz[] = {0, 6, 8, 32, 60, 82, 92, 100, 108, 112, 110, 72, 16, 20, 44, 56, 66, 74, 80, 88, 92, 98, 68, 16};
  int tempNc[] = {0, 6, 8, 32, 60, 82, 92, 100, 108, 112, 110, 72, 16, 20, 44, 56, 66, 74, 80, 88, 92, 98, 68, 16};

//  int tempNx[] = {0, -33, -67, -100, -133, -160}; //
//  int tempNy[] = {0, 207, 413, 620, 827, 893};    //
//  int tempNz[] = {0, -1, -3, -4, -5, -30};        //
//  int tempNc[] = {0, 20, 50, 90, 129, 179};       //

  // 배열 길이 계산
  currentPatternLength = sizeof(tempNx) / sizeof(tempNx[0]);

  // 전역 버퍼로 복사
  for (int i = 0; i < currentPatternLength; i++) {
    Nx[i] = tempNx[i];
    Ny[i] = tempNy[i];
    Nz[i] = tempNz[i];
    Nc[i] = tempNc[i];
  }
}
