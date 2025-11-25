// 기존 수시변경 print encoder position -> print encoder position per 50ms 로 변경 
// Motor 1: X (LR), Motor 2: Y (SI), Motor 3: Z (AP), Motor 4: C (AP)

#include <AccelStepper.h>

// Config motor pin (DRIVER Mode) - Motor 1 (X), Motor 2 (Y)
const int motor1_stepPin = 38; // Step pin
const int motor1_dirPin = 40; // Direction pin

const int motor2_stepPin = 46; // Step pin
const int motor2_dirPin = 48; // Direction pin

// Config motor pin (FULL4WIRE Mode) - Motor 3 (Z), Motor 4 (C)
// NOTE: The pin assignments for 4-wire mode must be correct for the hardware setup.
// Assuming the motor3 and motor4 pin setups are placeholders or correct for the user's specific setup.
const int motor3_pin_1 = 22; // motor3_IN1
const int motor3_pin_2 = 24; // motor3_IN2
const int motor3_pin_3 = 26; // motor3_IN3
const int motor3_pin_4 = 28; // motor3_IN4

const int motor4_pin_1 = 30; // motor4_IN1
const int motor4_pin_2 = 32; // motor4_IN2
const int motor4_pin_3 = 34; // motor4_IN3
const int motor4_pin_4 = 36; // motor4_IN4


// 로터리 엔코더 핀 설정
const int encoderz_PinA = 19;
const int encoderz_PinB = 18;

// const int encoderc_PinA = 21; // Unused in this sketch
// const int encoderc_PinB = 20; // Unused in this sketch

// 엔코더 모니터링 모드 
bool encoderMonitorMode = false;

// 엔코더 카운터 변수
volatile long encoder_count = 0;
volatile long last_encoder_count = 0; 

// 엔코더 상태 출력 시간 관리
unsigned long lastEncoderPrintTime = 0;
const unsigned long encoderPrintInterval = 50; // 50ms마다 엔코더 값 출력 (변경 요구 사항 반영)


// 모터 동작 상태 변수
bool isRunning = false;
bool emergencyStop = false;

// AccelStepper 객체 생성
AccelStepper stepper1(AccelStepper::DRIVER, motor1_stepPin, motor1_dirPin); // Motor 1 (X): DRIVER
AccelStepper stepper2(AccelStepper::DRIVER, motor2_stepPin, motor2_dirPin); // Motor 2 (Y): DRIVER
// Renamed pins to avoid conflict, assuming standard 4-wire setup for NEMA 17/23 style
AccelStepper stepper3(AccelStepper::FULL4WIRE, motor3_pin_1, motor3_pin_3, motor3_pin_2, motor3_pin_4); // Motor 3 (Z): FULL4WIRE
AccelStepper stepper4(AccelStepper::FULL4WIRE, motor4_pin_1, motor4_pin_3, motor4_pin_2, motor4_pin_4); // Motor 4 (C): FULL4WIRE

// ==============================================================================
// 1. New Position Data (Converted to STEPS from 0.1mm units)
// **Position data is in 0.1mm units, so we divide by 10 to get mm.**
// Conversion Formulas (mm -> Steps):
// Dx' = Dx / 10 (mm)
// Ny' = Ny / 10 (mm)
// Nz' = Nz / 10 (mm)
// Nc' = Nc / 10 (mm)
// 
// Step Calculation:
// Nx = round(Dx' / 0.006) = round(Dx / 0.06)
// Ny = round(Ny' / 0.006) = round(Dy / 0.06)
// Nz = round(Nz' / 0.0184) = round(Hz / 0.184)
// Nc = round(Nc' / 0.0201) = round(Hc / 0.201)
// Note: Steps are rounded to the nearest integer.
// ==============================================================================

// Motor 1 (X) Steps: Dx/0.06
int Nx[] = {0, -33, -67, -100, -133, -160, -187, -213, -240, -100, 40, 180, 320, 247, 173, 100, 27, 40, 53, 67, 80, 53, 27, 0, -27, -27, 47, 120, 193, 267, 280, 293, 307, 320, 207, 93, -20, -133, -100, -67, -33};
// Motor 2 (Y) Steps: Dy/0.06
int Ny[] = {0, 207, 413, 620, 827, 893, 960, 1027, 1093, 1087, 1080, 1073, 1067, 1073, 1080, 1087, 1093, 1180, 1267, 1353, 1440, 1427, 1413, 1400, 1387, 1387, 1353, 1320, 1287, 1253, 1260, 1267, 1273, 1280, 1027, 773, 520, 267, 200, 133, 67};
// Motor 3 (Z) Steps: Hz/0.184
int Nz[] = {0, -1, -3, -4, -5, -30, -54, -79, -103, -118, -133, -148, -163, -249, -334, -420, -505, -492, -478, -465, -451, -313, -174, -35, 103, 103, 11, -82, -174, -266, -254, -242, -230, -217, -167, -117, -67, -16, -12, -8, -4};
// Motor 4 (C) Steps: Hc/0.201
int Nc[] = {0, 20, 50, 90, 129, 179, 239, 299, 338, 388, 448, 517, 607, 657, 697, 726, 746, 786, 806, 816, 826, 846, 856, 866, 886, 886, 856, 816, 766, 697, 597, 507, 388, 328, 269, 209, 149, 100, 60, 30, 10};


// 배열 크기 계산 (모든 배열의 크기는 동일해야 함)
const int arraySize = sizeof(Nx) / sizeof(Nx[0]);

// 작동 시간 측정 변수
unsigned long startTime = 0;
unsigned long endTime = 0;

// 함수 선언
void stopMotors();
void printFormattedTime(unsigned long timeInMs);
void calculateMaxMin();
void repeatMaxMin();
void moveMotorByArray();
void moveSingleMotor(int motorNum);
void checkSerialInput();
void printEncoderPositionPeriodically();
void Encoder_z_CW();
void Encoder_z_CCW();


void setup() {
    Serial.begin(115200);
    Serial.println("-------------------------------------");
    Serial.println("Stepper Motor and Encoder Initialized");

    // 모터 속도 및 가속 설정 (가속 및 속도는 하드웨어에 맞춰 조정 필요)
    stepper1.setMaxSpeed(1600.0);    // 최대 속도 (steps per second)
    stepper1.setAcceleration(800.0); // 가속도 (steps per second^2)

    stepper2.setMaxSpeed(1600.0);    // 최대 속도 (steps per second)
    stepper2.setAcceleration(800.0); // 가속도 (steps per second^2)

    stepper3.setMaxSpeed(1000.0);    // 최대 속도 (steps per second)
    stepper3.setAcceleration(500.0); // 가속도 (steps per second^2)

    stepper4.setMaxSpeed(1000.0);    // 최대 속도 (steps per second)
    stepper4.setAcceleration(500.0); // 가속도 (steps per second^2)

    // 배열 길이 일치 여부 확인
    if (sizeof(Nx) / sizeof(Nx[0]) == arraySize &&
        sizeof(Ny) / sizeof(Ny[0]) == arraySize &&
        sizeof(Nz) / sizeof(Nz[0]) == arraySize &&
        sizeof(Nc) / sizeof(Nc[0]) == arraySize) {
        Serial.println("All arrays have the same length.");
    } else {
        Serial.println("Error: Array lengths do not match! Check the new data arrays.");
    }

    // Config a rotary encdoer pin
    pinMode(encoderz_PinA, INPUT_PULLUP);
    pinMode(encoderz_PinB, INPUT_PULLUP);
    
    // Config the interrupt
    attachInterrupt(digitalPinToInterrupt(encoderz_PinA), Encoder_z_CW, RISING);
    attachInterrupt(digitalPinToInterrupt(encoderz_PinB), Encoder_z_CCW, RISING);

    // 배열의 이론적 작동 시간 계산 및 출력 (100ms * arraySize)
    int theoreticalTime = arraySize * 100; // 각 위치당 100ms로 변경
    Serial.print("Expected operation time: ");
    printFormattedTime(theoreticalTime);

    // Min-Max calculation (Only for Motor 1 (X) array Nx)
    calculateMaxMin();

    Serial.println("Available commands:");
    Serial.println("a: Operate all motors simultaneously (100ms interval)");
    Serial.println("1: Operate only motor 1 (X)");
    Serial.println("2: Operate only motor 2 (Y)");
    Serial.println("3: Operate only motor 3 (Z)");
    Serial.println("4: Operate only motor 4 (C)");
    Serial.println("e: Print current encoder value once");
    Serial.println("r: Reset encoder value");
    Serial.println("w: Encoder toggle on/off (Prints changes every 50ms when ON)");
    Serial.println("s: Emergency stop !!");
    Serial.println("m: Repeat motor movements with max and min values of Nx");

}


void loop() {
    
    // 모터가 동작 중이 아닐 때만 주기적 엔코더 출력 확인
    if (!isRunning) {
        printEncoderPositionPeriodically();
    }
    
    // 시리얼 입력 확인 및 처리
    checkSerialInput();

    // 비상 정지 상태 확인 (loop 시작 시점에서는 checkSerialInput에서 처리하도록 변경)
    if (emergencyStop) {
        // 비상 정지 상태일 때는 모터 동작 중지
        stopMotors();
        isRunning = false;
        Serial.println("Emergency stop activated. Motors stopped.");
        emergencyStop = false; // 비상 정지 상태 해제
        Serial.println("Ready for new commands.");
    }
}

// 50ms마다 엔코더 값 출력 함수
void printEncoderPositionPeriodically() {
    if (encoderMonitorMode) {
        unsigned long currentTime = millis();
        if (currentTime - lastEncoderPrintTime >= encoderPrintInterval) {
            // 엔코더 값이 변경되었는지 확인
            if (encoder_count != last_encoder_count) {
                Serial.print("E,");
                Serial.print(currentTime);
                Serial.print(",");
                Serial.println(encoder_count);
                last_encoder_count = encoder_count;
            }
            lastEncoderPrintTime = currentTime;
        }
    }
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
        if (!isRunning) { // 모터가 동작 중이 아닐 때만 다른 명령 처리
            if (command == 'a') {
                Serial.println("Moving all motors simultaneously (100ms interval)");
                
                // 시작 시간 기록
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
                printFormattedTime(totalTime);
            }
            else if (command == 'e') {
                // 엔코더 값 출력 명령
                Serial.print("Current encoder count: ");
                Serial.println(encoder_count);
            }
            else if (command == 'r') {
                // 엔코더 값 리셋 명령
                encoder_count = 0;
                last_encoder_count = 0;
                Serial.println("Encoder count reset to 0");
            }
            else if (command == '1') {
                // 모터 1(X)만 작동
                Serial.println("Moving only motor 1 (X)");
                startTime = millis();
                isRunning = true;
                Serial.println("Time(ms), Encoder Value");
                moveSingleMotor(1);
                isRunning = false;
                endTime = millis();

                unsigned long totalTime = endTime - startTime;
                Serial.print("Total execution time: ");
                printFormattedTime(totalTime);
            }
            else if (command == '2') {
                // 모터 2(Y)만 작동
                Serial.println("Moving only motor 2 (Y)");
                startTime = millis();
                isRunning = true;
                Serial.println("Time(ms), Encoder Value");
                moveSingleMotor(2);
                isRunning = false;
                endTime = millis();

                unsigned long totalTime = endTime - startTime;
                Serial.print("Total execution time: ");
                printFormattedTime(totalTime);
            }

            else if (command == '3') {
                // 모터 3(Z)만 작동
                Serial.println("Moving only motor 3 (Z)");
                startTime = millis();
                isRunning = true;
                Serial.println("Time(ms), Encoder Value");
                moveSingleMotor(3);
                isRunning = false;
                endTime = millis();

                unsigned long totalTime = endTime - startTime;
                Serial.print("Total execution time: ");
                printFormattedTime(totalTime);
            }
            else if (command == '4') {
                // 모터 4(C)만 작동
                Serial.println("Moving only motor 4 (C)");
                startTime = millis();
                isRunning = true;
                Serial.println("Time(ms), Encoder Value");
                moveSingleMotor(4);
                isRunning = false;
                endTime = millis();

                unsigned long totalTime = endTime - startTime;
                Serial.print("Total execution time: ");
                printFormattedTime(totalTime);
            }

            else if (command == 'm') { // 'm' 입력 시 반복 동작 실행 (Motor 1: X)
                repeatMaxMin();
            }
            
            else if (command == 'w') {
                // 엔코더 값 변화 모니터링 모드 토글
                encoderMonitorMode = !encoderMonitorMode;
                if (encoderMonitorMode) {
                    Serial.println("Encoder monitoring mode: ON (Prints changes every 50ms)");
                    lastEncoderPrintTime = millis(); // 모니터링 시작 시간 초기화
                    last_encoder_count = encoder_count;
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
    
    // 모터가 완전히 멈출 때까지 run() 함수 호출 (비상 정지 시에는 대기하지 않도록 수정)
    // 비상 정지는 run() 함수가 호출되는 루프 외부에서 즉시 실행되어야 하므로, run()을 한번만 호출
    stepper1.run();
    stepper2.run();
    stepper3.run();
    stepper4.run();
    
    // 모터 출력 비활성화 (필요한 경우)
    // stepper1.disableOutputs();
    // stepper2.disableOutputs();
    // stepper3.disableOutputs();
    // stepper4.disableOutputs();
}


// 배열에 따라 모터 움직임 구현 함수
void moveMotorByArray() {
    const unsigned long moveInterval = 100; // 100ms 간격으로 다음 위치로 이동
    
    for (int i = 0; i < arraySize; i++) {
        // 배열 값은 이미 스텝 수로 변환되었으므로, 바로 moveTo에 사용
        stepper1.moveTo(Nx[i]); // Motor 1 (X)
        stepper2.moveTo(Ny[i]); // Motor 2 (Y)
        stepper3.moveTo(Nz[i]); // Motor 3 (Z)
        stepper4.moveTo(Nc[i]); // Motor 4 (C)

        unsigned long stepStartTime = millis();
        
        // 100ms 동안 비차단 방식으로 모터를 부드럽게 움직임
        while (millis() - stepStartTime < moveInterval) {
            
            // 이 루프 안에서도 시리얼 입력 확인 (비상 정지를 위한 필수)
            if (Serial.available() > 0) {
                char command = Serial.read();
                if (command == 's') { // 's'키를 비상 정지로 사용
                    Serial.println("Emergency stop activated!");
                    stopMotors();
                    return; // 함수 종료
                }
            }
            
            // AccelStepper run()을 지속적으로 호출하여 움직임 처리
            stepper1.run();
            stepper2.run();
            stepper3.run();
            stepper4.run();
            
            // 50ms마다 엔코더 위치 출력 (모터 동작 중에는 50ms 주기로 엔코더 값 출력)
            unsigned long currentTime = millis();
            if (currentTime - lastEncoderPrintTime >= encoderPrintInterval) {
                Serial.print(currentTime - startTime);
                Serial.print(",");
                Serial.println(encoder_count);
                lastEncoderPrintTime = currentTime;
            }
        }
    }

    Serial.println("Completed all movements based on array data.");
}



// 단일 모터 움직임 함수
void moveSingleMotor(int motorNum) {
    int* array;
    AccelStepper* stepper;
    const unsigned long moveInterval = 100; // 100ms 간격으로 다음 위치로 이동

    switch(motorNum) {
        case 1: array = Nx; stepper = &stepper1; break;
        case 2: array = Ny; stepper = &stepper2; break;
        case 3: array = Nz; stepper = &stepper3; break;
        case 4: array = Nc; stepper = &stepper4; break;
        default: return;
    }
    
    for (int i = 0; i < arraySize; i++) {
        long targetPosition = array[i]; // 이미 스텝 수로 변환됨
        stepper->moveTo(targetPosition);

        unsigned long stepStartTime = millis();
        while (millis() - stepStartTime < moveInterval) {
            if (Serial.available() > 0) {
                char command = Serial.read();
                if (command == 's') {
                    Serial.println("Emergency stop activated!");
                    stopMotors();
                    return;
                }
            }
            stepper->run();
            
            // 50ms마다 엔코더 위치 출력
            unsigned long currentTime = millis();
            if (currentTime - lastEncoderPrintTime >= encoderPrintInterval) {
                Serial.print(currentTime - startTime);
                Serial.print(",");
                Serial.println(encoder_count);
                lastEncoderPrintTime = currentTime;
            }
        }
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


// Nx 배열의 최대값과 최소값 계산
int maxNx = -100000;
int minNx = 100000;

// Min-Max calculation function (Only for Motor 1 (X) array Nx)
void calculateMaxMin() {
    for (int i = 0; i < arraySize; i++) {
        if (Nx[i] > maxNx) maxNx = Nx[i];
        if (Nx[i] < minNx) minNx = Nx[i];
    }
    Serial.print("Max step value in Nx (Motor 1): ");
    Serial.println(maxNx);
    Serial.print("Min step value in Nx (Motor 1): ");
    Serial.println(minNx);
}


// 최대값과 최소값으로 모터 1(X)와 2(Y)를 반복 동작시키는 함수
void repeatMaxMin() {
    Serial.println("Repeating motor 1 and 2 movements with max and min values of Nx.");
    for (int repeat = 0; repeat < 10; repeat++) {
        // 최대값으로 동작
        long targetPositionMax = maxNx;
        stepper1.moveTo(targetPositionMax);
        // Motor 2는 Motor 1과 반대 방향으로 움직이도록 설정 (예시)
        stepper2.moveTo(-targetPositionMax); 

        while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0) {
            if (Serial.available() > 0) {
                char command = Serial.read();
                if (command == 's') { // 's'키를 비상 정지로 사용
                    Serial.println("Emergency stop activated!");
                    stopMotors();
                    return; // 함수 종료
                }
            }
            stepper1.run();
            stepper2.run();
        }
        Serial.println("Completed movement to max value.");
        Serial.print("Encoder Count: ");
        Serial.println(encoder_count);

        // 최소값으로 동작
        long targetPositionMin = minNx;
        stepper1.moveTo(targetPositionMin);
        stepper2.moveTo(-targetPositionMin);

        while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0) {
            if (Serial.available() > 0) {
                char command = Serial.read();
                if (command == 's') { // 's'키를 비상 정지로 사용
                    Serial.println("Emergency stop activated!");
                    stopMotors();
                    return; // 함수 종료
                }
            }
            stepper1.run();
            stepper2.run();
        }
        Serial.println("Completed movement to min value.");
        Serial.print("Encoder Count: ");
        Serial.println(encoder_count);
    }
    stopMotors();
    Serial.println("Max/Min repetition finished.");
}


// 로터리 엔코더 시계방향(CW) 회전 감지 인터럽트 함수
void Encoder_z_CW() {
    if (digitalRead(encoderz_PinB) == LOW) {
        encoder_count++;
    }
}


// 로터리 엔코더 반시계방향(CCW) 회전 감지 인터럽트 함수
void Encoder_z_CCW() {
    if (digitalRead(encoderz_PinA) == LOW) {
        encoder_count--;
    }
}
