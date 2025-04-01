// 

#include <AccelStepper.h>

// (DRIVER Mode)
const int motor1_stepPin = 38; // Step pin
const int motor1_dirPin = 40; // Direction pin

const int motor2_stepPin = 30; // Step pin
const int motor2_dirPin = 32; // Direction pin

// 모터 핀 설정 (FULL4WIRE Mode)
const int motor1_pin_1 = 2; // motor1_IN1
const int motor1_pin_2 = 3; // motor1_IN2
const int motor1_pin_3 = 4; // motor1_IN3
const int motor1_pin_4 = 5; // motor1_IN4

const int motor2_pin_1 = 6; // motor2_IN1
const int motor2_pin_2 = 7; // motor2_IN2
const int motor2_pin_3 = 8; // motor2_IN3
const int motor2_pin_4 = 9; // motor2_IN4

// 로터리 엔코더 핀 설정
const int encoderz_PinA = 19;
const int encoderz_PinB = 18;

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
AccelStepper stepper1(AccelStepper::DRIVER, motor1_stepPin, motor1_dirPin);
AccelStepper stepper2(AccelStepper::DRIVER, motor2_stepPin, motor2_dirPin);

// Array C
int C1[] = {24, 26, 26, 30, 32, 36, 40, 44, 48, 50, 54, 56, 58, 60, 60, 62, 64, 70, 76, 80, 84, 86, 88, 94, 98, 102, 108, 112, 116, 120, 126, 130, 130, 134, 138, 144, 150, 150, 152, 154, 156, 158, 162, 164, 168, 168, 170, 172, 174, 172, 174, 176, 180, 184, 186, 186, 186, 186, 186, 184, 180, 176, 172, 168, 164, 160, 154, 150, 142, 138, 134, 130, 124, 114, 108, 104, 100, 98, 92, 86, 80, 74, 68, 60, 56, 52, 48, 44, 40, 38, 32, 28, 24, 22, 20, 18, 16, 16, 16, 16, 16, 18, 22, 24, 28, 34, 40, 40, 44, 46, 50, 54, 56, 58, 62, 66, 68, 70, 76, 80, 80, 82, 92, 102, 104, 108, 110, 112, 114, 120, 128, 130, 134, 134, 138, 142, 144, 146, 146, 150, 154, 156, 158, 162, 160, 164, 166, 168, 172, 174, 174, 176, 178, 178, 176, 170, 164, 160, 156, 154, 148, 142, 136, 132, 126, 122, 114, 108, 104, 98, 94, 88, 84, 76, 72, 66, 62, 60, 54, 52, 48, 44, 40, 36, 32, 28, 24, 20, 16, 14, 10, 6, 6, 8, 12, 14, 16, 18, 24, 28, 32, 38, 40, 44, 46, 48, 52, 56, 56, 56, 60, 64, 68, 74, 78, 80, 84, 88, 90, 96, 98, 102, 104, 106, 108, 110, 114, 118, 126, 134, 138, 144, 144, 148, 150, 152, 156, 158, 160, 162, 164, 166, 168, 168, 168, 170, 174, 178, 180, 182, 184, 184, 188, 184, 184, 182, 174, 170, 164, 164, 158, 154, 146, 140, 128, 122, 116, 108, 98, 92, 84, 78, 74, 68, 62, 58, 56, 54, 50, 44, 42, 38, 36, 34, 32, 30, 26, 26, 22, 20, 20, 20, 20};

int C2[] = {24, 26, 26, 30, 32, 36, 40, 44, 48, 50, 54, 56, 58, 60, 60, 62, 64, 70, 76, 80, 84, 86, 88, 94, 98, 102, 108, 112, 116, 120, 126, 130, 130, 134, 138, 144, 150, 150, 152, 154, 156, 158, 162, 164, 168, 168, 170, 172, 174, 172, 174, 176, 180, 184, 186, 186, 186, 186, 186, 184, 180, 176, 172, 168, 164, 160, 154, 150, 142, 138, 134, 130, 124, 114, 108, 104, 100, 98, 92, 86, 80, 74, 68, 60, 56, 52, 48, 44, 40, 38, 32, 28, 24, 22, 20, 18, 16, 16, 16, 16, 16, 18, 22, 24, 28, 34, 40, 40, 44, 46, 50, 54, 56, 58, 62, 66, 68, 70, 76, 80, 80, 82, 92, 102, 104, 108, 110, 112, 114, 120, 128, 130, 134, 134, 138, 142, 144, 146, 146, 150, 154, 156, 158, 162, 160, 164, 166, 168, 172, 174, 174, 176, 178, 178, 176, 170, 164, 160, 156, 154, 148, 142, 136, 132, 126, 122, 114, 108, 104, 98, 94, 88, 84, 76, 72, 66, 62, 60, 54, 52, 48, 44, 40, 36, 32, 28, 24, 20, 16, 14, 10, 6, 6, 8, 12, 14, 16, 18, 24, 28, 32, 38, 40, 44, 46, 48, 52, 56, 56, 56, 60, 64, 68, 74, 78, 80, 84, 88, 90, 96, 98, 102, 104, 106, 108, 110, 114, 118, 126, 134, 138, 144, 144, 148, 150, 152, 156, 158, 160, 162, 164, 166, 168, 168, 168, 170, 174, 178, 180, 182, 184, 184, 188, 184, 184, 182, 174, 170, 164, 164, 158, 154, 146, 140, 128, 122, 116, 108, 98, 92, 84, 78, 74, 68, 62, 58, 56, 54, 50, 44, 42, 38, 36, 34, 32, 30, 26, 26, 22, 20, 20, 20, 20};

// 배열 크기 계산
const int arraySize = sizeof(C1) / sizeof(C1[0]);
//const int arraySize2 = sizeof(C1) / sizeof(C1[0]);

// 작동 시간 측정 변수
unsigned long startTime = 0;
unsigned long endTime = 0;

// 엔코더 상태 출력 시간 관리
//unsigned long lastEncoderPrintTime = 0;
//const unsigned long encoderPrintInterval = 50; // default: 500ms마다 엔코더 값 출력

void setup() {
    Serial.begin(115200);
    Serial.println("Stepper Motor and Encoder Initialized");

    // 모터 속도 및 가속 설정
    stepper1.setMaxSpeed(1000);      // 최대 속도 (steps per second)
    stepper1.setAcceleration(500);   // 가속도 (steps per second^2)

    stepper2.setMaxSpeed(1000);      // 최대 속도 (steps per second)
    stepper2.setAcceleration(500);   // 가속도 (steps per second^2)

    // Config a rotary encdoer pin
    pinMode(encoderz_PinA, INPUT_PULLUP);
    pinMode(encoderz_PinB, INPUT_PULLUP);
    
    // Config the interrupt
    attachInterrupt(digitalPinToInterrupt(encoderz_PinA), Encoder_z_CW, RISING);
    attachInterrupt(digitalPinToInterrupt(encoderz_PinB), Encoder_z_CCW, RISING);

    // C 배열의 이론적 작동 시간 계산 및 출력
    int theoreticalTime = arraySize * 50;  // 각 위치당 50ms
    Serial.print("이론적 작동 시간: ");
    printFormattedTime(theoreticalTime);

    Serial.println("사용 가능한 명령어:");
    Serial.println("c: 배열 C에 따라 모터 움직임 실행");
    Serial.println("1: 모터 1만 작동");
    Serial.println("2: 모터 2만 작동");
    Serial.println("e: 현재 엔코더 값 출력");
    Serial.println("r: 엔코더 값 리셋");
    Serial.println("w: Print encoder toggle on/off");
    Serial.println("s: 비상 정지 (모터 동작 중 언제든지 입력)");

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

    // 모터 실행
    stepper1.run();
    stepper2.run();
    
    // 50ms마다 엔코더 값 출력
    if (millis() - lastEncoderPrintTime >= encoderPrintInterval) {
        // 엔코더 값과 시간을 바로 Serial Monitor에 출력
        if (isRunning) {
            unsigned long currentTime = millis() - startTime;
            Serial.print(currentTime);
            Serial.print(", ");
            Serial.println(encoder_count);
        }
        
        // 콘솔에 현재 엔코더 값 변화 출력 (기존 코드)
        if (encoder_count != last_encoder_count) {
            Serial.print("Encoder position: ");
            Serial.println(encoder_count);
            last_encoder_count = encoder_count;
        }
        
        lastEncoderPrintTime = millis();
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
        if (!isRunning) {  // 모터가 동작 중이 아닐 때만 다른 명령 처리
            if (command == 'c') {
                Serial.println("Executing movement based on array C...");
                
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

                // 총 소요 시간 계산 및 출력
                unsigned long totalTime = endTime - startTime;
                Serial.print("Total execution time: ");
                Serial.print(totalTime);
                Serial.println(" milliseconds");
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
    
    // 모터가 완전히 멈출 때까지 run() 함수 호출
    while (stepper1.isRunning() || stepper2.isRunning()) {
        stepper1.run();
        stepper2.run();
    }
    
    // 모터 출력 비활성화
    stepper1.disableOutputs();
    stepper2.disableOutputs();
}


// 배열 C에 따라 모터 움직임 구현 함수
void moveMotorByArray() {
    for (int i = 0; i < arraySize; i++) {
        
        long targetPosition = C1[i]*4; // 배열 C의 값 사용
        long targetPosition2 = C2[i]*4; // 배열 C의 값 사용
        
        stepper1.moveTo(targetPosition);
        stepper2.moveTo(targetPosition2);

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
        }
    }

    Serial.println("Completed all movements based on array C.");
}


// 단일 모터만 움직이는 함수
void moveSingleMotor(int motorNum) {
    int* array = (motorNum == 1) ? C1 : C2;
    AccelStepper& stepper = (motorNum == 1) ? stepper1 : stepper2;
    
    for (int i = 0; i < arraySize; i++) {

        long targetPosition = array[i] * 4;
        stepper.moveTo(targetPosition);

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
            stepper.run();
        }
    }

    Serial.print("Completed all movements for motor ");
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

