// 'l': Start continuous looping (cycle)");
// 's': Emergency Stop and reset");

#include <AccelStepper.h>
#include <math.h> // for round() function

// ----------------------------------------------------
// 1. PIN CONFIGURATION (핀 설정)
// ----------------------------------------------------

// DRIVER Mode Pins (A4988/DRV8825 드라이버용)
// Motor 1 (Abdomen Lateral / x-axis)
const int motor1_stepPin = 30;
const int motor1_dirPin = 32;

// Motor 2 (Abdomen SI / y-axis)
const int motor2_stepPin = 22;
const int motor2_dirPin = 24;

// Motor 3 (Abdomen AP / z-axis) - High Torque
const int motor3_stepPin = 38; 
const int motor3_dirPin = 40;

// Motor 4 (Chest AP / c-axis) - High Torque
const int motor4_stepPin = 46;
const int motor4_dirPin = 48;

// Rotary Encoder Pins (엔코더는 Motor 3 (Abdomen AP)의 피드백으로 가정) - 수시로 바뀜
const int encoderz_PinA = 19;
const int encoderz_PinB = 18;


// ----------------------------------------------------
// 2. CONSTANTS AND ARRAYS (상수 및 배열)
// ----------------------------------------------------

// 원본 100ms 간격 데이터 (정수형으로 변환)
// M1(x/Lateral): 0.1mm 단위 (예: 4.0mm -> 40)
const int raw_pos_M1_x[] = {0, 40, 100, 180, 260, 360, 480, 600, 680, 780, 900, 1040, 1220, 1320, 1400, 1460, 1500, 1580, 1620, 1640, 1660, 1700, 1720, 1740, 1780, 1780, 1720, 1640, 1540, 1400, 1200, 1020, 780, 660, 540, 420, 300, 200, 120, 60, 20};
{0, -20, -40, -60, -80, -96, -112, -128, -144, -60, 24, 108, 192, 148, 104, 60, 16, 24, 32, 40, 48, 32, 16, 0, -16, -16, 28, 72, 116, 160, 168, 176, 184, 192, 124, 56, -12, -80, -60, -40, -20};
// M2(y/SI): 0.1mm 단위 (예: -2.0mm -> -20)
const int raw_pos_M2_y[] = {0, -20, -40, -60, -80, -96, -112, -128, -144, -60, 24, 108, 192, 148, 104, 60, 16, 24, 32, 40, 48, 32, 16, 0, -16, -16, 28, 72, 116, 160, 168, 176, 184, 192, 124, 56, -12, -80, -60, -40, -20};
{0, 124, 248, 372, 496, 536, 576, 616, 656, 652, 648, 644, 640, 644, 648, 652, 656, 708, 760, 812, 864, 856, 848, 840, 832, 832, 812, 792, 772, 752, 756, 760, 764, 768, 616, 464, 312, 160, 120, 80, 40};
// M3(z/Abdomen AP): 0.1mm 단위 (예: 12.4mm -> 124)
const int raw_pos_M3_z[] = {0, 124, 248, 372, 496, 536, 576, 616, 656, 652, 648, 644, 640, 644, 648, 652, 656, 708, 760, 812, 864, 856, 848, 840, 832, 832, 812, 792, 772, 752, 756, 760, 764, 768, 616, 464, 312, 160, 120, 80, 40};
{0, -25, -50, -75, -100, -550, -1000, -1450, -1900, -2175, -2450, -2725, -3000, -4575, -6150, -7725, -9300, -9050, -8800, -8550, -8300, -5750, -3200, -650, 1900, 1900, 200, -1500, -3200, -4900, -4675, -4450, -4225, -4000, -3075, -2150, -1225, -300, -225, -150, -75}/10;
// M4(c/Chest AP): 0.01mm 단위 (예: -0.25mm -> -25)
const int raw_pos_M4_c[] = {0, -25, -50, -75, -100, -550, -1000, -1450, -1900, -2175, -2450, -2725, -3000, -4575, -6150, -7725, -9300, -9050, -8800, -8550, -8300, -5750, -3200, -650, 1900, 1900, 200, -1500, -3200, -4900, -4675, -4450, -4225, -4000, -3075, -2150, -1225, -300, -225, -150, -75};
{0, 40, 100, 180, 260, 360, 480, 600, 680, 780, 900, 1040, 1220, 1320, 1400, 1460, 1500, 1580, 1620, 1640, 1660, 1700, 1720, 1740, 1780, 1780, 1720, 1640, 1540, 1400, 1200, 1020, 780, 660, 540, 420, 300, 200, 120, 60, 20};

const int RAW_SIZE = sizeof(raw_pos_M1_x) / sizeof(raw_pos_M1_x[0]);
const int INTERPOLATED_SIZE = 2 * RAW_SIZE - 1; 
const unsigned long STEP_INTERVAL_MS = 100;

// 모터별 보간된 위치 배열 (최종 목표 스텝 값 저장)
long g_motor1_positions[INTERPOLATED_SIZE];
long g_motor2_positions[INTERPOLATED_SIZE];
long g_motor3_positions[INTERPOLATED_SIZE];
long g_motor4_positions[INTERPOLATED_SIZE];

// 모터 스텝 변환 계수 (1mm = 5 steps 비율 유지)
// M1, M2, M3 (0.1 mm 단위): 0.1 mm * 5 = 0.5 Steps/단위
const float STEP_FACTOR_M123 = 0.5;
// M4 (0.01 mm 단위): 0.01 mm * 5 = 0.05 Steps/단위
const float STEP_FACTOR_M4 = 0.05;

// ----------------------------------------------------
// 3. CONTROL VARIABLES (제어 변수)
// ----------------------------------------------------

volatile long encoder_count = 0;
bool isRunning = false;
bool emergencyStop = false;
bool isLooping = false; // 무한 반복 상태 변수

// ----------------------------------------------------
// 4. ACCELSTEPPER & OBJECTS (모터 객체)
// ----------------------------------------------------

// AccelStepper 객체 생성 (모두 DRIVER 모드 사용)
AccelStepper stepper1(AccelStepper::DRIVER, motor1_stepPin, motor1_dirPin);
AccelStepper stepper2(AccelStepper::DRIVER, motor2_stepPin, motor2_dirPin);
AccelStepper stepper3(AccelStepper::DRIVER, motor3_stepPin, motor3_dirPin);
AccelStepper stepper4(AccelStepper::DRIVER, motor4_stepPin, motor4_dirPin);

// ----------------------------------------------------
// 5. INTERRUPT SERVICE ROUTINES (ISR) - 엔코더 인터럽트
// ----------------------------------------------------

void Encoder_z_CW() {
    if (digitalRead(encoderz_PinB) == LOW) {
        encoder_count++;
    }
}

void Encoder_z_CCW() {
    if (digitalRead(encoderz_PinA) == LOW) {
        encoder_count--;
    }
}


// ----------------------------------------------------
// 6. HELPER FUNCTIONS (도우미 함수)
// ----------------------------------------------------

// 선형 보간을 수행하고 스텝으로 변환하는 함수
void interpolateData(const int raw_data[], long interpolated_data[], float step_factor) {
    for (int i = 0; i < RAW_SIZE; i++) {
        // 1. 원본 데이터 포인트 저장 (50ms 지점)
        // 스텝으로 변환 후 round() 처리하여 가장 가까운 정수 스텝으로 설정
        interpolated_data[2 * i] = (long)round(raw_data[i] * step_factor);

        // 2. 인접한 두 점 사이에 보간값 계산 및 저장 (25ms 지점)
        if (i < RAW_SIZE - 1) {
            float p1_steps = raw_data[i] * step_factor;
            float p2_steps = raw_data[i + 1] * step_factor;
            // 선형 보간 후 round() 처리
            interpolated_data[2 * i + 1] = (long)round((p1_steps + p2_steps) / 2.0);
        }
    }
}

// 모터 정지 함수
void stopMotors() {
    stepper1.stop();
    stepper2.stop();
    stepper3.stop();
    stepper4.stop();

    // 모터가 완전히 멈출 때까지 run() 호출 (감속 정지)
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

// 시간을 포맷팅하여 출력하는 함수
void printFormattedTime(unsigned long timeInMs) {
    unsigned long ms = timeInMs % 1000;
    unsigned long totalSeconds = timeInMs / 1000;
    unsigned long seconds = totalSeconds % 60;
    unsigned long minutes = (totalSeconds / 60) % 60;
    
    Serial.print(minutes); Serial.print("m ");
    Serial.print(seconds); Serial.print("s ");
    Serial.print(ms); Serial.println("ms");
}


// ----------------------------------------------------
// 7. CORE MOTION LOGIC (핵심 모션 로직)
// ----------------------------------------------------

// 4축 동시 움직임 구현 함수
void moveMotorByArray() {
    // 50ms마다 현재 시각, 4개 축 목표 스텝, Motor 3 엔코더 값 출력
    Serial.println("Time(ms), M1(SI) Target Steps, M2(Lat) Target Steps, M3(AP) Target Steps, M4(Chest) Target Steps, M3 Encoder Steps");
    
    // 무한 반복 (Emergency Stop 전까지)
    while (isLooping) {
        for (int i = 0; i < INTERPOLATED_SIZE; i++) {
            if (emergencyStop) return; // 비상 정지 확인

            // 1. 목표 위치 설정 (절대 스텝 위치)
            stepper1.moveTo(g_motor1_positions[i]); // x-axis
            stepper2.moveTo(g_motor2_positions[i]); // y-axis
            stepper3.moveTo(g_motor3_positions[i]); // z-axis
            stepper4.moveTo(g_motor4_positions[i]); // c-axis

            unsigned long stepStartTime = millis();
            
            // 2. 50ms 동안 모터 구동 및 동기화 유지
            while (millis() - stepStartTime < STEP_INTERVAL_MS) {
                
                // 시리얼 입력 확인 (비상 정지 처리)
                if (Serial.available() > 0) {
                    char command = Serial.read();
                    if (command == 's') { 
                        emergencyStop = true;
                        Serial.println("Emergency stop command received!");
                        stopMotors();
                        return; 
                    }
                }

                // 모터 구동 (비차단 방식)
                stepper1.run();
                stepper2.run();
                stepper3.run();
                stepper4.run();
            }

            // 3. 50ms 간격으로 출력
            Serial.print(millis()); Serial.print(", ");
            Serial.print(g_motor1_positions[i]); Serial.print(", ");
            Serial.print(g_motor2_positions[i]); Serial.print(", ");
            Serial.print(g_motor3_positions[i]); Serial.print(", ");
            Serial.print(g_motor4_positions[i]); Serial.print(", ");
            Serial.println(encoder_count); 
        }
        // 사이클 완료 후 다시 시작
    }
    Serial.println("Movement loop stopped.");
}

// ----------------------------------------------------
// 8. SETUP & LOOP
// ----------------------------------------------------

void setup() {
    Serial.begin(115200);
    Serial.println("-------------------------------------");
    Serial.println("4D Phantom Controller Initializing...");

    // STEP_FACTOR를 적용하여 데이터 보간 및 스텝 변환
    interpolateData(raw_pos_M1_x, g_motor1_positions, STEP_FACTOR_M123);
    interpolateData(raw_pos_M2_y, g_motor2_positions, STEP_FACTOR_M123);
    interpolateData(raw_pos_M3_z, g_motor3_positions, STEP_FACTOR_M123);
    interpolateData(raw_pos_M4_c, g_motor4_positions, STEP_FACTOR_M4);

    // 모터 속도 및 가속 설정 (모든 축에 동일하게 설정)
    stepper1.setMaxSpeed(2000); stepper1.setAcceleration(1000);
    stepper2.setMaxSpeed(2000); stepper2.setAcceleration(1000);
    stepper3.setMaxSpeed(2000); stepper3.setAcceleration(1000);
    stepper4.setMaxSpeed(2000); stepper4.setAcceleration(1000);

    // 엔코더 인터럽트 설정
    pinMode(encoderz_PinA, INPUT_PULLUP);
    pinMode(encoderz_PinB, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(encoderz_PinA), Encoder_z_CW, RISING);
    attachInterrupt(digitalPinToInterrupt(encoderz_PinB), Encoder_z_CCW, RISING);

    // 이론적 작동 시간 계산 및 출력
    unsigned long theoreticalTime = INTERPOLATED_SIZE * STEP_INTERVAL_MS;
    Serial.print("Expected single cycle time: ");
    printFormattedTime(theoreticalTime);

    Serial.println("Available commands:");
    Serial.println("'l': Start continuous looping (cycle)");
    Serial.println("'s': Emergency Stop and reset");
}


void loop() {
    // 시리얼 입력 확인
    if (Serial.available() > 0) {
        char command = Serial.read();

        if (command == 's') {
            emergencyStop = true;
            isLooping = false; // 루프 중지
        }

        if (emergencyStop) {
            if (isRunning) {
                stopMotors();
                isRunning = false;
                Serial.println("Emergency stop activated. Motors stopped.");
            }
            emergencyStop = false;
            Serial.println("Ready for new commands (Type 'l' to start cycle).");
        } else if (command == 'l' && !isRunning) {
            Serial.println("Starting 4-axis synchronized cycle loop...");
            
            // 엔코더 초기화 및 시작 시간 기록
            encoder_count = 0;

            isRunning = true;
            isLooping = true;
            
            moveMotorByArray(); // 무한 루프 시작
            
            isRunning = false;
            isLooping = false; // moveMotorByArray에서 빠져나온 경우 (s 입력)
        }
    }
}
