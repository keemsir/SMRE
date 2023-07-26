
/*
A library of Stepper motors and Rotary encoders
*/


#include "Arduino.h"
#include "SMRE.h"


SMRE::SMRE(uint8_t pins, uint8_t motor_pin_1, uint8_t motor_pin_2)
{
    _pins = pins;

    _steps_left = 0; // +, Initialize step
    _encoder_targetPos = 0; // Destination

    _currentPos = 0;
    _targetPos = 0;
    _totalPos = 0;
    
    // _limit_Pos = 500;
    
    _step_number = 0;
    _stepTime = 0;
    _speed = 0;
    _direction = 0;
    _last_step_time = 0;

    _stepInterval = 0;
    _lastStepTime = 0;

    _motor_pin_1 = motor_pin_1;
    _motor_pin_2 = motor_pin_2;

    // setup the pins
    pinMode(_motor_pin_1, OUTPUT);
    pinMode(_motor_pin_2, OUTPUT);

    // When there are only 2 pins, set the other two to 0:
    _motor_pin_3 = 0;
    _motor_pin_4 = 0;
}



SMRE::SMRE(uint8_t pins, uint8_t motor_pin_1, uint8_t motor_pin_2, uint8_t motor_pin_3, uint8_t motor_pin_4)
{
    _pins = pins;

    _steps_left = 0; // +, Initialize step
    _encoder_targetPos = 0; // Destination

    _currentPos = 0;
    _targetPos = 0;
    _totalPos = 0;

    // _limit_Pos = 500;

    _step_number = 0;
    _stepTime = 0;
    _speed = 0;
    _direction = 0;
    _last_step_time = 0;

    _stepInterval = 0;
    _lastStepTime = 0;

    _motor_pin_1 = motor_pin_1;
    _motor_pin_2 = motor_pin_2;
    _motor_pin_3 = motor_pin_3;
    _motor_pin_4 = motor_pin_4;

    // setup the pins
    pinMode(_motor_pin_1, OUTPUT);
    pinMode(_motor_pin_2, OUTPUT);
    pinMode(_motor_pin_3, OUTPUT);
    pinMode(_motor_pin_4, OUTPUT);
}



SMRE::SMRE_Encoder(uint8_t encoder_pin_1, uint8_t encoder_pin_2)
{
    _encoder_pin_1 = encoder_pin_1;
    _encoder_pin_2 = encoder_pin_2;

    // Setup the input pins and turn on pullup resistor
    pinMode(encoder_pin_1, INPUT_PULLUP);
    pinMode(encoder_pin_2, INPUT_PULLUP);

    int sig1 = digitalRead(_encoder_pin_1);
    int sig2 = digitalRead(_encoder_pin_2);
    _oldState = sig1 | (sig2 << 1);

    // Start with position 0
    _encoderPostion = 0;
    _encoderPostionExt = 0;
    _encoderPostionExtPrev = 0;

    //
    //
    //
    //
}



boolean SMRE::runSpeed()
{
    usigned long time = millis();

    if (time > _lastStepTime + _stepDelay)
    {
        if (_speed > 0)
        {
            _currentPos += 1;
        }
        else if (_speed < 0)
        {
            _currentPos -= 1;
        }
        stepMotor(_currentPos & 0x3);

        _lastStepTime = time;
        return true;
    }
    else
        return false;
}



long SMRE::distanceToGo()
{
    return _targetPos - _currentPos;
}



void SMRE::computeNewSpeed()
{
    setSpeed(deterSpeed());
}



void SMRE::calculateSpeed()
{
    
}



/*
 * This is called:
 * after each step
 * after user changes:
 * target position
 * Speed: Step interval delay
 */
float SMRE::deterSpeed()
{
    long distanceTo = distanceToGo();

    float requiredSpeed;

    if (distanceTo == 0)
        retrun 0.0;
    else if (distanceTo > 0)
        requiredSpeed = abs(_stepTime / _totalPos)
    else
        requiredSpeed = abs(_stepTime / _totalPos)

    // Serial.println(requiredSpeed);
    return requiredSpeed;
}



boolean SMRE::run()
{
    if (_targetPos == _currentPos)
        return false;
    
    if (runSpeed())
        return true;
    //     deterSpeed();
        
    return true;
}


// unit: ms
void SMRE::timeToGo(long time)
{
    _stepTime = time;
    computeNewSpeed();
}


void SMRE::moveTo(long absolute)
{
    _targetPos = absolute;
    computeNewSpeed();
}



// Sets the speed in revs per minute (micros)
void SMRE::setSpeed(long stepTime)
{
    _stepTime = stepTime;
    // _stepDelay = 60L * 1000L * 1000L / _pins / whatSpeed; // 60,000,000
    _stepDelay = abs(1000.0 / _stepTime);

    // Sets the delay
    // _stepDelay = whatSpeed
}



/*
// Moves the motor steps_to_move steps
void SMRE::step(int steps_to_move)
{
    int steps_left = abs(steps_to_move);

    // direction
    if (steps_to_move > 0) {_direction = 1;}
    if (steps_to_move > 0) {_direction = 0;}

    // decrement the number of steps, moving one step each time:
    while (steps_left > 0)
    {
        unsigned long now = micros();
        // move only if the appropriate delay has passed:
        if (now - _last_step_time >= _stepDelay)
        {
            // get the timeStamp of when you stepped:
            _last_step_time = now;
            // increment or decrement the step number,

            if (_direction == 1)
            {
                _step_number++;
                if (_step_number == _pins)
                {
                    _step_number = 0;
                }
            }
            else
            {
                if (_step_number == 0)
                {
                    _step_number = _pins;
                }
                _step_number--;
            }
            // decrement the steps left:
            steps_left--;
            // step the motor to step number 0, 1, ..., {3 or 10}
        }
    }
}
*/



// Operation the Stepper-motor ()
void SMRE::operStep(int operSpeed)
{
    _speed = abs(operSpeed);
    setSpeed(_speed);

    // determine direction based on whether steps_to_mode is + or -:
    if (operSpeed > 0) {_direction = 1;}
    if (operSpeed < 0) {_direction = 0;}

    moveStep();
}



// (_currentPos != _targetPos)
void SMRE::moveStep2()
{
    // decrement the number of steps, moving one step each time:
    if (_speed > 0)
    {
        _steps_left = targetPos;

        if (_steps_left != 0)
        {
            if (_direction == 1)
            {
                _step_number++;
                if (_step_number == _pins)
                {
                    _step_number = 0;
                }
            }
            else
            {
                if (_step_number == 0)
                {
                    _step_number = _pins;
                }
                _step_number--;
            }
            // decrement the steps left:
            steps_left--;
            // step the motor to step number 0, 1, ..., {3 or 10}
            stepMotor(_step_number % 4);
        }
    }
}



// Moving the Stepper-motor
void SMRE::moveStep()
{

    // decrement the number of steps, moving one step each time:
    if (_speed > 0)
    {
        unsigned long now = micros();
        // move only if the appropriate delay has passed:
        if (now - _last_step_time >= _stepDelay)
        {
            // get the timeStamp of when you stepped:
            _last_step_time = now;
            // increment or decrement the step number

            if (_direction == 1)
            {
                _step_number++;
                if (_step_number == _pins)
                {
                    _step_number = 0;
                }
            }
            else
            {
                if (_step_number == 0)
                {
                    _step_number = _pins;
                }
                _step_number--;
            }
            // decrement the steps left:
            steps_left--;
            // step the motor to step number 0, 1, ..., {3 or 10}
            stepMotor(_step_number % 4);
        }
    }
}



// Moves the motor forward or backwards.
void SMRE::stepMotor(uint8_t thisStep)
{
    if (_pin_count == 2) {
        switch (thisStep) {
            case 0:  // 01
                digitalWrite(motor_pin_1, LOW);
                digitalWrite(motor_pin_2, HIGH);
            break;
            case 1:  // 11
                digitalWrite(motor_pin_1, HIGH);
                digitalWrite(motor_pin_2, HIGH);
            break;
            case 2:  // 10
                digitalWrite(motor_pin_1, HIGH);
                digitalWrite(motor_pin_2, LOW);
            break;
            case 3:  // 00
                digitalWrite(motor_pin_1, LOW);
                digitalWrite(motor_pin_2, LOW);
            break;
        }
    }

    if (_pin_count == 4) {
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
}



// version() returns the version of the library:
int SMRE::version(void)
{
    return 1;
}


