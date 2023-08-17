

// SMRE.cpp
//
//
// Copyright (C) 2023 Maengee Kim
// $Id: SMRE.cpp,v 1.0 2023/07/01 keemsir Exp $
//
//
// A library of Stepper motors and Rotary encoders


#include "SMRE.h"

//// Parameter setting ////

void SMRE::moveTo(long absolute)
{
    _targetPos = absolute;
    computeNewSpeed();
}


void SMRE::cycleTime(long absoluteTime)
{
    _targetTime = absoluteTime;
    computeNewSpeed();
}


void SMRE::moveToEncoder(long absoluteEncoder)
{
    _targetPosEncoder = absoluteEncoder;
    // _targetPos = absoluteEncoder;
    computeNewSpeed();
}


// Relative position
void SMRE::move(long relative)
{
    moveTo(_currentPos + relative);
}


// Emergency button
void SMRE::EMERGENCY_Button()
{
    _EMERGENCY = true;
}


// Implements steps according to the current speed
// You must call this at least once per step
// returns true if a step occurred
boolean SMRE::runSpeed()
{
    unsigned long time = micros(); // millis()

    if (time > _lastStepTime + _stepInterval)
    {
        if (_speed > 0)
        {
            // Clockwise
            _currentPos += 1;
            // _currentPosEncoder += 1;
        }
        else if (_speed < 0)
        {
            // Anticlockwise
            _currentPos -= 1;
            // _currentPosEncoder -= 1;
        }
        step(_currentPos & 0x3); // Bottom 2 bits (same as mod 4, but works with + and - numbers)
        // step(_currentPosEncoder & 0x3);

        _lastStepTime = time;
        return true;
    }
    else
        return false;
}


boolean SMRE::runSpeedEncoder()
{
    unsigned long time = micros(); // millis()

    if (time > _lastStepTime + _stepInterval)
    {
        if (_speed > 0)
        {
            // Clockwise
            // _currentPos += 1;
            _currentPosEncoder += 1;
        }
        else if (_speed < 0)
        {
            // Anticlockwise
            // _currentPos -= 1;
            _currentPosEncoder -= 1;
        }
        // step(_currentPos & 0x3); // Bottom 2 bits (same as mod 4, but works with + and - numbers)
        step(_currentPosEncoder & 0x3);

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


long SMRE::distanceToGoEncoder()
{
    return _targetPosEncoder - _currentPosEncoder;
}


long SMRE::timeToGo()
{
    return _targetTime;
}


long SMRE::targetPosition()
{
    return _targetPos;
}


long SMRE::targetPositionEncoder()
{
    return _targetPosEncoder;
}


long SMRE::currentPosition()
{
    return _currentPos;
}


long SMRE::currentPositionEncoder()
{
    return _currentPosEncoder;
}


// Useful during initialisations or after initial positioning
void SMRE::setCurrentPosition(long position)
{
    _currentPos = position;
}


void SMRE::computeNewSpeed()
{
    // setSpeed(desiredSpeed());
    setSpeed(timeSpeed());
}


// Work out and return a new speed.
// Subclasses can override if they want
// Implement acceleration, deceleration and max speed
// Negative speed is anticlockwise
// This is called:
//  after each step
//  after user changes:
//      maxSpeed
//      acceleration
//      target position (relative or absolute)
float SMRE::desiredSpeed()
{
    long distanceTo = distanceToGo();

    // Max possible speed that can still decelerate in the available distance
    float requiredSpeed;

    if (distanceTo == 0)
        return 0.0; // Were there
    else if (distanceTo > 0) // Clockwise
        requiredSpeed = sqrt(2.0 * distanceTo * _acceleration);
    else  // Anticlockwise
        requiredSpeed = -sqrt(2.0 * -distanceTo * _acceleration);

    if (requiredSpeed > _speed)
    {
        // Need to accelerate in clockwise direction
        if (_speed == 0)
            requiredSpeed = sqrt(2.0 * _acceleration);
        else
            requiredSpeed = _speed + abs(_acceleration / _speed);
        if (requiredSpeed > _maxSpeed)
            requiredSpeed = _maxSpeed;
    }
    else if (requiredSpeed < _speed)
    {
        // Need to accelerate in anticlockwise direction
        if (_speed == 0)
            requiredSpeed = -sqrt(2.0 * _acceleration);
        else
            requiredSpeed = _speed - abs(_acceleration / _speed);
        if (requiredSpeed < -_maxSpeed)
            requiredSpeed = -_maxSpeed;
    }
    return requiredSpeed;
}



// This is called:
//  after user changes:
//      _targetPos (moveTo) 
//      _targetTime (cycleTime)
float SMRE::timeSpeed()
{
    long distanceTo = distanceToGo();
    long time = timeToGo();

    float requiredSpeed;

    if (distanceTo == 0)
        return 0.0;
    else if (distanceTo > 0)
        requiredSpeed = abs(distanceTo / time);
    else
        requiredSpeed = -abs(distanceTo / time);

    // Serial.print("Distance: ");
    // Serial.println(distanceTo);

    // Serial.print("Time: ");
    // Serial.println(time);

    return requiredSpeed;
}



// Run the motor to implement speed and acceleration in order to proceed to the target position
// You must call this at least once per step, preferably in your main loop
// If the motor is in the desired position, the cost is very small
// returns true if we are still running to position
boolean SMRE::run()
{
    if (_EMERGENCY == false)
        if (_targetPos == _currentPos)
            return false;

        // if (_targetPosEncoder == _currentPosEncoder)
        //     return false;

        if (runSpeed())
            // computeNewSpeed();
        return true;
    return false;
}


boolean SMRE::runEncoder()
{
    if (_EMERGENCY == false)
        if (_targetPosEncoder == _currentPosEncoder)
            return false;

        if (runSpeedEncoder())
            // computeNewSpeed();
        return true;
    return false;
}


SMRE::SMRE(uint8_t pins, uint8_t pin1, uint8_t pin2, uint8_t pin3, uint8_t pin4)
{
    _pins = pins;
    _currentPos = 0;
    _targetPos = 0;
    _currentPosEncoder = 0;
    _targetPosEncoder = 0;
    _targetTime = 1.0;
    _speed = 0.0;
    _maxSpeed = 1.0;
    _acceleration = 1.0;
    _stepInterval = 0;
    _lastStepTime = 0;
    _pin1 = pin1;
    _pin2 = pin2;
    _pin3 = pin3;
    _pin4 = pin4;
    _EMERGENCY = false;
    enableOutputs();
}



SMRE::SMRE(void (*forward)(), void (*backward)())
{
    _pins = 0;
    _currentPos = 0;
    _targetPos = 0;
    _currentPosEncoder = 0;
    _targetPosEncoder = 0;
    _targetTime = 1.0;
    _speed = 0.0;
    _maxSpeed = 1.0;
    _acceleration = 1.0;
    _stepInterval = 0;
    _lastStepTime = 0;
    _pin1 = 0;
    _pin2 = 0;
    _pin3 = 0;
    _pin4 = 0;
    _EMERGENCY = false;
    _forward = forward;
    _backward = backward;
}



void SMRE::setMaxSpeed(float speed)
{
    _maxSpeed = speed;
    computeNewSpeed();
}


void SMRE::setAcceleration(float acceleration)
{
    _acceleration = acceleration;
    computeNewSpeed();
}


void SMRE::setSpeed(float speed)
{
    _speed = speed;
    // if (_speed != 0)
    _stepInterval = abs(10000.0 / _speed);
    // else
    //     _stepInterval = 0.0;
}


float SMRE::speed()
{
    return _speed;
}


// Subclasses can override
void SMRE::step(uint8_t step)
{
    switch (_pins)
    {
        case 0:
            step0();
            break;
        case 1:
            step1(step);
            break;
        
        case 2:
            step2(step);
            break;
        
        case 4:
            step4(step);
            break;  
    }
}


// 0 pin step function (ie for functional usage)
void SMRE::step0()
{
    if (_speed > 0)
    {
        _forward();
    }
    else
    {
        _backward();
    }
}


// 1 pin step function (ie for stepper drivers)
// This is passed the current step number (0 to 3)
// Subclasses can override
void SMRE::step1(uint8_t step)
{
    digitalWrite(_pin2, _speed > 0); // Direction
    // Caution 200ns setup time 
    digitalWrite(_pin1, HIGH);
    // Caution, min Step pulse width for 3967 is 1microsec
    // Delay 1microsec
    delayMicroseconds(1);
    digitalWrite(_pin1, LOW);
}


// 2 pin step function
// This is passed the current step number (0 to 3)
// Subclasses can override
void SMRE::step2(uint8_t step)
{
    switch (step)
    {
        case 0: /* 01 */
            digitalWrite(_pin1, LOW);
            digitalWrite(_pin2, HIGH);
            break;

        case 1: /* 11 */
            digitalWrite(_pin1, HIGH);
            digitalWrite(_pin2, HIGH);
            break;

        case 2: /* 10 */
            digitalWrite(_pin1, HIGH);
            digitalWrite(_pin2, LOW);
            break;

        case 3: /* 00 */
            digitalWrite(_pin1, LOW);
            digitalWrite(_pin2, LOW);
            break;
    }
}


// 4 pin step function
// This is passed the current step number (0 to 3)
// Subclasses can override
void SMRE::step4(uint8_t step)
{
    switch (step)
    {
        case 0:    // 1010
            digitalWrite(_pin1, HIGH);
            digitalWrite(_pin2, LOW);
            digitalWrite(_pin3, HIGH);
            digitalWrite(_pin4, LOW);
            break;

        case 1:    // 0110
            digitalWrite(_pin1, LOW);
            digitalWrite(_pin2, HIGH);
            digitalWrite(_pin3, HIGH);
            digitalWrite(_pin4, LOW);
            break;

        case 2:    //0101
            digitalWrite(_pin1, LOW);
            digitalWrite(_pin2, HIGH);
            digitalWrite(_pin3, LOW);
            digitalWrite(_pin4, HIGH);
            break;

        case 3:    //1001
            digitalWrite(_pin1, HIGH);
            digitalWrite(_pin2, LOW);
            digitalWrite(_pin3, LOW);
            digitalWrite(_pin4, HIGH);
            break;
    }
}


// Prevents power consumption on the outputs
void SMRE::disableOutputs()
{
    if (! _pins) return;

        digitalWrite(_pin1, LOW);
        digitalWrite(_pin2, LOW);
        
        if (_pins == 4)
        {
            digitalWrite(_pin3, LOW);
            digitalWrite(_pin4, LOW);
        }
}


void SMRE::enableOutputs()
{
    if (! _pins) return;

    pinMode(_pin1, OUTPUT);
    pinMode(_pin2, OUTPUT);
    if (_pins == 4)
    {
        pinMode(_pin3, OUTPUT);
        pinMode(_pin4, OUTPUT);
    }
}


// Blocks until the target position is reached
void SMRE::runToPosition()
{
    while (run())
        ;
}


boolean SMRE::runSpeedToPosition()
{
    return _targetPos!=_currentPos ? SMRE::runSpeed() : false;
}


// Blocks until the new target position is reached
void SMRE::runToNewPosition(long position)
{
    moveTo(position);
    runToPosition();
}



// 
// 
// fake encoder position
// 
// 
// 
// 
// 
// 
// 


