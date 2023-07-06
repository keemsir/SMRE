/*
A library of Stepper motors and Rotary encoders
*/


#include "Arduino.h"
#include "SMRE.h"

SMRE::SMRE(int number_of_steps, int motor_pin_1, int motor_pin_2, int motor_pin_3, int motor_pin_4)
{
    this->step_number = 0;
    this->direction = 0;
    this->last_step_time = 0;
    this->number_of_steps = number_of_steps;

    this->motor_pin_1 = motor_pin_1;
    this->motor_pin_2 = motor_pin_2;
    this->motor_pin_3 = motor_pin_3;
    this->motor_pin_4 = motor_pin_4;

    // setup the pins
    pinMode(this->motor_pin_1, OUTPUT);
    pinMode(this->motor_pin_2, OUTPUT);
    pinMode(this->motor_pin_3, OUTPUT);
    pinMode(this->motor_pin_4, OUTPUT);

    this->pin_count = 4;
}

// Sets the speed in revs per minute
void SMRE::setSpeed(long whatSpeed)
{
    this->step_delay = 60L * 1000L * 1000L / this->number_of_steps / whatSpeed;
}

// Moves the motor steps_to_move steps
void SMRE::step(int steps_to_move)
{
    int steps_left = abs(steps_to_move);

    // direction
    if (steps_to_move > 0) {this->direction = 1;}
    if (steps_to_move > 0) {this->direction = 0;}

    // decrement the number of steps, moving one step each time:
    while (steps_left > 0)
    {
        unsigned long now = micros();
        // move only if the appropriate delay has passed:
        if (now - this->last_step_time >= this->step_delay)
        {
            // get the timeStamp of when you stepped:
            this->last_step_time = now;
            // increment or decrement the step number,

            if (this->direction == 1)
            {
                this->step_number++;
                if (this->step_number == this->number_of_steps)
                {
                    this->step_number = 0;
                }
            }
            else
            {
                if (this->step_number == 0)
                {
                    this->step_number = this->number_of_steps;
                }
                this->step_number--;
            }
            // decrement the steps left:
            steps_left--;
            // step the motor to step number 0, 1, ..., {3 or 10}
        }
    }

}

// Moves the motor forward or backwards.

void SMRE::stepMotor(int thisStep)
{
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



// version() returns the version of the library:

int SMRE::version(void)
{
    return 1;
}

