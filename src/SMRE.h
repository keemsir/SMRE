
#ifndef SMRE_h
#define SMRE_h


class SMRE {
    public:
        // constructors:
        SMRE(int number_of_steps, int motor_pin_1, int motor_pin_2);
        SMRE(int number_of_steps, int motor_pin_1, int motor_pin_2, int motor_pin_3, int motor_pin_4);

        // speed setter method:
        void setSpeed(long whatSpeed);

        // mover method:
        void step(int number_of_steps);

        // Operating Stepper-motor
        void operStep(int configSpeed);
        
        void moveStep();

        int version(void);

    private:
        void stepMotor(int this_step);

        int direction;
        int speed;
        unsigned long step_delay;
        int number_of_steps;
        int pin_count;
        int step_number;

        // motor pin numbers:
        int motor_pin_1;
        int motor_pin_2;
        int motor_pin_3;
        int motor_pin_4;

        unsigned long last_step_time;

        // save the step
        int steps_left;

};

#endif
