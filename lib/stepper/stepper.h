#ifndef STEPPER_H
#define STEPPER_H

#include <Arduino.h>


class stepper {
    private:
        //PINS
        byte DIR_PIN;
        byte STEP_PIN;
        byte EN_PIN;
        byte INDEX_PIN;

        //Variables
        int maxFreq;
        int minFreq ;
        int rampSteps;
        int rampLevels;


        unsigned long stepperPosition;
        unsigned long stepperPositionLimit;

    public:
        stepper(byte DIR_PIN,byte STEP_PIN,byte EN_PIN,byte INDEX_PIN);
        void init();

        void setHome();
        void setLimit();
        void setMaxFreq(int maxFreq);
        void setMinFreq(int minFreq);
        void setRampSteps(int rampSteps);
        void setRampLevels(int rampLevels);

        void moveStepper(unsigned long numSteps, bool stepperDir);
        void moveStepperUNSAFE(unsigned long numSteps, bool stepperDir);



};

#endif