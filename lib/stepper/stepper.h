#ifndef STEPPER_H
#define STEPPER_H

#include <Arduino.h>
#include <SoftwareSerial.h>
#include "regDefs.h"


class stepper {
    private:
        //PINS
        byte DIR_PIN;
        byte STEP_PIN;
        byte EN_PIN;
        byte INDEX_PIN;
        byte TX_PIN;

        //Variables
        int maxFreq;
        int minFreq ;
        int rampSteps;
        int rampLevels;


        unsigned long stepperPosition;
        unsigned long stepperPositionLimit;


        SoftwareSerial * dataSend;
    public:
        stepper(byte DIR_PIN,byte STEP_PIN,byte EN_PIN,byte INDEX_PIN);
        stepper(byte DIR_PIN,byte STEP_PIN,byte EN_PIN,byte INDEX_PIN, byte TX_PIN);
        void init();

        bool isUART();

        void setHome();
        void setLimit();
        void setMaxFreq(int maxFreq);
        void setMinFreq(int minFreq);
        void setRampSteps(int rampSteps);
        void setRampLevels(int rampLevels);

        unsigned long getPosition();
        unsigned long getPositionLimit();

        void enableStepper();
        void disableStepper();
        void moveStepper(unsigned long numSteps, bool stepperDir);
        void moveStepperUNSAFE(unsigned long numSteps, bool stepperDir);

        uint8_t calcCRC(uint8_t data[]);
        void sendData(uint8_t regAddr,uint32_t data);
        void writeGCONF(bool i_scale_analog, bool internal_rsense, bool en_spreadcycle, bool direction, bool index_otpw,bool index_step, bool pdn_disable, bool mstep_reg_select, bool multistep_filt, bool test_mode);
        void writeIHOLD_RUN(uint8_t IHOLD,uint8_t IRUN, uint8_t IHOLDDELAY);
        void writeVACTUAL(bool direction, uint32_t speed);
};

#endif