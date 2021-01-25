#include "stepper.h"
#include <Arduino.h>


unsigned long pulseCount;
ICACHE_RAM_ATTR void countPulses() {
    pulseCount++;
}


stepper::stepper(byte DIR_PIN,byte STEP_PIN,byte EN_PIN,byte INDEX_PIN){
    this->DIR_PIN=DIR_PIN;
    this->STEP_PIN=STEP_PIN;
    this->EN_PIN=EN_PIN;
    this->INDEX_PIN=INDEX_PIN;
    maxFreq = 10000;
    minFreq = 1000;
    rampSteps = 3000;
    rampLevels = 100;
    pulseCount = 0;
    stepperPosition = 0;
    stepperPositionLimit = 10000;
}

void stepper::init(){
    attachInterrupt(digitalPinToInterrupt(INDEX_PIN), countPulses, RISING);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(EN_PIN, OUTPUT);
    digitalWrite(EN_PIN,1);
}

void stepper::setHome(){
    stepperPosition =0;
}

void stepper::setLimit(){
    stepperPositionLimit=stepperPosition;
}

void stepper::setMaxFreq(int maxFreq){
    this->maxFreq=maxFreq;
}

void stepper::setMinFreq(int minFreq){
    this->minFreq=minFreq;
}

void stepper::setRampSteps(int rampSteps){
    this->rampSteps=rampSteps;
}
void stepper::setRampLevels(int rampLevels){
    this->rampLevels=rampLevels;
}

void stepper::moveStepper(unsigned long numSteps, bool stepperDir){
    unsigned long pulseDestination;
    int stepDifference;
    int freqDifference;
    int y=0;
    int rampFreq= (maxFreq-minFreq)/rampLevels;
    int rampStepsLevel= rampSteps/rampLevels;
    int nextRampStep = rampStepsLevel;
    bool ramping = 0;
    int i =minFreq;
    if(stepperDir){
        if(stepperPosition < numSteps) pulseDestination = stepperPosition;
        else pulseDestination = numSteps;
    }else{
        if(stepperPosition+numSteps>stepperPositionLimit) pulseDestination = stepperPositionLimit-stepperPosition;
        else pulseDestination = numSteps;
    }

    pulseCount=0;
    digitalWrite(DIR_PIN, stepperDir);
    digitalWrite(EN_PIN, 0);
    analogWriteFreq(i);
    analogWrite(STEP_PIN,512);
    while(pulseCount<pulseDestination){
        if(!ramping){
        if (pulseCount >= rampSteps/2){
            ramping =1;
            nextRampStep=rampStepsLevel*(rampLevels-y);
        }
        if(pulseCount >= nextRampStep){
            if(i<maxFreq){
            nextRampStep+=rampStepsLevel;
            i+=rampFreq;
            y++;
            analogWriteFreq(i);
            analogWrite(STEP_PIN,512);      
            }else{
            ramping =1;
            nextRampStep=rampStepsLevel;
            }
        }
        }else{
        if(pulseCount >= pulseDestination-rampSteps+nextRampStep){
            if(i>minFreq){
            nextRampStep+=rampStepsLevel;
            i-=rampFreq;
            analogWriteFreq(i);
            analogWrite(STEP_PIN,512);      
            }     
        }
        }
        yield();
        
    }
    analogWrite(STEP_PIN,0);
    stepperPosition= stepperDir ? stepperPosition - pulseDestination : stepperPosition + pulseDestination;
    delay(10);
    }

void stepper::moveStepperUNSAFE(unsigned long numSteps, bool stepperDir){
    unsigned long pulseDestination;
    int stepDifference;
    int freqDifference;
    int y=0;
    int rampFreq= (maxFreq-minFreq)/rampLevels;
    int rampStepsLevel= rampSteps/rampLevels;
    int nextRampStep = rampStepsLevel;
    bool ramping = 0;
    int i =minFreq;
    pulseDestination = numSteps;


    pulseCount=0;
    digitalWrite(DIR_PIN, stepperDir);
    digitalWrite(EN_PIN, 0);
    analogWriteFreq(i);
    analogWrite(STEP_PIN,512);
    while(pulseCount<pulseDestination){
        if(!ramping){
        if (pulseCount >= rampSteps/2){
            ramping =1;
            nextRampStep=rampStepsLevel*(rampLevels-y);
        }
        if(pulseCount >= nextRampStep){
            if(i<maxFreq){
            nextRampStep+=rampStepsLevel;
            i+=rampFreq;
            y++;
            analogWriteFreq(i);
            analogWrite(STEP_PIN,512);      
            }else{
            ramping =1;
            nextRampStep=rampStepsLevel;
            }
        }
        }else{
        if(pulseCount >= pulseDestination-rampSteps+nextRampStep){
            if(i>minFreq){
            nextRampStep+=rampStepsLevel;
            i-=rampFreq;
            analogWriteFreq(i);
            analogWrite(STEP_PIN,512);      
            }     
        }
        }
        yield();
        
    }
    analogWrite(STEP_PIN,0);
    delay(10);
}




