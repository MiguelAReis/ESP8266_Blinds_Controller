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
    this->TX_PIN =255;

}
stepper::stepper(byte DIR_PIN,byte STEP_PIN,byte EN_PIN,byte INDEX_PIN, byte TX_PIN){
    this->DIR_PIN=DIR_PIN;
    this->STEP_PIN=STEP_PIN;
    this->EN_PIN=EN_PIN;
    this->INDEX_PIN=INDEX_PIN;
    this->TX_PIN=TX_PIN;
}

void stepper::init(){
    maxFreq = 10000;
    minFreq = 1000;
    rampSteps = 3000;
    rampLevels = 100;
    pulseCount = 0;
    stepperPosition = 0;
    stepperPositionLimit = 10000;
    attachInterrupt(digitalPinToInterrupt(INDEX_PIN), countPulses, RISING);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(EN_PIN, OUTPUT);
    digitalWrite(EN_PIN,1);

    if(isUART()){
        Serial.println("IS UART");
        SoftwareSerial *dataSend = new SoftwareSerial(-1, TX_PIN); 
        this->dataSend= dataSend;
        this->dataSend->begin(50000);
        //digitalWrite(TX_PIN,0);    
        } 
}

bool stepper::isUART(){
    return this->TX_PIN!=255;
}

void stepper::setHome(){
    this->stepperPosition =0;
}

void stepper::setLimit(){
    this->stepperPositionLimit=this->stepperPosition;
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
unsigned long stepper::getPosition(){
    return this->stepperPosition;
}
unsigned long stepper::getPositionLimit(){
    return this->stepperPositionLimit;
}
void stepper::enableStepper(){
    digitalWrite(EN_PIN, 0);
}
void stepper::disableStepper(){
    digitalWrite(EN_PIN, 1);
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
    enableStepper();
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
    enableStepper();
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

uint8_t stepper::calcCRC(uint8_t data[]) {
	uint8_t crc = 0;
	for (uint8_t i = 0; i < 7; i++) {
		uint8_t currentByte = data[i];
		for (uint8_t j = 0; j < 8; j++) {
			if ((crc >> 7) ^ (currentByte & 0x01)) {
				crc = (crc << 1) ^ 0x07;
			} else {
				crc = (crc << 1);
			}
			crc &= 0xff;
			currentByte = currentByte >> 1;
		} 
	}
	return crc;
}

void stepper::sendData(uint8_t regAddr,uint32_t data){
    uint8_t dataArray[8] ={SYNC,SLAVEADDR,(uint8_t)(BASEADDR|regAddr),(uint8_t)(data>>24), (uint8_t)(data>>16), (uint8_t)(data>>8), (uint8_t)(data>>0), 0x00};
    dataArray[7]=calcCRC(dataArray);
    //digitalWrite(TX_PIN,1);
    //delay(10);
    for(uint8_t i=0;i<8;i++) dataSend->write(dataArray[i]);
    //delay(10);
    //digitalWrite(TX_PIN,0);

}

void stepper::writeGCONF(bool i_scale_analog, bool internal_rsense, bool en_spreadcycle, bool direction, bool index_otpw,bool index_step, bool pdn_disable, bool mstep_reg_select, bool multistep_filt, bool test_mode){
    sendData(GCONF,(test_mode<<9)|(multistep_filt<<8)|(mstep_reg_select<<7)|(pdn_disable<<6)|(index_step<<5)|(index_otpw<<4)|(direction<<3)|(en_spreadcycle<<2)|(internal_rsense<<1)|i_scale_analog);
}
void stepper::writeIHOLD_RUN(uint8_t IHOLD,uint8_t IRUN, uint8_t IHOLDDELAY){
    sendData(IHOLD_RUN,((IHOLDDELAY&0x0F)<<16)|((IRUN&0x1F)<<8)|(IHOLD));
}
void stepper::writeVACTUAL(bool direction, uint32_t speed){
    sendData(VACTUAL,(direction<<24)|(speed&0x7FFFFF));
}