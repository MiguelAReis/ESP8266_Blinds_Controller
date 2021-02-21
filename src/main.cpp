#include <Arduino.h>
#include <stepper.h>

bool meme;
stepper stepper(12,16,14,5,13);


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  stepper.init();
  //stepper.setMaxFreq(100);
  delay(1000);
	stepper.writeGCONF(1,0,0,0,0,0,1,1,1,0);
	stepper.writeIHOLD_RUN(5,5,0);

	stepper.writeCHOPCONF(0,0,0,1,5,0,0,0,5,3);
}

void loop() {
	stepper.enableStepper();
	stepper.moveStepperUNSAFE(1600,0);
	stepper.writeGCONF(1,0,0,meme,0,0,1,1,1,0);
	meme=!meme;
	
}