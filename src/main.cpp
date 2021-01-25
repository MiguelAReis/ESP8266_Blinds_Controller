#include <Arduino.h>
#include <stepper.h>


stepper stepper(12,16,14,5);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  stepper.init();


}

void loop() {
    stepper.moveStepper(6400,0);
    delay(2000);
    stepper.moveStepper(6400,1);
    delay(2000);


}