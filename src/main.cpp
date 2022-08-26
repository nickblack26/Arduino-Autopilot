#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <Plane.h>
#include <Wire.h>
#include <utility/imumaths.h>
#include <Servo.h>

Plane plane;
uint8_t aDelay;
unsigned long current_time, prev_time, dt;

void loopRate(int freq) {
  //DESCRIPTION: Regulate main loop rate to specified frequency in Hz
  /*
   * It's good to operate at a constant loop rate for filters to remain stable and whatnot. Interrupt routines running in the
   * background cause the loop rate to fluctuate. This function basically just waits at the end of every loop iteration until 
   * the correct time has passed since the start of the current loop for the desired loop rate in Hz. 2kHz is a good rate to 
   * be at because the loop nominally will run between 2.8kHz - 4.2kHz. This lets us have a little room to add extra computations
   * and remain above 2kHz, without needing to retune all of our filtering parameters.
   */
  float invFreq = 1.0/freq*1000000.0;
  unsigned long checker = micros();
  
  //Sit in loop until appropriate time has passed
  while (invFreq > (checker - current_time)) {
    checker = micros();
  }
}

void setup() {
    Serial.begin(9600);
    delay(100);
    while(!plane.start(29.84))
    delay(100);
}

void loop() {
    plane.printCurrentState();
    plane.getData();
    // plane.getAbsoluteAttitude();
    plane.getDesiredState();
    plane.getErrors();
    plane.makeCorrections();

    // Regulates loop rate to 200
    // loopRate(2000);
    delay(100);
}
