#include "Arduino.h"
#include <EEPROM.h>
#include <Plane.h>
#include <SPI.h>
#include <Wire.h>
#include <utility/imumaths.h>

int buttonApin = 2;         // the number of the pushbutton pin

void Plane::moveServo(Servo *aServo, uint8_t value) {
    aServo->write(value); 
}

float Plane::kalman(float u) { 
    static const float r = 40; 
    static const float h = 10; 
    static float q = 10; 
    static float p = 0; 
    static float u_hat = 0; 
    static float k = 0;

    k = p * h / (h * p * h + r);
    u_hat = u_hat + k*(u-h*u_hat);

    p = (1 - k * h) * p + q;

    return u_hat;
}

float Plane::filter(float &oldValue, float &currentValue) {
    return 0.95 * oldValue + 0.05 * currentValue;
}

void Plane::getIMUdata() {
    imu::Vector<3> gyr = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    // thetaM=-atan2(acc.x()/9.8,acc.z()/9.8)/2/3.141592654*360;
    // phiM=-atan2(acc.y()/9.8,acc.z()/9.8)/2/3.141592654*360;
    // phiFnew=.95*phiFold+.05*phiM;
    // dt=(millis()-millisOld)/1000.;
    // millisOld=millis();
    // theta=(theta+gyr.y()*dt)*.95+thetaM*.05;
    // phi=(phi-gyr.x()*dt)*.95+ phiM*.05;
    // thetaG=thetaG+gyr.y()*dt;
    // phiG=phiG-gyr.x()*dt;
    
    // phiRad=phi/360*(2*3.14);
    // thetaRad=theta/360*(2*3.14);
    
    // Xm=mag.x()*cos(thetaRad)-mag.y()*sin(phiRad)*sin(thetaRad)+mag.z()*cos(phiRad)*sin(thetaRad);
    // Ym=mag.y()*cos(phiRad)+mag.z()*sin(phiRad);
    
    // psi=atan2(Ym,Xm)/(2*3.14)*360;

    // phiFold=phiFnew;
    // thetaFold=thetaFnew;
    // Serial.print(acc.x()/9.8);
    // Serial.print(",");
    // Serial.print(acc.y()/9.8);
    // Serial.print(",");
    // Serial.print(acc.z()/9.8);
    // Serial.print(",");
    // Serial.print(thetaM);
    // Serial.print(",");
    // Serial.print(phiM);
    // Serial.print(",");
    // Serial.print(thetaFnew);
    // Serial.print(",");
    // Serial.print(phiFnew);
    // Serial.print(",");
    // Serial.print(thetaG);
    // Serial.print(",");
    // Serial.print(phiG);
    // Serial.print(",");
    // Serial.print(theta);
    // Serial.print(",");
    // Serial.print(phi);
    // Serial.print(",");
    // Serial.println(psi);
}

void Plane::getData() {
    imu::Quaternion quat = bno.getQuat();
    imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

    bmp.getEvent(&event);
    pressure = event.pressure;
    bmp.getTemperature(&temperature);
    altitude = bmp.pressureToAltitude(seaLevelPressure, pressure, temperature);

    q0 = quat.w();
    q1 = quat.x();
    q2 = quat.y();
    q3 = quat.z();

    /* GET ROLL VALUES */
    roll_current = atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2));
    roll_current = roll_current / (2 * PI) * 360;
    roll_current = filter(roll_prev, roll_current);
    roll_prev = roll_current;

    /* GET PITCH VALUES */
    pitch_current = asin(2 * (q0 * q2 - q3 * q1));
    pitch_current = pitch_current / (2 * PI) * 360;
    pitch_current = filter(pitch_prev, pitch_current);
    pitch_prev = pitch_current;

    /* GET YAW VALUES */
    yaw_current = atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3));
    yaw_current = yaw_current / (2 * PI) * 360;
    yaw_current = filter(yaw_prev, yaw_current);
    yaw_prev = yaw_current;

    /* GET HEADING VALUES */
    heading_current = yaw_current * -1;
    heading_current = filter(heading_prev, heading_current);
    if (heading_current < 0) {
        heading_current = 360 + heading_current;
    } 
    heading_prev = heading_current;

    /* GET MISSION TIME */
    elapsed_time = (micros() - start_time) / 10000;
}

void Plane::getErrors() { 
    roll_error = roll_current - roll_target;
    roll_error_base = roll_base - roll_current;
    pitch_error = pitch_current - pitch_target; 
    pitch_error_base = pitch_base - pitch_current;
    yaw_error = yaw_current - yaw_target;
    yaw_error_base = yaw_base - yaw_current;
    heading_error = heading_current - heading_target;
    heading_error_base = (heading_base - heading_current) * -1;
}

void Plane::setPhase(phase phase) { flight_phase = phase; };

void Plane::getPhase(uint8_t *phase_value) {
    *phase_value = flight_phase;
};

void Plane::getPressure() {
    bmp.getEvent(&event);
    pressure = event.pressure;
}

void Plane::getTemperature() { 
    bmp.getTemperature(&temperature);
    temperature = kalman(temperature);
}

void Plane::getAltitude() { altitude = bmp.pressureToAltitude(seaLevelPressure, pressure, temperature); }

void Plane::printCurrentState() {
    Serial.print("Mission Time: ");
    Serial.println(elapsed_time);

    Serial.print("\nroll: ");
    Serial.print(roll_current, 1);
    Serial.print("\terror: ");
    Serial.print(roll_error, 1);
    Serial.print("\tbase: ");
    Serial.print(roll_error_base, 1);
    Serial.println("");

    Serial.print("pitch: ");
    Serial.print(pitch_current, 1);
    Serial.print("\terror: ");
    Serial.print(pitch_error, 1);
    Serial.print("\tbase: ");
    Serial.print(pitch_error_base, 1);
    Serial.println("");

    Serial.print("yaw: ");
    Serial.print(yaw_current, 1);
    Serial.print("\terror: ");
    Serial.print(yaw_error, 1);
    Serial.print("\tbase: ");
    Serial.print(yaw_error_base, 1);
    Serial.println("");

    Serial.print("head: ");
    Serial.print(heading_current, 0);
    Serial.print("\terror: ");
    Serial.print(heading_error, 0);
    Serial.print("\tbase: ");
    Serial.print(heading_error_base, 0);
    Serial.println("");

    // Serial.print("pressure: ");
    // Serial.print(pressure);
    // Serial.print("\ttemperature: ");
    // Serial.print(temperature);
    // Serial.print("\taltitude: ");
    // Serial.print(altitude);
    Serial.println("");
    Serial.println("");
}

void Plane::calibratePlane() {
    uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;
    int eeAddress = 0;
    long bnoID;

    adafruit_bno055_offsets_t calibrationData;

    EEPROM.get(eeAddress, bnoID);
    sensor_t sensor;


    bno.getSensor(&sensor);
    if (bnoID != sensor.sensor_id) {
        Serial.println("\nNo Calibration Data for this sensor exists in EEPROM");
        delay(500);
    } else {
        Serial.println("\nFound Calibration for this sensor in EEPROM.");
        eeAddress += sizeof(long);
        EEPROM.get(eeAddress, calibrationData);

        Serial.println("\n\nRestoring Calibration data to the BNO055...");
        bno.setSensorOffsets(calibrationData);

        Serial.println("\n\nCalibration data loaded into BNO055");
        return;
    }
     
    while(digitalRead(buttonApin) == HIGH) {
        bno.getEvent(&event);
        bno.getCalibration(&system, &gyro, &accel, &mag);
        Serial.print("X: ");
        Serial.print(event.orientation.x, 4);
        Serial.print("\tY: ");
        Serial.print(event.orientation.y, 4);
        Serial.print("\tZ: ");
        Serial.print(event.orientation.z, 4);
        Serial.print("\t\tSys:");
        Serial.print(system, DEC);
        Serial.print(" G:");
        Serial.print(gyro, DEC);
        Serial.print(" A:");
        Serial.print(accel, DEC);
        Serial.print(" M:");
        Serial.println(mag, DEC);
        delay(100);
    }

        // while(!bno.isFullyCalibrated()) {
            // bno.getCalibration(&system, &gyro, &accel, &mag);
            // Serial.print("Sys:");
            // Serial.print(system, DEC);
            // Serial.print(" G:");
            // Serial.print(gyro, DEC);
            // Serial.print(" A:");
            // Serial.print(accel, DEC);
            // Serial.print(" M:");
            // Serial.println(mag, DEC);
        // }
 
        bno.setSensorOffsets(calibrationData);   
}

bool Plane::start(float pressure = SENSORS_PRESSURE_SEALEVELHPA) {
    if (pressure != SENSORS_PRESSURE_SEALEVELHPA) {
        seaLevelPressure = pressure / 0.02953;
    }
     
    // if (!bmp.begin()) {
    //     Serial.print("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!\n");
    //     return false;
    // }
    
    if (!bno.begin()) {
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!\n");
        return false;
    }
    
    bno.setExtCrystalUse(true);
    aileron_left.attach(3);

    // moveServo(&aileron_left, 0);

    pinMode(buttonApin, INPUT_PULLUP);  
    calibratePlane();
    
    // get raw data
    for (uint8_t i = 0; i < 250; i++) {
       getData();
    }

    /* SET BASELINE NUMBERS */
    roll_base = roll_current;
    pitch_base = pitch_current;
    yaw_base = yaw_current;
    heading_base = heading_base;

    /* START MISSION TIMER */
    start_time = micros();
    elapsed_time = 0;

    return true;
}
