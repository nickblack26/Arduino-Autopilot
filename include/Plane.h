#ifndef __Plane_H__
#define __Plane_H__

#include "Adafruit_BMP085_U.h"
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>

class Plane {
  public:
    /* How quickly to run sensors based phase of flight*/
    typedef enum {
        WAITING = 200,
        TAKEOFF = 50,
        CLIMBING = 100,
        CRUISE = 200,
        BASE = 100,
        FINAL = 50,
        EMERGENCY = 10,
    } phase;

  private:
    Adafruit_BNO055 bno = Adafruit_BNO055(55);
    Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
    sensors_event_t event;
    phase flight_phase = WAITING;

    /* SERVOS */
    Servo aileron_left;
    Servo aileron_right;
    Servo elevator;
    Servo rudder;

    float pressure;
    float temperature;
    float seaLevelPressure;
    float altitude;

    /* HELPERS */
    float TRIM_ARRAY[100];

    /* MAX VALUES */
    uint8_t MAX_ROLL = 30;
    uint8_t MAX_PITCH = 30;
    
    /* HEADING VALUES */
    float heading_target = 0;
    float heading_current;
    float heading_prev = 0;
    float heading_error;
    float heading_base;
    float heading_error_base;

    /* ROLL VALUES */
    float roll_target = 0;
    float roll_current;
    float roll_prev = 0;
    float roll_error;
    float roll_base;
    float roll_error_base;
    float roll_trim = 0;

    /* PITCH VALUES */
    float pitch_target = 0;
    float pitch_current;
    float pitch_prev = 0;
    float pitch_error;
    float pitch_base;
    float pitch_error_base;
    float pitch_trim = 0;

    /* YAW VALUES */
    float yaw_target = 0;
    float yaw_current;
    float yaw_prev = 0;
    float yaw_error;
    float yaw_base;
    float yaw_error_base;
    float yaw_trim = 0;

    /* SPEED VALUES */
    float speed_target;
    float speed_current;
    float speed_prev = 0;
    float speed_error;
    float velocity = 0;

    /* RAW QUATERNION VALUES */
    float q0 = 0.0;
    float q1 = 0.0;
    float q2 = 0.0;
    float q3 = 0.0;

    /* TIME VALUES */
    float start_time;
    float elapsed_time; 

  public:
    void moveServo(Servo *servo, uint8_t value);

    float kalman(float u);

    /*!
     * @brief Low pass filter to filter out noise
     * @param oldValue pass in the previous value
     * @param currentValue pass in the current value
     *
     */
    float filter(float &oldValue, float &currentValue);
    

    void getIMUdata();

    /*!
     * @brief Retrieves raw data from all sensors and passes it through a low pass filter to limit noise
    */
    void getData();

    void getErrors();

    /*!
     * @brief Changes the flight phase to set how fast to run the computer
     * @param phase set the phase based on enum type
     */
    void setPhase(phase phase);

    /*!
     * @brief Gets the value of the flight phase to update the computer speed
     * @return return an integer contained in the enum
     */
    void getPhase(uint8_t *phase_value);

    /*!
     * @brief Starts initialization of plane
     * @return Returns true if successful
     */
    void getPressure();

    /*!
     * @brief Returns temperature of ambient air
     * @return Returns float of ambient air temperature
     */
    void getTemperature();

    /*!
     * @brief Returns temperature of ambient air
     * @return Filtered Alitude
     */
    void getAltitude();

    /*!
     * @brief Prints current state of plane
     */
    void printCurrentState();

    /*!
     * @brief Starts initialization of plane
     * @param pressure Set a pressure for current area
     * @return Returns true if successful
     */
    bool start(float pressure);

    void test();

    /*!
     * @brief Calibrates plane sensors
     * @param recalibrate Read calibration data from memory or redo calibration steps
     * @return Returns true if calibration is complete
     */
    void calibratePlane();

    
};

#endif