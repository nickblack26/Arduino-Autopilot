#ifndef __Motor_H__
#define __Motor_H__

#include <cstdint>

class Motor {
  private:
    int MAX_RPM;
    int MAX_BURST_CURRENT;
    int MAX_CONT_CURRENT;
    int currentRPM;

  public:
    enum Type { BRUSHLESS, BRUSHED };
    /*!
     * @brief Motor Constructor
     * @param rpm, type, max_burst_current, max_cont_current
     */
    Motor(int rpm, Motor::Type type, int max_burst_current, int max_cont_current);
    ~Motor();
};

#endif