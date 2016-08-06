/* MPU/BNO Easy Rover Code
 by Ted Meyers (8/4/2016)
 https://github.com/TedMeyers/XYZ-robo

 Copyright (c) 2016, Ted Meyers

 license: Cola-Ware - Use this code however you'd like. If you 
 find it useful you can buy me a Coke some time.

 IMPORTANT: Define USE_MPU or USE_BNO below!

 This library contains most of the necessary features for creating
 an autonomous rover.
*/
#ifndef _XYZ_EASY_ROVER_H__
#define _XYZ_EASY_ROVER_H__

// --------------------------------
// --------- IMPORTANT! -----------
// --------------------------------
// This defines the type of
// gyro to use (MPU605 or BNO-055)
//
// Only define one of these!
// --------------------------------
//
//#define USE_MPU
//#define USE_BNO
#define USE_MPU

// ----------------------------
#if (ARDUINO >= 100)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif
// ----------------------------

#ifdef USE_MPU
  #include "XYZ_MPU6050.h"
  #define DEFAULT_ADDRESS MPU6050_ADDRESS_B
  #define IMU_ADDR MPU6050_ADDRESS_A
#endif
#ifdef USE_BNO
  #include "XYZ_BNO055.h"
  #define DEFAULT_ADDRESS BNO055_ADDRESS_B
  #define IMU_ADDR BNO055_ADDRESS_A
#endif

#include "XYZ_RoverServos.h"
#include "XYZ_WheelEncoder.h"
#include "XYZ_Button.h"
#include "XYZ_LED.h"
// ----------------------------

// ----------------------------
// The following are TUNING PARAMETERS that may need to be adjusted!
// ---------------------------- 
// These values are good: 1.1, 0.3, 0.1
 // Or maybe better: 2.4, 0.3, 0.6
#define STEERING_P 2.4
#define STEERING_I 0.3
#define STEERING_D 0.6
#define STEERING_I_CUTOFF 5
#define STEERING_I_LIMIT 60

// ----------------------------
// END TUNING PARAMETERS!
// ----------------------------

// This is the maximum action array size
#define ACTIONS_SIZE 50

#define DEFAULT_THROTTLE_CENTER_VALUE 0
#define DEFAULT_STEERING_CENTER_VALUE -18

#define IMU_UPDATE_TIME_MS 10
#define SERVO_ADJ_TIME_MS 10

// The following are various state values.
//
#define STEERING_MODE_CLOSEST 0
#define STEERING_MODE_LEFT 1
#define STEERING_MODE_RIGHT 2


class XYZ_EasyRover
{
  public:
    XYZ_EasyRover();

    void setSteeringPID(float kp=STEERING_P, float ki=STEERING_I, float kd=STEERING_D);
    int setupI2C_IMU(int address=DEFAULT_ADDRESS);
    void reset();
    void resetHeadings();
    void resetActions();

    int setAction(int index, int32_t distance, float heading_deg);
    int addAction(int32_t distance, float heading_deg);

    void setSteeringModeDirection(uint8_t steering_mode_dir);

    void updateAll();
    void updateAction();
    void updateIMU();
    void updateIMUInstant();
    void updateServos();
    void updateCalibrationLED();

    float calcSteeringAngle();
    float normalizeDeg180(float h_deg);
    float normalizeDeg360(float h_deg);
    float calcHeadingDiffDeg(float cur_deg, float to_deg);
    float calcHeadingDiff360Deg(float cur_deg, float to_deg);
    void readCalibrationStats(uint8_t *stats);


    XYZ_RoverServos *getServos() { return &_xyz_servos; }
    XYZ_WheelEncoder *getWheelEncoder() { return &_xyz_wheel_encoder; }
    XYZ_Button *getButton() { return &_xyz_button; }
    XYZ_LED *getLED() { return &_xyz_led; }

    void setSteeringCenterValue(int center_value) { _steering_center_value = center_value; }
    void setThrottleCenterValue(int center_value) { _throttle_center_value = center_value; }
    void setSteeringAngleDeg(int deg) { _cur_steering_angle_deg = deg; updateSteeringAngle(); }
    void setThrottlePercent(int percent) { _cur_throttle_percent = percent; updateThrottlePercent(); }
    void updateSteeringAngle() {  setToSteeringAngle(_cur_steering_angle_deg); }
    void updateThrottlePercent() { setToThrottlePercent(_cur_throttle_percent); }

    void restartActions() { reset(); resetHeadings(); _actions_index = 0; }
    int disableActions() { int tmp = _actions_index; _actions_index = -1; return tmp; }
    int enableActionIndex(int index) { _actions_index = index; return index; }
    int incrementActionIndex() { return ++_actions_index; }
    int getActionIndex() { return _actions_index; }
    bool getActionsEnabled() { return (_actions_index>=0 && _actions_index<ACTIONS_SIZE); }
    int32_t getActionDistanceTicks(int index) { return _distance_tick_actions[index]; }
    int32_t getActionDistanceTicks() { return _distance_tick_actions[_actions_index]; }
    float getActionHeadingDeg(int index) { return _heading_deg_actions[index]; }
    float getActionHeadingDeg() { return _heading_deg_actions[_actions_index]; }

    float getCurrentGyroHeadingDeg() {return _cur_gyro_heading_deg;}
    float getCurrentHeadingDeg() {return _cur_rel_heading_deg;}
    int32_t getMarkedTick() { return _last_wheel_encoder_cnt; }
    int32_t getCurrentTicks() {return (getTotalTicks() - _last_wheel_encoder_cnt);}
    int32_t getTotalTicks() {return getWheelEncoder()->getEncoderCount();}

    void setLeftTurnMode() {setSteeringModeDirection(STEERING_MODE_LEFT);}
    void setRightTurnMode() {setSteeringModeDirection(STEERING_MODE_RIGHT);}
    void setClosestTurnMode() {setSteeringModeDirection(STEERING_MODE_CLOSEST);}

    // Only call these functions when not running actions (when actions_index is -1)
    // Calling them while running actions will mess with the actionw
    void setToHeadingDeg(float h_deg) {_to_rel_heading_deg = normalizeDeg360(h_deg);}
    void setToSteeringAngle(int angle) { _xyz_servos.setToSteeringAngle(_steering_center_value+angle); }
    void setToThrottlePercent(int percent) { _xyz_servos.setToThrottlePercent(_throttle_center_value+percent); }
    int32_t markCurrentTicks() {_last_wheel_encoder_cnt = getTotalTicks(); return _last_wheel_encoder_cnt;}

  private:
    #ifdef USE_MPU
        XYZ_MPU6050 _xyz_imu;
    #endif
    #ifdef USE_BNO
        XYZ_BNO055 _xyz_imu;
    #endif

    XYZ_RoverServos _xyz_servos;
    XYZ_WheelEncoder _xyz_wheel_encoder;
    XYZ_Button _xyz_button;         // Button
    XYZ_LED _xyz_led;               // led

    int _cur_steering_angle_deg;
    int _cur_throttle_percent;

    bool _is_I2C_IMU_setup_flag;    // IMU setup flag (true iff IMU setup success)

    float _start_gyro_heading_deg;  // The starting heading (or heading at last reset -- used to calc _cur_rel_heading_deg)
    float _cur_gyro_heading_deg;    // The current gyro heading from the IMU
    float _cur_rel_heading_deg;     // Current heading used by rover, calculated from _start_gyro_heading_deg and _cur_gyro_heading_deg
    float _last_rel_heading_deg;    // Last heading, used in steering PID loop (to update in turns)
    float _to_rel_heading_deg;      // The heading to turn to

    int _steering_center_value;
    int _throttle_center_value;
    float _steer_integ;  // Steering PID control loop integral value
    float _steering_Kp;  // Steering PID control loop constant (P)
    float _steering_Ki;  // Steering PID control loop constant (I)
    float _steering_Kd;  // Steering PID control loop constant (D)

    uint8_t _steering_mode;         // Steering mode (CLOSEST/LEFT/RIGHT)

    int32_t _last_wheel_encoder_cnt;    // Encoder count for marking distances
    int32_t _update_wheel_encoder_cnt;  // Encoder count at last position update

    int _total_actions;
    int _actions_index;
    int32_t _distance_tick_actions[ACTIONS_SIZE];
    float   _heading_deg_actions[ACTIONS_SIZE];
};

#endif