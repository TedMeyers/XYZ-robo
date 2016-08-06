/* MPU/BNO Rover Code
 by Ted Meyers (4/4/2016)
 https://github.com/TedMeyers/XYZ-robo

 Copyright (c) 2016, Ted Meyers

 license: Cola-Ware - Use this code however you'd like. If you 
 find it useful you can buy me a Coke some time.

 IMPORTANT: Define USE_MPU or USE_BNO below!

 This library contains most of the necessary features for creating
 an autonomous rover.
*/
#ifndef _XYZ_ROVER_H__
#define _XYZ_ROVER_H__

// --------------------------------
// --------- IMPORTANT! -----------
// --------------------------------
// This defines the type of
// gyro to use (MPU605 or BNO-055)
//
// BNO is preferred!
//
// Only define one of these!
// --------------------------------
//
// MPU = Type 0,
// BNO = Type 1
//
#define IMU_TYPE 0
#if (IMU_TYPE == 0)
  #define USE_MPU
#else
  #define USE_BNO
#endif

//#define USE_PRINT_CB

// -----------------------------------------------------
// The following flags determine if or how
// the RC transmitter can be used to enable
// or disable the rover.
//
// USE_RX_MUX: enables multiplexer capability (being able to
// turn off auto-driver and turn on RC control).
// See XYZ_RoverMux for disable/enable on valid Tx signal.
// -----------------------------------------------------
#define USE_RX_MUX 0

// If USE_RX_MUX is 0 (false) the following are valid:
// -----------------------------------------------------
// USE_RX_AUX_DISABLE: disables auto-driving when the
// auziliary channel is switched high. Also, set 
// USE_RX_SIGNAL_DISABLE to 1 to only allow auto when
// valid tx signal, or 0 to allow regardless.
//
// USE_RX_SIGNAL_DISABLE: disables auto-driving when the
// transmitter is turned on.
// Personally, I like 1/0; Competition == 1/1
// -----------------------------------------------------
#define USE_RX_AUX_DISABLE 0
#define USE_RX_SIGNAL_DISABLE 0
// -----------------------------------------------------

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

#if (USE_RX_MUX)
  #include "XYZ_RoverMux.h"
#endif
#include "XYZ_RxDecoder.h"
#include "XYZ_RoverServos.h"
#include "XYZ_WheelEncoder.h"
#include "XYZ_Button.h"
#include "XYZ_LED.h"
// ----------------------------

// ----------------------------
// The following are TUNING PARAMETERS that may need to be adjusted!
// ---------------------------- 
#define SLOW_THROTTLE_PERCENT 20
#define NORMAL_THROTTLE_PERCENT 40
#define REVERSE_THROTTLE_PERCENT 0

// These values are good: 1.1, 0.3, 0.1
 // Or maybe better: 2.4, 0.3, 0.6
#define STEERING_P 2.4
#define STEERING_I 0.3
#define STEERING_D 0.6
#define STEERING_I_CUTOFF 5
#define STEERING_I_LIMIT 60

#define HEADING_THRESHOLD 10.0
#define HARD_TURN_THRESH_DEG 15.0
// ----------------------------
// END TUNING PARAMETERS!
// ----------------------------

// WP_THRESH_FT should be less than WP_FOLLOW_OFFSET_FT
#define DEFAULT_WP_THRESH_FT 11.0
#define DEFAULT_WP_FOLLOW_OFFSET_FT 12.0

#define DEFAULT_THROTTLE_CENTER_PERCENT 0
#define DEFAULT_STEERING_CENTER_ANGLE -18

#define IMU_UPDATE_TIME_MS 10
#define SERVO_ADJ_TIME_MS 10

// ----------------------------
// define to have the code set the stop flag (if stopped)
//#define CHECK_FOR_STOP
// define to have the code set the obstacle override flag if stopped flag is set
//#define SET_OBSTACLE_OVERRIDE
// ----------------------------

// The following are various state values.
//
#define STEERING_MODE_CLOSEST 0
#define STEERING_MODE_LEFT 1
#define STEERING_MODE_RIGHT 2

#define ACTION_STATE_NONE 0
#define ACTION_STATE_STOP 1
#define ACTION_STATE_DIST 2
#define ACTION_STATE_WAYP 3
#define ACTION_STATE_TURN 4
#define ACTION_STATE_WAIT 5
#define ACTION_STATE_BTN  6
#define ACTION_STATE_CUST 7
#define ACTION_STATE_sTOP 8

#define ROVER_MODE_RUN 0
#define ROVER_MODE_AUTO_HOLD 1
#define ROVER_MODE_MANUAL_HOLD 2
#define ROVER_MODE_ALL_HOLD 3

// This can be decreased if memory runs short;
// it specifies the maximum number of waypoints allowed.
 //
#define WP_PATH_SIZE 40


class XYZ_Rover
{
  public:
    XYZ_Rover();

    #if (USE_RX_MUX)
      XYZ_RoverMux *getMux() { return &_xyz_mux; }
      XYZ_RxDecoder *getRx() { return _xyz_mux.getRx(); }
      XYZ_RoverServos *getServos() { return _xyz_mux.getServos(); }

      void setToSteeringAngle(int angle) { _xyz_mux.setToSteeringAngle(_steering_center_angle+angle); }
      void setToThrottlePercent(int percent) { _xyz_mux.setToThrottlePercent(percent); }
      void servoReset() { _xyz_mux.reset(); }
    #else
      XYZ_RxDecoder *getRx() { return &_xyz_rx; }
      XYZ_RoverServos *getServos() { return &_xyz_servos; }

      void setToSteeringAngle(int angle) { _xyz_servos.setToSteeringAngle(_steering_center_angle+angle); }
      void setToThrottlePercent(int percent) { _xyz_servos.setToThrottlePercent(percent); }
      void servoReset() { _xyz_servos.reset(); }
    #endif

    XYZ_WheelEncoder *getWheelEncoder() { return &_xyz_wheel_encoder; }
    XYZ_Button *getButton() { return &_xyz_button; }
    XYZ_LED *getLED() { return &_xyz_led; }

    void setThrottleCenterPercent(int center_percent) { _throttle_center_percent = center_percent; }
    void setSteeringCenterAngle(int center_angle) { _steering_center_angle = center_angle; }

    void setSteeringPID(float kp=STEERING_P, float ki=STEERING_I, float kd=STEERING_D);

    void setWaypointConstantsFt(float thresh_ft, float offset_ft) {_wp_thresh_ft=thresh_ft; _wp_follow_offset_ft=offset_ft;}

    int setupI2C_IMU(int address=DEFAULT_ADDRESS);
    void reset();

    void setUpdateCB(void (*functionPtr)(int)) {_updateCB = functionPtr;}

    void setSlowThrottlePercent(int percent) { _slow_throttle_percent = percent; }
    void setNormalThrottlePercent(int percent) { _normal_throttle_percent = percent; }
    void setReverseThrottlePercent(int percent) { _reverse_throttle_percent = percent; }

    void setTickToFeetConversion(float val_ft) {_ticks_per_ft = val_ft;}
    float convertTicksToFeet(int32_t ticks) {return ticks/_ticks_per_ft;}
    int32_t convertFeetToTicks(float d_ft) {return (int32_t)(d_ft*_ticks_per_ft);}

    float getCurrentMagHeadingDeg() {return _cur_mag_heading_deg;}
    float getCurrentHeadingDeg() {return _cur_rel_heading_deg;}
    void setToHeadingDeg(float h_deg) {_to_rel_heading_deg = normalizeDeg360(h_deg);}
    void resetHeadings();

    bool isStopped() {return _stopped_flag;}
    void setObstacleOverride(bool isOn) {_obstacle_override_flag = isOn;}
    bool isObstacleOverride() {return _obstacle_override_flag;}

    void setToZeroPosition() {_x_pos_ft = 0.0; _y_pos_ft = 0.0;}
    void setPositionFt(float x_ft, float y_ft) {_x_pos_ft = x_ft; _y_pos_ft = y_ft;}
    float getXPositionFt() {return _x_pos_ft;}
    float getYPositionFt() {return _y_pos_ft;}
    float getXWaypointFt() {return _x_wp_ft;}
    float getYWaypointFt() {return _y_wp_ft;}
    void resetCurrentWaypointIndex() { _c_wp_idx = 1; }
    int getCurrentWaypointIndex() {return _c_wp_idx;}
    void addWaypointPathFt(float x_ft, float y_ft, float t_ft) {setWaypointPathFt(_c_wp_idx++, x_ft, y_ft, t_ft);}
    
    void clearWaypoints();
    void setWaypointPathFt(int i, float x_ft, float y_ft, float t_ft);
    void moveWaypointPathFt(int i, float dx_ft, float dy_ft);

    void brakeStop(int duration_ms);
    void rollStop(int duration_ms);

    void setSteeringModeDirection(uint8_t steering_mode_dir);
    void setLeftTurnMode() {setSteeringModeDirection(STEERING_MODE_LEFT);}
    void setRightTurnMode() {setSteeringModeDirection(STEERING_MODE_RIGHT);}
    void setClosestTurnMode() {setSteeringModeDirection(STEERING_MODE_CLOSEST);}
 
    void turnToLeftDeg(float angle_deg);
    void turnToRightDeg(float angle_deg);
    void turnToClosestDeg(float heading_deg);
    void setStraight();
    void setLeftTurnDeg(float angle_deg);
    void setRightTurnDeg(float angle_deg);
    void setAngleTurnDeg(float angle_deg);

    bool testToHeading(bool testNeg, float toHeadingDeg);

    void driveWaypointPath(int i);
    void driveWaypointLineFt(float x0_ft, float y0_ft, 
        float x1_ft, float y1_ft, float slow_thresh_ft);  // slow_thresh_ft is the slowdown threshold

    void waitForWaypointFt(float toX_ft, float toY_ft, float dist_ft);
    bool checkForWaypointFt(float toX_ft, float toY_ft, float thresh_ft);
    void waitForCustom(bool (*customWaitPtr)());
    void waitForButtonPress();
    void waitForHeadingDeg(float toHead_deg);
    void waitForTimeMs(uint32_t t_ms);
    void waitForTicks(int32_t numTicks);
    void waitForDistanceFt(float dist_ft);
    void waitForCustom();

    void updateAll();
    void updateAllBasic();

    void updatePosition();
    void updateIMU();
    void updateServos();
    void updateMux();
    void updateCalibrationLED();

    void readCalibrationStats(uint8_t *stats);

    int32_t getMarkedTick() { return _last_wheel_encoder_cnt; }
    int32_t markCurrentTicks() {_last_wheel_encoder_cnt = getTotalTicks(); return _last_wheel_encoder_cnt;}
    int32_t getCurrentTicks() {return (getTotalTicks() - _last_wheel_encoder_cnt);}
    int32_t getTotalTicks() {return getWheelEncoder()->getEncoderCount();}
    void setTotalTicks(int32_t val) { getWheelEncoder()->setEncoderCount(val); }
    void resetTotalTicks() { getWheelEncoder()->resetEncoderCount(); }
    
    uint8_t getActionState() {return _rover_action_state;}
    uint8_t getRoverMode() {return _rover_mode;}
    void setRoverModeRun() { setRoverModeAutoHold(false); }
    void setRoverModeHold() { setRoverModeAutoHold(true); }
    void setRoverModeAutoHold(bool b);
    void setRoverModeManualHold(bool b);
    void updateRoverMode();

    void updateSteeringAngle();
    void updateThrottlePercent(int throttle_percent);
    void updateThrottlePercent();

    float normalizeDeg180(float h_deg);
    float normalizeDeg360(float h_deg);
    float calcHeadingDiffDeg(float cur_deg, float to_deg);
    float calcHeadingDiff360Deg(float cur_deg, float to_deg);

    float getHeadingToPosDeg(float x_ft, float y_ft);
    float getDistanceFt(float x0_ft, float y0_ft, float x1_ft, float y1_ft);
    float getDistanceFromFt(float x_ft, float y_ft) {return getDistanceFt(_x_pos_ft, _y_pos_ft, x_ft, y_ft);}


    #ifdef USE_PRINT_CB
        void (*printIntCB)(int);
        void (*printLongCB)(long);
        void (*printFloatCB)(float);
        void (*printStringCB)(char *);
        void (*printlnCB)(char *);
    #endif

  private:
    #ifdef USE_MPU
        XYZ_MPU6050 _xyz_imu;
    #endif
    #ifdef USE_BNO
        XYZ_BNO055 _xyz_imu;
    #endif

    void (*_updateCB)(int);         // Update callback function

    #if (USE_RX_MUX)
       XYZ_RoverMux _xyz_mux;       // Throttle and steering servos (with mux) & wheel encoders
    #else
       XYZ_RxDecoder _xyz_rx;
       XYZ_RoverServos _xyz_servos;
    #endif
    XYZ_WheelEncoder _xyz_wheel_encoder;
    XYZ_Button _xyz_button;         // Button
    XYZ_LED _xyz_led;               // led

    int _throttle_center_percent;
    int _steering_center_angle;

    float _wp_thresh_ft;            // waypoint threshold in feet
    float _wp_follow_offset_ft;     // waypoint follow distance in feet

    bool _is_I2C_IMU_setup_flag;    // IMU setup flag (true iff IMU setup success)
    bool _stopped_flag;             // Set when the wheel encoders are not changing
    bool _obstacle_override_flag;   // Set when an obsstacle detected
    bool _slow_flag;                // Set when rover needs to go slow
    bool _hard_turn_flag;           // Set when rover needs to make a hard turn
    bool _is_reverse_flag;          // Set when rover is backing up
    bool _use_cur_throttle_flag;    // Set when current throttle is to be used instead of default

    int _cur_throttle_percent;      // Current throttle value (in percent)
    int _slow_throttle_percent;     // Slow default throttle value (in percent)
    int _normal_throttle_percent;   // Normal default throttle value (in percent)
    int _reverse_throttle_percent;  // Reverse default throttle value (in percent)

    float _start_mag_heading_deg;   // The starting heading (or heading at last reset -- used to calc _cur_rel_heading_deg)
    float _cur_mag_heading_deg;     // The current magnetic heading from the IMU
    float _cur_rel_heading_deg;     // Current heading used by rover, calculated from _start_mag_heading_deg and _cur_mag_heading_deg
    float _last_rel_heading_deg;    // Last heading, used in steering PID loop (to update in turns)
    float _to_rel_heading_deg;      // The heading to turn to

    float _steer_integ;  // Steering PID control loop integral value
    float _steering_Kp;  // Steering PID control loop constant (P)
    float _steering_Ki;  // Steering PID control loop constant (I)
    float _steering_Kd;  // Steering PID control loop constant (D)

    bool _auto_hold_mode;           // Is auto hold enabled
    bool _manual_hold_mode;         // Is manual hold enabled
    uint8_t _rover_mode;            // Current rover mode (MANUAL_HOLD / AUTO_HOLD / RUN)
    uint8_t _steering_mode;         // Steering mode (CLOSEST/LEFT/RIGHT)
    uint8_t _rover_action_state;    // Internal state (current large scale action being performed)

    int32_t _last_wheel_encoder_cnt;    // Encoder count for marking distances
    int32_t _update_wheel_encoder_cnt;  // Encoder count at last position update
    int32_t _move_check_cnt;            // Enc count for determining if rover is stopped/stuck

    float _ticks_per_ft;  // Factor for converting encoder ticks to feet
    float _x_pos_ft;      // Current X position of rover in feet
    float _y_pos_ft;      // Current Y position of rover in feet

    // Waypoint Values
    float _x_wp_ft; // Current waypoint X position in feet
    float _y_wp_ft; // Current waypoint Y position in feet
    int _c_wp_idx;  // Current array index
    float _wp_path_x_ft[WP_PATH_SIZE];  // X position of waypoints in feet
    float _wp_path_y_ft[WP_PATH_SIZE];  // Y position of waypoints in feet
    float _wp_path_t_ft[WP_PATH_SIZE];  // Threshold to waypoints in feet

    void waitForTimeMsInternal(uint32_t t_ms);
};

#endif