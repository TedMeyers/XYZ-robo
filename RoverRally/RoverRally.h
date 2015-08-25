
/* BNO Rover Code
 by Ted Meyers (5/19/2015)
 license: Beerware - Use this code however you'd like. If you 
 find it useful you can buy me a beer some time.
*/
#ifndef _ROVER_RALLY_H__
#define _ROVER_RALLY_H__

// ----------------------------
// Only define one of these (gyros)...
#define USE_MPU
//#define USE_BNO
// ----------------------------

// ----------------------------
// define to have the code set the stop flag (if stopped)
//#define CHECK_FOR_STOP
// define to have the code set the obstacle override flag if stopped flag is set
//#define SET_OBSTACLE_OVERRIDE
// ----------------------------


#if (ARDUINO >= 100)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif
#include <Servo.h>
#ifdef USE_MPU
  #include "helper_3dmath.h"
  #include "XMPU6050.h"
  #include "XYZ_MPU6050.h"
  #define DEFAULT_ADDRESS MPU6050_ADDRESS_A
#endif
#ifdef USE_BNO
  #include "XYZ_BNO055.h"
  #define DEFAULT_ADDRESS BNO055_ADDRESS_A
#endif

#define RAD_TO_DEG 57.2957795
#define DEG_TO_RAD 0.0174532925

#define DEFAULT_WP_THRESH 11.0
#define DEFAULT_WP_FOLLOW_OFFSET 12.0

#define DEFAULT_STEER_CENTER 90
#define DEFAULT_STEER_MIN 55
#define DEFAULT_STEER_MAX 125
#define TURN_THRESH 15.0

#define DEFAULT_THROTTLE_CENTER 1500
#define DEFAULT_THROTTLE_MIN 1200
#define DEFAULT_THROTTLE_MAX 1800
#define DEFAULT_THROTTLE_MIN_FWD 80
#define DEFAULT_THROTTLE_MIN_REV 185
#define DEFAULT_THROTTLE_SCALE 2.0

#define STEER_ADJ_TIME 10.0
#define STEERING_P 1.2
#define STEERING_I 0.3
#define STEERING_D 0.1
#define STEERING_I_CUTOFF 5
#define STEERING_I_LIMIT 60
#define STEERING_EPS 1
#define HEADING_THRESHOLD 10.0

#define STEER_CLOSEST 0
#define STEER_LEFT 1
#define STEER_RIGHT 2

#define STATE_NONE 0
#define STATE_STOP 1
#define STATE_DIST 2
#define STATE_WAYP 3
#define STATE_TURN 4
#define STATE_WAIT 5
#define STATE_BTN  6
#define STATE_CUST 7

#define MODE_WAIT 0
#define MODE_RUN 1

 #define WP_PATH_SIZE 20


class RoverRally
{
  public:
    RoverRally();

    void setSteeringPID(float kp=STEERING_P, float ki=STEERING_I, float kd=STEERING_D);
    void setSteeringValues(int center=DEFAULT_STEER_CENTER, int min=DEFAULT_STEER_MIN, int max=DEFAULT_STEER_MAX);
    void setThrottleValues(int center=DEFAULT_THROTTLE_CENTER, 
        int minFwd=DEFAULT_THROTTLE_MIN_FWD, int minRev=DEFAULT_THROTTLE_MIN_REV,
        int min=DEFAULT_THROTTLE_MIN, int max=DEFAULT_THROTTLE_MAX, int scale=DEFAULT_THROTTLE_SCALE);
    
    void setWaypointConstants(float thresh, float offset) {_wp_thresh=thresh; _wp_follow_offset=offset;}


    int setupRover(int thrPin, int steerPin, int btnPin, int ledPin, int address=DEFAULT_ADDRESS);
    void reset();

    void setUpdateCB(void (*functionPtr)(int)) {_updateCB = functionPtr;}

    void setTickToDistanceConversion(float val) {_ticks_per_distance = val;}
    float convertTicksToDistance(int32_t ticks) {return ticks/_ticks_per_distance;}
    int32_t convertDistanceToTicks(float d) {return (int32_t)(d*_ticks_per_distance);}

    void setStartHeading(float h) {_startMagHeading = h;}
    float getCurrentMagHeading() {return _curMagHeading;}
    float getCurrentHeading() {return _curRelHeading;}
    void resetHeading() {_startMagHeading=_curMagHeading; _toRelHeading=_curMagHeading;}

    bool isStopped() {return _stoppedFlag;}
    void setObstacleOverride(bool isOn) {_obstacleOverride = isOn;}
    bool isObstacleOverride() {return _obstacleOverride;}

    void resetPosition() {_x_pos = 0.0; _y_pos = 0.0;}
    void setPosition(float x, float y) {_x_pos = x; _y_pos = y;}
    float getXPosition() {return _x_pos;}
    float getYPosition() {return _y_pos;}
    float getXWaypoint() {return _x_wp;}
    float getYWaypoint() {return _y_wp;}
    int getCurrentWaypointIndex() {return _c_wp;}
    void addWaypointPath(float x, float y, float t) {setWaypointPath(_c_wp++, x, y, t);}
    
    void clearWaypoints();
    void setWaypointPath(int i, float x, float y, float t);
    void moveWaypointPath(int i, float dx, float dy);

    void backupStraight(float dist) {backup(false, false, dist);}
    void backupLeft(float angle) {backup(true, true, angle);}
    void backupRight(float angle) {backup(true, false, angle);}

    void backAround(bool isLeft, float d1, float d2);
    void backup(bool isTurn, bool isLeft, float angleDist);
    void brakeStop(int duration);
    void rollStop();

    void setLeft() {setDirection(STEER_LEFT);}
    void setRight() {setDirection(STEER_RIGHT);}
    void setClosest() {setDirection(STEER_CLOSEST);}

    void setReverse() {_isReverse = true;}
    void setForward() {_isReverse = false;}
    void setSlowThrottleOn(bool isOn) {_slowOn = isOn;}
    void setSlowThrottlePercent(float percent) {_slowThrottle = calcThrottleFromPercent(percent);}
    void setThrottlePercent(float percent) {_setThrottle = calcThrottleFromPercent(percent);}

    int calcThrottleFromPercent(float percent);
    void setDirection(int dir);
 
    void turnToLeft(float angle);
    void turnToRight(float angle);
    void turnToClosest(float heading);
    void setStraight();
    void setLeftTurn(int angle);
    void setRightTurn(int angle);
    void setHeadingTurn(int hdg);
    void setAngleTurn(int angle);

    float setToHeading(float toHead);
    bool testToHeading(bool testNeg);

    void driveWaypointPath() {driveWaypointPath(_c_wp-1);}
    void driveWaypointPath(int i);
    void driveWaypointLine(float x0, float y0, float x1, float y1, float thresh);  // thresh is the slowdown threshold

    void waitForWaypoint(float toX, float toY, float dist);
    void waitForCustom(bool (*customWaitPtr)());
    void waitForButtonPress();
    void waitForHeading(float toHead);
    void waitForTime(uint32_t val);
    void waitForTicks(uint32_t numTicks);
    void waitForDistance(float dist);
    void waitForCustom();

    void updateAll();
    void updateAllBasic();
    void adjustThrottle();
    void adjustSteering();
    void updateCalibrationLED();
    void updateSteering();
    void updateThrottle();
    void updatePosition();

    int normalizeSteering(int amount);
    int normalizeThrottle(int amount);

    float normalizeDeg180(float h);
    float normalizeDeg360(float heading);
    float calcHeadingDiff(float cur, float to);

    float getHeadingTo(float x, float y);
    float getDistance(float x0, float y0, float x1, float y1);

    float getDistanceFrom(float x, float y) {return getDistance(_x_pos, _y_pos, x, y);}

    void readCalibrationStats(uint8_t *stats);

    int calcThrottleAbsOffset(int val);

    uint32_t markCurrentTicks() {_lastWheelEncoderCount = _wheel_encoder_counter; return _lastWheelEncoderCount;}
    uint32_t getCurrentTicks() {return (_wheel_encoder_counter - _lastWheelEncoderCount);}
    uint32_t getTotalTicks() {return _wheel_encoder_counter;}
    void resetTicks() {_wheel_encoder_counter = 0; _updateWheelEncoderCount = 0;}
    void setWheelEncoderCount(uint32_t val) {_wheel_encoder_counter = val;}

    uint8_t getState() {return _state;}
    uint8_t getMode() {return _curMode;}
    void setModeRun() {_curMode = MODE_RUN;}
    void setModeWait() {_curMode = MODE_WAIT;}


  private:
    #ifdef USE_MPU
        XYZ_MPU6050 _xyz_imu
    #endif
    #ifdef USE_BNO
        XYZ_BNO055 _xyz_imu;
    #endif

    void (*_updateCB)(int);

    Servo _throttleServo; 
    Servo _steeringServo;

    int _buttonPin;
    int _ledPin;

    int _th_center;
    int _th_minFwd;
    int _th_minRev;
    int _th_min;
    int _th_max;
    int _th_scale;

    int _steer_center;
    int _steer_min;
    int _steer_max;

    float _wp_thresh;           // waypoint threshold in feet
    float _wp_follow_offset;    // waypoint follow distance in feet

    uint32_t _start_time;        // start of run
    uint32_t _imu_update_time;   // sensor update time
    uint32_t _steer_adj_time;
    uint32_t _moveCheckTime;

    bool _obstacleOverride;
    bool _slowOn;
    bool _hardTurnOn;
    bool _isReverse;
    bool _throttleOverride;
    bool _stoppedFlag;  // Set when the wheel encoders are not changing
    int _slowThrottle;  // Slow throttle to use (like when turning)
    int _setThrottle;   // The last set throttle
    int _toThrottle;    // The throttle to set to
    int _curThrottle;   // The currently set throttle
    int _toSteer;
    int _curSteer;
    int _steerDir;

    float _startMagHeading;
    float _curMagHeading;
    float _curRelHeading;
    float _lastRelHeading;
    float _toRelHeading;
    float _headingChange;
    float _steerInteg;

    float _steering_Kp;
    float _steering_Ki;
    float _steering_Kd;

    uint8_t _curMode;
    uint8_t _state;

    uint32_t _lastWheelEncoderCount;
    uint32_t _updateWheelEncoderCount;
    uint32_t _moveCheckCount;

    float _ticks_per_distance;
    float _x_pos;
    float _y_pos;

    int _c_wp;
    float _x_wp;
    float _y_wp;
    float _wp_path_x[WP_PATH_SIZE];
    float _wp_path_y[WP_PATH_SIZE];
    float _wp_path_t[WP_PATH_SIZE];

    uint32_t _wheel_encoder_counter;
};

#endif