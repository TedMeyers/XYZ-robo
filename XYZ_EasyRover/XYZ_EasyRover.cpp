/* BNO/MPU Easy Rover Code
 by Ted Meyers (4/4/2016)
 https://github.com/TedMeyers/XYZ-robo

 Copyright (c) 2016, Ted Meyers

 license: Cola-Ware - Use this code however you'd like. If you 
 find it useful you can buy me a Coke some time.

 IMPORTANT: Go to XYZ_EasyRover.h and set the gyro type to be use.  This
            setting gets compiled in, and you have to pick one.
*/
#include "XYZ_EasyRover.h"

/***************************************************************************
 CONSTRUCTOR
 ***************************************************************************/
XYZ_EasyRover::XYZ_EasyRover()
{
  _steering_center_value = DEFAULT_STEERING_CENTER_VALUE;
  _cur_throttle_percent = DEFAULT_THROTTLE_CENTER_VALUE;

  _is_I2C_IMU_setup_flag = false;
  _start_gyro_heading_deg = 0.0;

  _steering_Kp = STEERING_P;
  _steering_Ki = STEERING_I*(SERVO_ADJ_TIME_MS/1000.0);
  _steering_Kd = STEERING_D/(SERVO_ADJ_TIME_MS/1000.0);
  _actions_index = -1; 

  _steering_mode = STEERING_MODE_CLOSEST;

  _xyz_wheel_encoder.resetEncoderCount();
  _xyz_servos.reset();
  reset();
  resetActions();
}

void XYZ_EasyRover::setSteeringPID(float kp, float ki, float kd) 
{
  _steering_Kp = kp;
  _steering_Ki = ki*(SERVO_ADJ_TIME_MS/1000.0);
  _steering_Kd = kd/(SERVO_ADJ_TIME_MS/1000.0);
}

int XYZ_EasyRover::setupI2C_IMU(int address)
{
  if (_is_I2C_IMU_setup_flag) return 1;

  // Initialise the IMU sensor
  _xyz_imu.setupI2C();

  int status = 0;
  #ifdef USE_MPU
    status = (_xyz_imu.setup(address))?0:1;
  #endif
  #ifdef USE_BNO
    if (!_xyz_imu.setup(address)) {
      status = 0;
    } else {
      _xyz_imu.setMode(XYZ_BNO055::IMU);  // IMU or NDOF
      status = _xyz_imu.getMode();
    }
  #endif

  if (status > 1) {
    _is_I2C_IMU_setup_flag = true;
  }

  return status;
}

void XYZ_EasyRover::reset()
{
  uint16_t cur_cnt = getTotalTicks();
  _last_wheel_encoder_cnt = cur_cnt;
  _update_wheel_encoder_cnt = cur_cnt;

  _steer_integ = 0.0;

  _start_gyro_heading_deg = 0.0;
  _cur_rel_heading_deg = 0.0;
  _to_rel_heading_deg = 0.0;
  _last_rel_heading_deg = 0.0;
}

void XYZ_EasyRover::resetHeadings()
{
  updateIMUInstant();
  _start_gyro_heading_deg = _cur_gyro_heading_deg;
  _cur_rel_heading_deg = 0.0;
  _to_rel_heading_deg = _cur_rel_heading_deg;
  _last_rel_heading_deg = _cur_rel_heading_deg;
}

void XYZ_EasyRover::resetActions() {
  for (int i=0; i<ACTIONS_SIZE; i++) {
    _distance_tick_actions[i] = -1;
    _heading_deg_actions[i] = 0.0;
  }
  _actions_index = -1;
}

int XYZ_EasyRover::setAction(int index, int32_t distance_ticks, float heading_deg)
{
  if (index >= 0 && index < ACTIONS_SIZE) {
    _distance_tick_actions[index] = distance_ticks;
    _heading_deg_actions[index] = heading_deg;
    return index;
  }
  return -1;
}

// Find the first empty entry and set the action.
int XYZ_EasyRover::addAction(int32_t distance_ticks, float heading_deg)
{
  int i=0;
  for (i=0; i<ACTIONS_SIZE; i++) {
    if (_distance_tick_actions[i]<0) break;
  }
  return setAction(i, distance_ticks, heading_deg);
}

void XYZ_EasyRover::setSteeringModeDirection(uint8_t steering_mode_dir)
{
  if (_steering_mode != steering_mode_dir) {
    if (steering_mode_dir == STEERING_MODE_LEFT) _steering_mode = STEERING_MODE_LEFT;
    else if (steering_mode_dir == STEERING_MODE_RIGHT) _steering_mode = STEERING_MODE_RIGHT;
    else _steering_mode = STEERING_MODE_CLOSEST;
  }
}


void XYZ_EasyRover::updateAll()
{
  updateAction();
  updateIMU();
  updateServos();
  updateCalibrationLED();
}

void XYZ_EasyRover::updateAction()
{
  if (_actions_index >= 0 && _actions_index < ACTIONS_SIZE) {
    int32_t c = getCurrentTicks();
    int32_t d = _distance_tick_actions[_actions_index];
    if (c < d) {
      setToHeadingDeg(_heading_deg_actions[_actions_index]);
      _cur_steering_angle_deg = (int) calcSteeringAngle();
    } else {
      _actions_index++;
      markCurrentTicks();
    }
  }
}

void XYZ_EasyRover::updateIMU()
{
  static uint32_t _s_imu_update_time_ms = 0;

  if ((millis() - _s_imu_update_time_ms) >= IMU_UPDATE_TIME_MS) {
    _s_imu_update_time_ms = millis();
    updateIMUInstant();
  }  
}

void XYZ_EasyRover::updateIMUInstant() {
    #ifdef USE_BNO
      _cur_gyro_heading_deg = normalizeDeg360(_xyz_imu.readHeading_deg());
    #endif
    #ifdef USE_MPU
      _xyz_imu.updateYPR(); 
      _cur_gyro_heading_deg = normalizeDeg360(_xyz_imu.getHeading());
    #endif
    _cur_rel_heading_deg = calcHeadingDiff360Deg(_cur_gyro_heading_deg, _start_gyro_heading_deg);
}

void XYZ_EasyRover::updateServos()
{
  static uint32_t _s_servo_update_time_ms = 0;

  if ((millis() - _s_servo_update_time_ms) >= SERVO_ADJ_TIME_MS) {
    _s_servo_update_time_ms = millis();
    updateSteeringAngle();
    updateThrottlePercent();
  }  
}

void XYZ_EasyRover::updateCalibrationLED() 
{
  bool b = false;
  #ifdef USE_BNO
    uint8_t stats[4];
    readCalibrationStats(stats);
    b = (stats[1]==3); //((stats[0]==3) && (stats[1]==3) && (stats[3]==3));
  #endif
  #ifdef USE_MPU
    static float cal1 = 0.0;
    float cal2 = _xyz_imu.getHeading();
    b = (abs(cal1-cal2) < 0.07);
    cal1 = cal2;
  #endif
  _xyz_led.set(b);
}

float XYZ_EasyRover::calcSteeringAngle()
{
  float hc_deg = -calcHeadingDiffDeg(_to_rel_heading_deg, _cur_rel_heading_deg);
  uint8_t dir = _steering_mode;
  if (hc_deg > -20 && hc_deg < 20) {
    dir = STEERING_MODE_CLOSEST;
  } else if ((dir == STEERING_MODE_LEFT) && (hc_deg < -20)) {
    hc_deg += 360.0;
  } else if ((dir == STEERING_MODE_RIGHT) && (hc_deg > 20)) {
    hc_deg -= 360.0;
  }
  if (hc_deg > 60.0) hc_deg = 60.0;
  if (hc_deg < -60.0) hc_deg = -60.0;

  float hd_deg = calcHeadingDiffDeg(_cur_rel_heading_deg, _last_rel_heading_deg);
  _last_rel_heading_deg = _cur_rel_heading_deg;
  if (hc_deg < STEERING_I_CUTOFF && hc_deg > -STEERING_I_CUTOFF) _steer_integ -= hc_deg;

  float iTerm = (_steer_integ * _steering_Ki);
  if (iTerm > STEERING_I_LIMIT) iTerm = STEERING_I_LIMIT;
  if (iTerm < -STEERING_I_LIMIT) iTerm = -STEERING_I_LIMIT;

  hc_deg = (hc_deg * _steering_Kp) - iTerm + (hd_deg * _steering_Kd);
  
  // Center dead zone...
  if ((hc_deg > -0.2) && (hc_deg < 0.2)) {
    hc_deg = 0.0;
  }
  return hc_deg;
}

float XYZ_EasyRover::normalizeDeg180(float d_deg)
{
  if (d_deg >  180.0) d_deg -= 360.0;
  if (d_deg >  180.0) d_deg -= 360.0;
  if (d_deg < -180.0) d_deg += 360.0;
  if (d_deg < -180.0) d_deg += 360.0;
  return d_deg;
}
float XYZ_EasyRover::normalizeDeg360(float heading)
{
  if (heading < 0) heading += 360;
  if (heading < 0) heading += 360;
  if (heading > 360) heading -= 360;
  if (heading > 360) heading -= 360;
  return heading;
}
float XYZ_EasyRover::calcHeadingDiffDeg(float cur_deg, float to_deg)
{
  return normalizeDeg180(cur_deg - to_deg);
}
float XYZ_EasyRover::calcHeadingDiff360Deg(float cur_deg, float to_deg) {
  return normalizeDeg360(cur_deg - to_deg);
}

void XYZ_EasyRover::readCalibrationStats(uint8_t *stats) {
  #ifdef USE_BNO
    _xyz_imu.readCalibration(stats);
  #endif
}
