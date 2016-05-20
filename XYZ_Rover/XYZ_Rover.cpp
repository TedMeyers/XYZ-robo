/* BNO/MPU Rover Code
 by Ted Meyers (4/4/2016)
 https://github.com/TedMeyers/XYZ-robo

 Copyright (c) 2016, Ted Meyers

 license: Cola-Ware - Use this code however you'd like. If you 
 find it useful you can buy me a Coke some time.

 IMPORTANT: Go to XYZ_Rover.h and set the gyro type to be use.  This
            setting gets compiled in, and you have to pick one.
*/
#include "XYZ_Rover.h"

/***************************************************************************
 CONSTRUCTOR
 ***************************************************************************/
XYZ_Rover::XYZ_Rover()
{
  _updateCB = 0;

  _throttle_center_percent = DEFAULT_THROTTLE_CENTER_PERCENT;
  _steering_center_angle = DEFAULT_STEERING_CENTER_ANGLE;

  _slow_throttle_percent = SLOW_THROTTLE_PERCENT;
  _normal_throttle_percent = NORMAL_THROTTLE_PERCENT;
  _reverse_throttle_percent = REVERSE_THROTTLE_PERCENT;

  _is_I2C_IMU_setup_flag = false;
  _start_time_ms = 0;
  _start_mag_heading_deg = 0.0;

  _steering_Kp = STEERING_P;
  _steering_Ki = STEERING_I*(SERVO_ADJ_TIME_MS/1000.0);
  _steering_Kd = STEERING_D/(SERVO_ADJ_TIME_MS/1000.0);

  _ticks_per_ft = 1.0;

  resetTotalTicks();
  setWaypointConstantsFt(DEFAULT_WP_THRESH_FT, DEFAULT_WP_FOLLOW_OFFSET_FT);

  reset();
  clearWaypoints();
  addWaypointPathFt(0.0, 0.0, 0.0);
}

void XYZ_Rover::setSteeringPID(float kp, float ki, float kd) 
{
  _steering_Kp = kp;
  _steering_Ki = ki*(SERVO_ADJ_TIME_MS/1000.0);
  _steering_Kd = kd/(SERVO_ADJ_TIME_MS/1000.0);
}

int XYZ_Rover::setupI2C_IMU(int address)
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

void XYZ_Rover::reset()
{
  // CENTER SERVOS

  _slow_flag = false;
  _hard_turn_flag = false;
  _obstacle_override_flag = false;
  _is_reverse_flag = false;
  _stopped_flag = false;
  _use_cur_throttle_flag = false;

  _cur_throttle_percent = 0;

  _c_wp_idx = 1;
  _x_wp_ft = 0.0;
  _y_wp_ft = 0.0;

  _steer_integ = 0.0;

  uint16_t cur_cnt = getTotalTicks();
  _last_wheel_encoder_cnt = cur_cnt;
  _update_wheel_encoder_cnt = cur_cnt;
  _move_check_cnt = 0;

  _rover_action_state = ACTION_STATE_NONE;
  _rover_mode = ROVER_MODE_AUTO_HOLD;
  _auto_hold_mode = true;
  _manual_hold_mode = false;
  _steering_mode = STEERING_MODE_CLOSEST;

  _start_mag_heading_deg = 0.0;
  _cur_rel_heading_deg = 0.0;
  _to_rel_heading_deg = 0.0;
  _last_rel_heading_deg = 0.0;

  setToZeroPosition();
  servoReset();

  _start_time_ms = millis();
}

void XYZ_Rover::resetHeadings() {
  updateIMU();
  _start_mag_heading_deg = _cur_mag_heading_deg;
  _cur_rel_heading_deg = 0.0;
  _to_rel_heading_deg = _cur_rel_heading_deg;
  _last_rel_heading_deg = _cur_rel_heading_deg;
}


void XYZ_Rover::setRoverModeAutoHold(bool b) 
{
  _auto_hold_mode = b;
  updateRoverMode();
}
void XYZ_Rover::setRoverModeManualHold(bool b) 
{
  _manual_hold_mode = b;
  updateRoverMode();
}
void XYZ_Rover::updateRoverMode() 
{
  if (_auto_hold_mode && _manual_hold_mode) _rover_mode = ROVER_MODE_ALL_HOLD;
  else if (_auto_hold_mode) _rover_mode = ROVER_MODE_AUTO_HOLD;
  else if (_manual_hold_mode) _rover_mode = ROVER_MODE_MANUAL_HOLD;
  else _rover_mode = ROVER_MODE_RUN;
  updateThrottlePercent();
  updateSteeringAngle();
}

void XYZ_Rover::clearWaypoints() 
{
  for (int i=0; i<WP_PATH_SIZE; i++) {
    _wp_path_x_ft[i] = 0.0;
    _wp_path_y_ft[i] = 0.0;
    _wp_path_t_ft[i] = -1.0;
  }  
  _wp_path_t_ft[0] = 0;

}
void XYZ_Rover::setWaypointPathFt(int i, float x_ft, float y_ft, float t_ft)
{
  _wp_path_x_ft[i] = x_ft;
  _wp_path_y_ft[i] = y_ft;
  _wp_path_t_ft[i] = t_ft;
}
void XYZ_Rover::moveWaypointPathFt(int i, float dx_ft, float dy_ft)
{
  _wp_path_x_ft[i] += dx_ft;
  _wp_path_y_ft[i] += dy_ft;
}

void XYZ_Rover::setSteeringModeDirection(uint8_t steering_mode_dir)
{
  if (_steering_mode != steering_mode_dir) {
    if (steering_mode_dir == STEERING_MODE_LEFT) _steering_mode = STEERING_MODE_LEFT;
    else if (steering_mode_dir == STEERING_MODE_RIGHT) _steering_mode = STEERING_MODE_RIGHT;
    else _steering_mode = STEERING_MODE_CLOSEST;
  }
}

void XYZ_Rover::turnToLeftDeg(float angleDeg)
{
  uint8_t steering_mode_dir = _steering_mode;
  setLeftTurnDeg(angleDeg);
  waitForHeadingDeg(_to_rel_heading_deg);
  _steering_mode = steering_mode_dir;
}
void XYZ_Rover::turnToRightDeg(float angleDeg)
{
  uint8_t steering_mode_dir = _steering_mode;
  setRightTurnDeg(angleDeg);
  waitForHeadingDeg(_to_rel_heading_deg);
  _steering_mode = steering_mode_dir;
}
void XYZ_Rover::turnToClosestDeg(float headingDeg)
{
  uint8_t steering_mode_dir = _steering_mode;
  setClosestTurnMode();
  setToHeadingDeg(headingDeg);
  waitForHeadingDeg(_to_rel_heading_deg);
  _steering_mode = steering_mode_dir;
}
void XYZ_Rover::setStraight()
{
  setClosestTurnMode();
  setAngleTurnDeg(0);
}
void XYZ_Rover::setLeftTurnDeg(int angleDeg)
{
  setLeftTurnMode();
  setAngleTurnDeg(-angleDeg);
}
void XYZ_Rover::setRightTurnDeg(int angleDeg)
{
  setRightTurnMode();
  setAngleTurnDeg(angleDeg);
}
void XYZ_Rover::setAngleTurnDeg(int angleDeg)
{
  int headingDeg = normalizeDeg360(_to_rel_heading_deg + angleDeg);
  setToHeadingDeg(headingDeg);
}

bool XYZ_Rover::testToHeading(bool testNeg, float toHeadingDeg) {
  float deg = calcHeadingDiffDeg(_cur_rel_heading_deg, toHeadingDeg);
  return (testNeg) ? (deg < -HEADING_THRESHOLD) : (deg > HEADING_THRESHOLD);
}


// Duration is how long in millis to brake
void XYZ_Rover::brakeStop(int duration_ms) 
{
  _rover_action_state = ACTION_STATE_sTOP;
  updateAll();
  int dir = getServos()->getThrottleDirection();
  bool tmp = _use_cur_throttle_flag;
  _use_cur_throttle_flag = true;
  if (dir >= 0) {
    updateThrottlePercent(-50);
    waitForTimeMsInternal(200);
    updateThrottlePercent(0);
    waitForTimeMsInternal(200);
    updateThrottlePercent(-80);
  } else if (dir < 0) {
    updateThrottlePercent(50);
    waitForTimeMsInternal(200);
    updateThrottlePercent(0);
    waitForTimeMsInternal(200);
    updateThrottlePercent(80);
  }

  waitForTimeMsInternal(duration_ms);

  updateThrottlePercent(0);
  _use_cur_throttle_flag = tmp;
  _rover_action_state = ACTION_STATE_NONE;
  updateAll();
}

// Duration in millis is how long to roll
void XYZ_Rover::rollStop(int duration_ms)
{
  _rover_action_state = ACTION_STATE_sTOP;
  updateAll();
  bool tmp = _use_cur_throttle_flag;
  _use_cur_throttle_flag = true;
  updateThrottlePercent(0);
  waitForTimeMsInternal(duration_ms);
  _use_cur_throttle_flag = tmp;
  _rover_action_state = ACTION_STATE_NONE;
  updateAll();
}


void XYZ_Rover::waitForHeadingDeg(float toHeadDeg)
{
  _rover_action_state = ACTION_STATE_TURN;
  updateAll();
  if (toHeadDeg != _to_rel_heading_deg) {
    setToHeadingDeg(toHeadDeg);
  }

  float diff = calcHeadingDiffDeg(_cur_rel_heading_deg, toHeadDeg);
  bool testNeg = (diff < 0.0);

  while (testToHeading(testNeg, toHeadDeg)) {  
    updateAll();
  }
  _rover_action_state = ACTION_STATE_NONE;
  updateAll();
}

void XYZ_Rover::waitForButtonPress()
{
  _rover_action_state = ACTION_STATE_BTN;

  uint32_t time = millis();
  while (!_xyz_button.waitForPress(1)) {
    if ((millis() - time) > 100) {
      updateCalibrationLED();
      time = millis();
    }
    updateAll();
  }
  _rover_action_state = ACTION_STATE_NONE;
  updateAll();
}

void XYZ_Rover::waitForTimeMs(uint32_t msecs)
{
  _rover_action_state = ACTION_STATE_WAIT;
  waitForTimeMsInternal(msecs);
  _rover_action_state = ACTION_STATE_NONE;
  updateAll();
}

void XYZ_Rover::waitForTimeMsInternal(uint32_t msecs)
{
  updateAll();
  uint32_t endtime_ms = millis() + msecs;
  while (millis() < endtime_ms) {
    updateAll();
  }
}
void XYZ_Rover::waitForDistanceFt(float dist_ft)
{
  int32_t ticks = convertFeetToTicks(dist_ft);
  waitForTicks(ticks);
}

void XYZ_Rover::waitForTicks(int32_t numTicks)
{
  _rover_action_state = ACTION_STATE_DIST;
  markCurrentTicks();
  updateAll();
  if (numTicks > 0) {
    while (getCurrentTicks() < numTicks) {
      updateAll();
    }
  } else if (numTicks < 0) {
    while (getCurrentTicks() > numTicks) {
      updateAll();
    }    
  }
  _rover_action_state = ACTION_STATE_NONE;
  updateAll();
}

void XYZ_Rover::waitForCustom(bool (*customWaitPtr)())
{
  _rover_action_state = ACTION_STATE_CUST;
  updateAll();
  while (customWaitPtr()) {
    updateAll();
  }
  _rover_action_state = ACTION_STATE_NONE;
  updateAll();
}

void XYZ_Rover::waitForWaypointFt(float toX_ft, float toY_ft, float thresh_ft)
{
  _rover_action_state = ACTION_STATE_WAYP;
  _x_wp_ft = toX_ft;
  _y_wp_ft = toY_ft;

  updateAll();
  while (!checkForWaypointFt(toX_ft, toY_ft, thresh_ft)) {
    updateAll();
  }
  _rover_action_state = ACTION_STATE_NONE;
  updateAll();
}

// Drive is like a waitFor function (it calls updateAll a lot)
//
// This function creates a series of short term temporary waypoints
// along the line between the two given points.  It calls
// waitForWAypoint to drive to each temporary waypoint.
//
void XYZ_Rover::driveWaypointLineFt(float x0_ft, float y0_ft, 
  float x1_ft, float y1_ft, float slow_thresh_ft)
{
  _rover_action_state = ACTION_STATE_WAYP;
  _x_wp_ft = x1_ft;
  _y_wp_ft = y1_ft;
  updateAll();

  float d1_ft = getDistanceFromFt(x1_ft, y1_ft); 
  float dt_ft = getDistanceFt(x0_ft, y0_ft, x1_ft, y1_ft);
  float dx = (x1_ft-x0_ft)/dt_ft;
  float dy = (y1_ft-y0_ft)/dt_ft;

  while (!_obstacle_override_flag) {
    float d_ft = (dt_ft - d1_ft) + _wp_follow_offset_ft;
    float tmp_x_wp_ft = x0_ft + (d_ft*dx);
    float tmp_y_wp_ft = y0_ft + (d_ft*dy);

    while (!_obstacle_override_flag) {
      updateAll();
      d1_ft = getDistanceFromFt(x1_ft, y1_ft);
      _slow_flag = (d1_ft < slow_thresh_ft);
      if (d1_ft < _wp_thresh_ft) break;
      updateAll();
      if (checkForWaypointFt(tmp_x_wp_ft, tmp_y_wp_ft, _wp_thresh_ft)) break;
    }
    if (d1_ft < _wp_thresh_ft) break;
  }
  _rover_action_state = ACTION_STATE_NONE;
  updateAll();
}

bool XYZ_Rover::checkForWaypointFt(float toX_ft, float toY_ft, float thresh_ft)
{
  if (_obstacle_override_flag) return true;

  int headingDeg = getHeadingToPosDeg(toX_ft, toY_ft);
  setToHeadingDeg(headingDeg);

  float d_ft = getDistanceFromFt(toX_ft, toY_ft);
  return (d_ft <= thresh_ft);
}

// Drive is like a waitFor function (it calls updateAll a lot)
void XYZ_Rover::driveWaypointPath(int i)
{
  updateAll();
  _c_wp_idx = (i+1);
  while (_c_wp_idx > 0 && _c_wp_idx < WP_PATH_SIZE && _wp_path_t_ft[_c_wp_idx] >= 0) {
    int c0_idx = _c_wp_idx-1;
    float x0_ft = _wp_path_x_ft[c0_idx];
    float y0_ft = _wp_path_y_ft[c0_idx];
    float x1_ft = _wp_path_x_ft[_c_wp_idx];
    float y1_ft = _wp_path_y_ft[_c_wp_idx];

    driveWaypointLineFt(x0_ft, y0_ft, x1_ft, y1_ft, _wp_path_t_ft[_c_wp_idx]);
    if (_obstacle_override_flag) break; else _c_wp_idx++;
  }
  updateAll();
}


void XYZ_Rover::updateAll() {
  if (_updateCB) (*_updateCB)(_rover_action_state);
  else updateAllBasic();
}

void XYZ_Rover::updateAllBasic()
{
  updateIMU();
  updatePosition();
  updateServos();
  updateMux();
}

void XYZ_Rover::updateIMU()
{
  static uint32_t _s_imu_update_time_ms = 0;

  if ((millis() - _s_imu_update_time_ms) >= IMU_UPDATE_TIME_MS) {
    _s_imu_update_time_ms = millis();
    #ifdef USE_BNO
      _cur_mag_heading_deg = normalizeDeg360(_xyz_imu.readHeading());
    #endif
    #ifdef USE_MPU
      _xyz_imu.updateYPR(); 
      _cur_mag_heading_deg = normalizeDeg360(_xyz_imu.getHeading());
    #endif
    _cur_rel_heading_deg = calcHeadingDiff360Deg(_cur_mag_heading_deg, _start_mag_heading_deg);
  }  
}

void XYZ_Rover::updateServos()
{
  static uint32_t _s_servo_update_time_ms = 0;

  if ((millis() - _s_servo_update_time_ms) >= SERVO_ADJ_TIME_MS) {
    _s_servo_update_time_ms = millis();
    updateSteeringAngle();
    updateThrottlePercent();
  }  
}


// Update the MUX (for doing a rover hold).
//
// NOTE: This also can do a rover hold by
// checking the aux channel or just the
// presence of a Tx signal.
void XYZ_Rover::updateMux()
{
  #if (USE_RX_MUX)
    _xyz_mux.update();
    setRoverModeManualHold(_xyz_mux.isManualMode());
  #elif (USE_RX_AUX_DISABLE)
    if (getRx()->checkSignal()) {
      setRoverModeManualHold(getRx()->getAuxiliaryMicros() > 1700);
    } else {
      setRoverModeManualHold(USE_RX_SIGNAL_DISABLE);
    }
  #elif (USE_RX_SIGNAL_DISABLE)
    setRoverModeManualHold(getRx()->checkSignal());
  #endif
}

void XYZ_Rover::updateCalibrationLED() 
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

void XYZ_Rover::updatePosition() {
  uint16_t cur_cnt = getTotalTicks();
  if (_update_wheel_encoder_cnt != cur_cnt) {
    float h_rad = _cur_rel_heading_deg * DEG_TO_RAD;
    float d_ft = (cur_cnt - _update_wheel_encoder_cnt) / _ticks_per_ft;
    _x_pos_ft += d_ft * cos(h_rad);
    _y_pos_ft += d_ft * sin(h_rad);
    _update_wheel_encoder_cnt = cur_cnt;
    _is_reverse_flag = (d_ft < 0);
  }
}

void XYZ_Rover::updateSteeringAngle()
{
  int st_ang = 0;
  if (_rover_mode != ROVER_MODE_RUN) {
    setToSteeringAngle(st_ang);
    return;
  }

  float hc_deg = -calcHeadingDiffDeg(_to_rel_heading_deg, _cur_rel_heading_deg);
  uint8_t dir = _steering_mode;
  if (hc_deg > -5 && hc_deg < 5) dir = STEERING_MODE_CLOSEST;
  if ((dir == STEERING_MODE_LEFT) && (hc_deg < -5)) {
    hc_deg += 360.0;
  } else if ((dir == STEERING_MODE_RIGHT) && (hc_deg > 5)) {
    hc_deg -= 360.0;
  }

  float hd_deg = calcHeadingDiffDeg(_cur_rel_heading_deg, _last_rel_heading_deg);
  _last_rel_heading_deg = _cur_rel_heading_deg;
  if (hc_deg < STEERING_I_CUTOFF && hc_deg > -STEERING_I_CUTOFF) _steer_integ -= hc_deg;

  float iTerm = (_steer_integ * _steering_Ki);
  if (iTerm > STEERING_I_LIMIT) iTerm = STEERING_I_LIMIT;
  if (iTerm < -STEERING_I_LIMIT) iTerm = -STEERING_I_LIMIT;

  hc_deg = (hc_deg * _steering_Kp) - iTerm + (hd_deg * _steering_Kd);
  
  if (_is_reverse_flag) hc_deg *= -1.0;
  if ((hc_deg < -0.2) || (hc_deg > 0.2)) {
    st_ang = (int)hc_deg;
  }
  setToSteeringAngle(st_ang);

  // Set flag for hard turns
  if (hc_deg < 0.0) hc_deg *= -1.0;
  _hard_turn_flag = (hc_deg > HARD_TURN_THRESH_DEG);
}

void XYZ_Rover::updateThrottlePercent()
{
  int th_pct = _normal_throttle_percent;

  if (_rover_mode != ROVER_MODE_RUN) {
    th_pct = _throttle_center_percent;
  } else if (_use_cur_throttle_flag) {
    th_pct = _cur_throttle_percent;
  } else {
    // Adjust throttle for turns/straights
    if (_slow_flag || _hard_turn_flag) {
      th_pct = _slow_throttle_percent;
    } else if (_is_reverse_flag) {
      th_pct = _reverse_throttle_percent;
    }
  }
  setToThrottlePercent(th_pct);
}

void XYZ_Rover::updateThrottlePercent(int throttle_percent)
{
  _cur_throttle_percent = throttle_percent;

  if (_rover_mode != ROVER_MODE_RUN) {
    setToThrottlePercent(_throttle_center_percent);
  } else {
    setToThrottlePercent(_cur_throttle_percent);
  }
}

float XYZ_Rover::normalizeDeg180(float d_deg)
{
  if (d_deg >  180.0) d_deg -= 360.0;
  if (d_deg >  180.0) d_deg -= 360.0;
  if (d_deg < -180.0) d_deg += 360.0;
  if (d_deg < -180.0) d_deg += 360.0;
  return d_deg;
}
float XYZ_Rover::normalizeDeg360(float heading)
{
  if (heading < 0) heading += 360;
  if (heading < 0) heading += 360;
  if (heading > 360) heading -= 360;
  if (heading > 360) heading -= 360;
  return heading;
}
float XYZ_Rover::calcHeadingDiffDeg(float cur_deg, float to_deg)
{
  return normalizeDeg180(cur_deg - to_deg);
}
float XYZ_Rover::calcHeadingDiff360Deg(float cur_deg, float to_deg) {
  return normalizeDeg360(cur_deg - to_deg);
}

float XYZ_Rover::getHeadingToPosDeg(float x_ft, float y_ft)
{
  float dx_ft = (x_ft - _x_pos_ft);
  float dy_ft = (y_ft - _y_pos_ft);
  float ang_rad = 0.0;
  if (dx_ft != 0.0 || dy_ft != 0.0) ang_rad = atan2(dy_ft, dx_ft);
  float deg = ang_rad * RAD_TO_DEG;
  return deg;
}

float XYZ_Rover::getDistanceFt(float x0_ft, float y0_ft, float x1_ft, float y1_ft)
{
  float dx_ft = (x1_ft - x0_ft);
  float dy_ft = (y1_ft - y0_ft);
  float d_ft2 = dx_ft*dx_ft + dy_ft*dy_ft;
  float d_ft = sqrt(d_ft2);
  if (d_ft == 0.0) d_ft = 0.01;
  return d_ft;
}

void XYZ_Rover::readCalibrationStats(uint8_t *stats) {
  #ifdef USE_BNO
    _xyz_imu.readCalibration(stats);
  #endif
}
