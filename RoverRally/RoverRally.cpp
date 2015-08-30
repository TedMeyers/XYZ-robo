/* BNO/MPU Rover Code
 by Ted Meyers (5/19/2015)
 https://github.com/TedMeyers/XYZ-robo
 license: Beerware - Use this code however you'd like. If you 
 find it useful you can buy me a beer some time.

 IMPORTANT: Go to RoverRally.h and set the gyro type to be use.  This
            setting gets compiled in, and you have to pick one.
*/
#include "RoverRally.h"

/***************************************************************************
 CONSTRUCTOR
 ***************************************************************************/
RoverRally::RoverRally()
{
  setSteeringValues();
  setThrottleValues();
  setWaypointConstants(DEFAULT_WP_THRESH, DEFAULT_WP_FOLLOW_OFFSET);

  _updateCB = 0;

  _start_time = 0;
  _imu_update_time = 0;
  _steer_adj_time = 0;
  _imu_update_time = 0;
  _moveCheckTime = 0;
  _isReverse = false;

  _steering_Kp = STEERING_P;
  _steering_Ki = STEERING_I*(STEER_ADJ_TIME/1000.0);
  _steering_Kd = STEERING_D/(STEER_ADJ_TIME/1000.0);

  _ticks_per_distance = 1.0;
  _wheel_encoder_counter = 0.0;

  _slowThrottle = _th_center;
  _setThrottle = _th_center;

  reset();
  clearWaypoints();
  addWaypointPath(0.0, 0.0, 0.0);
}



void RoverRally::setSteeringPID(float kp, float ki, float kd) {
  _steering_Kp = kp;
  _steering_Ki = ki*(STEER_ADJ_TIME/1000.0);
  _steering_Kd = kd/(STEER_ADJ_TIME/1000.0);
}
void RoverRally::setSteeringValues(int center, int min, int max) {
  _steer_center = center;
  _steer_min = min;
  _steer_max = max;
}
void RoverRally::setThrottleValues(int center, int minFwd, int minRev,int min, int max, int scale) {
  _th_center = center;
  _th_minFwd = minFwd;
  _th_minRev = minRev;
  _th_min = min;
  _th_max = max;
  _th_scale = scale;
}


int RoverRally::setupRover(int thrPin, int steerPin, int btnPin, int ledPin, int address) {
  _buttonPin = btnPin;
  _ledPin = ledPin;

  _throttleServo.attach(thrPin);
  _steeringServo.attach(steerPin);
  _throttleServo.write(_th_center); 
  _steeringServo.write(_steer_center); 
 
  // Initialise the IMU sensor
  int status = 0;
  #ifdef USE_MPU
    _xyz_imu.setupI2C();
    status = (_xyz_imu.setup(address))?0:1;
  #endif
  #ifdef USE_BNO
    if (!_xyz_imu.setup(address)) {
      status = 0;
    } else {
      _xyz_imu.setMode(XYZ_BNO055::IMU);
      status = 1;      
    }
  #endif

  pinMode(_buttonPin, INPUT_PULLUP);

  pinMode(_ledPin, OUTPUT);
  digitalWrite(_ledPin, LOW);
  
  return status;
}

void RoverRally::reset() {
  _toThrottle = _th_center;
  _toSteer = _steer_center;
  _slowOn = false;
  _hardTurnOn = false;
  _throttleOverride = false;
  _curThrottle = 0;
  _curSteer = 0;
  _obstacleOverride = false;
  _isReverse = false;
  _stoppedFlag = false;

  _c_wp = 0;
  _x_wp = 0.0;
  _y_wp = 0.0;

  _steerInteg = 0.0;
  _lastRelHeading = 0.0;

  _lastWheelEncoderCount = _wheel_encoder_counter;
  _updateWheelEncoderCount = _wheel_encoder_counter;
  _moveCheckCount = 0;

  _steerDir = STEER_CLOSEST;
  _state = STATE_NONE;
  _curMode = MODE_WAIT;

  resetPosition();
  resetHeading();

  _throttleServo.write(_th_center); 
  _steeringServo.write(_steer_center);
  _start_time = millis();
}


void RoverRally::clearWaypoints() {
  for (int i=0; i<WP_PATH_SIZE; i++) {
    _wp_path_x[i] = 0.0;
    _wp_path_y[i] = 0.0;
    _wp_path_t[i] = -1.0;
  }  
}
void RoverRally::setWaypointPath(int i, float x, float y, float t) {
  _wp_path_x[i] = x;
  _wp_path_y[i] = y;
  _wp_path_t[i] = t;
}
void RoverRally::moveWaypointPath(int i, float dx, float dy) {
  _wp_path_x[i] += dx;
  _wp_path_y[i] += dy;
}


void RoverRally::backAround(bool isLeft, float d1, float d2) {
  int slow = _slowThrottle;
  int norm = _setThrottle;
  bool isOverride = _throttleOverride;
  _throttleOverride = true;

  int offset = calcThrottleAbsOffset(slow) + 4.0;
  int throttle = _th_center + offset;

  if (isLeft) backupLeft(90.0); else backupRight(90.0);
  brakeStop(300);

  _slowThrottle = slow;
  _setThrottle = throttle;
  waitForDistance(d1);

  _slowThrottle = slow;
  _setThrottle = throttle;
  if (isLeft) turnToLeft(90.0); else turnToRight(90.0);

  _slowThrottle = slow;
  _setThrottle = throttle;
  waitForDistance(d2);

  _slowThrottle = slow;
  _setThrottle = norm;
  _throttleOverride = isOverride;
  _obstacleOverride = false;
}


void RoverRally::backup(bool isTurn, bool isLeft, float angleDist) {
  int slow = _slowThrottle;
  int norm = _setThrottle;
  int throttle = _th_center - calcThrottleAbsOffset(slow);

  int dir = _steerDir;
  bool isOverride = _throttleOverride;
  _throttleOverride = true;
  _isReverse = true;
  brakeStop(50);

  _slowThrottle = throttle;
  _setThrottle = throttle;
  if (isTurn) {
    if (isLeft) setRightTurn(angleDist); else setLeftTurn(angleDist);
    waitForHeading(_toRelHeading);
  } else {
    waitForDistance(angleDist);
  }
  rollStop();    
  waitForTime(400);

  if (isTurn) _steerDir = dir;
  _isReverse = false;
  _throttleOverride = isOverride;
  _obstacleOverride = false;
  _slowThrottle = slow;
  _setThrottle = norm;

}

void RoverRally::brakeStop(int duration) {
  bool tmp = _throttleOverride;
  _throttleOverride = true;
  if (_curThrottle > _th_center) {
    setThrottlePercent(-50);
    waitForTime(200);
    setThrottlePercent(0);
    waitForTime(200);
    setThrottlePercent(-80);
  } else if (_curThrottle < _th_center) {
    setThrottlePercent(50);
    waitForTime(200);
    setThrottlePercent(0);
    waitForTime(200);
    setThrottlePercent(80);    
  }
  waitForTime(duration);
  setThrottlePercent(0);
  waitForTime(200);
  _throttleOverride = tmp;
}

void RoverRally::rollStop() {
  bool tmp = _throttleOverride;
  _throttleOverride = true;
  setThrottlePercent(0);
  waitForTime(200);
  _throttleOverride = tmp;
}


int RoverRally::calcThrottleFromPercent(float percent) {
  int amt = (int)(percent*_th_scale);
  if (amt < 0) amt -= _th_minRev;
  else if (amt > 0) amt += _th_minFwd;
  return normalizeThrottle(amt);
}

void RoverRally::setDirection(int dir) {
  if (_steerDir != dir) {
    if (dir == STEER_LEFT) _steerDir = STEER_LEFT;
    else if (dir == STEER_RIGHT) _steerDir = STEER_RIGHT;
    else _steerDir = STEER_CLOSEST;
  }
}

void RoverRally::turnToLeft(float angle) {
  int dir = _steerDir;
  setLeftTurn(angle);
  waitForHeading(_toRelHeading);
  _steerDir  = dir;
}
void RoverRally::turnToRight(float angle) {
  int dir = _steerDir;
  setRightTurn(angle);
  waitForHeading(_toRelHeading);
  _steerDir  = dir;
}
void RoverRally::turnToClosest(float heading) {
  int dir = _steerDir;
  setClosest();
  setHeadingTurn(heading);
  waitForHeading(_toRelHeading);
  _steerDir  = dir;
}
void RoverRally::setStraight() {
  setClosest();
  setAngleTurn(0);
}
void RoverRally::setLeftTurn(int angle) {
  setLeft();
  setAngleTurn(-angle);
}
void RoverRally::setRightTurn(int angle) {
  setRight();
  setAngleTurn(angle);
}
void RoverRally::setHeadingTurn(int hdg) {
  _toRelHeading = normalizeDeg180(hdg);
  _lastRelHeading = _curRelHeading;
}
void RoverRally::setAngleTurn(int angle) {
  _toRelHeading = normalizeDeg180(_toRelHeading + angle);
  _lastRelHeading = _curRelHeading;
}

float RoverRally::setToHeading(float toHead) {
  _toRelHeading = normalizeDeg360(toHead);
  return calcHeadingDiff(_curRelHeading, _toRelHeading);
}
bool RoverRally::testToHeading(bool testNeg) {
  float d = calcHeadingDiff(_curRelHeading, _toRelHeading);
  return (testNeg) ? (d < -HEADING_THRESHOLD) : (d > HEADING_THRESHOLD);
}


void RoverRally::driveWaypointPath(int i) {
  _c_wp = (i+1);
  while (_c_wp > 0 && _c_wp < WP_PATH_SIZE && _wp_path_t[_c_wp] >= 0) {
    int c0 = _c_wp-1;
    float x0 = _wp_path_x[c0];
    float y0 = _wp_path_y[c0];
    float x1 = _wp_path_x[_c_wp];
    float y1 = _wp_path_y[_c_wp];

    _state = STATE_WAYP;
    updateAll();
    driveWaypointLine(x0, y0, x1, y1, _wp_path_t[_c_wp]);
    if (_obstacleOverride) break; else _c_wp++;
  }
  _state = STATE_NONE;
  _slowOn = false;
}
void RoverRally::driveWaypointLine(float x0, float y0, float x1, float y1, float thresh) {
  float d1 = getDistanceFrom(x1, y1);
  float dt = getDistance(x0, y0, x1, y1);
  float dx = (x1-x0)/dt;
  float dy = (y1-y0)/dt;

  while (d1 > _wp_thresh) {
    _state = STATE_WAYP;
    updateAll();
    
    d1 = getDistanceFrom(x1, y1);
    _slowOn = (d1 < thresh);
    
    float d = (dt - d1) + _wp_follow_offset;
    float xwp = x0 + (d*dx);
    float ywp = y0 + (d*dy);
    updateAll();
    waitForWaypoint(xwp, ywp, _wp_thresh);
    if (_obstacleOverride) {_state = STATE_NONE; return;}
  }
  _state = STATE_NONE;
}

void RoverRally::waitForWaypoint(float toX, float toY, float thresh) {
  _state = STATE_WAYP;
  _x_wp = toX;
  _y_wp = toY;
  float d = thresh+1;   // Go through loop at leas once
  while (d > thresh) {
    updateAll();
    if (_obstacleOverride) {_state=STATE_NONE; return;}

    _toRelHeading = normalizeDeg180(getHeadingTo(_x_wp, _y_wp));
    d = getDistanceFrom(_x_wp, _y_wp);
  }
  _state = STATE_NONE;
}

void RoverRally::waitForCustom(bool (*customWaitPtr)()) {
  _state = STATE_CUST;
  while (!customWaitPtr()) {
    updateAll();
    delay(1);
  }
  _state = STATE_NONE;
}
void RoverRally::waitForButtonPress() {
  _state = STATE_BTN;

  uint8_t stats[4];

  uint32_t time = millis();
  int curBtn = digitalRead(_buttonPin);
  while (digitalRead(_buttonPin) == curBtn) {
    if ((millis() - time) > 100) {
      updateCalibrationLED();
      time = millis();
    }
    updateAll();
  }
  _state = STATE_NONE;
}

void RoverRally::waitForHeading(float toHead) {
  _state = STATE_TURN;
  updateAll();
  bool testNeg = (setToHeading(toHead) < 0.0);
  while (testToHeading(testNeg)) {  
    updateAll();
  }
  _state = STATE_NONE;
}
void RoverRally::waitForTime(uint32_t msecs) {
  _state = STATE_WAIT;
  uint32_t endtime = millis() + msecs;
  updateAll();
  while (millis() < endtime) {
    updateAll();
  }
  _state = STATE_NONE;
}
void RoverRally::waitForTicks(uint32_t numTicks) {
  _state = STATE_DIST;
  updateAll();  
  markCurrentTicks();
  updateAll();
  while (getCurrentTicks() < numTicks) {
    updateAll();
  }
  _state = STATE_NONE;
}
void RoverRally::waitForDistance(float dist) {
  int32_t ticks = convertDistanceToTicks(dist);
  waitForTicks(ticks);
}

void RoverRally::updateAll() {
  if (_updateCB) (*_updateCB)(_state);
  else updateAllBasic();
}

void RoverRally::updateAllBasic() {
  if ((millis() - _imu_update_time) >= 10) {
    _imu_update_time = millis();
    #ifdef USE_BNO
      _curMagHeading = normalizeDeg180(_xyz_imu.readHeading());
      _curRelHeading = normalizeDeg180(_curMagHeading - _startMagHeading);
    #endif
    #ifdef USE_MPU
      _xyz_imu.updateYPR(); 
      _curMagHeading = normalizeDeg180(_xyz_imu.getHeading());
      _curRelHeading = normalizeDeg180(_curMagHeading - _startMagHeading);
    #endif

    updateSteering();
    updateThrottle();
    updatePosition();      
  }

  #ifdef CHECK_FOR_STOP
  // ----------------------------------------------
  if ((millis() - _moveCheckTime) > 200) {
    _moveCheckTime = millis();
    uint32_t c = _wheel_encoder_counter;

    if ((_curThrottle != _th_center) && ((millis() - _start_time) > 500)) {
      _stoppedFlag = ((c - _moveCheckCount) < 10);
      #ifdef SET_OBSTACLE_OVERRIDE
      if (_stoppedFlag) _obstacleOverride = true;      
      #endif
    }
    _moveCheckCount = c;
  }
  // ----------------------------------------------
  #endif

  if (_curMode == MODE_RUN) {
    adjustSteering();
    adjustThrottle();  
  }
}

void RoverRally::updateCalibrationLED() {
  bool b = false;
  #ifdef USE_BNO
    uint8_t stats[4];
    readCalibrationStats(stats);
    b = (stats[0]==3); //((stats[0]==3) && (stats[1]==3) && (stats[3]==3));
  #endif
  #ifdef USE_MPU
    static float cal1 = 0.0;
    float cal2 = _xyz_imu.getHeading();
    b = (abs(cal1-cal2) < 0.07);
    cal1 = cal2;
  #endif
  digitalWrite(_ledPin, (b)?HIGH:LOW); 
}

void RoverRally::updatePosition() {
  if (_updateWheelEncoderCount < _wheel_encoder_counter) {
    float h = _curRelHeading * DEG_TO_RAD;
    float d = (_wheel_encoder_counter - _updateWheelEncoderCount) / _ticks_per_distance;
    if (_isReverse) d *= -1.0;
    _x_pos += d * cos(h);
    _y_pos += d * sin(h);
    _updateWheelEncoderCount = _wheel_encoder_counter;

  } else if (_updateWheelEncoderCount > _wheel_encoder_counter) {
    _updateWheelEncoderCount = _wheel_encoder_counter;
  }
}

void RoverRally::updateSteering() {
  uint32_t now = millis();
  uint32_t delt_t = now - _steer_adj_time;

  if (delt_t > STEER_ADJ_TIME) { 
    float hc = -normalizeDeg180(_toRelHeading - _curRelHeading);
    int dir = _steerDir;
    if (hc > -5 && hc < 5) dir = STEER_CLOSEST;
    if ((dir == STEER_LEFT) && (hc < -5)) {
      hc += 360.0;
    } else if ((dir == STEER_RIGHT) && (hc > 5)) {
      hc -= 360.0;
    }
  
    float hd = (_curRelHeading - _lastRelHeading);
    if (hc < STEERING_I_CUTOFF && hc > -STEERING_I_CUTOFF) _steerInteg -= hc;

    float iTerm = (_steerInteg * _steering_Ki);
    if (iTerm > STEERING_I_LIMIT) iTerm = STEERING_I_LIMIT;
    if (iTerm < -STEERING_I_LIMIT) iTerm = -STEERING_I_LIMIT;

    hc = (hc * _steering_Kp) - iTerm + (hd * _steering_Kd);
    _headingChange = hc;
    _lastRelHeading = _curRelHeading;
    
    if (_isReverse) hc *= -1.0;
    if ((hc < -0.2) || (hc > 0.2)) {
      _toSteer = normalizeSteering((int)hc);
    } else {
      _toSteer = _steer_center;
    }

    // Set flag for hard turns
    if (hc < 0.0) hc *= -1.0;
    _hardTurnOn = (hc > TURN_THRESH);

    _steer_adj_time = now;
  }
}

void RoverRally::updateThrottle() {
  if (_curMode == MODE_WAIT) {
    _toThrottle = _th_center;
  } else {
    // Adjust throttle for turns/straights
    if (_throttleOverride) {
      _toThrottle = _setThrottle;
    } else {
      _toThrottle = (_slowOn || _hardTurnOn) ? _slowThrottle : _setThrottle;
    }
  }
}

void RoverRally::adjustThrottle() {
  if (_curThrottle != _toThrottle) {
    _curThrottle = _toThrottle;
    _throttleServo.write(_curThrottle);
  }
}
void RoverRally::adjustSteering() {
  if (_curSteer != _toSteer) {
    _steeringServo.write(_toSteer);
    _curSteer = _toSteer;
  }
}

int RoverRally::normalizeSteering(int amount) {
  if ((amount < 0) && (amount > -STEERING_EPS)) amount = -STEERING_EPS;
  if ((amount > 0) && (amount <  STEERING_EPS)) amount = STEERING_EPS;

  int steering = _steer_center + amount;
  if (steering < _steer_min)  steering = _steer_min;
  if (steering > _steer_max) steering = _steer_max;
  return steering;
}
int RoverRally::normalizeThrottle(int amount) {
  int throttle = _th_center + amount;
  if (throttle < _th_min) throttle = _th_min;
  if (throttle > _th_max) throttle = _th_max;
  return throttle;
}

float RoverRally::normalizeDeg180(float h) {
  while (h > 180.0)  h -= 360.0;
  while (h < -180.0) h += 360.0;
  return h;
}
float RoverRally::normalizeDeg360(float heading) {
  if (heading < 0) heading += 360;
  if (heading < 0) heading += 360;
  if (heading > 360) heading -= 360;
  if (heading > 360) heading -= 360;
  return heading;
}
float RoverRally::calcHeadingDiff(float cur, float to) {
  float d = (cur - to);
  if (d <= -180.0) d += 360.0;
  if (d >   180.0) d -= 360.0;
  return d;
}


float RoverRally::getHeadingTo(float x, float y) {
  float dx = (x - _x_pos);
  float dy = (y - _y_pos);
  float ang = 0.0;
  if (dx != 0.0 || dy != 0.0) ang = atan2(dy, dx);
  float deg = ang * RAD_TO_DEG;
  return deg;
}

float RoverRally::getDistance(float x0, float y0, float x1, float y1) {
  float dx = (x1 - x0);
  float dy = (y1 - y0);
  float d2 = dx*dx + dy*dy;
  float d = sqrt(d2);
  return d;
}

void RoverRally::readCalibrationStats(uint8_t *stats) {
  #ifdef USE_BNO
    _xyz_imu.readCalibration(stats);
  #endif
}

int RoverRally::calcThrottleAbsOffset(int val) {
  int offset = (val - _th_center);
  if (offset < 0) offset *= -1;
  return offset;
}
