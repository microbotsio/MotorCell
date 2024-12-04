#include "MotorCell.h"
uint8_t mc_num = 0;
uint8_t speed_percentlast = 0U;

MotorCell::MotorCell(byte IN, byte OUT, byte FR) {
  _IN = IN;
  _OUT = OUT;
  _FR = FR;
}

void MotorCell::Init() {
  uint32_t rpm_speed = 0U;
  delay(500);
  mc_num++;
  _motor_num = mc_num;
  pinMode(_OUT, INPUT_PULLUP); /*Set up OUT-pin as input*/

  pinMode(_FR, OUTPUT);    /*Set up FR-pin as output*/
  digitalWrite(_FR, HIGH); /*Set up initial direction*/

  pinMode(_IN, OUTPUT);
#if defined(ARDUINO_ARCH_ESP32)
  ledcAttach(_IN, MOTOR_BASE_FREQ, MOTOR_TIMER);
  ledcWrite(_IN, 0);
  Reset(255);
  rpm_speed = pulseIn(_OUT, HIGH); /*Wait for the next pulse to get rpm-time*/

  if (rpm_speed > 0) {
    rpm_speed = (60000000 / (rpm_speed * 4u)); /*Get RPM value - MicroSeconds rpm-time multiply by Pole value(4) */
    for (int sii = 0; sii < SPEED_FILTER; sii++) {
      _array_average_speed[sii] = rpm_speed;
    }
  }
#endif


  Serial.print("MC#");
  Serial.print(_motor_num);
  Serial.println("Completed");
}


#if defined(ARDUINO_ARCH_ESP32)
uint16_t MotorCell::Spin(uint8_t speed_percent) {
  uint32_t rpm_speed = 0U;
  uint32_t timer_TT = 0;
  uint32_t average_speed = 0U;
  if (speed_percentlast != speed_percent) {
    if (speed_percent > 100U) {
      _motorspeed = 255U;
    } else {
      _motorspeed = (speed_percent * 255U) / 100U;
    }
    ledcWrite(_IN, _motorspeed);
  } else {
    /*Skip*/
  }
  speed_percentlast = speed_percent;

  rpm_speed = pulseIn(_OUT, HIGH); /*Wait for the next pulse to get rpm-time*/

  if ((rpm_speed > 0U) && (rpm_speed < 14000U) && (_motorspeed >= 5U)) {
    _array_average_speed[_si] = (60000000 / (rpm_speed * 4u)); /*Get RPM value - MicroSeconds rpm-time multiply by Pole value(4) */
    for (int sii = 0; sii < SPEED_FILTER; sii++) {
      average_speed = average_speed + _array_average_speed[sii];
    }
    average_speed = average_speed / SPEED_FILTER;
    _si++;
    if (_si >= SPEED_FILTER) {
      _si = 0;
    }

    Serial.print("MC#");
    Serial.print(_motor_num);
    Serial.print(" @ ");
    Serial.print(average_speed / 1000);
    Serial.println(" kRPM");
    _error_flag = 0;
    rpm_speed = 0;
  } else {
    if (_error_flag == 0) {
      speed_percentlast = 0;
      if (_motorspeed < 5U) {
        Serial.print("MC#");
        Serial.print(_motor_num);
        Serial.println(" Speed Percentage too low");
        _error_flag = 1;
      } else {
        Serial.print("MC#");
        Serial.print(_motor_num);
        Serial.println(" Hualt Error - Reseting..");
        Reset(255);
      }
    } else {
      /*wait*/
    }
  }
  return average_speed;
}
#else
void MotorCell::Spin(uint8_t speed_percent) {
  speed_percent = speed_percent / 10;
  delayMicroseconds(7);
  if ((_si >= 10) && (speed_percent != 0)) {
    _si = 0;
    digitalWrite(_IN, HIGH); /*Set up initial direction*/
  } else if (_si == speed_percent) {
    digitalWrite(_IN, LOW); /*Set up initial direction*/
  } else {
  }
  _si++;
}
#endif

#if defined(ARDUINO_ARCH_ESP32)
uint16_t MotorCell::SpinPID(uint16_t speed_rpm_target) {
  uint32_t rpm_speed = 0U;
  double error = 0.0;
  double derivative = 0.0;
  double output = 0.0;
  uint32_t average_speed = 0U;
  uint32_t currentTime = millis();                // Current time in milliseconds
  uint32_t timeChange = currentTime - _lastTime;  // Time change since the last call

  if (speed_rpm_target < 2000U) {
    speed_rpm_target = 2000U;
    Serial.print(" Speed too low");
  }

  rpm_speed = pulseIn(_OUT, HIGH); /*Wait for the next pulse to get rpm-time*/
  if ((_u_output > 0) && (rpm_speed == 0u)) {
    Serial.print("MC#");
    Serial.print(_motor_num);
    Serial.println(" Hualt Error - Reseting..");
    Reset(50U);
  } else {

    Serial.print(" MC#");
    Serial.print(_motor_num);
    if ((_u_output > 0) && (rpm_speed > 0)) {
      _array_average_speed[_si] = (60000000 / (rpm_speed * 4u)); /*Get RPM value - MicroSeconds rpm-time multiply by Pole value(4) */
      for (int sii = 0; sii < SPEED_FILTER; sii++) {
        average_speed = average_speed + _array_average_speed[sii];
      }
      average_speed = average_speed / SPEED_FILTER;
      _si++;
      if (_si >= SPEED_FILTER) {
        _si = 0;
      }
    } else {
      average_speed = rpm_speed;
      for (int sii = 0; sii < SPEED_FILTER; sii++) {
        _array_average_speed[sii] = rpm_speed;
      }
    }
    if (timeChange != 0) {

      error = (double)speed_rpm_target - (double)average_speed; /*Calculate Error from target*/

      derivative = (error - _previous_error) / timeChange;

      _integral += error * timeChange;
      if (_integral > INT_LIMIT) {
        _integral = INT_LIMIT;
      } else if (_integral < 0) {
        _integral = 0;
      } else {
        /*Skip*/
      }

      output = ((Kp * error) + (Ki * _integral) + (Kd * derivative)); /*PID calculation*/
      _u_output = (uint32_t)output;
      if (_u_output > 255U) {
        _u_output = 255U;
        if (speed_rpm_target > average_speed) {
          if (_spin_error < 350) {
            _spin_error++;
          } else {
            Serial.print(" Speed cannot be reached - Reduce Speed or Load ");
          }
        }
      } else if (_u_output < 0U) {
        _u_output = 0U;
        _spin_error = 0U;
      } else {
        _spin_error = 0U;
      }

      Serial.print(" @ ");
      Serial.print(average_speed / 1000);
      Serial.println(" kRPM");

      ledcWrite(_IN, _u_output);

      _previous_error = error;
      _lastTime = currentTime;
    }
  }

  // Serial.print(" average_speed");
  // Serial.print(average_speed);
  // Serial.print(" rpm_speed");
  // Serial.print(rpm_speed);
  // Serial.print(" error");
  // Serial.print(error);
  // Serial.print(" (Kp * error)");
  // Serial.print(Kp * error);
  // Serial.print(" (Ki * _integral) ");
  // Serial.print(Ki * _integral);
  // Serial.print(" (_integral) ");
  // Serial.print(_integral);
  // Serial.print(" (Kd * derivative) ");
  // Serial.print(Kd * derivative);
  // Serial.print(" output ");
  // Serial.print(output);
  return average_speed;
}

void MotorCell::Reset(uint8_t speed_percent) {
  ledcWrite(_IN, 0U);
  delay(50);
  ledcWrite(_IN, speed_percent);
  delay(200);
}
#endif

void MotorCell::Pulse(uint8_t p_pin, uint8_t ms_duration) {
  pinMode(p_pin, OUTPUT);    /*Set up FR-pin as output*/
  digitalWrite(p_pin, HIGH); /*Set up initial direction*/
  delay(ms_duration);
  digitalWrite(p_pin, LOW); /*Set up initial direction*/
}


void MotorCell::ReverseSpin() {
  digitalWrite(_FR, !digitalRead(_FR));
}


#if defined(ARDUINO_ARCH_ESP32)
void MotorCell::MaxSpin() {
  ledcWrite(_IN, 255);
}
#else
void MotorCell::MaxSpin() {
  pinMode(_IN, OUTPUT);
  digitalWrite(_IN, 1);
}
#endif

uint16_t MotorCell::RPMRead() {
  uint16_t u_rpm_speed = pulseIn(_OUT, HIGH);
  u_rpm_speed = (15000000 / u_rpm_speed); /*Get RPM value - MicroSeconds rpm-time multiply by Pole value(4) - (60000000 / (rpm_speed * 4u)) */
  Serial.print(u_rpm_speed);
  Serial.println(" RPM");
  return u_rpm_speed;
}
