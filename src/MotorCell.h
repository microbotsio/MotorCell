#ifndef MOTORCELL_H
#define MOTORCELL_H

#include <Arduino.h>

#define MOTOR_TIMER 8
#define MOTOR_BASE_FREQ 30000
#define MC_MAX_NUM 10

#define SPEED_FILTER 50

#define Kp 0.008
#define Ki 0.000009
#define Kd 0.05
#define INT_LIMIT 50000000.0

class MotorCell {
private:
  byte _IN;
  byte _OUT;
  byte _FR;
  bool _error_flag = 0;
  uint8_t _si = 0U;
  uint8_t _motor_num = 0U;
  uint8_t _motorspeed = 0U;
  uint16_t _spin_error = 0U;
  uint32_t _u_output = 0U;
  uint32_t _lastTime = 0U;
  uint32_t _array_average_speed[SPEED_FILTER] = { 20000U };
  double _previous_error = 0.0;
  double _integral = 0.0;

public:
  MotorCell(byte IN, byte OUT, byte FR);
  void Init();
  void ReverseSpin();
  void MaxSpin();
  void Pulse(uint8_t p_pin, uint16_t ms_duration);
  uint16_t RPMRead();
#if defined(ARDUINO_ARCH_ESP32)
  void Reset(uint8_t speed_percent);
  uint16_t Spin(uint8_t speed_percent);
  uint16_t SpinPID(uint16_t speed_rpm_target);
#else
  void Spin(uint8_t speed_percent);
#endif
};
#endif
