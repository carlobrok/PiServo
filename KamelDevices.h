#ifndef KAMELI2C_H
#define KAMELI2C_H

#include <stdint.h>       	// int8_t, uint8_t, uint16_t, ...

int kamelI2Copen(int devId);

int pca9685_setup(int address);

inline int get_register(uint8_t pin) {
	 return (pin >= 16 ? 0xFA : 0x6 + 4 * pin);
}

class Servo {
public:
  Servo(int pca9685_fd, uint8_t pin);

  Servo(int pca9685_fd, uint8_t pin, uint16_t max_angle, float min_ms, float max_ms);

  ~Servo();

  void set_angle(int angle);

  void off();

private:
  int m_fd;
  int m_pin;
  int m_max_angle;
  float m_min_ms;
  float m_max_ms;
};


#endif
