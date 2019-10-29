#include <sys/ioctl.h>				// ioctl
#include <fcntl.h>						// fcntl, O_RDWR
#include <cstdint>						// int8_t, uint8_t, uint16_t, ...
#include <cerrno>							// errno
#include <cstring>						// strerror
#include <linux/i2c-dev.h>		// i2c_smbus_...
#include <pca9685.h>
#include <wiringPiI2C.h>
#include <iostream>

#include "KamelDevices.h"



// returns a file descriptor of the opened I2C device

int kamelI2Copen(int devId) {
	const char *device;
	int fd;

	//device = "/dev/i2c-0";	// Older Raspberry Pi models
	device = "/dev/i2c-1";	// Raspberry Pi 3B+

	if((fd = open(device, O_RDWR)) < 0)	{			// open "/dev/i2c-1"
		std::cout << "Unable to open " << device << ": " << strerror(errno) << std::endl;
		return -1;
	}

	if (ioctl (fd, I2C_SLAVE, devId) < 0) {			// set device address of fd to devId
		std::cout << "Unable to open device " << devId << ": " << strerror(errno) << std::endl;
		return -1;
	}

	return fd;
}



#define PCA9685_MODE1 0x0

// Function copied from https://github.com/Reinbert/pca9685 library, modified for specific usage
// Does not setup the wiringPi node, only works with the "advanced" functions of the library
int pca9685_setup(int address) {
  int fd = kamelI2Copen(address);
  if (fd < 0) return fd;

  // Setup the chip. Enable auto-increment of registers.
	int settings = wiringPiI2CReadReg8(fd, PCA9685_MODE1) & 0x7F;
	int autoInc = settings | 0x20;

	wiringPiI2CWriteReg8(fd, PCA9685_MODE1, autoInc);

  pca9685PWMFreq(fd, 50);

  return fd;
}


Servo::Servo(int pca9685_fd, uint8_t pin) {
  m_fd = pca9685_fd;
  m_pin = pin;
	m_max_angle = 180;
  m_min_ms = 1;
  m_max_ms = 2;
}

Servo::Servo(int pca9685_fd, uint8_t pin, uint16_t max_angle, float min_ms, float max_ms) {
  m_fd = pca9685_fd;
  m_pin = pin;
  m_max_angle = max_angle;
  m_min_ms = min_ms;
  m_max_ms = max_ms;
}

Servo::~Servo() {
  off();
}

void Servo::set_angle(int angle) {
  int off = (int)(4096 * ((angle*1.0 / m_max_angle) * ((m_max_ms - m_min_ms) / 20.0) + (m_min_ms / 20.0)) + 0.5f);

  pca9685PWMWrite(m_fd, m_pin, 0, off);
}

void Servo::off() {
  pca9685FullOff(m_fd, m_pin, 1);
}
