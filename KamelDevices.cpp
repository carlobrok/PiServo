#include <sys/ioctl.h>				// ioctl
#include <fcntl.h>						// fcntl, O_RDWR
#include <cstdint>						// int8_t, uint8_t, uint16_t, ...
#include <cerrno>							// errno
#include <cstring>						// strerror
#include <linux/i2c-dev.h>		// i2c_smbus_...
//#include <pca9685.h>
//#include <wiringPiI2C.h>
#include <iostream>
#include <thread>
#include <chrono>

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
#define PCA9685_PRESCALE 0xFE

int pca9685_setup(int address) {
  int fd = kamelI2Copen(address);
  if (fd < 0) return fd;

  // Setup the chip. Enable auto-increment of registers.
	int settings = i2c_smbus_read_byte_data(fd, PCA9685_MODE1) & 0x7F;
	int autoInc = settings | 0x20;

	i2c_smbus_write_byte_data(fd, PCA9685_MODE1, autoInc);


	// Set the frequency

	// Min freq = 40, max freq = 1000
	int freq = 50;

	// To set pwm frequency we have to set the prescale register. The formula is:
	// prescale = round(osc_clock / (4096 * frequency))) - 1 where osc_clock = 25 MHz
	// Further info here: http://www.nxp.com/documents/data_sheet/PCA9685.pdf Page 24
	int prescale = (int)(25000000.0f / (4096 * freq) - 0.5f);

	// Get settings and calc bytes for the different states.
	settings = i2c_smbus_read_byte_data(fd, PCA9685_MODE1) & 0x7F;	// Set restart bit to 0
	int sleep	= settings | 0x10;									// Set sleep bit to 1
	int wake 	= settings & 0xEF;									// Set sleep bit to 0
	int restart = wake | 0x80;										// Set restart bit to 1

	// Go to sleep, set prescale and wake up again.
	i2c_smbus_write_byte_data(fd, PCA9685_MODE1, sleep);
	i2c_smbus_write_byte_data(fd, PCA9685_PRESCALE, prescale);
	i2c_smbus_write_byte_data(fd, PCA9685_MODE1, wake);

	// Now wait a millisecond until oscillator finished stabilizing and restart PWM.
	std::this_thread::sleep_for(std::chrono::milliseconds(1));
	i2c_smbus_write_byte_data(fd, PCA9685_MODE1, restart);

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

  //pca9685PWMWrite(m_fd, m_pin, 0, off);

	// Write to on and off registers and mask the 12 lowest bits of data to overwrite full-on and off
	i2c_smbus_write_word_data(m_fd, get_register(m_pin), 0 & 0x0FFF);
	i2c_smbus_write_word_data(m_fd, get_register(m_pin) + 2, off & 0x0FFF);
}

void Servo::off() {
  //pca9685FullOff(m_fd, m_pin, 1);

	int reg = get_register(m_pin) + 3;		// LEDX_OFF_H
	int state = i2c_smbus_read_byte_data(m_fd, reg);

	// Set bit 4 to 1 or 0 accordingly
	state |= 0x10;

	i2c_smbus_write_byte_data(m_fd, reg, state);
}
