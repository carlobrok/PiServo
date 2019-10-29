#include <iostream>
#include <unistd.h>

#include "KamelDevices.h"


int main() {

  int servo_fd = pca9685_setup(0x40);

  Servo s1(servo_fd, 2, 180, 0.7, 2.6);

  while(1) {
    s1.set_angle(90);
    std::cout << "90" << '\n';
    usleep(1000*1000);
    s1.set_angle(0);
    std::cout << "0" << '\n';
    usleep(1000*1000);
    s1.set_angle(180);
    std::cout << "180" << '\n';
    usleep(1000*1000);
    pca9685FullOff(servo_fd, 2, 1);
    usleep(1000*1000);
  }
}
