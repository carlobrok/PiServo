#include <iostream>
#include <unistd.h>
#include <chrono>
#include <thread>
#include "KamelDevices.h"


int main() {

  int servo_fd = pca9685_setup(0x40);

  Servo s1(servo_fd, 0);

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  while(1) {
    s1.set_angle(90);
    std::cout << "90" << '\n';
    usleep(1000*1000);
    s1.set_angle(0);
    std::cout << "60" << '\n';
    usleep(1000*1000);
    s1.set_angle(180);
    std::cout << "120" << '\n';
    usleep(1000*1000);
    s1.off();
    usleep(1000*1000);
  }
}
