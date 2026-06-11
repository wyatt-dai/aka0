// Author: Zhihang Shao <dio_ro@outlook.com>
// Source: ported from aka-rk3588/motor/motor.cpp
// Description: 电机高层 API，委托给 PWM 或 UART 驱动后端
#include "motor.hpp"
#include "pwm_motor_driver.hpp"
#include "uart_motor_driver.hpp"
#include <cstdlib>
#include <algorithm>

Motor::Motor(MotorDriverType type, const std::string& device) {
    switch (type) {
    case MotorDriverType::PWM:
        // PWM 后端使用 sysfs 路径；device 为空时用默认 pwmchip4
        driver_.reset(new PwmMotorDriver(
            device.empty() ? "/sys/class/pwm/pwmchip4/" : device));
        break;
    case MotorDriverType::UART:
    default:
        driver_.reset(new UartMotorDriver(device));
        break;
    }
}

Motor::~Motor() = default;

void Motor::forward(int speed) {
    drive(speed, speed);
}

void Motor::backward(int speed) {
    drive(-speed, -speed);
}

void Motor::left(int speed) {
    // Spin left: right wheel forward, left wheel backward
    drive(-speed, speed);
}

void Motor::right(int speed) {
    // Spin right: left wheel forward, right wheel backward
    drive(speed, -speed);
}

void Motor::brake() {
    driver_->brake();
}

void Motor::standby() {
    driver_->standby();
}

void Motor::drive(int left_speed, int right_speed) {
    // 死区映射：非零速度拉到 [min_speed_, 100]
    auto map = [&](int v) -> int {
        if (v == 0)
            return 0;
        int sign = (v > 0) ? 1 : -1;
        int mag = std::abs(v);
        // 线性映射：[1,100] -> [min_speed_, 100]
        mag = min_speed_ + (mag - 1) * (100 - min_speed_) / 99;
        if (mag > 100)
            mag = 100;
        return sign * mag;
    };
    driver_->drive(map(left_speed), map(right_speed));
}
