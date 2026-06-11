#ifndef PWM_MOTOR_DRIVER_HPP
#define PWM_MOTOR_DRIVER_HPP

#include "motor_driver.hpp"
#include <string>

// sysfs PWM motor driver (4-channel H-bridge).
// Original aka0 motor backend, refactored behind the MotorDriver interface.
//
// Source: aka0-ref commits 755a885, 9c69f3f
class PwmMotorDriver : public MotorDriver {
  public:
    // pwm_path: e.g. "/sys/class/pwm/pwmchip4/"
    explicit PwmMotorDriver(const std::string& pwm_path = "/sys/class/pwm/pwmchip4/");
    ~PwmMotorDriver() override;

    void drive(int left_speed, int right_speed) override;
    void brake() override;
    void standby() override;

  private:
    void init_pwm(int pwm_id);
    void set_pwm_duty_cycle(int pwm_id, int duty_cycle);
    void set_pwm_enable(int pwm_id, bool enable);
    void set_speed(int pwm_id, int speed);

    std::string PWM_PATH;
    const int PERIOD = 10000; // 10kHz

    // PWM IDs for left and right wheels
    const int LEFT_WHEEL_BACKWARD = 0;
    const int LEFT_WHEEL_FORWARD = 1;
    const int RIGHT_WHEEL_BACKWARD = 2;
    const int RIGHT_WHEEL_FORWARD = 3;
};

#endif // PWM_MOTOR_DRIVER_HPP
