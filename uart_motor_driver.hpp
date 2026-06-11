#ifndef UART_MOTOR_DRIVER_HPP
#define UART_MOTOR_DRIVER_HPP

#include "motor_driver.hpp"
#include <string>
#include <cstdint>

// UART-based motor driver for ESP32-C3 chassis controller.
// Source: ported from aka-rk3588/motor/uart_motor_driver.{hpp,cpp}
//
// Protocol (framing from esp32_base_control/tests/test_uart.py):
//   Frame: [0xAA] [0x55] [CMD] [LEN] [PAYLOAD...] [CHK]
//   CHK   = CMD ^ LEN ^ payload[0] ^ ... ^ payload[n-1]
//
// Commands used:
//   CMD_INIT     (0x01)  payload: none         → must call before any motion
//   CMD_CONFIG   (0x02)  payload: PPR(2B BE) + FREQ(2B BE)
//   CMD_SET_SPEED(0x10)  payload: motor_id(1B) + speed(int16 BE)  per-motor
//   CMD_BRAKE    (0x12)  payload: motor_id (0=M1, 1=M2)
//   CMD_STOP     (0x11)  payload: motor_id
//   CMD_RESET    (0xFF)  payload: none
//
// speed_scale converts the Motor [-100,100] range to the ESP32 PWM value.
class UartMotorDriver : public MotorDriver {
  public:
    // uart_dev:    e.g. "/dev/ttyS1" (SG2002 chassis controller UART)
    // speed_scale: Motor speed 100 maps to this PWM value (ESP32 uses [-255,255]).
    //               Default 150 means 100% → PWM 150 (≈60% duty), slower and safer.
    // ppr:         Encoder pulses per revolution (default 4680 for 1:90 gearbox)
    // pwm_freq:    PWM frequency in Hz (default 20000)
    explicit UartMotorDriver(const std::string& uart_dev = "/dev/ttyS1",
                             int speed_scale = 150,
                             uint16_t ppr = 4680,
                             uint16_t pwm_freq = 20000);
    ~UartMotorDriver() override;

    void drive(int left_speed, int right_speed) override;
    void brake() override;
    void standby() override;

  private:
    // Low-level UART helpers
    bool open_uart(const std::string& dev);
    void close_uart();
    void send_frame(uint8_t cmd, const uint8_t* payload, uint8_t len);
    bool recv_frame(uint8_t* cmd_out, uint8_t* payload_out, uint8_t* len_out, int timeout_ms = 100);

    // High-level protocol commands
    void cmd_init();
    void cmd_config(uint16_t ppr, uint16_t pwm_freq);
    void cmd_set_speeds(int16_t m1_rpm, int16_t m2_rpm);
    void cmd_stop(uint8_t motor_id);
    void cmd_brake(uint8_t motor_id);

    int fd_;
    int speed_scale_;
    uint16_t ppr_;
    uint16_t pwm_freq_;
};

#endif // UART_MOTOR_DRIVER_HPP
