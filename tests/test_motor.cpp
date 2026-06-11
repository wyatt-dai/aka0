// Author: Zhihang Shao <dio_ro@outlook.com>
// Source: aka0-ref commit 64f8dab
// Description: 电机测试工具，自动测试所有电机动作

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <unistd.h>
#include "motor.hpp"
#include "logger.hpp"

int main(int argc, char** argv) {
    if (argc < 2) {
        printf("Usage:\n");
        printf("  %s forward [speed] [dev]    - 前进 (默认50)\n", argv[0]);
        printf("  %s backward [speed] [dev]   - 后退\n", argv[0]);
        printf("  %s left [speed] [dev]       - 左转\n", argv[0]);
        printf("  %s right [speed] [dev]      - 右转\n", argv[0]);
        printf("  %s drive <L> <R> [dev]      - 差速驱动 (正前负后, 范围-100~100)\n", argv[0]);
        printf("  %s stop [dev]               - 停车\n", argv[0]);
        printf("  %s test [dev]               - 自动测试全部动作\n", argv[0]);
        printf("  dev 默认 /dev/ttyS1，可指定其它串口逐个排查\n");
        return 0;
    }

    // 解析串口设备：各命令的设备名都放在该命令参数之后的最后一个位置
    const char* dev = "/dev/ttyS1";
    if (strcmp(argv[1], "test") == 0 || strcmp(argv[1], "stop") == 0) {
        if (argc >= 3) dev = argv[2];          // test/stop: argv[2]=dev
    } else if (strcmp(argv[1], "drive") == 0) {
        if (argc >= 5) dev = argv[4];          // drive L R dev
    } else {
        if (argc >= 4) dev = argv[3];          // forward/backward/left/right speed dev
    }

    printf("使用串口: %s\n", dev);
    Motor motor(MotorDriverType::UART, dev);

    if (strcmp(argv[1], "test") == 0) {
        int speed = 50;
        printf("=== 电机自动测试 ===\n");

        printf("[1/5] 前进 (speed=%d, 2秒)...\n", speed);
        motor.forward(speed);
        sleep(2);

        printf("[2/5] 后退 (speed=%d, 2秒)...\n", speed);
        motor.backward(speed);
        sleep(2);

        printf("[3/5] 左转 (speed=%d, 2秒)...\n", speed);
        motor.left(speed);
        sleep(2);

        printf("[4/5] 右转 (speed=%d, 2秒)...\n", speed);
        motor.right(speed);
        sleep(2);

        printf("[5/5] 差速驱动 L=60 R=30 (2秒)...\n");
        motor.drive(60, 30);
        sleep(2);

        motor.standby();
        printf("=== 测试完成 ===\n");
        return 0;
    }

    int speed = (argc >= 3) ? atoi(argv[2]) : 50;

    if (strcmp(argv[1], "forward") == 0) {
        printf("前进 speed=%d\n", speed);
        motor.forward(speed);
    } else if (strcmp(argv[1], "backward") == 0) {
        printf("后退 speed=%d\n", speed);
        motor.backward(speed);
    } else if (strcmp(argv[1], "left") == 0) {
        printf("左转 speed=%d\n", speed);
        motor.left(speed);
    } else if (strcmp(argv[1], "right") == 0) {
        printf("右转 speed=%d\n", speed);
        motor.right(speed);
    } else if (strcmp(argv[1], "drive") == 0 && argc >= 4) {
        int l = atoi(argv[2]);
        int r = atoi(argv[3]);
        printf("差速驱动 L=%d R=%d\n", l, r);
        motor.drive(l, r);
    } else if (strcmp(argv[1], "stop") == 0) {
        printf("停车\n");
        motor.standby();
    }

    printf("按 Ctrl-C 停止\n");
    while (true) {
        sleep(1);
    }
    return 0;
}
