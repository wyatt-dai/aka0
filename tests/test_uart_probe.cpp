// Author: ported for SG2002 bring-up
// Description: 串口电机控制器探测工具。
//   开一个 UART，发一帧 INIT (AA 55 01 00 01)，读原始回包并尝试按
//   [AA][55][CMD][LEN][PAYLOAD...][CHK] 解析，用来确认哪个 /dev/ttySx
//   接的是 ESP32-C3 电机控制器。
//
//   用法: test_uart_probe [dev=/dev/ttyS1] [baud=115200]
//   判据: 收到以 AA 55 开头、且异或校验正确的帧 => 就是电机控制器。

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/select.h>

static const uint8_t SOF0 = 0xAA;
static const uint8_t SOF1 = 0x55;
static const uint8_t CMD_INIT = 0x01;

static speed_t baud_const(int baud) {
    switch (baud) {
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        default:     return B115200;
    }
}

static int open_uart(const char* dev, int baud) {
    int fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) { perror("open"); return -1; }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fd, &tty) != 0) { perror("tcgetattr"); close(fd); return -1; }

    speed_t b = baud_const(baud);
    cfsetospeed(&tty, b);
    cfsetispeed(&tty, b);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_oflag &= ~OPOST;
    tty.c_lflag = 0;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    tcflush(fd, TCIOFLUSH);
    if (tcsetattr(fd, TCSANOW, &tty) != 0) { perror("tcsetattr"); close(fd); return -1; }
    return fd;
}

// 等一个字节，超时返回 false
static bool wait_byte(int fd, uint8_t* b, int ms) {
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(fd, &fds);
    struct timeval tv = { ms / 1000, (ms % 1000) * 1000 };
    if (select(fd + 1, &fds, nullptr, nullptr, &tv) <= 0) return false;
    return read(fd, b, 1) == 1;
}

int main(int argc, char** argv) {
    const char* dev = (argc >= 2) ? argv[1] : "/dev/ttyS1";
    int baud = (argc >= 3) ? atoi(argv[2]) : 115200;

    printf("探测 %s @ %d 8N1\n", dev, baud);
    int fd = open_uart(dev, baud);
    if (fd < 0) { printf("结果: 无法打开 %s\n", dev); return 1; }

    usleep(100000);
    tcflush(fd, TCIOFLUSH);

    // 发 INIT 帧: AA 55 01 00 01  (CHK = CMD ^ LEN = 0x01 ^ 0x00 = 0x01)
    uint8_t frame[5] = { SOF0, SOF1, CMD_INIT, 0x00, (uint8_t)(CMD_INIT ^ 0x00) };
    ssize_t w = write(fd, frame, sizeof(frame));
    printf("已发送 INIT 帧 (%zd 字节): AA 55 01 00 01\n", w);

    // 读最多 2 秒的原始回包
    printf("等待回包 (最多 2 秒)...\n");
    uint8_t raw[256];
    int n = 0;
    int budget_ms = 2000;
    while (n < (int)sizeof(raw) && budget_ms > 0) {
        uint8_t b;
        if (wait_byte(fd, &b, 50)) {
            raw[n++] = b;
        } else {
            budget_ms -= 50;
            if (n > 0) break;  // 已收到数据且静默 50ms，认为一帧结束
        }
    }

    if (n == 0) {
        printf("结果: 无任何回包 —— 这个口大概率不是电机，或控制器没上电/波特率不对。\n");
        close(fd);
        return 2;
    }

    printf("收到 %d 字节原始数据:\n  ", n);
    for (int i = 0; i < n; i++) printf("%02X ", raw[i]);
    printf("\n");

    // 在原始字节里找 AA 55 帧头
    int idx = -1;
    for (int i = 0; i + 1 < n; i++) {
        if (raw[i] == SOF0 && raw[i + 1] == SOF1) { idx = i; break; }
    }
    if (idx < 0) {
        printf("结果: 收到数据但没有 AA 55 帧头 —— 可能是控制台口或其它设备，不是电机。\n");
        close(fd);
        return 3;
    }

    // 解析帧: [AA][55][CMD][LEN][PAYLOAD..][CHK]
    if (idx + 4 >= n) {
        printf("结果: 找到 AA 55 帧头但数据不完整，疑似电机但未收全，建议重试。\n");
        close(fd);
        return 4;
    }
    uint8_t cmd = raw[idx + 2];
    uint8_t len = raw[idx + 3];
    if (idx + 4 + len >= n) {
        printf("结果: 帧头 OK (CMD=0x%02X LEN=%u) 但负载/校验未收全，疑似电机，建议重试。\n", cmd, len);
        close(fd);
        return 4;
    }
    uint8_t chk = raw[idx + 4 + len];
    uint8_t calc = cmd ^ len;
    for (int i = 0; i < len; i++) calc ^= raw[idx + 4 + i];

    printf("解析帧: CMD=0x%02X LEN=%u CHK=0x%02X (计算=0x%02X)\n", cmd, len, chk, calc);
    if (chk == calc) {
        printf("结果: ✓ 校验通过 —— %s 就是电机控制器 (ESP32-C3)。\n", dev);
        close(fd);
        return 0;
    } else {
        printf("结果: 有 AA 55 帧头但校验不符 —— 可能是噪声或波特率略偏，疑似但不确定。\n");
        close(fd);
        return 5;
    }
}
