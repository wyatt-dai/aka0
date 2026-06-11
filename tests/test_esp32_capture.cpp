// Author: tgoskits sg2002 port
// Description: 独立验证 ESP32 摄像头驱动 (/dev/cvi-camera0)
//   - 不依赖 cvi-base / cvi-sys / cvi-tpu
//   - 仅打开 cvi-camera0 调 INIT/GET_INFO/GET_FRAME ioctl
//   - 把抓到的 JPEG 原样写到指定路径
//
// Usage:
//   test_esp32_capture [device] [output.jpg] [count]
//   test_esp32_capture                          # 默认 /dev/cvi-camera0 -> /data/esp32.jpg 1帧
//   test_esp32_capture /dev/cvi-camera0 /data/x.jpg 5

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>

#define ESP32_CAM_INIT       1
#define ESP32_CAM_GET_INFO   2
#define ESP32_CAM_GET_FRAME  3
#define FRAME_MAX_SIZE       (2 * 1024 * 1024)

struct ESP32CameraInfo {
    uint16_t width;
    uint16_t height;
    uint8_t  format;
    uint8_t  connected;
};

int main(int argc, char** argv) {
    const char* dev = "/dev/cvi-camera0";
    const char* out = "/data/esp32.jpg";
    int count = 1;
    if (argc > 1) dev = argv[1];
    if (argc > 2) out = argv[2];
    if (argc > 3) count = atoi(argv[3]);

    printf("[esp32] open %s\n", dev);
    int fd = open(dev, O_RDWR);
    if (fd < 0) {
        fprintf(stderr, "open %s failed: %s\n", dev, strerror(errno));
        return 1;
    }

    printf("[esp32] ioctl INIT (this sends 'init' + 'ping' to ESP32 over UART3)\n");
    int rc = ioctl(fd, ESP32_CAM_INIT, 0);
    if (rc < 0) {
        fprintf(stderr, "ioctl INIT failed: %s (rc=%d)\n", strerror(errno), rc);
        close(fd);
        return 2;
    }
    printf("[esp32] INIT ok\n");

    struct ESP32CameraInfo info;
    memset(&info, 0, sizeof(info));
    rc = ioctl(fd, ESP32_CAM_GET_INFO, &info);
    if (rc < 0) {
        fprintf(stderr, "ioctl GET_INFO failed: %s\n", strerror(errno));
        close(fd);
        return 3;
    }
    printf("[esp32] info: %ux%u fmt=%u connected=%u\n",
           info.width, info.height, info.format, info.connected);

    uint8_t* buf = (uint8_t*)malloc(FRAME_MAX_SIZE);
    if (!buf) {
        fprintf(stderr, "malloc failed\n");
        close(fd);
        return 4;
    }

    for (int i = 0; i < count; i++) {
        printf("[esp32] grabbing frame %d/%d ...\n", i + 1, count);
        int len = ioctl(fd, ESP32_CAM_GET_FRAME, buf);
        if (len < 0) {
            fprintf(stderr, "ioctl GET_FRAME failed: %s\n", strerror(errno));
            free(buf);
            close(fd);
            return 5;
        }
        printf("[esp32] got %d bytes\n", len);
        if (len <= 4) {
            fprintf(stderr, "frame too small, ignoring\n");
            continue;
        }
        // 保存最后一帧
        if (i == count - 1) {
            FILE* fp = fopen(out, "wb");
            if (!fp) {
                fprintf(stderr, "fopen %s failed: %s\n", out, strerror(errno));
                free(buf);
                close(fd);
                return 6;
            }
            size_t w = fwrite(buf, 1, (size_t)len, fp);
            fclose(fp);
            printf("[esp32] wrote %zu bytes to %s\n", w, out);
            // 头 8 字节是 JPEG SOI 0xFFD8FFE0... 看看
            printf("[esp32] head: %02x %02x %02x %02x %02x %02x %02x %02x\n",
                   buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);
        }
    }

    free(buf);
    close(fd);
    printf("[esp32] done.\n");
    return 0;
}
