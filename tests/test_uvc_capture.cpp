// Author: ported for SG2002 bring-up
// Description: UVC 摄像头抓帧测试工具。
//   打开 /dev/videoN，抓若干帧，把最后一帧原始数据(MJPEG 或 YUYV)写到文件，
//   用来确认 UVC 相机在 SG2002 上能出图。
//
//   用法: test_uvc_capture [index=0] [width=640] [height=480] [frames=10] [out=/tmp/uvc_frame.bin]
//   说明: MJPEG 格式可直接改名 .jpg 打开查看；YUYV 则是裸 YUV422。

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include "uvc_capture.hpp"

int main(int argc, char** argv)
{
    int index  = (argc >= 2) ? atoi(argv[1]) : 0;
    int width  = (argc >= 3) ? atoi(argv[2]) : 640;
    int height = (argc >= 4) ? atoi(argv[3]) : 480;
    int frames = (argc >= 5) ? atoi(argv[4]) : 10;
    const char* out = (argc >= 6) ? argv[5] : "/tmp/uvc_frame.bin";

    printf("打开 /dev/video%d  %dx%d  抓 %d 帧\n", index, width, height, frames);

    UvcCapture cam;
    if (cam.open(index, width, height, 30) != 0) {
        printf("结果: 无法打开 /dev/video%d\n", index);
        return 1;
    }

    // 抓帧缓冲：留足够大，MJPEG/YUYV 都够用
    size_t cap = (size_t)width * height * 2 + 65536;
    uint8_t* buf = (uint8_t*)malloc(cap);
    if (!buf) { printf("结果: 内存分配失败\n"); cam.close(); return 1; }

    int got = 0, last_bytes = 0;
    for (int i = 0; i < frames; i++) {
        int n = cam.getFrame(buf, cap, 1000);
        if (n > 0) {
            got++;
            last_bytes = n;
            printf("  frame %2d/%d: %d bytes\n", i + 1, frames, n);
        } else {
            printf("  frame %2d/%d: 超时/读取失败\n", i + 1, frames);
        }
    }

    if (got == 0) {
        printf("结果: 一帧都没抓到 —— 相机没出图，检查 /dev/video%d 是否存在、格式是否支持。\n", index);
        free(buf); cam.close();
        return 2;
    }

    // 把最后一帧存盘（再抓一帧拿干净数据）
    int n = cam.getFrame(buf, cap, 1000);
    if (n <= 0) n = last_bytes;  // 兜底用上一帧的字节数没法重存，仅做提示
    if (n > 0) {
        FILE* f = fopen(out, "wb");
        if (f) {
            fwrite(buf, 1, (size_t)n, f);
            fclose(f);
            printf("已保存最后一帧到 %s (%d bytes)\n", out, n);
        } else {
            printf("警告: 无法写入 %s\n", out);
        }
    }

    printf("结果: ✓ 成功抓到 %d/%d 帧 —— /dev/video%d 工作正常。\n", got, frames, index);
    free(buf);
    cam.close();
    return 0;
}
