// UvcCapture implementation using V4L2 (linux).
// Derived from linuxcam_simple_test.c - verified working on SG2002.
#include "uvc_capture.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>
#include <time.h>

#define BUFFER_COUNT 4

// ── open ──────────────────────────────────────────────────────────────────────
int UvcCapture::open(int device_index, int width, int height, int fps)
{
    width_  = width;
    height_ = height;

    // Build device path: /dev/video0, /dev/video1, ...
    char dev_path[32];
    snprintf(dev_path, sizeof(dev_path), "/dev/video%d", device_index);

    // 1. Open device (blocking)
    fd_ = ::open(dev_path, O_RDWR);
    if (fd_ < 0) {
        fprintf(stderr, "[UvcCapture] open %s: %s\n", dev_path, strerror(errno));
        return -1;
    }

    // 2. Query capability
    struct v4l2_capability cap;
    if (ioctl(fd_, VIDIOC_QUERYCAP, &cap) < 0) {
        fprintf(stderr, "[UvcCapture] QUERYCAP: %s\n", strerror(errno));
        close(); return -1;
    }
    printf("[UvcCapture] driver=%s  card=%s  bus=%s\n", cap.driver, cap.card, cap.bus_info);

    // 3. Set format - try MJPEG first, fall back to YUYV
    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = width;
    fmt.fmt.pix.height = height;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;

    if (ioctl(fd_, VIDIOC_S_FMT, &fmt) < 0) {
        fprintf(stderr, "[UvcCapture] MJPEG not supported, trying YUYV...\n");
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
        if (ioctl(fd_, VIDIOC_S_FMT, &fmt) < 0) {
            fprintf(stderr, "[UvcCapture] S_FMT: %s\n", strerror(errno));
            close(); return -1;
        }
    }

    // Read back actual format
    if (ioctl(fd_, VIDIOC_G_FMT, &fmt) < 0) {
        fprintf(stderr, "[UvcCapture] G_FMT: %s\n", strerror(errno));
        close(); return -1;
    }
    printf("[UvcCapture] %ux%u fmt=0x%08X size=%u\n",
           fmt.fmt.pix.width, fmt.fmt.pix.height,
           fmt.fmt.pix.pixelformat, fmt.fmt.pix.sizeimage);
    width_  = fmt.fmt.pix.width;
    height_ = fmt.fmt.pix.height;

    // 4. Request buffers
    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof(req));
    req.count = BUFFER_COUNT;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (ioctl(fd_, VIDIOC_REQBUFS, &req) < 0 || req.count < 2) {
        fprintf(stderr, "[UvcCapture] REQBUFS failed\n");
        close(); return -1;
    }
    buffer_count_ = req.count;

    // 5. Map buffers
    buffers_ = static_cast<Buffer*>(calloc(buffer_count_, sizeof(Buffer)));
    for (int i = 0; i < buffer_count_; i++) {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (ioctl(fd_, VIDIOC_QUERYBUF, &buf) < 0) {
            fprintf(stderr, "[UvcCapture] QUERYBUF %d: %s\n", i, strerror(errno));
            close(); return -1;
        }

        buffers_[i].length = buf.length;
        buffers_[i].start = mmap(NULL, buf.length, PROT_READ | PROT_WRITE,
                                 MAP_SHARED, fd_, buf.m.offset);
        if (buffers_[i].start == MAP_FAILED) {
            fprintf(stderr, "[UvcCapture] mmap %d: %s\n", i, strerror(errno));
            close(); return -1;
        }
    }

    // 6. Enqueue all buffers
    for (int i = 0; i < buffer_count_; i++) {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (ioctl(fd_, VIDIOC_QBUF, &buf) < 0) {
            fprintf(stderr, "[UvcCapture] QBUF %d: %s\n", i, strerror(errno));
            close(); return -1;
        }
    }

    // 7. Start streaming
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd_, VIDIOC_STREAMON, &type) < 0) {
        fprintf(stderr, "[UvcCapture] STREAMON: %s\n", strerror(errno));
        close(); return -1;
    }

    printf("[UvcCapture] streaming %dx%d on %s\n", width_, height_, dev_path);
    return 0;
}

// ── close ─────────────────────────────────────────────────────────────────────
void UvcCapture::close()
{
    if (fd_ >= 0) {
        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        ioctl(fd_, VIDIOC_STREAMOFF, &type);
    }

    if (buffers_) {
        for (int i = 0; i < buffer_count_; i++) {
            if (buffers_[i].start && buffers_[i].start != MAP_FAILED)
                munmap(buffers_[i].start, buffers_[i].length);
        }
        free(buffers_);
        buffers_ = nullptr;
    }
    buffer_count_ = 0;

    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

// ── getFrame ──────────────────────────────────────────────────────────────────
int UvcCapture::getFrame(uint8_t* buf, size_t cap, int timeout_ms)
{
    if (fd_ < 0) return -1;

    // Use select() for timeout
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(fd_, &fds);
    struct timeval tv;
    tv.tv_sec  = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    int ret = select(fd_ + 1, &fds, nullptr, nullptr, &tv);
    if (ret <= 0) return -1;  // timeout or error

    // Dequeue a frame
    struct v4l2_buffer vbuf;
    memset(&vbuf, 0, sizeof(vbuf));
    vbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    vbuf.memory = V4L2_MEMORY_MMAP;

    if (ioctl(fd_, VIDIOC_DQBUF, &vbuf) < 0)
        return -1;

    int size = vbuf.bytesused;

    // Copy to caller buffer
    size_t copy_len = (size_t)size < cap ? (size_t)size : cap;
    memcpy(buf, buffers_[vbuf.index].start, copy_len);

    // Re-enqueue
    ioctl(fd_, VIDIOC_QBUF, &vbuf);

    return static_cast<int>(copy_len);
}
