#ifndef UVC_CAPTURE_HPP
#define UVC_CAPTURE_HPP

#include <cstdint>
#include <cstddef>

// UVC camera capture wrapper using V4L2 (linux).
// After open(), call getFrame() in a loop; it fills a caller-supplied buffer
// with raw MJPEG bytes.  Call close() when done.
class UvcCapture {
public:
    UvcCapture() {}
    ~UvcCapture() { close(); }

    // Open device at zero-based index with requested format.
    // Returns 0 on success, -1 on failure.
    int open(int device_index = 0,
             int width = 640,
             int height = 480,
             int fps = 30);

    void close();

    // Block until the next frame arrives (or timeout_ms elapses).
    // Copies MJPEG data into buf (capacity cap).
    // Returns number of bytes written, or -1 on error/timeout.
    int getFrame(uint8_t* buf, size_t cap, int timeout_ms = 200);

    int width()  const { return width_;  }
    int height() const { return height_; }

private:
    int fd_ = -1;

    // V4L2 mmap buffers
    struct Buffer {
        void* start = nullptr;
        size_t length = 0;
    };
    Buffer* buffers_ = nullptr;
    int buffer_count_ = 0;

    int width_  = 640;
    int height_ = 480;
};

#endif // UVC_CAPTURE_HPP
