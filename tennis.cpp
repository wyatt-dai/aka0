// Author: Zhihang Shao <dio_ro@outlook.com>
// Source: ported from aka-rk3588/tennis.cpp onto aka0 (SG2002 / CVI runtime)
// Description: 完整捡球比赛状态机
//   追球(CHASE_BALL) → 抓球(GRAB) → 找红桶(FIND_BUCKET)
//   → 趋近桶(APPROACH_BUCKET) → 放球(DEPOSIT) → 回到追球，循环
//
// 主要特性（移植自 rk3588）：
//   - 平滑连续差速转向（远距离差速、近距离轴转），不再脉冲式停转
//   - 动态速度 base_speed()：离得越近开得越慢，BRAKE 区锁最低速
//   - 停车对准：球需落在中心偏右 STOP_CENTER_OFFSET（夹爪偏置），确认 N 帧后制动抓取
//   - 对准卡死检测 + 踢一脚（ALIGN_KICK）脱困
//   - 太近自动后退（REVERSE），避免挤到球
//   - 丢球后沿最后方向搜索，长时间丢失原地扫描
//   - HSV 红桶检测 + 趋近 + 防抖确认 + 丢桶重搜
//   - 抓球后保持夹住，送到桶边再松手（arm.grab 持球 / arm.release 放球）
//   - Ctrl-C 安全退出
//
// 相机/推理为 aka0 原生：CVI runtime + VI(NV21) 或 ESP32-CAM(BGR)，640x640。
// 注意：下方 area_* 阈值沿用 rk3588（其相机为 640x480）的调好值，
//       换到 SG2002 的相机后建议用 test-yolo 读实际 area 重新标定。

#include <stdio.h>
#include <math.h>
#include <time.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <algorithm>
#include <vector>
#include <opencv2/opencv.hpp>
#include "cviruntime.h"
#include "motor.hpp"
#include "arm.hpp"
#include "detect.hpp"
#include "bucket_detect.hpp"

#if USE_ESP32_CAMERA
#include "esp32_capture.hpp"
#elif USE_UVC_CAMERA
#include "uvc_capture.hpp"
#else
#include "vi_capture.hpp"
#endif
#include "logger.hpp"

// ── Frame / model ──────────────────────────────────────────────────────────
static const int FRAME_WIDTH  = 640;
static const int FRAME_HEIGHT = 640;

#if USE_UVC_CAMERA
// UVC 相机的原始抓取分辨率（之后 imdecode + resize 到 FRAME_WIDTH x FRAME_HEIGHT）
static const int UVC_CAP_WIDTH  = 640;
static const int UVC_CAP_HEIGHT = 480;
#endif

// ── Chase 控制参数（映射后：快轮20，慢轮15）────────────────────────────────
static const int   CHASE_SPEED_FAR  = 4;   // →映射17
static const int   CHASE_SPEED_NEAR = 3;   // →映射16

static const float AREA_FAR         = 0.02f;
static const float AREA_NEAR        = 0.35f;
static const float AREA_BRAKE       = 0.20f;

static const int   BRAKE_SPEED      = 3;
static const float AREA_STOP        = 0.28f;
static const float AREA_REVERSE     = 0.50f;
static const int   REVERSE_SPEED    = 2;   // →映射15

static const float AREA_STOP_EXIT   = 0.20f;
static const int   STOP_CONFIRM_CNT = 4;
static const int   BRAKE_PULSE_US   = 350000;
static const int   STOP_CENTER_OFFSET = 120;  // 目标：球落在中心偏右（夹爪偏置）

static const float K_TURN              = 6.0f;
static const int   MAX_TURN_BIAS_FAR   = 3;   // →映射16
static const int   MAX_TURN_BIAS_NEAR  = 1;   // →映射15
static const int   CENTER_DEAD_ZONE    = 5;
static const int   STOP_CENTER_ZONE    = 20;
static const int   ALIGN_PIVOT_SPD     = 2;   // →映射15
static const int   ALIGN_PIVOT_MIN     = 2;   // →映射15

static const int   SEARCH_FRAMES    = 25;
static const int   SEARCH_PIVOT_SPD = 3;   // →映射16

static const int   ALIGN_STALL_FRAMES  = 20;
static const int   ALIGN_STALL_MOVE_PX = 10;
static const int   ALIGN_KICK_SPD      = 3;   // →映射16
static const int   ALIGN_KICK_US       = 180000;

// ── Bucket 趋近参数 ──────────────────────────────────────────────────────────
static const float BUCKET_AREA_DEPOSIT  = 0.90f; // 桶够大 → 放球
static const float BUCKET_AREA_BRAKE    = 0.70f; // 桶很大 → 减速防撞
static const int   BUCKET_APPROACH_SPD  = 4;   // →映射17
static const int   BUCKET_BRAKE_SPD     = 2;   // →映射16
static const float BUCKET_K_TURN        = 5.0f;
static const int   BUCKET_MAX_BIAS      = 3;   // →映射16
static const int   BUCKET_SEARCH_SPD    = 3;   // →映射16
static const int   BUCKET_LOST_FRAMES   = 10;
static const int   BUCKET_CONFIRM_CNT   = 3;
static const int   BUCKET_MIN_AREA      = 1000; // HSV 最小连通域像素

// ── 游戏状态机 ───────────────────────────────────────────────────────────────
enum class GameState {
    CHASE_BALL,      // 追球，直到抓到
    FIND_BUCKET,     // 旋转搜索红色桶
    APPROACH_BUCKET, // 趋近桶
    DEPOSIT,         // 放球（单次），完成后回到 CHASE_BALL
};

static const char* game_name(GameState s) {
    switch (s) {
        case GameState::CHASE_BALL:      return "CHASE_BALL";
        case GameState::FIND_BUCKET:     return "FIND_BUCKET";
        case GameState::APPROACH_BUCKET: return "APPROACH_BUCKET";
        case GameState::DEPOSIT:         return "DEPOSIT";
    }
    return "?";
}

// ── 信号处理用全局指针 ───────────────────────────────────────────────────────
#if USE_ESP32_CAMERA
static ESP32Capture* g_capture = nullptr;
#elif USE_UVC_CAMERA
static UvcCapture* g_uvc_capture = nullptr;
#else
static VICapture* g_vi_capture = nullptr;
#endif
static Motor*           g_motor = nullptr;
static Arm*             g_arm   = nullptr;
static CVI_MODEL_HANDLE g_model = nullptr;

static void cleanup_and_exit() {
    if (g_motor) g_motor->standby();
#if USE_ESP32_CAMERA
    if (g_capture) g_capture->deinit();
#elif USE_UVC_CAMERA
    if (g_uvc_capture) g_uvc_capture->close();
#else
  #if USE_VPSS_RESIZE
    if (g_vi_capture) g_vi_capture->deinitVpssResize();
  #endif
    if (g_vi_capture) g_vi_capture->deinit();
#endif
    if (g_model) CVI_NN_CleanupModel(g_model);
}

static void signal_handler(int /*sig*/) {
    cleanup_and_exit();
    exit(0);
}

// ── 计时辅助 ─────────────────────────────────────────────────────────────────
static long elapsed_us(const struct timeval& start) {
    struct timeval now; gettimeofday(&now, nullptr);
    return (now.tv_sec - start.tv_sec) * 1000000L + (now.tv_usec - start.tv_usec);
}

#if USE_UVC_CAMERA
// Letterbox 参数：用于将检测坐标从模型输入空间映射回原始相机画面
static int   g_lb_pad_x  = 0;
static int   g_lb_pad_y  = 0;
static float g_lb_scale  = 1.0f;
static int   g_cam_w     = UVC_CAP_WIDTH;
static int   g_cam_h     = UVC_CAP_HEIGHT;

// letterbox: 等比缩放 + 灰色(114)填充，与 rk3588 decode_mjpeg 行为一致
static void letterbox(const cv::Mat& src, cv::Mat& dst,
                      int out_w, int out_h,
                      int& pad_x, int& pad_y, float& scale) {
    scale = std::min((float)out_w / src.cols, (float)out_h / src.rows);
    int nw = (int)(src.cols * scale);
    int nh = (int)(src.rows * scale);
    pad_x = (out_w - nw) / 2;
    pad_y = (out_h - nh) / 2;

    dst = cv::Mat(out_h, out_w, CV_8UC3, cv::Scalar(114, 114, 114));
    cv::Mat resized;
    cv::resize(src, resized, cv::Size(nw, nh));
    resized.copyTo(dst(cv::Rect(pad_x, pad_y, nw, nh)));
}

// 从 UVC 相机抓一帧 -> 解码 -> letterbox 到 FRAME_WIDTH x FRAME_HEIGHT 的 BGR。
// UVC 出 MJPEG 时用 imdecode；个别相机出 YUYV 时按 YUYV->BGR 转。
// 返回 true 表示 bgr 有效。
static bool uvc_grab_bgr(UvcCapture& cam, cv::Mat& bgr) {
    static std::vector<uint8_t> buf(UVC_CAP_WIDTH * UVC_CAP_HEIGHT * 2 + 65536);
    int n = cam.getFrame(buf.data(), buf.size(), 300);
    if (n <= 0) return false;

    cv::Mat decoded;
    // MJPEG: 以 FF D8 开头 -> JPEG 解码
    if (n >= 2 && buf[0] == 0xFF && buf[1] == 0xD8) {
        cv::Mat raw(1, n, CV_8UC1, buf.data());
        decoded = cv::imdecode(raw, cv::IMREAD_COLOR);
    } else if (n == UVC_CAP_WIDTH * UVC_CAP_HEIGHT * 2) {
        // YUYV422 -> BGR
        cv::Mat yuyv(UVC_CAP_HEIGHT, UVC_CAP_WIDTH, CV_8UC2, buf.data());
        cv::cvtColor(yuyv, decoded, cv::COLOR_YUV2BGR_YUYV);
    } else {
        return false;
    }
    if (decoded.empty()) return false;

    g_cam_w = decoded.cols;
    g_cam_h = decoded.rows;

    if (decoded.cols != FRAME_WIDTH || decoded.rows != FRAME_HEIGHT) {
        letterbox(decoded, bgr, FRAME_WIDTH, FRAME_HEIGHT,
                  g_lb_pad_x, g_lb_pad_y, g_lb_scale);
    } else {
        bgr = decoded;
        g_lb_pad_x = 0; g_lb_pad_y = 0; g_lb_scale = 1.0f;
    }
    return true;
}
#endif

// ── 动态前进速度：area 越大越慢，到 BRAKE 区锁最低速 ─────────────────────────
static int base_speed(float area_ratio) {
    if (area_ratio >= AREA_BRAKE) return BRAKE_SPEED;
    float t = (area_ratio - AREA_FAR) / (AREA_BRAKE - AREA_FAR);
    t = std::max(0.0f, std::min(1.0f, t));
    return (int)(CHASE_SPEED_FAR + t * (BRAKE_SPEED - CHASE_SPEED_FAR));
}

static int g_stop_center_offset = STOP_CENTER_OFFSET;  // 可通过命令行覆盖

static void usage(char** argv) {
    LOGI("Usage:");
    LOGI("  %s <model.cvimodel> [vi_channel|esp32] [uart_dev] [arm_dev] [offset]", argv[0]);
    LOGI("  Example: %s tennis.cvimodel 0 /dev/ttyS1 /dev/ttyS2 120", argv[0]);
    LOGI("  offset: ball target x-offset from frame center (default=%d, negative=left)", STOP_CENTER_OFFSET);
}

// ── Main ──────────────────────────────────────────────────────────────────────
int main(int argc, char** argv)
{
    if (argc < 2) { usage(argv); return 1; }

    const char* model_path = argv[1];
    // argv[2]: VI 通道号（数字）或 "esp32"（仅作记录，相机类型由编译宏决定）
    int         vi_channel = (argc >= 3 && strcmp(argv[2], "esp32") != 0) ? atoi(argv[2]) : 0;
    const char* uart_dev   = (argc >= 4) ? argv[3] : "/dev/ttyS1";
    const char* arm_dev    = (argc >= 5) ? argv[4] : "/dev/ttyS2";
    if (argc >= 6) g_stop_center_offset = atoi(argv[5]);

    signal(SIGINT,  signal_handler);
    signal(SIGTERM, signal_handler);

    // ── 电机（ESP32-C3 UART 底盘）──────────────────────────────────────────────
    Motor motor(MotorDriverType::UART, uart_dev);
    g_motor = &motor;
    // 最小可动速度，低于此值电机转不动。换车时用 test-motor 测后修改此值
    motor.set_min_speed(15);
    LOGI("Motor initialized (UART %s)  min_speed=%d", uart_dev, motor.get_min_speed());

    // ── 机械臂 ─────────────────────────────────────────────────────────────────
    Arm arm(arm_dev);
    g_arm = &arm;
    arm.grab_pos();  // 归位到待抓取姿势
    LOGI("Arm initialized (%s)", arm_dev);

    // ── 相机 ───────────────────────────────────────────────────────────────────
#if USE_ESP32_CAMERA
    ESP32Capture capture;
    g_capture = &capture;
    if (capture.init() != 0) { LOGE("Failed to init ESP32 camera"); return 1; }
    capture.setResize(FRAME_WIDTH, FRAME_HEIGHT);
#elif USE_UVC_CAMERA
    UvcCapture uvc_capture;
    g_uvc_capture = &uvc_capture;
    // UVC 相机走 /dev/videoN；vi_channel 复用为 video 设备号
    if (uvc_capture.open(vi_channel, UVC_CAP_WIDTH, UVC_CAP_HEIGHT, 30) != 0) {
        LOGE("Failed to open UVC camera /dev/video%d", vi_channel); return 1;
    }
    LOGI("UVC camera opened (/dev/video%d) %dx%d -> %dx%d",
         vi_channel, UVC_CAP_WIDTH, UVC_CAP_HEIGHT, FRAME_WIDTH, FRAME_HEIGHT);
    LOGI("ESP32 camera opened, resize -> %dx%d", FRAME_WIDTH, FRAME_HEIGHT);
#else
    VICapture vi_capture;
    g_vi_capture = &vi_capture;
    if (vi_capture.init() != CVI_SUCCESS) { LOGE("VI init failed"); return 1; }
  #if USE_VPSS_RESIZE
    if (vi_capture.initVpssResize(2560, 1440, FRAME_WIDTH, FRAME_HEIGHT) != CVI_SUCCESS) {
        LOGE("VPSS init failed"); vi_capture.deinit(); return 1;
    }
  #endif
    LOGI("VI camera opened (chn=%d) -> %dx%d", vi_channel, FRAME_WIDTH, FRAME_HEIGHT);
#endif

    // ── 加载 CVI 模型 ───────────────────────────────────────────────────────────
    CVI_MODEL_HANDLE model;
    if (CVI_NN_RegisterModel(model_path, &model) != CVI_RC_SUCCESS) {
        LOGE("Failed to load model: %s", model_path);
        cleanup_and_exit();
        return 1;
    }
    g_model = model;

    CVI_TENSOR *input_tensors, *output_tensors;
    int32_t input_num, output_num;
    CVI_NN_GetInputOutputTensors(model, &input_tensors, &input_num,
                                 &output_tensors, &output_num);
    CVI_TENSOR* input = CVI_NN_GetTensorByName(CVI_NN_DEFAULT_TENSOR,
                                               input_tensors, input_num);
    CVI_SHAPE input_shape = CVI_NN_TensorShape(input);
    int model_h = input_shape.dim[2];
    int model_w = input_shape.dim[3];
    LOGI("Model input %dx%d", model_w, model_h);

    CVI_SHAPE* output_shape = (CVI_SHAPE*)calloc(output_num, sizeof(CVI_SHAPE));
    for (int i = 0; i < output_num; i++)
        output_shape[i] = CVI_NN_TensorShape(&output_tensors[i]);

    const int classes_num = 1;  // 单类别：网球
    const int channel_size = model_h * model_w;

    // ── 预热相机（丢弃前若干帧让传感器稳定）────────────────────────────────────
    LOGI("Warming up camera...");
    usleep(1000 * 1000);
    {
        cv::Mat warm;
        for (int i = 0; i < 15; i++) {
#if USE_ESP32_CAMERA
            capture.getFrameAsBGR(warm);
#elif USE_UVC_CAMERA
            uvc_grab_bgr(uvc_capture, warm);
#else
            vi_capture.getFrameAsBGR(vi_channel, warm);
#endif
            usleep(30000);
        }
    }

    int  frame_idx = 0;

    // ── 丢球后搜索用的"最后一次看到"记录 ────────────────────────────────────────
    int  last_offset     = 0;
    int  last_seen_frame = -999;

    // ── 停车状态 ───────────────────────────────────────────────────────────────
    bool stopped          = false;
    int  stop_confirm_cnt = 0;
    int  align_cnt        = 0;

    // ── ALIGN 卡死检测环形缓冲 ──────────────────────────────────────────────────
    static const int STALL_BUF = 30;
    int  align_off_buf[STALL_BUF] = {};
    int  align_off_head = 0;

    // ── 游戏状态 ───────────────────────────────────────────────────────────────
    GameState game_state  = GameState::CHASE_BALL;
    int  bucket_lost_cnt  = 0;   // 连续找不到桶的帧数
    int  bucket_confirm   = 0;   // 连续看到桶的帧数（防抖）

    LOGI("STOP_CENTER_OFFSET=%d", g_stop_center_offset);
    LOGI("Entering main loop. Ctrl-C to exit.");

    while (true) {
        struct timeval t_start;
        gettimeofday(&t_start, nullptr);
        frame_idx++;

        // ── 抓取一帧 BGR（已 resize 到 FRAME_WIDTH x FRAME_HEIGHT）────────────────
        cv::Mat bgr;
#if USE_ESP32_CAMERA
        int cap_ret = capture.getFrameAsBGR(bgr);
        bool cap_ok = (cap_ret == 0 && bgr.data);
#elif USE_UVC_CAMERA
        bool cap_ok = uvc_grab_bgr(uvc_capture, bgr);
#else
        int cap_ret = vi_capture.getFrameAsBGR(vi_channel, bgr);
        bool cap_ok = (cap_ret == CVI_SUCCESS && bgr.data);
#endif
        if (!cap_ok) {
            LOGW("[Frame %d] No frame", frame_idx);
            usleep(10000);
            continue;
        }
        const int img_w  = bgr.cols;
        const int img_h  = bgr.rows;
        const int half_w = img_w / 2;

        // ── 桶状态机（FIND_BUCKET / APPROACH_BUCKET / DEPOSIT）──────────────────
        // 这几个状态不需要 YOLO，直接用 HSV 桶检测后跳过追球逻辑
        if (game_state == GameState::FIND_BUCKET ||
            game_state == GameState::APPROACH_BUCKET ||
            game_state == GameState::DEPOSIT)
        {
            if (game_state == GameState::DEPOSIT) {
                motor.standby();
                LOGI("[GAME] DEPOSIT - releasing ball...");
                if (g_arm) g_arm->release();   // 在抬起位张爪放球
                usleep(500000);
                if (g_arm) g_arm->grab_pos();  // 回到待抓取位
                // ── 重置，继续找下一个球 ──────────────────────────────────────
                game_state       = GameState::CHASE_BALL;
                stopped          = false;
                stop_confirm_cnt = 0;
                align_cnt        = 0;
                align_off_head   = 0;
                last_seen_frame  = -999;
                bucket_lost_cnt  = 0;
                bucket_confirm   = 0;
                LOGI("[GAME] -> CHASE_BALL (next round)");
                continue;
            }

            // FIND_BUCKET / APPROACH_BUCKET: 在 BGR 帧上做 HSV 红桶检测
            BucketResult br;
            bool bucket_visible = detect_bucket_bgr(bgr, br, BUCKET_MIN_AREA);

            if (game_state == GameState::FIND_BUCKET) {
                if (bucket_visible) {
                    bucket_confirm++;
                    bucket_lost_cnt = 0;
                    if (bucket_confirm >= BUCKET_CONFIRM_CNT) {
                        game_state     = GameState::APPROACH_BUCKET;
                        bucket_confirm = 0;
                        LOGI("[GAME] -> APPROACH_BUCKET  area=%.3f", br.area_ratio);
                    } else {
                        motor.standby();  // 稳住等确认
                    }
                } else {
                    bucket_confirm = 0;
                    bucket_lost_cnt++;
                    // 原地旋转搜索（始终向右转，可按场地调整）
                    motor.drive(BUCKET_SEARCH_SPD, -BUCKET_SEARCH_SPD);
                    LOGI("[GAME] FIND_BUCKET searching... lost=%d", bucket_lost_cnt);
                }
                continue;
            }

            // APPROACH_BUCKET
            if (!bucket_visible) {
                bucket_lost_cnt++;
                if (bucket_lost_cnt > BUCKET_LOST_FRAMES) {
                    game_state     = GameState::FIND_BUCKET;
                    bucket_confirm = 0;
                    LOGI("[GAME] APPROACH_BUCKET lost bucket -> FIND_BUCKET");
                } else {
                    motor.standby();  // 短暂丢失先停车等
                }
                continue;
            }
            bucket_lost_cnt = 0;

            if (br.area_ratio >= BUCKET_AREA_DEPOSIT) {
                // 足够近 → 制动停车，进入放球
                struct timeval tb2; gettimeofday(&tb2, nullptr);
                while (elapsed_us(tb2) < BRAKE_PULSE_US) {
                    motor.brake(); usleep(20000);
                }
                motor.standby();
                game_state = GameState::DEPOSIT;
                LOGI("[GAME] -> DEPOSIT  bucket area=%.3f", br.area_ratio);
                continue;
            }

            // 趋近：差速对准桶中心
            int bk_off  = br.cx - half_w;
            int bk_bias = (abs(bk_off) <= CENTER_DEAD_ZONE) ? 0
                        : (int)(BUCKET_K_TURN * bk_off / (float)half_w);
            bk_bias = std::max(-BUCKET_MAX_BIAS, std::min(BUCKET_MAX_BIAS, bk_bias));

            int bk_spd = (br.area_ratio >= BUCKET_AREA_BRAKE)
                         ? BUCKET_BRAKE_SPD : BUCKET_APPROACH_SPD;
            int bk_l = std::max(-100, std::min(100, bk_spd + bk_bias));
            int bk_r = std::max(-100, std::min(100, bk_spd - bk_bias));
            motor.drive(bk_l, bk_r);
            LOGI("[GAME] APPROACH_BUCKET  area=%.3f off=%d  L=%d R=%d",
                 br.area_ratio, bk_off, bk_l, bk_r);
            continue;
        }

        // ── CHASE_BALL：YOLO 推理 ───────────────────────────────────────────────
        // 预处理：letterbox 到模型输入尺寸 -> BGR2RGB -> split -> 填充 NCHW int8
        cv::Mat model_in;
        if (img_w == model_w && img_h == model_h) {
            model_in = bgr;
        } else {
            float lb = std::min((float)model_w / img_w, (float)model_h / img_h);
            int nw = (int)(img_w * lb), nh = (int)(img_h * lb);
            cv::Mat resized;
            cv::resize(bgr, resized, cv::Size(nw, nh));
            model_in = cv::Mat(model_h, model_w, CV_8UC3, cv::Scalar(114, 114, 114));
            resized.copyTo(model_in(cv::Rect((model_w - nw) / 2, (model_h - nh) / 2, nw, nh)));
        }

        cv::Mat rgb;
        cv::cvtColor(model_in, rgb, cv::COLOR_BGR2RGB);
        cv::Mat ch[3];
        for (int i = 0; i < 3; i++) ch[i] = cv::Mat(model_h, model_w, CV_8SC1);
        cv::split(rgb, ch);
        int8_t* iptr = (int8_t*)CVI_NN_TensorPtr(input);
        for (int i = 0; i < 3; i++)
            memcpy(iptr + i * channel_size, ch[i].data, channel_size);

        CVI_NN_Forward(model, input_tensors, input_num, output_tensors, output_num);

        std::vector<detection> dets;
        int det_num = getDetections(output_tensors, model_h, model_w, classes_num,
                                    output_shape[0], 0.5f, dets);
        NMS(dets, &det_num, 0.45f);
        correctYoloBoxes(dets, det_num, img_h, img_w, model_h, model_w);

        // ── 将检测坐标从 letterbox 空间映射回原始相机画面 ──────────────────────
#if USE_UVC_CAMERA
        if (g_lb_scale != 1.0f || g_lb_pad_x != 0 || g_lb_pad_y != 0) {
            for (int i = 0; i < det_num; i++) {
                float cx = dets[i].bbox.x, cy = dets[i].bbox.y;
                float w = dets[i].bbox.w, h = dets[i].bbox.h;
                float x1 = cx - w * 0.5f, y1 = cy - h * 0.5f;
                float x2 = cx + w * 0.5f, y2 = cy + h * 0.5f;
                x1 = std::max(0.0f, (x1 - g_lb_pad_x) / g_lb_scale);
                y1 = std::max(0.0f, (y1 - g_lb_pad_y) / g_lb_scale);
                x2 = std::min((float)g_cam_w, (x2 - g_lb_pad_x) / g_lb_scale);
                y2 = std::min((float)g_cam_h, (y2 - g_lb_pad_y) / g_lb_scale);
                dets[i].bbox.x = (x1 + x2) * 0.5f;
                dets[i].bbox.y = (y1 + y2) * 0.5f;
                dets[i].bbox.w = x2 - x1;
                dets[i].bbox.h = y2 - y1;
            }
        }
        const int cam_half_w = g_cam_w / 2;
#else
        const int cam_half_w = half_w;
#endif

        // ── 平滑差速转向 ────────────────────────────────────────────────────────
        if (!dets.empty()) {
            int best = 0;
            for (int i = 1; i < (int)dets.size(); i++)
                if (dets[i].bbox.w * dets[i].bbox.h >
                    dets[best].bbox.w * dets[best].bbox.h)
                best = i;

            const box& b     = dets[best].bbox;
#if USE_UVC_CAMERA
            float area_ratio = (b.w * b.h) / (float)(g_cam_w * g_cam_h);
#else
            float area_ratio = (b.w * b.h) / (float)(img_w * img_h);
#endif
            int   ball_cx    = (int)b.x;
            int   offset     = ball_cx - cam_half_w;   // <0 = 球在左

            last_offset     = offset;
            last_seen_frame = frame_idx;

            int spd = base_speed(area_ratio);

            const char* zone = (area_ratio >= AREA_REVERSE) ? "REVERSE" :
                               (area_ratio >= AREA_STOP)    ? "STOP"    :
                               (area_ratio >= AREA_BRAKE)   ? "BRAKE"   :
                               (area_ratio >= AREA_NEAR)    ? "NEAR"    :
                               (area_ratio >= AREA_FAR)     ? "FAR"     : "LOST";

            // ── 停车态：球离远才退出 ──────────────────────────────────────────
            if (stopped) {
                if (area_ratio >= AREA_REVERSE) {
                    stopped = false; stop_confirm_cnt = 0;
                    LOGI("[STATE] STOPPED->REVERSE  area=%.3f", area_ratio);
                    // fall through to REVERSE
                } else if (area_ratio < AREA_STOP_EXIT) {
                    stopped = false; stop_confirm_cnt = 0;
                    LOGI("[STATE] RESUME  area=%.3f zone=%s", area_ratio, zone);
                } else {
                    motor.standby();
                    continue;
                }
            }

            // ── 后退：球占画面过大 ────────────────────────────────────────────
            if (area_ratio >= AREA_REVERSE) {
                int rev_l = (offset > CENTER_DEAD_ZONE)  ? -REVERSE_SPEED + 1 :
                            (offset < -CENTER_DEAD_ZONE) ? -REVERSE_SPEED - 1 : -REVERSE_SPEED;
                int rev_r = (offset > CENTER_DEAD_ZONE)  ? -REVERSE_SPEED - 1 :
                            (offset < -CENTER_DEAD_ZONE) ? -REVERSE_SPEED + 1 : -REVERSE_SPEED;
                motor.drive(rev_l, rev_r);
                LOGI("[STATE] REVERSE  area=%.3f off=%d  L=%d R=%d",
                     area_ratio, offset, rev_l, rev_r);
                continue;
            }

            // ── 停车条件：够近且球落在目标偏置，连续确认 N 帧 ─────────────────
            int stop_off = offset - g_stop_center_offset;
            if (area_ratio >= AREA_STOP && abs(stop_off) <= STOP_CENTER_ZONE) {
                stop_confirm_cnt++;
                align_cnt = 0; align_off_head = 0;
                if (stop_confirm_cnt >= STOP_CONFIRM_CNT) {
                    LOGI("[STATE] BRAKING  area=%.3f off=%d  %dms...",
                         area_ratio, offset, BRAKE_PULSE_US / 1000);
                    struct timeval tb; gettimeofday(&tb, nullptr);
                    while (elapsed_us(tb) < BRAKE_PULSE_US) { motor.brake(); usleep(20000); }
                    motor.standby();
                    stopped = true;
                    LOGI("[STATE] STOPPED  area=%.3f  -> GRAB", area_ratio);
                    if (g_arm) g_arm->grab();   // 抓球（结束时夹住球）
                    game_state      = GameState::FIND_BUCKET;
                    bucket_lost_cnt = 0;
                    bucket_confirm  = 0;
                    LOGI("[GAME] -> FIND_BUCKET");
                } else {
                    motor.brake();
                }
                continue;
            } else if (area_ratio >= AREA_STOP && abs(stop_off) > STOP_CENTER_ZONE) {
                // 够近但未对准 → 比例轴转对准
                stop_confirm_cnt = 0;
                align_cnt++;

                // 卡死检测：记录 offset 历史，近 N 帧无移动则踢一脚
                align_off_buf[align_off_head % STALL_BUF] = offset;
                align_off_head++;
                bool stalled = false;
                if (align_cnt >= ALIGN_STALL_FRAMES) {
                    int mn = align_off_buf[0], mx = align_off_buf[0];
                    int check = std::min(align_cnt, STALL_BUF);
                    for (int i = 1; i < check; i++) {
                        int v = align_off_buf[i];
                        if (v < mn) mn = v;
                        if (v > mx) mx = v;
                    }
                    stalled = (mx - mn) < ALIGN_STALL_MOVE_PX;
                }

                if (stalled) {
                    int kick = (stop_off > 0) ? ALIGN_KICK_SPD : -ALIGN_KICK_SPD;
                    LOGI("[STATE] ALIGN_KICK  off=%d stop_off=%d kick=%d %dms",
                         offset, stop_off, kick, ALIGN_KICK_US / 1000);
                    struct timeval tk; gettimeofday(&tk, nullptr);
                    while (elapsed_us(tk) < ALIGN_KICK_US) { motor.drive(kick, -kick); usleep(20000); }
                    motor.brake();
                    align_off_head = 0; align_cnt = 0;
                    continue;
                }

                float t = std::min(1.0f, (float)abs(stop_off) / (float)cam_half_w);
                int pivot_spd = (int)(ALIGN_PIVOT_MIN + t * (ALIGN_PIVOT_SPD - ALIGN_PIVOT_MIN));
                int pivot = (stop_off > 0) ? pivot_spd : -pivot_spd;
                motor.drive(pivot, -pivot);
                LOGI("[STATE] ALIGN  off=%d stop_off=%d pivot=%d [%d]",
                     offset, stop_off, pivot, align_cnt);
                continue;
            } else {
                stop_confirm_cnt = 0; align_cnt = 0; align_off_head = 0;
            }

            // ── 追球 ──────────────────────────────────────────────────────────
            int bias = (abs(offset) <= CENTER_DEAD_ZONE) ? 0
                     : (int)(K_TURN * offset / (float)cam_half_w);

            int left_spd, right_spd;
            if (area_ratio >= AREA_NEAR) {
                // 近距离：纯轴转，双轮反向，避免差速丢球
                int pivot = std::max(-MAX_TURN_BIAS_NEAR, std::min(MAX_TURN_BIAS_NEAR, bias));
                left_spd = pivot; right_spd = -pivot;
            } else {
                // 远距离：差速，双轮同向，边走边修方向
                bias = std::max(-MAX_TURN_BIAS_FAR, std::min(MAX_TURN_BIAS_FAR, bias));
                left_spd  = std::max(-100, std::min(100, spd + bias));
                right_spd = std::max(-100, std::min(100, spd - bias));
            }
            motor.drive(left_spd, right_spd);

            long frame_us = elapsed_us(t_start);
            const char* steer = (area_ratio >= AREA_BRAKE) ? "pivot" : "diff";
            LOGI("[STATE] CHASE zone=%-6s area=%.3f off=%3d steer=%-5s L=%3d R=%3d fps=%.1f",
                 zone, area_ratio, offset, steer, left_spd, right_spd, 1e6f / frame_us);

        } else {
            int frames_lost = frame_idx - last_seen_frame;
            if (last_seen_frame >= 0 && frames_lost <= SEARCH_FRAMES) {
                // 刚丢失：沿最后看到球的方向快速转
                int pivot = (last_offset >= 0) ? SEARCH_PIVOT_SPD : -SEARCH_PIVOT_SPD;
                motor.drive(pivot, -pivot);
                LOGI("[STATE] SEARCH lost=%d/%d pivot=%s",
                     frames_lost, SEARCH_FRAMES, last_offset >= 0 ? "R" : "L");
            } else {
                // 长时间丢失：原地慢速扫描，每 60 帧反向
                int scan_dir = ((frame_idx / 60) % 2 == 0) ? 1 : -1;
                motor.drive(scan_dir * SEARCH_PIVOT_SPD, -scan_dir * SEARCH_PIVOT_SPD);
                LOGI("[STATE] SCAN  frame=%d dir=%s", frame_idx, scan_dir > 0 ? "R" : "L");
            }
        }
    }

    free(output_shape);
    cleanup_and_exit();
    return 0;
}
