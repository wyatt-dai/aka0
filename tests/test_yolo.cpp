// test_yolo.cpp — UVC/VI 摄像头 + YOLO 检测调试工具
// 采集一帧 → letterbox → YOLO 推理 → 打印坐标/置信度 → 保存标注图
//
// 用法:
//   test_yolo <model.cvimodel> [uvc_index=0] [output.jpg]
//
// 支持 VI / UVC 摄像头（由编译宏 USE_UVC_CAMERA 决定）

#include <opencv2/opencv.hpp>
#include "cviruntime.h"
#include "logger.hpp"
#include "detect.hpp"
#include <unistd.h>
#include <sys/stat.h>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <vector>

#if USE_UVC_CAMERA
#include "uvc_capture.hpp"
#else
#include "vi_capture.hpp"
#endif

// letterbox: 等比缩放 + 灰色(114)填充
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

int main(int argc, char** argv) {
    if (argc < 2) {
        printf("Usage: %s <model.cvimodel> [uvc_index=0] [output.jpg]\n", argv[0]);
        return 1;
    }
    const char* model_path = argv[1];
    int device_index = (argc >= 3) ? atoi(argv[2]) : 0;
    const char* output_path = (argc >= 4) ? argv[3] : "/root/images/test_yolo.jpg";

    // 确保输出目录存在
    {
        std::string path(output_path);
        size_t pos = path.rfind('/');
        if (pos != std::string::npos) mkdir(path.substr(0, pos).c_str(), 0755);
    }

    // ── 打开摄像头 ───────────────────────────────────────────────────────────
    const int CAP_W = 640;
    const int CAP_H = 480;
    const int MODEL_W = 640;
    const int MODEL_H = 640;

#if USE_UVC_CAMERA
    UvcCapture cam;
    if (cam.open(device_index, CAP_W, CAP_H, 30) != 0) {
        LOGE("Failed to open UVC camera /dev/video%d", device_index);
        return 1;
    }
    LOGI("UVC camera opened: /dev/video%d", device_index);
#else
    VICapture vi;
    if (vi.init() != CVI_SUCCESS) { LOGE("VI init failed"); return 1; }
#if USE_VPSS_RESIZE
    if (vi.initVpssResize(2560, 1440, MODEL_W, MODEL_H) != CVI_SUCCESS) {
        LOGE("VPSS init failed"); vi.deinit(); return 1;
    }
#endif
    LOGI("VI camera opened");
#endif

    // ── 预热 ─────────────────────────────────────────────────────────────────
    LOGI("Warming up camera (skip 15 frames)...");
    cv::Mat warm;
    for (int i = 0; i < 15; i++) {
#if USE_UVC_CAMERA
        static std::vector<uint8_t> wbuf(CAP_W * CAP_H * 2 + 65536);
        cam.getFrame(wbuf.data(), wbuf.size(), 500);
#else
        vi.getFrameAsBGR(device_index, warm);
#endif
        usleep(30000);
    }

    // ── 采集一帧 ─────────────────────────────────────────────────────────────
    cv::Mat raw_frame;  // 原始相机画面
    int cam_w = 0, cam_h = 0;

#if USE_UVC_CAMERA
    {
        static std::vector<uint8_t> buf(CAP_W * CAP_H * 2 + 65536);
        int n = cam.getFrame(buf.data(), buf.size(), 1000);
        cam.close();
        if (n <= 0) { LOGE("No frame from UVC camera"); return 1; }

        cv::Mat decoded;
        if (n >= 2 && buf[0] == 0xFF && buf[1] == 0xD8) {
            cv::Mat raw(1, n, CV_8UC1, buf.data());
            decoded = cv::imdecode(raw, cv::IMREAD_COLOR);
        } else if (n == CAP_W * CAP_H * 2) {
            cv::Mat yuyv(CAP_H, CAP_W, CV_8UC2, buf.data());
            cv::cvtColor(yuyv, decoded, cv::COLOR_YUV2BGR_YUYV);
        } else {
            LOGE("Unknown frame format (%d bytes)", n);
            return 1;
        }
        if (decoded.empty()) { LOGE("Decode failed"); return 1; }
        raw_frame = decoded;
        cam_w = decoded.cols;
        cam_h = decoded.rows;
    }
#else
    if (vi.getFrameAsBGR(device_index, raw_frame) != CVI_SUCCESS || raw_frame.empty()) {
        LOGE("Failed to capture frame from VI");
        vi.deinit();
        return 1;
    }
    cam_w = raw_frame.cols;
    cam_h = raw_frame.rows;
#if USE_VPSS_RESIZE
    vi.deinitVpssResize();
#endif
    vi.deinit();
#endif

    LOGI("Camera frame: %dx%d", cam_w, cam_h);

    // ── Letterbox ────────────────────────────────────────────────────────────
    cv::Mat lb_frame;
    int pad_x = 0, pad_y = 0;
    float lb_scale = 1.0f;

    if (cam_w != MODEL_W || cam_h != MODEL_H) {
        letterbox(raw_frame, lb_frame, MODEL_W, MODEL_H, pad_x, pad_y, lb_scale);
        LOGI("Letterbox: %dx%d -> %dx%d  scale=%.4f  pad=(%d,%d)",
             cam_w, cam_h, MODEL_W, MODEL_H, lb_scale, pad_x, pad_y);
    } else {
        lb_frame = raw_frame;
        LOGI("No letterbox needed (camera == model size)");
    }

    // ── 加载模型 ─────────────────────────────────────────────────────────────
    CVI_MODEL_HANDLE model;
    if (CVI_NN_RegisterModel(model_path, &model) != CVI_RC_SUCCESS) {
        LOGE("Failed to load model: %s", model_path);
        return 1;
    }
    CVI_TENSOR *input_tensors, *output_tensors;
    int32_t input_num, output_num;
    CVI_NN_GetInputOutputTensors(model, &input_tensors, &input_num,
                                 &output_tensors, &output_num);

    CVI_TENSOR* input = CVI_NN_GetTensorByName(CVI_NN_DEFAULT_TENSOR,
                                               input_tensors, input_num);
    CVI_SHAPE input_shape = CVI_NN_TensorShape(input);
    int model_h = input_shape.dim[2];
    int model_w = input_shape.dim[3];
    LOGI("Model input: %dx%d", model_w, model_h);

    CVI_SHAPE* output_shape = (CVI_SHAPE*)calloc(output_num, sizeof(CVI_SHAPE));
    for (int i = 0; i < output_num; i++)
        output_shape[i] = CVI_NN_TensorShape(&output_tensors[i]);

    // ── 预处理: BGR → RGB → split → int8 CHW ────────────────────────────────
    cv::Mat model_in;
    if (lb_frame.cols != model_w || lb_frame.rows != model_h) {
        cv::resize(lb_frame, model_in, cv::Size(model_w, model_h));
    } else {
        model_in = lb_frame;
    }

    cv::Mat rgb;
    cv::cvtColor(model_in, rgb, cv::COLOR_BGR2RGB);
    cv::Mat ch[3];
    int channel_size = model_h * model_w;
    for (int i = 0; i < 3; i++) ch[i] = cv::Mat(model_h, model_w, CV_8SC1);
    cv::split(rgb, ch);
    int8_t* iptr = (int8_t*)CVI_NN_TensorPtr(input);
    for (int i = 0; i < 3; i++)
        memcpy(iptr + i * channel_size, ch[i].data, channel_size);

    // ── 推理 ─────────────────────────────────────────────────────────────────
    LOGI("Running inference...");
    CVI_NN_Forward(model, input_tensors, input_num, output_tensors, output_num);

    // ── 后处理 ───────────────────────────────────────────────────────────────
    const int classes_num = 1;
    std::vector<detection> dets;
    int det_num = getDetections(output_tensors, model_h, model_w, classes_num,
                                output_shape[0], 0.5f, dets);
    NMS(dets, &det_num, 0.45f);
    correctYoloBoxes(dets, det_num, cam_h, cam_w, model_h, model_w);

    // ── letterbox 逆变换 → 原始相机坐标 ──────────────────────────────────────
    if (pad_x != 0 || pad_y != 0 || lb_scale != 1.0f) {
        for (size_t i = 0; i < dets.size(); i++) {
            float cx = dets[i].bbox.x, cy = dets[i].bbox.y;
            float w = dets[i].bbox.w, h = dets[i].bbox.h;
            float x1 = cx - w * 0.5f, y1 = cy - h * 0.5f;
            float x2 = cx + w * 0.5f, y2 = cy + h * 0.5f;
            x1 = std::max(0.0f, (x1 - pad_x) / lb_scale);
            y1 = std::max(0.0f, (y1 - pad_y) / lb_scale);
            x2 = std::min((float)cam_w, (x2 - pad_x) / lb_scale);
            y2 = std::min((float)cam_h, (y2 - pad_y) / lb_scale);
            dets[i].bbox.x = (x1 + x2) * 0.5f;
            dets[i].bbox.y = (y1 + y2) * 0.5f;
            dets[i].bbox.w = x2 - x1;
            dets[i].bbox.h = y2 - y1;
        }
    }

    // ── 输出结果 ─────────────────────────────────────────────────────────────
    float img_area = (float)(cam_w * cam_h);
    int half_w = cam_w / 2;

    LOGI("=== Detection Results ===");
    LOGI("Camera: %dx%d  Model: %dx%d  Letterbox: pad=(%d,%d) scale=%.4f",
         cam_w, cam_h, model_w, model_h, pad_x, pad_y, lb_scale);

    if (dets.empty()) {
        LOGI("No detection (conf_thresh=0.5)");
    } else {
        for (size_t i = 0; i < dets.size(); i++) {
            const box& b = dets[i].bbox;
            float area_ratio = (b.w * b.h) / img_area;
            int offset = (int)b.x - half_w;
            LOGI("Det[%zu]: conf=%.3f  cx=%.1f cy=%.1f w=%.1f h=%.1f  "
                 "area_ratio=%.4f  offset=%d",
                 i, dets[i].score, b.x, b.y, b.w, b.h, area_ratio, offset);
        }
    }

    // ── 画框保存标注图 ───────────────────────────────────────────────────────
    cv::Mat drawn = raw_frame.clone();

    for (size_t i = 0; i < dets.size(); i++) {
        const box& b = dets[i].bbox;
        int x1 = std::max(0, (int)(b.x - b.w / 2));
        int y1 = std::max(0, (int)(b.y - b.h / 2));
        int x2 = std::min(cam_w - 1, (int)(b.x + b.w / 2));
        int y2 = std::min(cam_h - 1, (int)(b.y + b.h / 2));

        // 绿色框
        cv::rectangle(drawn, cv::Point(x1, y1), cv::Point(x2, y2),
                      cv::Scalar(0, 255, 0), 2);

        // 中心点
        cv::circle(drawn, cv::Point((int)b.x, (int)b.y), 4,
                   cv::Scalar(0, 0, 255), -1);

        // 标签
        float area_ratio = (b.w * b.h) / img_area;
        char label[128];
        snprintf(label, sizeof(label), "%.2f area:%.3f off:%d",
                 dets[i].score, area_ratio, (int)b.x - half_w);
        cv::putText(drawn, label, cv::Point(x1, y1 - 8),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
    }

    // 画中心线和目标偏移线
    cv::line(drawn, cv::Point(half_w, 0), cv::Point(half_w, cam_h),
             cv::Scalar(255, 255, 0), 1);
    int target_x = half_w + 90;  // STOP_CENTER_OFFSET
    cv::line(drawn, cv::Point(target_x, 0), cv::Point(target_x, cam_h),
             cv::Scalar(0, 255, 255), 1);

    if (dets.empty()) {
        cv::putText(drawn, "No detection", cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
    }

    if (cv::imwrite(output_path, drawn)) {
        LOGI("Saved: %s", output_path);
    } else {
        LOGE("Failed to save: %s", output_path);
    }

    CVI_NN_CleanupModel(model);
    free(output_shape);
    return 0;
}
