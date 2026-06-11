#ifndef BUCKET_DETECT_HPP
#define BUCKET_DETECT_HPP

#include <opencv2/opencv.hpp>

// 红色桶检测结果
// Source: ported from aka-rk3588/test_cmds.{hpp,cpp} (detect_bucket_frame)
struct BucketResult {
    bool found;
    float area_ratio; // 红色连通域面积 / 整帧面积
    int cx, cy;       // bbox 中心 (帧坐标)
    int x, y, w, h;   // bounding box
};

// 在 BGR 图上做红色 HSV 检测，找最大红色连通域。
// 找到返回 true 并填充 out；min_area 为最小像素面积阈值。
//
// HSV 范围（OpenCV H∈[0,180]）：
//   区间1: H∈[0,10]   S∈[80,255] V∈[50,255]
//   区间2: H∈[170,180] S∈[80,255] V∈[50,255]
bool detect_bucket_bgr(const cv::Mat& bgr, BucketResult& out, int min_area = 1000);

#endif // BUCKET_DETECT_HPP
