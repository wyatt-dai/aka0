// Source: ported from aka-rk3588/test_cmds.cpp (HSV red bucket detection)
// Description: 红色桶检测，用 OpenCV HSV 双区间掩膜 + 连通域，取最大区域
#include "bucket_detect.hpp"
#include <vector>

bool detect_bucket_bgr(const cv::Mat& bgr, BucketResult& out, int min_area) {
    out.found = false;
    if (bgr.empty())
        return false;

    cv::Mat hsv;
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

    // 红色跨越 H 环两端，用两个区间合并
    cv::Mat mask1, mask2, mask;
    cv::inRange(hsv, cv::Scalar(0, 80, 50), cv::Scalar(10, 255, 255), mask1);
    cv::inRange(hsv, cv::Scalar(170, 80, 50), cv::Scalar(180, 255, 255), mask2);
    cv::bitwise_or(mask1, mask2, mask);

    // 形态学开运算去噪点
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (contours.empty())
        return false;

    // 取面积最大的连通域
    int best = -1;
    double best_area = 0;
    for (int i = 0; i < (int)contours.size(); i++) {
        double a = cv::contourArea(contours[i]);
        if (a > best_area) {
            best_area = a;
            best = i;
        }
    }
    if (best < 0 || best_area < min_area)
        return false;

    cv::Rect r = cv::boundingRect(contours[best]);
    float frame_area = (float)(bgr.cols * bgr.rows);

    out.found = true;
    out.area_ratio = (float)best_area / frame_area;
    out.x = r.x;
    out.y = r.y;
    out.w = r.width;
    out.h = r.height;
    out.cx = r.x + r.width / 2;
    out.cy = r.y + r.height / 2;
    return true;
}
