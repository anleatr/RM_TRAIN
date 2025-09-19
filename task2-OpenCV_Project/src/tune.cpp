#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <filesystem>

struct HSV_Thresh {
    int H_min = 147;
    int H_max = 180;
    int S_min = 83;
    int S_max = 255;
    int V_min = 141;
    int V_max = 255;
};

// 全局对象
HSV_Thresh g_thresh;
cv::Mat    g_src;        // 原始彩色图
cv::Mat    g_hsv;        // HSV 图

// 回调函数，每次拖动 Trackbar 都会调用
static void onTrackbar(int, void*)
{
    cv::Mat mask;
    cv::inRange(g_hsv,
                cv::Scalar(g_thresh.H_min, g_thresh.S_min, g_thresh.V_min),
                cv::Scalar(g_thresh.H_max, g_thresh.S_max, g_thresh.V_max),
                mask);

    cv::Mat res;
    cv::bitwise_and(g_src, g_src, res, mask);

    cv::imshow("Red HSV Tuner", res);
    cv::imwrite("mask_red.png", res);
}

// 创建并运行调节窗口
void createTunerWindow(const cv::Mat& bgr_img)
{
    g_src = bgr_img.clone();
    cv::cvtColor(g_src, g_hsv, cv::COLOR_BGR2HSV);

    const std::string winName = "Red HSV Tuner";
    cv::namedWindow(winName, cv::WINDOW_AUTOSIZE);

    // 创建 Trackbar
    cv::createTrackbar("Hmin", winName, &g_thresh.H_min, 180, onTrackbar);
    cv::createTrackbar("Hmax", winName, &g_thresh.H_max, 180, onTrackbar);
    cv::createTrackbar("Smin", winName, &g_thresh.S_min, 255, onTrackbar);
    cv::createTrackbar("Smax", winName, &g_thresh.S_max, 255, onTrackbar);
    cv::createTrackbar("Vmin", winName, &g_thresh.V_min, 255, onTrackbar);
    cv::createTrackbar("Vmax", winName, &g_thresh.V_max, 255, onTrackbar);

    // 初始化显示
    onTrackbar(0, nullptr);

    std::cout << "按 ESC 退出调节窗口\n";
    while (true) {
        int key = cv::waitKey(30);
        if (key == 27) break;   // ESC
    }
    cv::destroyWindow(winName);
}