#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <filesystem>

namespace fs = std::filesystem;

cv::Mat filterByArea(const cv::Mat& binary, double minArea = 100.0, double maxArea = 10000.0) {
    cv::Mat filtered = cv::Mat::zeros(binary.size(), binary.type());
    
    // 查找轮廓
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(binary, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    // 筛选轮廓
    for (size_t i = 0; i < contours.size(); i++) {
        double area = cv::contourArea(contours[i]);
        
        if (area >= minArea && area <= maxArea) {
            // 绘制符合条件的轮廓
            cv::drawContours(filtered, contours, i, cv::Scalar(255), -1);
        } 
    }
    
    return filtered;
}

cv::Mat filterByRectArea(const cv::Mat& binary,
    double minRectArea = 100.0,
    double maxRectArea = 500.0,
    double rectThr     = 0.85,
    double minAsp      = 0.3,
    double maxAsp      = 3.0)
{
    cv::Mat filtered = cv::Mat::zeros(binary.size(), binary.type());
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (size_t i = 0; i < contours.size(); ++i) {

        cv::RotatedRect rr = cv::minAreaRect(contours[i]);
        double rectArea = rr.size.area();   // ← 关键：用框面积
        if (rectArea < minRectArea || rectArea > maxRectArea) continue;

        cv::drawContours(filtered, contours, i, cv::Scalar(255), -1);
    }
    return filtered;
}

int main(){
    cv::Mat image = cv::imread("../resources/test_image_2.jpg");
    // resize
    int h = image.cols; 
    int w = image.rows;

    cv::Mat resized;
    cv::resize(image, resized, cv::Size(h/2, w/2));

    // gray
    cv::Mat gray;
    cv::cvtColor(resized, gray, cv::COLOR_BGR2GRAY);

    // binary
    cv::Mat binary;
    cv::threshold(gray, binary, 240, 255, cv::THRESH_TOZERO);

    // 面积筛
    double minArea = 200.0;   // 最小面积阈值
    double maxArea = 230.0; // 最大面积阈值
    
    cv::Mat filtered = filterByArea(binary, minArea, maxArea);

    // 形状筛
    cv::Mat finalMask = filterByRectArea(filtered, 100, 1000, 0.85, 0.3, 3.0);

    // 识别装甲板
    std::vector<std::vector<cv::Point>> lastContours;
    cv::findContours(finalMask, lastContours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (!lastContours.empty()) {
        // 先合并所有轮廓点，再算整体外接矩形
        std::vector<cv::Point> allPts;
        for (const auto& c : lastContours) allPts.insert(allPts.end(), c.begin(), c.end());
        cv::Rect overall = cv::boundingRect(allPts);         

        // 可视化
        cv::rectangle(resized, overall, cv::Scalar(0, 0, 255), 3);
        std::cout << "整体框 左上角:(" << overall.x << "," << overall.y
                  << ") 宽高:" << overall.width << "x" << overall.height << std::endl;
    }

    cv::imwrite("../results/1.png",filtered);
    cv::imwrite("../results/detect_result.png", resized);
    return 0;
}