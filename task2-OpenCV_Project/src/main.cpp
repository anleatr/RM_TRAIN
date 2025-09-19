#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <filesystem>

namespace fs = std::filesystem;

// ========= 工具：确保输出目录存在 =========
void ensureResultsDir() {
    if (!fs::exists("../results")) {
        fs::create_directory("../results");
        std::cout << "Created ../results directory.\n";
    }
}

// ========= 1. 颜色空间转换 =========
cv::Mat convertToGray(const cv::Mat& src) {
    cv::Mat gray;
    cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
    return gray;
}

cv::Mat convertToHSV(const cv::Mat& src) {
    cv::Mat hsv;
    cv::cvtColor(src, hsv, cv::COLOR_BGR2HSV);
    return hsv;
}

// ========= 2. 滤波 =========
cv::Mat applyBlur(const cv::Mat& src, int ksize = 3) {
    cv::Mat dst;
    cv::blur(src, dst, cv::Size(ksize, ksize));
    return dst;
}

cv::Mat applyGaussian(const cv::Mat& src, int ksize = 3, double sigma = 0) {
    cv::Mat dst;
    cv::GaussianBlur(src, dst, cv::Size(ksize, ksize), sigma);
    return dst;
}

// ========= 3. 红色区域提取 =========
cv::Mat redInHSV(const cv::Mat& bgr)       
{
    // 首先转化为hsv
    cv::Mat hsv;
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

    // 调参
    cv::Scalar lower_red(147, 83, 141);
    cv::Scalar upper_red(180, 255, 255);

    cv::Mat mask;
    cv::inRange(hsv, lower_red, upper_red, mask);

    cv::Mat res;
    cv::bitwise_and(bgr, bgr, res, mask);
    // cv::imshow("mask", res);
    // while (true) {
    //     int key = cv::waitKey(30);
    //     if (key == 27) break;   // ESC
    // }
    return res;   
}

// ========= 4. 红色外轮廓、Bounding Box =========
std::pair<cv::Mat, cv::Mat> visualizeRedContours(const cv::Mat& src,
    const cv::Mat& redMask,
    int lineWidth = 1)
{
    cv::Mat gray4red;
    cv::cvtColor(redMask, gray4red, cv::COLOR_BGR2GRAY);
    gray4red.convertTo(gray4red, CV_8U);
    cv::Mat binary;
    cv::threshold(gray4red, binary, 0, 255, cv::THRESH_BINARY);

    // 1. 找轮廓
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;          
    cv::findContours(binary.clone(), contours, hierarchy,
                     cv::RETR_TREE,          // 使用EXTRAL只有外部轮廓  
                     cv::CHAIN_APPROX_SIMPLE);

    // 2. 画轮廓 & 包围框
    cv::Mat canvas = src.clone();
    
    cv::drawContours(canvas, contours, -1,
        cv::Scalar(0, 255, 0), lineWidth);
    
    double areas = 0.0;
    cv::Mat bboxed = redMask.clone();
    // 做一下过滤
    for (const auto& cnt : contours){
        double are = cv::contourArea(cnt);
        areas += are;


        cv::Rect box = cv::boundingRect(cnt);
        cv::rectangle(bboxed, box, cv::Scalar(0, 0, 255), lineWidth);

        
    }

    std::cout << "The total area of the red zone is" << " " << areas << std::endl;

    return {canvas, bboxed};
}

// ========= 5. 高亮区域提取与形态学处理 =========
cv::Mat highlightProcessing(const cv::Mat& src)
{
    // 1. 得到高亮掩码
    cv::Mat hsv, v;
    cv::cvtColor(src, hsv, cv::COLOR_BGR2HSV);
    std::vector<cv::Mat> ch;
    cv::split(hsv, ch);
    v = ch[2];
    cv::Mat mask = v > 200;          // 单通道 0/255

    cv::Mat mask3;
    cv::cvtColor(mask, mask3, cv::COLOR_GRAY2BGR);

    cv::Mat result;
    cv::bitwise_and(src, mask3, result);

    return result;
}

void highlightPipeline(const cv::Mat& highlightColor, int& step)
{
    // 确保输出目录存在
    fs::create_directories("../results");

    // 1. 灰度化
    cv::Mat gray;
    cv::cvtColor(highlightColor, gray, cv::COLOR_BGR2GRAY);
    cv::imwrite("../results/" + std::to_string(++step) + "_highlightGray.png", gray);

    // 2. 二值化
    cv::Mat bin;
    cv::threshold(gray, bin, 1, 255, cv::THRESH_BINARY);  // 高光区非 0 像素→255
    cv::imwrite("../results/" + std::to_string(++step) + "_highlightBin.png", bin);

    // 3. 膨胀
    cv::Mat dilated;
    cv::Mat kernelD = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3));
    cv::dilate(bin.clone(), dilated, kernelD, cv::Point(-1,-1), 1);
    cv::imwrite("../results/" + std::to_string(++step) + "_highlightDilated.png", dilated);

    // 4. 腐蚀
    cv::Mat eroded;
    cv::Mat kernelE = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3));
    cv::erode(bin.clone(), eroded, kernelE, cv::Point(-1,-1), 1);
    cv::imwrite("../results/" + std::to_string(++step) + "_highlightEroded.png", eroded);

    // 5. 漫水
    cv::Mat flood = highlightColor.clone();
    cv::Mat mask = cv::Mat::zeros(flood.rows + 2, flood.cols + 2, CV_8U);
    cv::floodFill(flood, mask, cv::Point(0,0), cv::Scalar(0,255,0), 0, cv::Scalar(), cv::Scalar(), 4 | (255 << 8));
    cv::imwrite("../results/" + std::to_string(++step) + "_highlightFlood.png", flood);
}


// ========= 6. 绘制图形与轮廓 =========
void drawShapes(cv::Mat& canvas) {
    // 圆形、矩形、文字
    cv::circle(canvas, cv::Point(100, 100), 50, cv::Scalar(0, 255, 0), 3);
    cv::rectangle(canvas, cv::Point(200, 200), cv::Point(300, 300), cv::Scalar(255, 0, 0), 3);
    cv::putText(canvas, "Hello World", cv::Point(50, 400),
                cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
}

void drawContoursAndBoxes(cv::Mat& canvas,
                          const std::vector<std::vector<cv::Point>>& contours,
                          const std::vector<cv::Rect>& boxes) {
    cv::drawContours(canvas, contours, -1, cv::Scalar(0, 0, 255), 2);
    for (const auto& r : boxes) {
        cv::rectangle(canvas, r, cv::Scalar(0, 255, 0), 2);
    }
}

// ========= 7. 几何变换 =========
cv::Mat rotateImage(const cv::Mat& src, double angle) {
    cv::Point2f center(src.cols / 2.0, src.rows / 2.0);
    cv::Mat rot = cv::getRotationMatrix2D(center, angle, 1.0);
    cv::Mat dst;
    cv::warpAffine(src, dst, rot, src.size(), cv::INTER_LINEAR);
    return dst;
}

cv::Mat cropTopLeftQuarter(const cv::Mat& src) {
    cv::Rect roi(0, 0, src.cols / 2, src.rows / 2);
    return src(roi).clone();
}



int main() {
    ensureResultsDir();

    // 读取图像
    cv::Mat image = cv::imread("../resources/test_image.png");
    if (image.empty()) {
        std::cout << "无法加载图像！" << std::endl;
        return -1;
    }

    // 缩放
    double scale = 0.3;
    cv::Size newSize(static_cast<int>(image.cols * scale),
                     static_cast<int>(image.rows * scale));
    cv::resize(image, image, newSize, 0, 0, cv::INTER_AREA);

    int step = 0; // 用于命名

    // 1. 颜色空间转换
    cv::Mat gray = convertToGray(image);
    cv::imwrite("../results/" + std::to_string(++step) + "_gray.png", gray);

    cv::Mat hsv = convertToHSV(image);
    cv::imwrite("../results/" + std::to_string(++step) + "_hsv.png", hsv);

    // 2. 滤波
    cv::Mat blurImg = applyBlur(image);
    cv::imwrite("../results/" + std::to_string(++step) + "_mean_blur.png", blurImg);

    cv::Mat gaussImg = applyGaussian(image);
    cv::imwrite("../results/" + std::to_string(++step) + "_gaussian_blur.png", gaussImg);

    // // 3. 红色提取
    cv::Mat redMask = redInHSV(image);
    cv::imwrite("../results/" + std::to_string(++step) + "_redMask.png", redMask);

    // // 4. 轮廓, bbox, 面积
    auto [_canvas, bboxed] = visualizeRedContours(image, redMask);

    cv::imwrite("../results/" + std::to_string(++step) + "_redOutline.png", _canvas);
    cv::imwrite("../results/" + std::to_string(++step) + "_redBbox.png", bboxed);
    
    // // 5. 高亮区域处理
    cv::Mat highlightZone = highlightProcessing(image);
    cv::imwrite("../results/" + std::to_string(++step) + "_highlightZone.png", highlightZone);

    highlightPipeline(highlightZone, step);

    // // 6. 绘制图形
    cv::Mat shapesCanvas = image.clone();
    drawShapes(shapesCanvas);
    cv::imwrite("../results/" + std::to_string(++step) + "_shapes.png", shapesCanvas);

    // // 7. 旋转
    cv::Mat rotated = rotateImage(image, 35);
    cv::imwrite("../results/" + std::to_string(++step) + "_rotated35.png", rotated);

    // // 8. 裁剪
    cv::Mat cropped = cropTopLeftQuarter(image);
    cv::imwrite("../results/" + std::to_string(++step) + "_cropTopLeftQuarter.png", cropped);

    // // createTunerWindow(image)
    std::cout << "All results saved to ../results/\n";
    return 0;
}
