#include <opencv2/opencv.hpp>
#include <ceres/ceres.h>
#include <iostream>
#include <vector>
#include <cmath>


void show_video(cv::VideoCapture cap, double fps=60){ //一共211帧，前143帧有效
    int delay_ms = static_cast<int>(1000.0 / fps);        
    cv::Mat frame;
    double cx, cy;
    // int i =0;
    while (true) {
        auto t0 = cv::getTickCount();
        cap >> frame;              // 读取下一帧
        if (frame.empty()) break;  

        // 灰度
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        // 二值
        cv::Mat bin;
        cv::threshold(gray, bin, 50, 255, cv::THRESH_BINARY);

        // 找中心
        cv::Moments m = cv::moments(bin, true);
        if(m.m00 != 0){     
            // std::cout << i << std::endl; i++;    
            cx = m.m10 / m.m00;
            cy = m.m01 / m.m00;
            cv::circle(frame, cv::Point(static_cast<int>(cx), static_cast<int>(cy)), 1, cv::Scalar(0, 0, 255), -1);
        }
        cv::imshow("center", frame);
        int elapsed_ms = static_cast<int>((cv::getTickCount() - t0) * 1000 / cv::getTickFrequency());
        int sleep = std::max(1, delay_ms - elapsed_ms);
        cv::waitKey(sleep);
    }

    cv::destroyAllWindows();
}

void collectCentroids(cv::VideoCapture& cap,
                      int   thresh,
                      double fps,
                      std::vector<double>& t_arr,
                      std::vector<double>& x_arr,
                      std::vector<double>& y_arr)
{
    t_arr.clear();
    x_arr.clear();
    y_arr.clear();

    cv::Mat frame, gray, bin;
    int frameCount = 0;

    while (true) {
        cap >> frame;
        if (frame.empty()) break;

        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        cv::threshold(gray, bin, thresh, 255, cv::THRESH_BINARY);

        cv::Moments m = cv::moments(bin, true);
        if (m.m00 != 0) {
            double cx = m.m10 / m.m00;
            double cy = m.m01 / m.m00;
            t_arr.push_back(frameCount / fps);
            x_arr.push_back(cx);
            y_arr.push_back(720.0-cy);
        }
        ++frameCount;
    }
}

struct TrajectoryResidual {
    TrajectoryResidual(double t, double x, double y)
        : t_(t), x_(x), y_(y) {}

    template <typename T>
    bool operator()(const T* const vxvy, const T* const gk, T* residual) const {
        T t = T(t_);
        T vx = vxvy[0];
        T vy = vxvy[1];
        T g = gk[0];
        T k = gk[1];

        T one_minus_exp = T(1.0) - ceres::exp(-k * t);
        T x_model = vx * one_minus_exp / k + T(164);    // x0
        T y_model = (vy + (g/k)) * one_minus_exp / k - g/k * t  + T(593.048);

        residual[0] = T(x_) - x_model;
        residual[1] = T(y_) - y_model;
        return true;
    }

private:
    double t_, x_, y_;
};

void ComputePredictedValues(const std::vector<double>& t_arr,
                            const std::vector<double>& x_arr,
                            const std::vector<double>& y_arr,
                            double vx, double vy, double g, double k) {
    std::cout << "Predicted vs Actual:" << std::endl;
    
    for (size_t i = 0; i < t_arr.size(); ++i) {
        double t = t_arr[i];
        double one_minus_exp = 1.0 - ceres::exp(-k * t);
        double x_pred = vx * (1 - exp(-k * t)) / k + x_arr[0];
        double y_pred = (vy + (g/k)) * one_minus_exp / k - g/k * t  + y_arr[0];

        std::cout << "t=" << t << ", x_pred=" << x_pred << ", x_actual=" << x_arr[i]
                  << ", y_pred=" << y_pred << ", y_actual=" << y_arr[i] << std::endl;
    }
}

int main()
{
    const std::string videoPath = "../resources/video.mp4";   // 相对可执行文件的路径
    cv::VideoCapture cap(videoPath);
    if (!cap.isOpened()) {
        std::cerr << "Cannot open video: " << videoPath << std::endl;
        return -1;
    } 
    
    // show_video(cap, 15);
    // 取数据
    const double fps = 60.0;
    std::vector<double> t_arr, x_arr, y_arr;
    collectCentroids(cap, 50, fps, t_arr, x_arr, y_arr);  
    cap.release();

    for (size_t i = 0; i < t_arr.size(); ++i)
        std::cout << "t=" << t_arr[i] << ", x=" << x_arr[i] << ", y=" << y_arr[i] << '\n';

    // 初始参数估计
    double vxvy[2] = {0.0, 0.0};  // [vx, vy]
    double gk[2] = {100.0, 0.01};     // [g, k]

    // 运行优化问题
    ceres::Problem problem;

    for (size_t i = 0; i < t_arr.size(); ++i) {
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<TrajectoryResidual, 2, 2, 2>(
                new TrajectoryResidual(t_arr[i], x_arr[i], y_arr[i])),
            nullptr, vxvy, gk);
    }

    problem.SetParameterLowerBound(gk, 0, 100.0);
    problem.SetParameterLowerBound(gk, 1, 0.01);
    problem.SetParameterUpperBound(gk, 0, 1000.0);
    problem.SetParameterUpperBound(gk, 1, 1.0);

    // 运行优化
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    
    std::cout << "vx: " << vxvy[0] << "\n";
    std::cout << "vy: " << vxvy[1] << "\n";
    std::cout << "g: " << gk[0] << "\n";
    std::cout << "k: " << gk[1] << "\n";
    
    ComputePredictedValues(t_arr, x_arr, y_arr, vxvy[0], vxvy[1], gk[0], gk[1]);
    return 0;
}