// exp_fit.cc
#include <ceres/ceres.h>
#include <glog/logging.h>

#include <iostream>
#include <vector>
#include <random>
#include <cmath>

// ---------------- 1. 残差 functor ----------------
struct ExponentialResidual {
    ExponentialResidual(double x, double y) : x_(x), y_(y) {}
    
    template <typename T>
    bool operator()(const T* const ab, T* residual) const {
        // ab[0] = a, ab[1] = b
        residual[0] = T(y_) - ceres::exp(ab[0] * T(x_) + ab[1]);
        return true;
    }

private:
    double x_, y_;
};

// ---------------- 2. 主函数 ----------------
int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);

    // 生成带噪数据 y = exp(ax+b) + noise
    const double a_true = 0.3, b_true = 0.1;
    std::vector<double> x_data, y_data;
    std::default_random_engine gen;
    std::normal_distribution<double> noise(0.0, 0.2);

    for (int i = 0; i < 100; ++i) {
        double x = i / 100.0;
        x_data.push_back(x);
        y_data.push_back(std::exp(a_true * x + b_true) + noise(gen));
    }

    // 待估参数初值
    double ab[2] = {0.0, 0.0};

    // 构建最小二乘问题
    ceres::Problem problem;
    for (size_t i = 0; i < x_data.size(); ++i) {
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<ExponentialResidual, 1, 2>(
                new ExponentialResidual(x_data[i], y_data[i])),
            nullptr, ab);
    }

    // 求解
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    // 输出
    std::cout << summary.BriefReport() << '\n';
    std::cout << "Estimated a, b = " << ab[0] << ", " << ab[1] << '\n';
    return 0;
}