#include "ceres/ceres.h"
#include "glog/logging.h"

// 定义残差 functor
struct CostFunctor {
    template <typename T>
    bool operator()(const T* const x, T* residual) const {
        residual[0] = T(10.0) - x[0];
        return true;
    }
};

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    double x = 0.0; // 初始值
    
    ceres::Problem problem;
    // 使⽤ AutoDiff 包装残差函数
    problem.AddResidualBlock(new ceres::AutoDiffCostFunction<CostFunctor, 1, 1>(new CostFunctor),nullptr, &x);

    // 配置求解器
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << "\n";
    std::cout << "Final x = " << x << "\n";
    return 0;
}