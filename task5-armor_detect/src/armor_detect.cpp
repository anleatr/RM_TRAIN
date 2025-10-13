#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <torch/script.h>
#include <torch/torch.h>

class DigitalRecognition {
private:
    torch::jit::script::Module module;
    torch::Device device;
    const int IMAGE_COLS = 28;
    const int IMAGE_ROWS = 28;
public:
    /**
     * 默认使用CPU，可通过标志位开启使用GPU
     * @param use_cuda 是否使用GPU
     * @param model_path 模型文件路径
     */
    explicit DigitalRecognition(bool use_cuda = false,
                                const std::string &model_path = "/home/wtz/ros2_ws/src/armor/checkpoint/lenet.pt") : device(torch::kCPU) {
        if ((use_cuda) && (torch::cuda::is_available())) {
            std::cout << "CUDA is available! Training on GPU." << std::endl;
            device = torch::kCUDA;
        }
        module = torch::jit::load(model_path, device);
    }

    /**
     * 单张图片分类器
     * @param img 图片，cv::Mat类型
     * @return 分类结果
     */
    int matToDigital(cv::Mat &img) {// 输入为二值化图像
        // 正则化
        img.convertTo(img, CV_32FC1, 1.0f / 255.0f);

        // 模型用的是 28*28 的单通道灰度图
        cv::resize(img, img, cv::Size(IMAGE_COLS, IMAGE_ROWS));

        // 将 OpenCV 的 Mat 转换为 Tensor, 注意两者的数据格式
        // OpenCV: H*W*C 高度, 宽度, 通道数
        auto input_tensor = torch::from_blob(img.data, {1, IMAGE_COLS, IMAGE_ROWS, 1});

        // Tensor: N*C*H*W 数量, 通道数, 高度, 宽度
        // 数字表示顺序
        input_tensor = input_tensor.permute({0, 3, 1, 2}).to(device);

        // 添加数据
        std::vector<torch::jit::IValue> inputs;
        inputs.emplace_back(input_tensor);

        // 模型计算
        at::Tensor output = module.forward(inputs).toTensor();
        std::cout << output.slice(/*dim=*/1, /*start=*/0, /*end=*/7) << '\n';

        // 输出分类的结果
        int ans = output.argmax(1).item().toInt();
        std::cout << "当前机器人编号: " << ans+1 << std::endl;

        return ans;
    }
};

class ArmorDetectorNode : public rclcpp::Node {

public:
  ArmorDetectorNode() : Node("armor_detector"){
    // 订阅图像
    img_sub_ = create_subscription<sensor_msgs::msg::Image>("/image_raw", 10, std::bind(&ArmorDetectorNode::imageCallback, this, std::placeholders::_1));

    // 发布装甲板中心坐标
    armor_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("/armor_coordinate", 10);

    cv::namedWindow("armor_debug", cv::WINDOW_AUTOSIZE);
  }

  ~ArmorDetectorNode() { cv::destroyAllWindows(); }

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
      cv::Mat frame = cv_ptr->image;
      if (frame.empty()) return;

      /*-------------------------装甲板检测---------------------------*/
      std::vector<cv::Point2f> armor_corners;
      if (!detectArmorPlate(frame, armor_corners)) std::cout << "dont't detect armor" << std::endl; return; // 没检测到直接返回
      printArmorCorners(armor_corners);
      /* ------------------------pnp解算---------------------------------- */
      cv::Mat P_cam = CoordinateConvert(armor_corners); // pnp结算


      /*------------------------提取ROI---------------------------------------*/
      const int out_w = 200, out_h = 100;
      std::vector<cv::Point2f> dst_pts = {
          cv::Point2f(0, 0), cv::Point2f(out_w, 0), cv::Point2f(out_w, out_h),
          cv::Point2f(0, out_h)
      };
      cv::Mat M = cv::getPerspectiveTransform(armor_corners, dst_pts);
      cv::Mat armor_crop;
      cv::warpPerspective(frame, armor_crop, M, cv::Size(out_w, out_h)); // 裁减出装甲板区域

      /*------------------------数字识别---------------------------------------*/
      cv::Mat gray, binary;
      cv::cvtColor(armor_crop, gray, cv::COLOR_BGR2GRAY);
      cv::threshold(gray, binary, 50, 255, cv::THRESH_BINARY);

      cv::Mat temp = binary.clone();
      
      digitalRecognition.matToDigital(temp);

      // 可视化裁减后的图片
    //   cv::imshow("armor_debug", binary);
    //   cv::waitKey(1);
  }


  void printArmorCorners(const std::vector<cv::Point2f>& corners) {
    std::cout << "armor_corners: [";
    for (size_t i = 0; i < corners.size(); ++i) {
        std::cout << "(" << corners[i].x << ", " << corners[i].y << ")";
        if (i + 1 < corners.size()) std::cout << ", ";
    }
    std::cout << "]" << std::endl;
  }

  bool detectArmorPlate(const cv::Mat& frame,
                        std::vector<cv::Point2f>& armor_corners)
  {
      armor_corners.clear();
      if (frame.empty()) return false;

      /* ---------- 1. 预处理 ---------- */
      cv::Mat gray, binary;
      cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
      cv::threshold(gray, binary, 200, 255, cv::THRESH_BINARY);

      /* ---------- 2. 轮廓过滤 ---------- */
      std::vector<std::vector<cv::Point>> contours;
      cv::findContours(binary, contours, cv::RETR_EXTERNAL,
                      cv::CHAIN_APPROX_SIMPLE);

      const double min_area = 100.0;
      const double min_width = 10.0;
      std::vector<cv::RotatedRect> light_bars;

      for (const auto& cnt : contours)
      {
          double area = cv::contourArea(cnt);
          if (area < min_area) continue;
          
          cv::RotatedRect r = cv::minAreaRect(cnt);
          float short_side = std::min(r.size.width, r.size.height);
          if (short_side < min_width) continue;   // 太细，直接扔掉

          cv::drawContours(frame, std::vector<std::vector<cv::Point>>{cnt},
                     -1, cv::Scalar(0, 0, 255), 5);
          light_bars.push_back(r);
      }
      if (light_bars.size() < 2) return false;
    //   std::cout << light_bars.size() << std::endl;
      // 展示

      
      /* ---------- 3. 灯条匹配 ---------- */
      // 现在我们有vector<cv::RoraredRect> light_bars, 需要找到装甲板两侧灯条
      double max_pair_len = 0;
      cv::RotatedRect best1, best2;
      bool found = false;

      for (size_t i = 0; i < light_bars.size(); ++i)
      {
          for (size_t j = i + 1; j < light_bars.size(); ++j)
          {
              const auto& r1 = light_bars[i];
              const auto& r2 = light_bars[j];

              float angle_diff = std::abs(r1.angle - r2.angle);
              if (angle_diff > 90) angle_diff = 180 - angle_diff;
              if (angle_diff > 8.0f) continue;

              double len1 = std::max(r1.size.width, r1.size.height);
              double len2 = std::max(r2.size.width, r2.size.height);
              double pair_len = len1 + len2;

              if (pair_len > max_pair_len)
              {
                  max_pair_len = pair_len;
                  best1 = r1;
                  best2 = r2;
                  found = true;
              }
          }
      }
      if (!found) return false;

      /* ---------- 4. 生成 4 个顶点 ---------- */
      std::vector<cv::Point2f> pts;
      for (const auto& r : {best1, best2})
      {
          cv::Point2f v[4];
          r.points(v);
          pts.insert(pts.end(), v, v + 4);
      }
      cv::RotatedRect armor_box = cv::minAreaRect(pts);

      /* 仅把高度延长 1 倍（上下各一倍） */

      // TODO: 这种粗暴的形式只能用于车站立的时候，如果是侧视，宽和高就反了， 因此应该用灯条的外接矩形的朝向作为宽高判断方式
      armor_box.size.width *= 2.4f;   // 如果 OpenCV 把 height 视为长边，可改成 width
      armor_box.size.height *= 0.7f;
      cv::Point2f pts_[4];          // 1. 临时 C 数组
      armor_box.points(pts_);       // 2. OpenCV 填充
      armor_corners.assign(pts_, pts_ + 4);

      // 画装甲板
      std::vector<cv::Point> armor_int_corners;
      cv::Mat(armor_corners).convertTo(armor_int_corners, CV_32S);  // 转整数

    // 3. 画边缘线框（更醒目）
      cv::polylines(frame,
                std::vector<std::vector<cv::Point>>{armor_int_corners},
                true,                    // 闭合
                cv::Scalar(0, 0, 255),   // 红色边框
                3, cv::LINE_AA);

      cv::imshow("armor_debug", frame);
      cv::waitKey(1);
      return true;
  }

  cv::Mat CoordinateConvert(std::vector<cv::Point2f>& imagePoints){

      // 相机内参矩阵 (fx, fy, cx, cy)
      cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 
                              718.0, 0, 2673.0, 
                              0, 560.0, 2673.0, 
                              0, 0, 1);

      // 相机畸变系数 (可以忽略畸变设置为0，若不考虑畸变)
      cv::Mat distCoeffs = (cv::Mat_<double>(1, 5) << 
                          0, 0, 0, 0, 0);

      //物体的真实尺寸 
      //小装甲板
      // std::vector<cv::Point3f> objectPoints;
      // objectPoints.push_back(cv::Point3f(67.50F, -28.50F, 0));            // 右上角
      // objectPoints.push_back(cv::Point3f(67.50F, 28.50F, 0));             // 右下角
      // objectPoints.push_back(cv::Point3f(-67.50F, 28.50F, 0));            // 左下角
      // objectPoints.push_back(cv::Point3f(-67.50F, -28.50F, 0));           // 左上角
      //大装甲板
      std::vector<cv::Point3f> objectPoints;
      objectPoints.push_back(cv::Point3f(-115.00F, -28.50F, 0));           // 左上角
      objectPoints.push_back(cv::Point3f(115.00F, -28.50F, 0));            // 右上角
      objectPoints.push_back(cv::Point3f(115.00F, 28.50F, 0));             // 右下角
      objectPoints.push_back(cv::Point3f(-115.00F, 28.50F, 0));            // 左下角
      
      // 定义旋转向量和位移向量
      cv::Mat rvec, tvec;
      cv::Mat P_center_in_cam;
      // 使用PnP算法估算物体的旋转和位移向量
      bool success = cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);
      if (success) {
        cv::Mat R;
        cv::Rodrigues(rvec, R);  // 旋转向量 -> 旋转矩阵
        // 装甲板中心在世界坐标系下的坐标
        cv::Mat P_world = (cv::Mat_<double>(3, 1) << 0, 0, 0);
        // 装甲板中心在相机坐标系下的坐标
        cv::Mat P_cam = R * P_world + tvec;  // 这就是 tvec，但方向是“世界->相机”
        P_center_in_cam = -R.t() * tvec;  // 这才是“装甲板中心在相机坐标系下的坐
        std::cout << "装甲板中心在相机坐标系下的坐标 (X, Y, Z): "
        << P_center_in_cam.at<double>(0) << ", "
        << P_center_in_cam.at<double>(1) << ", "
        << P_center_in_cam.at<double>(2) << " mm" << std::endl;

      } else {
          std::cout << "PnP calculation failed!" << std::endl;
      }
      
      return P_center_in_cam;

  } 

  DigitalRecognition digitalRecognition;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr armor_pub_;
};

  
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmorDetectorNode>());
  rclcpp::shutdown();
  return 0;
}