#include <pcl/common/time.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>

#define PCL_NO_PRECOMPILE

#include "patchwork/patchwork.hpp"  // Your implementation
#include "tools/kitti_loader.hpp"
#include "tools/pcd_loader.hpp"

using std::placeholders::_1;
using PointType = pcl::PointXYZL;

using RosPointCloud2 = sensor_msgs::msg::PointCloud2;

class InterfaceNode : public rclcpp::Node {
 public:
  std::shared_ptr<PatchWork<PointType>> patchwork_;

  InterfaceNode() : Node("patchwork_benchmark_node") {
    // Declare and get parameters
    seq_ = this->declare_parameter<std::string>("seq", "00");
    acc_filename_ = this->declare_parameter<std::string>("acc_filename", "");
    pcd_savepath_ = this->declare_parameter<std::string>("pcd_savepath", "");

    // Publishers
    cloud_pub_ = create_publisher<RosPointCloud2>("lidar_out", 10);
    ground_pub_ = create_publisher<RosPointCloud2>("ground", 10);
    nonground_pub_ = create_publisher<RosPointCloud2>("non_ground", 10);
    labeled_pub_ = create_publisher<RosPointCloud2>("/labeled_cloud", 10);

    // Subscriber
    cloud_sub_ = create_subscription<RosPointCloud2>(
        "/patchwork/lidar", 10, std::bind(&InterfaceNode::pointcloudCallback, this, _1));

    RCLCPP_INFO(this->get_logger(), "Patchwork Benchmark Node Initialized.");
  }

 private:
  void pointcloudCallback(const RosPointCloud2::SharedPtr msg) {
    pcl::PointCloud<PointType> pc_curr;
    pcl::fromROSMsg(*msg, pc_curr);

    pcl::PointCloud<PointType> pc_ground, pc_non_ground, pc_labeled;
    double time_taken;

    patchwork_->estimate_ground(pc_curr, pc_ground, pc_non_ground, time_taken);
    double hz = 1.0 / time_taken;

    RCLCPP_INFO(this->get_logger(),
                "[%s] Time: %.2f ms | Hz: %.2f | Points: %zu -> %zu",
                msg->header.frame_id.c_str(),
                time_taken * 1000.0,
                hz,
                pc_curr.size(),
                pc_ground.size());

    // Publish input
    RosPointCloud2 out_msg;
    pcl::toROSMsg(pc_curr, out_msg);
    out_msg.header = msg->header;
    cloud_pub_->publish(out_msg);

    ground_pub_->publish(to_msg(pc_ground, msg->header));
    nonground_pub_->publish(to_msg(pc_non_ground, msg->header));

    for (auto &pt : pc_ground.points) pt.label = 1;
    for (auto &pt : pc_non_ground.points) pt.label = 0;
    pc_labeled = pc_ground;
    pc_labeled.insert(pc_labeled.end(), pc_non_ground.begin(), pc_non_ground.end());
    labeled_pub_->publish(to_msg(pc_labeled, msg->header));
  }

  RosPointCloud2 to_msg(const pcl::PointCloud<PointType> &cloud,
                        const std_msgs::msg::Header &header) {
    RosPointCloud2 msg;
    pcl::toROSMsg(cloud, msg);
    msg.header = header;
    return msg;
  }

  std::string seq_;
  std::string acc_filename_;
  std::string pcd_savepath_;
  std::string data_path_;

  bool evaluate_semantickitti_;
  bool save_flag_;

  // ROS 2 publishers and subscriber
  rclcpp::Publisher<RosPointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<RosPointCloud2>::SharedPtr ground_pub_;
  rclcpp::Publisher<RosPointCloud2>::SharedPtr nonground_pub_;
  rclcpp::Publisher<RosPointCloud2>::SharedPtr labeled_pub_;

  rclcpp::Subscription<RosPointCloud2>::SharedPtr cloud_sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<InterfaceNode>();

  bool evaluate_semantickitti = node->declare_parameter<bool>("evaluate_semantickitti");
  bool save_flag = node->declare_parameter<bool>("save_flag", false);
  bool use_sor_before_save = node->declare_parameter<bool>("use_sor_before_save", false);
  std::string dataset_path =
      node->declare_parameter<std::string>("dataset_path", "/path/to/SemKITTI");
  std::string seq = fs::path(dataset_path).filename().string();

  if (evaluate_semantickitti) {
    rclcpp::Publisher<RosPointCloud2>::SharedPtr tp_pub;
    rclcpp::Publisher<RosPointCloud2>::SharedPtr fp_pub;
    rclcpp::Publisher<RosPointCloud2>::SharedPtr fn_pub;
    tp_pub = node->create_publisher<RosPointCloud2>("benchmark/TP", 1);
    fp_pub = node->create_publisher<RosPointCloud2>("benchmark/FP", 1);
    fn_pub = node->create_publisher<RosPointCloud2>("benchmark/FN", 1);

    using PointEvalType = PointXYZILID;
    auto patchwork_eval = std::make_shared<PatchWork<PointEvalType>>(node.get());

    // Setting data loader and paths to be saved
    KittiLoader loader(dataset_path);
    const char *home_dir = std::getenv("HOME");
    if (home_dir == nullptr) {
      std::cerr << "Error: HOME environment variable not set." << std::endl;
      return 1;
    }
    std::string abs_save_dir = std::string(home_dir) + "/patchwork";
    fs::create_directory(abs_save_dir);
    std::string output_filename = abs_save_dir + "/" + seq + ".txt";

    const int N = loader.size();
    for (int n = 0; rclcpp::ok() && n < N; ++n) {
      pcl::PointCloud<PointEvalType> cloud;
      loader.get_cloud(n, cloud);

      pcl::PointCloud<PointEvalType> ground, non_ground;
      double time_taken;
      patchwork_eval->estimate_ground(cloud, ground, non_ground, time_taken);

      double precision, recall, precision_naive, recall_naive;
      calculate_precision_recall(cloud, ground, precision, recall);
      calculate_precision_recall(cloud, ground, precision_naive, recall_naive, false);

      RCLCPP_INFO(node->get_logger(), "[%d] Precision: %.2f | Recall: %.2f", n, precision, recall);

      ofstream ground_output(output_filename, ios::app);
      ground_output << n << "," << time_taken << "," << precision << "," << recall << ","
                    << precision_naive << "," << recall_naive;
      ground_output << std::endl;
      ground_output.close();

      pcl::PointCloud<PointEvalType> TP;
      pcl::PointCloud<PointEvalType> FP;
      pcl::PointCloud<PointEvalType> FN;
      pcl::PointCloud<PointEvalType> TN;
      discern_ground(ground, TP, FP);
      discern_ground(non_ground, FN, TN);

      // if (save_flag) {  // To make `.label` file
      //   if (use_sor_before_save) {
      //     pcl::PointCloud<PointType>::Ptr filtered;
      //     filtered.reset(new pcl::PointCloud<PointType>());
      //     filter_by_sor(ground, *filtered);
      //     save_ground_label(abs_save_dir, n, cloud, *filtered);
      //   } else {
      //     save_ground_label(abs_save_dir, n, cloud, ground);
      //   }
      // }

      const auto curr_time = rclcpp::Clock().now();
      sensor_msgs::msg::PointCloud2 ros_msg;
      pcl::toROSMsg(TP, ros_msg);
      ros_msg.header.frame_id = "velodyne_link";
      ros_msg.header.stamp = curr_time;
      tp_pub->publish(ros_msg);

      pcl::toROSMsg(FP, ros_msg);
      ros_msg.header.frame_id = "velodyne_link";
      ros_msg.header.stamp = curr_time;
      fp_pub->publish(ros_msg);

      pcl::toROSMsg(FN, ros_msg);
      ros_msg.header.frame_id = "velodyne_link";
      ros_msg.header.stamp = curr_time;
      fn_pub->publish(ros_msg);
    }

    RCLCPP_INFO(node->get_logger(), "Finished SemanticKITTI Evaluation.");
    rclcpp::shutdown();
    return 0;
  } else {
    node->patchwork_ = std::make_shared<PatchWork<PointType>>(node.get());
  }

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
