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

class InterfaceNode : public rclcpp::Node {
 public:
  InterfaceNode() : Node("patchwork_benchmark_node") {
    // Declare and get parameters
    declare_parameter<std::string>("algorithm", "patchwork");
    declare_parameter<std::string>("seq", "00");
    declare_parameter<bool>("is_kitti", true);
    declare_parameter<bool>("save_flag", false);
    declare_parameter<std::string>("acc_filename", "");
    declare_parameter<std::string>("pcd_savepath", "");

    get_parameter("algorithm", algorithm_);
    get_parameter("seq", seq_);
    get_parameter("is_kitti", is_kitti_);
    get_parameter("save_flag", save_flag_);
    get_parameter("acc_filename", acc_filename_);
    get_parameter("pcd_savepath", pcd_savepath_);

    patchwork_ = std::make_shared<PatchWork<PointType>>(this);

    // Publishers
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/benchmark/cloud", 10);
    ground_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/patchwork/ground", 10);
    nonground_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/patchwork/non_ground", 10);
    labeled_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/benchmark/labeled_cloud", 10);
    tp_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/benchmark/TP", 10);
    fp_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/benchmark/FP", 10);
    fn_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/benchmark/FN", 10);
    precision_pub_ = create_publisher<visualization_msgs::msg::Marker>("/precision", 1);
    recall_pub_ = create_publisher<visualization_msgs::msg::Marker>("/recall", 1);

    // Subscriber
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/patchwork/cloud", 10, std::bind(&InterfaceNode::pointcloudCallback, this, _1));

    RCLCPP_INFO(this->get_logger(), "Patchwork Benchmark Node Initialized.");
  }

 private:
  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
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
    sensor_msgs::msg::PointCloud2 out_msg;
    pcl::toROSMsg(pc_curr, out_msg);
    out_msg.header = msg->header;
    cloud_pub_->publish(out_msg);

    // Benchmark eval
    if (is_kitti_) {
      pcl::PointCloud<PointType> TP, FP, FN, TN;
      TP.header = FP.header = FN.header = TN.header = pc_curr.header;

      double precision, recall, precision_naive, recall_naive;
      calculate_precision_recall(pc_curr, pc_ground, precision, recall);
      calculate_precision_recall(pc_curr, pc_ground, precision_naive, recall_naive, false);

      RCLCPP_INFO(this->get_logger(), "Precision: %.2f | Recall: %.2f", precision, recall);

      discern_ground(pc_ground, TP, FP);
      discern_ground(pc_non_ground, FN, TN);

      tp_pub_->publish(to_msg(TP, msg->header));
      fp_pub_->publish(to_msg(FP, msg->header));
      fn_pub_->publish(to_msg(FN, msg->header));
      publish_score("p", precision);
      publish_score("r", recall);

      if (save_flag_) {
        std::map<int, int> pc_curr_gt_counts, g_est_gt_counts;
        double accuracy;
        save_all_accuracy(
            pc_curr, pc_ground, acc_filename_, accuracy, pc_curr_gt_counts, g_est_gt_counts);
        // Optionally: save PCD
      }

    } else {
      ground_pub_->publish(to_msg(pc_ground, msg->header));
      nonground_pub_->publish(to_msg(pc_non_ground, msg->header));

      for (auto &pt : pc_ground.points) pt.label = 1;
      for (auto &pt : pc_non_ground.points) pt.label = 0;
      pc_labeled = pc_ground;
      pc_labeled.insert(pc_labeled.end(), pc_non_ground.begin(), pc_non_ground.end());
      labeled_pub_->publish(to_msg(pc_labeled, msg->header));
    }
  }

  sensor_msgs::msg::PointCloud2 to_msg(const pcl::PointCloud<PointType> &cloud,
                                       const std_msgs::msg::Header &header) {
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(cloud, msg);
    msg.header = header;
    return msg;
  }

  void publish_score(const std::string &mode, double value) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = patchwork_->frame_patchwork;
    marker.header.stamp = now();
    marker.ns = "benchmark_ns";
    marker.id = (mode == "p") ? 0 : 1;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = (mode == "p") ? 28.5 : 25.0;
    marker.pose.position.y = 30.0;
    marker.pose.position.z = 1.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.z = 5.0;
    marker.color.g = 1.0;
    marker.color.a = 1.0;
    marker.text = mode + ": " + std::to_string(value);

    if (mode == "p") {
      precision_pub_->publish(marker);
    } else if (mode == "r") {
      recall_pub_->publish(marker);
    }
  }

  std::shared_ptr<PatchWork<PointType>> patchwork_;
  std::string algorithm_, seq_, acc_filename_, pcd_savepath_;
  bool is_kitti_, save_flag_;

  // ROS 2 publishers and subscriber
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_, ground_pub_,
      nonground_pub_, labeled_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr tp_pub_, fp_pub_, fn_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr precision_pub_, recall_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InterfaceNode>());
  rclcpp::shutdown();
  return 0;
}
