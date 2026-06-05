// Per-scan RANSAC ground-plane segmentation for NDT input clouds.
//
// Replaces the earlier Python prototype (pointcloud_ground_filter,
// numpy + sensor_msgs_py.point_cloud2). The Python version spent
// the bulk of its time in PCL <-> Python conversion (~500ms per
// 100k-point scan) which caused system-wide real-time backlog —
// scanmatcher / EKF / RViz all started dropping messages.
//
// This C++ version uses pcl::fromROSMsg + pcl::SACSegmentation with
// SACMODEL_PERPENDICULAR_PLANE (axis = (0,0,1), eps_angle = 30°) so
// the plane has to be near-horizontal. Inliers are then removed via
// pcl::ExtractIndices and the remainder is republished. Typical
// runtime: ~3-8 ms per scan on a modern desktop.
//
// Parameters mirror the Python version so existing launch / docs stay
// usable.

#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/multi_array_dimension.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>


using std::placeholders::_1;
using PointT = pcl::PointXYZI;
using CloudT = pcl::PointCloud<PointT>;

class RansacGroundFilter : public rclcpp::Node
{
public:
  RansacGroundFilter()
  : rclcpp::Node("ransac_ground_filter")
  {
    declare_parameter<std::string>("input_topic",        "/morai/lidar/points");
    declare_parameter<std::string>("output_topic",       "/lidar/no_ground");
    declare_parameter<std::string>("diagnostics_topic",  "/lidar/ground_filter/diagnostics");

    // RANSAC hyper-parameters
    declare_parameter<int>   ("max_iter",            50);
    declare_parameter<double>("distance_threshold",  0.20);
    // SACMODEL_PERPENDICULAR_PLANE accepts planes whose normal is
    // within eps_angle of the configured axis. axis = (0,0,1) so the
    // plane is forced near-horizontal.
    declare_parameter<double>("eps_angle_deg",       30.0);
    declare_parameter<int>   ("min_inliers",         500);

    // Optional: clip to a z-band before RANSAC so e.g. walls don't
    // win the plane vote on cluttered scans. Wide default ≈ disabled.
    declare_parameter<double>("z_min_for_ransac",   -3.0);
    declare_parameter<double>("z_max_for_ransac",    0.5);

    input_topic_   = get_parameter("input_topic").as_string();
    output_topic_  = get_parameter("output_topic").as_string();
    diag_topic_    = get_parameter("diagnostics_topic").as_string();

    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&RansacGroundFilter::onCloud, this, _1));
    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic_, rclcpp::SensorDataQoS());
    diag_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      diag_topic_, 10);

    RCLCPP_INFO(get_logger(),
      "ransac_ground_filter started: %s -> %s",
      input_topic_.c_str(), output_topic_.c_str());
  }

private:
  void onCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    const auto t0 = std::chrono::steady_clock::now();

    CloudT::Ptr cloud(new CloudT());
    pcl::fromROSMsg(*msg, *cloud);
    const size_t n_total = cloud->size();
    if (n_total == 0) {
      pub_->publish(*msg);
      publishDiag(0, 0, 0, 0.0);
      return;
    }

    // Pre-clip a z-band so SACSegmentation considers only candidate
    // ground points; the full cloud is filtered against the resulting
    // plane afterwards.
    const double z_min = get_parameter("z_min_for_ransac").as_double();
    const double z_max = get_parameter("z_max_for_ransac").as_double();

    pcl::PointIndices::Ptr candidate_indices(new pcl::PointIndices());
    candidate_indices->indices.reserve(n_total);
    for (size_t i = 0; i < n_total; ++i) {
      const float z = (*cloud)[i].z;
      if (z >= z_min && z <= z_max) {
        candidate_indices->indices.push_back(static_cast<int>(i));
      }
    }
    if (candidate_indices->indices.empty()) {
      pub_->publish(*msg);
      publishDiag(n_total, n_total, 0, elapsedMs(t0));
      return;
    }

    // RANSAC plane fit on the candidate band only.
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(get_parameter("max_iter").as_int());
    seg.setDistanceThreshold(
      get_parameter("distance_threshold").as_double());
    seg.setAxis(Eigen::Vector3f(0.0f, 0.0f, 1.0f));
    seg.setEpsAngle(
      get_parameter("eps_angle_deg").as_double() * M_PI / 180.0);
    seg.setInputCloud(cloud);
    seg.setIndices(candidate_indices);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::ModelCoefficients::Ptr coeffs(new pcl::ModelCoefficients());
    seg.segment(*inliers, *coeffs);

    const int n_inliers_in_band = static_cast<int>(inliers->indices.size());
    if (n_inliers_in_band < get_parameter("min_inliers").as_int()) {
      // No good ground plane — pass through unchanged.
      pub_->publish(*msg);
      publishDiag(n_total, n_total, n_inliers_in_band, elapsedMs(t0));
      return;
    }

    // Re-test the plane against the FULL cloud so points outside the
    // pre-clip band that nonetheless lie on the ground plane get
    // removed too.
    if (coeffs->values.size() < 4) {
      pub_->publish(*msg);
      publishDiag(n_total, n_total, n_inliers_in_band, elapsedMs(t0));
      return;
    }
    const float a = coeffs->values[0];
    const float b = coeffs->values[1];
    const float c = coeffs->values[2];
    const float d = coeffs->values[3];
    const double thr = get_parameter("distance_threshold").as_double();

    pcl::PointIndices::Ptr full_inliers(new pcl::PointIndices());
    full_inliers->indices.reserve(n_total);
    for (size_t i = 0; i < n_total; ++i) {
      const auto & p = (*cloud)[i];
      const float dist = std::abs(a * p.x + b * p.y + c * p.z + d);
      if (dist < thr) {
        full_inliers->indices.push_back(static_cast<int>(i));
      }
    }

    // Remove inliers, keep the rest.
    CloudT::Ptr kept(new CloudT());
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(full_inliers);
    extract.setNegative(true);
    extract.filter(*kept);

    sensor_msgs::msg::PointCloud2 out;
    pcl::toROSMsg(*kept, out);
    out.header = msg->header;
    pub_->publish(out);

    publishDiag(n_total, kept->size(),
                static_cast<int>(full_inliers->indices.size()),
                elapsedMs(t0));
  }

  void publishDiag(size_t total, size_t kept, int plane_inliers,
                   double dt_ms)
  {
    std_msgs::msg::Float64MultiArray msg;
    std_msgs::msg::MultiArrayDimension dim;
    dim.label  = "total_pts, kept_pts, kept_ratio, plane_inliers, dt_ms";
    dim.size   = 5;
    dim.stride = 5;
    msg.layout.dim.push_back(dim);
    const double ratio = total > 0
      ? static_cast<double>(kept) / static_cast<double>(total) : 0.0;
    msg.data = {
      static_cast<double>(total),
      static_cast<double>(kept),
      ratio,
      static_cast<double>(plane_inliers),
      dt_ms,
    };
    diag_->publish(msg);
  }

  static double elapsedMs(std::chrono::steady_clock::time_point t0)
  {
    const auto t1 = std::chrono::steady_clock::now();
    return std::chrono::duration<double, std::milli>(t1 - t0).count();
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string diag_topic_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr diag_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RansacGroundFilter>());
  rclcpp::shutdown();
  return 0;
}
