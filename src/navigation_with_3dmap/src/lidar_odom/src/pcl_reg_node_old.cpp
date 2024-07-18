#include <small_gicp/pcl/pcl_registration.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <small_gicp/util/downsampling_omp.hpp>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "tf2_msgs/msg/tf_message.hpp"
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>

namespace small_gicp {
class LidarOdometryNode : public rclcpp::Node
{

  public:
    LidarOdometryNode() : Node("lidar_odometry_node")
    {
      RCLCPP_INFO(this->get_logger(), "lidar_odometry_node");

      parameter_initilization();

      double CorrespondenceRandomness;
      double MaxCorrespondenceDistance;
      double VoxelResolution;
      const int numthreads = 4;
      std::string RegistrationType;
      std::string scan_topic_name;
      std::string pointcloud2_topic_name;
      std::string odom_topic_name;
      std::string odom_frame_id;
      std::string base_frame_id;

      this->get_parameter("CorrespondenceRandomness", CorrespondenceRandomness);
      this->get_parameter("MaxCorrespondenceDistance", MaxCorrespondenceDistance);
      this->get_parameter("VoxelResolution", VoxelResolution);
      this->get_parameter("RegistrationType", RegistrationType);
      this->get_parameter("scan_topic_name", scan_topic_name);
      this->get_parameter("odom_topic_name", odom_topic_name);
      this->get_parameter("pointcloud2_topic_name", pointcloud2_topic_name);
      this->get_parameter("odom_frame_id", odom_frame_id);
      this->get_parameter("base_frame_id", base_frame_id);

      RCLCPP_INFO(this->get_logger(), "===== Configuration =====");

      RCLCPP_INFO(this->get_logger(), "CorrespondenceRandomness: %.4f", CorrespondenceRandomness);
      RCLCPP_INFO(this->get_logger(), "MaxCorrespondenceDistance: %.4f", MaxCorrespondenceDistance);
      RCLCPP_INFO(this->get_logger(), "VoxelResolution %.4f", VoxelResolution);
      RCLCPP_INFO(this->get_logger(), "RegistrationType: %s", RegistrationType.c_str());
      RCLCPP_INFO(this->get_logger(), "scan_topic_name: %s", scan_topic_name.c_str());
      RCLCPP_INFO(this->get_logger(), "odom_topic_name: %s", odom_topic_name.c_str());
      RCLCPP_INFO(this->get_logger(), "pointcloud2_topic_name: %s", pointcloud2_topic_name.c_str());
      RCLCPP_INFO(this->get_logger(), "odom_frame_id: %s", odom_frame_id.c_str());
      RCLCPP_INFO(this->get_logger(), "base_frame_id: %s", base_frame_id.c_str());

      // Declare registrator
      reg.setNumThreads(numthreads);
      reg.setCorrespondenceRandomness(CorrespondenceRandomness);
      reg.setMaxCorrespondenceDistance(MaxCorrespondenceDistance);
      reg.setVoxelResolution(VoxelResolution);
      reg.setRegistrationType(RegistrationType.c_str());  // or "GICP" (default = "GICP")

      pass_x.setFilterFieldName("x");
      pass_x.setFilterLimits(-20.0, 20.0); // Adjust these limits as needed
      pass_y.setFilterFieldName("y");
      pass_y.setFilterLimits(-20.0, 20.0); // Adjust these limits as needed
      T = Eigen::Isometry3d::Identity();

      odom_publisher = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_name, 100);
      scan_subscriber = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        scan_topic_name, 2, std::bind(&LidarOdometryNode::scan_callback, this, std::placeholders::_1)
      );
      pointcloud_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(pointcloud2_topic_name, 100);
      odometry_transform_publisher_ = this->create_publisher<tf2_msgs::msg::TFMessage>(
      "/tf", rclcpp::SystemDefaultsQoS());
      realtime_odometry_transform_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(odometry_transform_publisher_);
    
    }

    private:
      Eigen::Isometry3d T;
      rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher;
      rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr odometry_transform_publisher_;
      std::shared_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>> realtime_odometry_transform_publisher_;
      rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher;
      rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_subscriber;
      RegistrationPCL<pcl::PointXYZ, pcl::PointXYZ> reg;
      pcl::PassThrough<pcl::PointXYZ> pass_x;
      pcl::PassThrough<pcl::PointXYZ> pass_y;

      void parameter_initilization() {
        this->declare_parameter<double>("CorrespondenceRandomness", 20);
        this->declare_parameter<double>("MaxCorrespondenceDistance", 1.0);
        this->declare_parameter<double>("VoxelResolution", 1.0);
        this->declare_parameter<std::string>("RegistrationType", "VGICP");
        this->declare_parameter<std::string>("scan_topic_name", "/livox/lidar");
        this->declare_parameter<std::string>("odom_topic_name", "scan_odom");
        this->declare_parameter<std::string>("pointcloud2_topic_name", "Modified_PointCloud2");
        this->declare_parameter<std::string>("odom_frame_id", "odom");
        this->declare_parameter<std::string>("base_frame_id", "base_link");
      }


enum class Axis
      {
        X = 0,
        Y = 1,
        Z = 2
      };

      enum class Plane
      {
        XY = 0,
        YZ = 1,
        ZX = 2
      };

      void pointcloud_range_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, Axis axis, double min_range, double max_range, bool is_negative=false)
      {
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud);
        switch (axis)
        {
          case Axis::X:
            pass.setFilterFieldName("x");
            pass.setFilterLimits(min_range, max_range);
            break;
        
          case Axis::Y:
            pass.setFilterFieldName("y");
            pass.setFilterLimits(min_range, max_range);
            break;
          
          case Axis::Z:
            pass.setFilterFieldName("z");
            pass.setFilterLimits(min_range, max_range);
            break;
        }
        pass.setNegative(is_negative);
        pass.filter(*cloud);
      }

      void pointcloud_tinker_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
      {
        pcl::CropBox<pcl::PointXYZ> boxFilter;
        boxFilter.setMin(Eigen::Vector4f(-0.7, -0.25, -0.3, 1.0));
        boxFilter.setMax(Eigen::Vector4f(0.18, 0.25, 0.6, 1.0));
        boxFilter.setInputCloud(cloud);
        boxFilter.setNegative(true);
        boxFilter.filter(*cloud);
      }

      void pointcloud_angle_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, Plane plane, double min_angle, double max_angle, bool is_negative=false)
      // the minimun and maximum angle to be reserved on plane
      {
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
        double angle_current;

        // 计算每个点的角度并添加到 inliers 中
        for (size_t i = 0; i < cloud->points.size(); ++i)
        {
          switch (plane)
          {
            case Plane::XY:
              angle_current = atan2(cloud->points[i].y, cloud->points[i].x);
              break;
            case Plane::YZ:
              angle_current = atan2(cloud->points[i].z, cloud->points[i].y);
              break;
            case Plane::ZX:
              angle_current = atan2(cloud->points[i].x, cloud->points[i].z);
              break;
          }
          if (min_angle <= angle_current && angle_current <= max_angle)
          {
            inliers->indices.push_back(i);
          }
        }

        // 设置过滤器参数
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(is_negative);
        extract.filter(*cloud);
      }

      void scan_callback(const sensor_msgs::msg::PointCloud2::SharedPtr scan_msg) {
        // RCLCPP_INFO(this->get_logger(), "Resgistering");
        pcl::PointCloud<pcl::PointXYZ>::Ptr points_pcl(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*scan_msg, *points_pcl);
        // pass_x.setInputCloud(points_pcl);
        // pass_x.filter(*points_pcl);
        // pass_y.setInputCloud(points_pcl);
        // pass_y.filter(*points_pcl);
        pointcloud_tinker_filter(points_pcl);
        // publish_pointcloud2(points_pcl);
        pcl::PointCloud<pcl::PointXYZ>::Ptr target = voxelgrid_sampling_omp(*points_pcl, 0.25);
        auto target_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>(*target);
        
        if (!reg.getInputTarget()) {
            RCLCPP_INFO(this->get_logger(), "No initial cloud found, set as initial");
            reg.setInputTarget(target_cloud);           
        }
        reg.setInputSource(target_cloud);
        // RCLCPP_INFO(this->get_logger(), "Begin regeister");
        auto aligned = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        reg.align(*aligned);
        // RCLCPP_INFO(this->get_logger(), "Resgister Finished");
        T = T * Eigen::Isometry3d(reg.getFinalTransformation().cast<double>());
        reg.swapSourceAndTarget();
        publish_odometry();
        // publish_tf();
      }

      void publish_odometry() {
        std::string fixed_id = "odom";

        nav_msgs::msg::Odometry odom_msg;

        odom_msg.header.frame_id = fixed_id;
        odom_msg.child_frame_id = "livox";
        odom_msg.header.stamp = this->get_clock()->now();

         // Set position
        odom_msg.pose.pose.position.x = T.translation().x();
        odom_msg.pose.pose.position.y = T.translation().y();
        odom_msg.pose.pose.position.z = T.translation().z();
        odom_msg.pose.covariance = {
              0.15, 0.0, 0.0, 0.0, 0.0, 0.0,
              0.0, 0.15, 0.0, 0.0, 0.0, 0.0,
              0.0, 0.0, 0.15, 0.0, 0.0, 0.0,
              0.0, 0.0, 0.0, 0.15, 0.0, 0.0,
              0.0, 0.0, 0.0, 0.0, 0.15, 0.0,
              0.0, 0.0, 0.0, 0.0, 0.0, 0.15
          };

        // Set orientation
        Eigen::Quaterniond q(T.rotation());
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();

        odom_publisher->publish(odom_msg);
      }

      void publish_tf(){
        if (realtime_odometry_transform_publisher_->trylock())
          {
            auto & odometry_transform_message = realtime_odometry_transform_publisher_->msg_;
            odometry_transform_message.transforms.resize(1);
            odometry_transform_message.transforms.front().header.frame_id = "odom";
            odometry_transform_message.transforms.front().child_frame_id =  "base_link";
            auto & transform = realtime_odometry_transform_publisher_->msg_.transforms.front();
            transform.header.stamp = this->get_clock()->now();
            transform.transform.translation.x = T.translation().x();
            transform.transform.translation.y = T.translation().y();
            Eigen::Quaterniond q(T.rotation());
            transform.transform.rotation.x = q.x();
            transform.transform.rotation.y = q.y();
            transform.transform.rotation.z = q.z();
            transform.transform.rotation.w = q.w();
            realtime_odometry_transform_publisher_->unlockAndPublish();
          }
      }

      void publish_pointcloud2(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud){
          sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();

          // Convert the pcl::PointCloud to the PointCloud2 message
          pcl::toROSMsg(*cloud, *cloud_msg);

          // Set the header values
          cloud_msg->header.stamp = this->get_clock()->now();
          cloud_msg->header.frame_id = "laser"; // Change this to your frame id

          // Publish the message
          pointcloud_publisher->publish(*cloud_msg);
        }

};
}
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<small_gicp::LidarOdometryNode>());
  rclcpp::shutdown();
  return 0;
}