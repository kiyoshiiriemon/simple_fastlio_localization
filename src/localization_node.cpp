#include <boost/circular_buffer.hpp>
#include "simple_lio_loc.h"
#include "loc_types.h"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <Eigen/Geometry>
#include <pcl/io/pcd_io.h>

class FastLIOHandler : public rclcpp::Node
{
public:
    FastLIOHandler() : Node("fast_lio_handler"), odom_buffer_(10), cloud_buffer_(10)
    {
        this->declare_parameter("map_file", rclcpp::PARAMETER_STRING);
        this->declare_parameter("initial_pose", rclcpp::PARAMETER_STRING);
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/Odometry", 10, std::bind(&FastLIOHandler::odomCallback, this, std::placeholders::_1));
        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/cloud_registered_body", 10, std::bind(&FastLIOHandler::cloudCallback, this, std::placeholders::_1));
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/estimated_pose", 10);
        map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/map_cloud",
            rclcpp::QoS(rclcpp::KeepLast(1)).transient_local()
        );

        std::string map_file_path;
        if (!this->get_parameter("map_file", map_file_path))
        {
            RCLCPP_ERROR(this->get_logger(), "parameter map_file not specified");
            rclcpp::shutdown();
            return;
        }

        if (loc_.loadMap(map_file_path))
        {
            publish_map(map_file_path);
        }

        Eigen::Isometry3d initial_pose = Eigen::Isometry3d::Identity();
        std::string initial_pose_str;
        if (this->get_parameter("initial_pose", initial_pose_str))
        {
            initial_pose = parse_posestr(initial_pose_str);
        }
        loc_.setInitialPose(initial_pose);
    }

    void publish_map(const std::string &map_file_path)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        if (pcl::io::loadPCDFile<pcl::PointXYZI>(map_file_path, *map_cloud) == -1)
        {
            std::cerr << "Failed to load map file: " << map_file_path << std::endl;
            return;
        }

        sensor_msgs::msg::PointCloud2 map_msg;
        pcl::toROSMsg(*map_cloud, map_msg);
        map_msg.header.frame_id = "map";
        map_pub_->publish(map_msg);
        std::cerr << "Published map cloud with " << map_cloud->size() << " points" << std::endl;
    }

    Eigen::Isometry3d parse_posestr(const std::string &pose_str)
    {
        double x = 0.0, y = 0.0, z = 0.4;
        double qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;

        std::istringstream iss(pose_str);
        iss >> x >> y >> z >> qx >> qy >> qz >> qw;
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        pose.translation() << x, y, z;

        RCLCPP_INFO(this->get_logger(), "parse pose str trans=(%f %f %f) quat=(%f %f %f %f)", x, y, z, qx, qy, qz, qw);

        Eigen::Quaterniond q(qw, qx, qy, qz);
        if (q.norm() == 0)
        {
            std::cerr << "invalid quaternion" << std::endl;
            q = Eigen::Quaterniond(1, 0, 0, 0);
        }
        else
        {
            q.normalize();
        }
        pose.rotate(q);

        return pose;
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received Odometry data");
        const auto &position = msg->pose.pose.position;
        const auto &orientation = msg->pose.pose.orientation;
        RCLCPP_INFO(this->get_logger(), "LIO Position: x=%f, y=%f, z=%f", position.x, position.y, position.z);
        RCLCPP_INFO(this->get_logger(), "LIO Orientation: x=%f, y=%f, z=%f, w=%f", orientation.x, orientation.y, orientation.z, orientation.w);
        odom_buffer_.push_back(*msg);
        update();
    }

    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        simple_lio_localization::PointCloudPCL::Ptr cloud(new simple_lio_localization::PointCloudPCL);
        pcl::fromROSMsg(*msg, *cloud);

        RCLCPP_INFO(this->get_logger(), "Received PointCloud2 data with %ld points", cloud->size());
        cloud_buffer_.push_back(cloud);
        update();
    }

    void update()
    {
        if (odom_buffer_.empty() || cloud_buffer_.empty())
        {
            RCLCPP_INFO(this->get_logger(), "No data to update");
            return;
        }

        const auto &odom = odom_buffer_.front();
        const auto &cloud = cloud_buffer_.front();
        simple_lio_localization::Pose3d lio_pose = simple_lio_localization::Pose3d::Identity();
        lio_pose.translation() << odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z;
        lio_pose.rotate(Eigen::Quaterniond(odom.pose.pose.orientation.w, odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z));
        loc_.update(*cloud, lio_pose);
        Eigen::Isometry3d pose = loc_.getPose();

        RCLCPP_INFO(this->get_logger(), "Pose: x=%f, y=%f, z=%f", pose.translation().x(), pose.translation().y(), pose.translation().z());
        odom_buffer_.pop_front();
        cloud_buffer_.pop_front();

        publish_estimated_pose(odom.header.stamp, pose);
        publish_transform(odom.header.stamp, loc_.getLIOToMap());
    }

    void publish_estimated_pose(const rclcpp::Time &stamp, const simple_lio_localization::Pose3d &pose)
    {
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = stamp;
        pose_msg.header.frame_id = "map";
        pose_msg.pose.position.x = pose.translation().x();
        pose_msg.pose.position.y = pose.translation().y();
        pose_msg.pose.position.z = pose.translation().z();
        Eigen::Quaterniond q(pose.rotation());
        pose_msg.pose.orientation.w = q.w();
        pose_msg.pose.orientation.x = q.x();
        pose_msg.pose.orientation.y = q.y();
        pose_msg.pose.orientation.z = q.z();
        pose_pub_->publish(pose_msg);
    }

    void publish_transform(const rclcpp::Time &stamp, const simple_lio_localization::Pose3d &lio_to_map)
    {
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.stamp = stamp;
        transformStamped.header.frame_id = "map";
        transformStamped.child_frame_id = "camera_init";
        transformStamped.transform.translation.x = lio_to_map.translation().x();
        transformStamped.transform.translation.y = lio_to_map.translation().y();
        transformStamped.transform.translation.z = lio_to_map.translation().z();
        Eigen::Quaterniond q(lio_to_map.rotation());
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();
        tf_broadcaster_->sendTransform(transformStamped);
    }

private:
    simple_lio_localization::SimpleLIOLoc loc_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    boost::circular_buffer<nav_msgs::msg::Odometry> odom_buffer_;
    boost::circular_buffer<simple_lio_localization::PointCloudPCL::Ptr> cloud_buffer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FastLIOHandler>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
