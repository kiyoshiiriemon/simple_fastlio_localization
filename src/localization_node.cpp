#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <mutex>
#include <boost/circular_buffer.hpp>
#include <pcl/io/pcd_io.h>
#include "simple_lio_loc.h"
#include "loc_types.h"

static Eigen::Isometry3d pose_from_odom(const nav_msgs::msg::Odometry &odom)
{
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() << odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z;
    Eigen::Quaterniond q(odom.pose.pose.orientation.w, odom.pose.pose.orientation.x,
                         odom.pose.pose.orientation.y, odom.pose.pose.orientation.z);
    pose.rotate(q);
    return pose;
}

class FastLIOHandler : public rclcpp::Node
{
public:
    FastLIOHandler() : Node("fast_lio_handler"), odom_buffer_(1), cloud_buffer_(1)
    {
        this->declare_parameter<std::string>("map_file", "");
        this->declare_parameter<std::string>("initial_pose", "");
        this->declare_parameter<int>("frames_accumulate", 1);
        this->declare_parameter<double>("min_registration_distance", 0);
        this->declare_parameter<double>("min_registration_interval_sec", 0.0);
        this->declare_parameter<bool>("asynchronous_registration", false);
        this->declare_parameter<bool>("publish_2d_pose", false);
        this->declare_parameter<bool>("visualize_registration_result", false);
        this->declare_parameter<bool>("enable_sound", false);
        this->declare_parameter<bool>("enable_lio_only_update", false);
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                "/Odometry", 10, std::bind(&FastLIOHandler::odomCallback, this, std::placeholders::_1));
        cloud_odom_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/cloud_registered", 10, std::bind(&FastLIOHandler::cloudCallback, this, std::placeholders::_1));
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/estimated_pose", 10);
        map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/map_cloud",
                                                                         rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());
        registration_pub_   = this->create_publisher<sensor_msgs::msg::PointCloud2>("/loc_registered_cloud", 1);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

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
        simple_lio_localization::Params params;
        params.frames_accumulate = this->get_parameter("frames_accumulate").as_int();
        params.min_registration_distance = this->get_parameter("min_registration_distance").as_double();
        params.min_registration_interval_sec = this->get_parameter("min_registration_interval_sec").as_double();
        params.max_accumulate_frames = params.frames_accumulate;
        RCLCPP_INFO(this->get_logger(), "frames_accumulate: %d", params.frames_accumulate);
        RCLCPP_INFO(this->get_logger(), "min_registration_distance: %f", params.min_registration_distance);
        RCLCPP_INFO(this->get_logger(), "min_registration_interval_sec: %f", params.min_registration_interval_sec);
        loc_.setParams(params);
        loc_.setRegistrationDoneCallback(std::bind(&FastLIOHandler::registrationCallback, this, std::placeholders::_1));

        if (this->get_parameter("asynchronous_registration").as_bool()) {
            RCLCPP_INFO(this->get_logger(), "Asynchronous registration enabled");
            loc_.startAsynchronousRegistration();
        }
        publish_2d_pose_ = this->get_parameter("publish_2d_pose").as_bool();
        if (publish_2d_pose_) {
            RCLCPP_INFO(this->get_logger(), "2D pose projection enabled");
        } else {
            RCLCPP_INFO(this->get_logger(), "2D pose projection disabled");
        }
        visualize_registration_result_ = this->get_parameter("visualize_registration_result").as_bool();
        if (visualize_registration_result_) {
            RCLCPP_INFO(this->get_logger(), "Registration result visualization enabled");
        } else {
            RCLCPP_INFO(this->get_logger(), "Registration result visualization disabled");
        }
        enable_sound_ = this->get_parameter("enable_sound").as_bool();
        if (enable_sound_) {
            RCLCPP_INFO(this->get_logger(), "Sound notification enabled");
        } else {
            RCLCPP_INFO(this->get_logger(), "Sound notification disabled");
        }
        lio_only_update_ = this->get_parameter("enable_lio_only_update").as_bool();
        if (lio_only_update_) {
            RCLCPP_INFO(this->get_logger(), "LIO only update enabled");
        } else {
            RCLCPP_INFO(this->get_logger(), "LIO only update disabled");
        }
    }

    void terminate() {
        loc_.terminate();
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

        std::string processed_str = pose_str;
        std::replace(processed_str.begin(), processed_str.end(), ',', ' ');
        
        std::istringstream iss(processed_str);
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

    Eigen::Isometry3d projectTo2D(const Eigen::Isometry3d& pose_3d)
    {
        Eigen::Isometry3d pose_2d = Eigen::Isometry3d::Identity();
        pose_2d.translation() << pose_3d.translation().x(), pose_3d.translation().y(), 0.0;
        Eigen::Matrix3d rot_matrix = pose_3d.rotation();
        double yaw = std::atan2(rot_matrix(1, 0), rot_matrix(0, 0));
        Eigen::AngleAxisd rotation_z(yaw, Eigen::Vector3d::UnitZ());
        pose_2d.rotate(rotation_z);
        return pose_2d;
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received Odometry data");
        const auto &position = msg->pose.pose.position;
        const auto &orientation = msg->pose.pose.orientation;
        RCLCPP_INFO(this->get_logger(), "LIO Position: x=%f, y=%f, z=%f", position.x, position.y, position.z);
        RCLCPP_INFO(this->get_logger(), "LIO Orientation: x=%f, y=%f, z=%f, w=%f", orientation.x, orientation.y, orientation.z, orientation.w);
        odom_buffer_.push_back(*msg);

        if (publish_2d_pose_) {
            Eigen::Isometry3d odom3d = pose_from_odom(*msg);
            Eigen::Isometry3d odom2d = projectTo2D(odom3d);
            publish_transform(msg->header.stamp, odom2d, "odom", "base_link");
        }

        update(msg->header.stamp);
    }

    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        lio_frame_ = msg->header.frame_id;
        simple_lio_localization::PointCloudPCL::Ptr cloud(new simple_lio_localization::PointCloudPCL);
        pcl::fromROSMsg(*msg, *cloud);

        RCLCPP_INFO(this->get_logger(), "Received PointCloud2 data with %ld points", cloud->size());
        cloud_buffer_.push_back(cloud);
        update(msg->header.stamp);
    }

    void registrationCallback(const simple_lio_localization::RegistrationResult &result)
    {
        std::lock_guard<std::mutex> lock(registered_cloud_mutex_);
        loc_registered_cloud_ = result.pc_registered;
        loc_registered_cloud_timestamp_ = result.timestamp;
        colorizePointCloud(loc_registered_cloud_, result.converged);
        playRegistrationSound(result.converged);
    }

    void update(const rclcpp::Time &stamp)
    {
        {
            std::lock_guard<std::mutex> lock(registered_cloud_mutex_);
            if (!loc_registered_cloud_.empty()) {
                sensor_msgs::msg::PointCloud2 cloud_msg;
                pcl::toROSMsg(loc_registered_cloud_, cloud_msg);
                cloud_msg.header.frame_id = "map";
                cloud_msg.header.stamp = rclcpp::Time(loc_registered_cloud_timestamp_);
                registration_pub_->publish(cloud_msg);
                loc_registered_cloud_.clear();
            }
        }

        if (lio_only_update_ && !odom_buffer_.empty() && cloud_buffer_.empty()) {
            const auto &odom = odom_buffer_.back();
            loc_.updateLIO(pose_from_odom(odom));
            RCLCPP_INFO(this->get_logger(), "LIO only update");
        } else if (odom_buffer_.empty() || cloud_buffer_.empty()) {
            return;
        } else {
            const auto &odom = odom_buffer_.back();
            const auto &cloud = cloud_buffer_.back();
            simple_lio_localization::Pose3d lio_pose = pose_from_odom(odom);
            double timestamp = stamp.seconds();
            loc_.update(*cloud, lio_pose, timestamp, simple_lio_localization::CoordinateFrame::LIO);
            odom_buffer_.clear();
            cloud_buffer_.clear();
        }
        Eigen::Isometry3d pose = loc_.getPose();
        RCLCPP_INFO(this->get_logger(), "Pose: x=%f, y=%f, z=%f", pose.translation().x(), pose.translation().y(), pose.translation().z());

        publish_estimated_pose(stamp, pose);
        publish_transform(stamp, loc_.getLIOToMap(), "map", lio_frame_);
        if (publish_2d_pose_) {
            publish_2d_transform(stamp, loc_.getLIOToMap());
        }
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

    void publish_transform(const rclcpp::Time &stamp, const simple_lio_localization::Pose3d &lio_to_map, const std::string frame_id, const std::string child_frame_id)
    {
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.stamp = stamp;
        transformStamped.header.frame_id = frame_id;
        transformStamped.child_frame_id = child_frame_id;
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

    void publish_2d_transform(const rclcpp::Time &stamp, const Eigen::Isometry3d &lio_to_map)
    {
        Eigen::Isometry3d lio_to_map2d = projectTo2D(lio_to_map);
        publish_transform(stamp, lio_to_map2d, "map", "odom");
    }

    void colorizePointCloud(simple_lio_localization::PointCloudPCL &cloud, bool success)
    {
        if (!visualize_registration_result_) return;
        
        if (!success) {
            for (auto &point : cloud.points) {
                point.intensity = 0.0f;
            }
        }
    }

    void playRegistrationSound(bool success)
    {
        if (!enable_sound_) return;
        
        std::string package_path = ament_index_cpp::get_package_share_directory("simple_fastlio_localization");
        std::string sound_file;
        
        if (success) {
            sound_file = package_path + "/sounds/popi.wav";
        } else {
            sound_file = package_path + "/sounds/pipi.wav";
        }
        
        std::string command = "aplay " + sound_file + " &";
        std::system(command.c_str());
    }

private:
    simple_lio_localization::SimpleLIOLoc loc_;
    std::string lio_frame_;
    bool publish_2d_pose_;
    bool visualize_registration_result_;
    bool enable_sound_;
    bool lio_only_update_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr registration_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    boost::circular_buffer<nav_msgs::msg::Odometry> odom_buffer_;
    boost::circular_buffer<simple_lio_localization::PointCloudPCL::Ptr> cloud_buffer_;
    std::mutex registered_cloud_mutex_;
    simple_lio_localization::PointCloudPCL loc_registered_cloud_;
    double loc_registered_cloud_timestamp_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FastLIOHandler>();
    rclcpp::spin(node);
    node->terminate();
    rclcpp::shutdown();
    return 0;
}

