#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_broadcaster.h>

#include <mutex>
#include <boost/circular_buffer.hpp>
#include "simple_lio_loc.h"
#include "loc_types.h"

class FastLIOHandler
{
public:
    FastLIOHandler() : nh_("~"), odom_buffer_(10), cloud_buffer_(10)
    {
        odom_sub_  = nh_.subscribe("/Odometry", 10, &FastLIOHandler::odomCallback, this);
        cloud_sub_ = nh_.subscribe("/cloud_registered_body", 10, &FastLIOHandler::cloudCallback, this);
        pose_pub_  = nh_.advertise<geometry_msgs::PoseStamped>("/estimated_pose", 10);
        map_pub_   = nh_.advertise<sensor_msgs::PointCloud2>("/map_cloud", 1, true);
        registration_pub_   = nh_.advertise<sensor_msgs::PointCloud2>("/loc_registered_cloud", 1);
        std::string map_file_path;
        if (!nh_.getParam("map_file", map_file_path))
        {
            ROS_ERROR("parameter map_file not specified");
            abort();
        }
        if (loc_.loadMap(map_file_path)) {
            publish_map(map_file_path);
        }

        Eigen::Isometry3d initial_pose = Eigen::Isometry3d::Identity();
        std::string initial_pose_str;
        if (nh_.getParam("initial_pose", initial_pose_str))
        {
            initial_pose = parse_posestr(initial_pose_str);
        }
        loc_.setInitialPose(initial_pose);
        simple_lio_localization::Params params;
        params.min_registration_distance = 2.0;
        int nframes;
        if (nh_.getParam("frames_accumulate", nframes)) {
            if (nframes > 0) {
                params.frames_accumulate = nframes;
            } else {
                params.frames_accumulate = 1;
            }
        }
        if (nh_.getParam("min_registration_distance", params.min_registration_distance)) {
            ROS_INFO("min registration distance: %f", params.min_registration_distance);
        }
        ROS_INFO("accumulate frames: %d", params.frames_accumulate);
        loc_.setParams(params);
        loc_.setRegistrationDoneCallback(std::bind(&FastLIOHandler::registrationCallback, this, std::placeholders::_1));
        bool asynchronous_registration = false;
        if (nh_.getParam("asynchronous_registration", asynchronous_registration)) {
            if (asynchronous_registration) loc_.startAsynchronousRegistration();
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
            return;
        }

        sensor_msgs::PointCloud2 map_msg;
        pcl::toROSMsg(*map_cloud, map_msg);
        map_msg.header.frame_id = "map";
        map_pub_.publish(map_msg);
    }

    Eigen::Isometry3d parse_posestr(const std::string &pose_str)
    {
        double x = 0.0, y = 0.0, z = 0.4;
        double qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;

        std::istringstream iss(pose_str);
        iss >> x >> y >> z >> qx >> qy >> qz >> qw;
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        pose.translation() << x, y, z;

        ROS_INFO("parse pose str trans=(%f %f %f) quat=(%f %f %f %f)", x, y, z, qx, qy, qz, qw);

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

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        ROS_INFO("Received Odometry data");
        const auto& position = msg->pose.pose.position;
        const auto& orientation = msg->pose.pose.orientation;
        ROS_INFO("LIO Position: x=%f, y=%f, z=%f", position.x, position.y, position.z);
        ROS_INFO("LIO Orientation: x=%f, y=%f, z=%f, w=%f", orientation.x, orientation.y, orientation.z, orientation.w);
        odom_buffer_.push_back(*msg);
        update();
    }

    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        simple_lio_localization::PointCloudPCL::Ptr cloud(new simple_lio_localization::PointCloudPCL);
        pcl::fromROSMsg(*msg, *cloud);

        ROS_INFO("Received PointCloud2 data with %ld points", cloud->size());
        cloud_buffer_.push_back(cloud);
        update();
    }

    void registrationCallback(const simple_lio_localization::RegistrationResult &result)
    {
        std::lock_guard<std::mutex> lock(registered_cloud_mutex_);
        ROS_INFO("Registration done callback time=%fms, converged=%d", result.elapsed_sec*1000, result.converged);
        loc_registered_cloud_ = result.pc_registered;
    }

    void update()
    {
        {
            //publish registered cloud if any
            std::lock_guard<std::mutex> lock(registered_cloud_mutex_);
            if (!loc_registered_cloud_.empty()) {
                sensor_msgs::PointCloud2 cloud_msg;
                pcl::toROSMsg(loc_registered_cloud_, cloud_msg);
                cloud_msg.header.frame_id = "map";
                registration_pub_.publish(cloud_msg);
            }
            loc_registered_cloud_.clear();
        }
        int odom_size = odom_buffer_.size();
        int cloud_size = cloud_buffer_.size();
        ROS_INFO("Odom buffer size: %d, Cloud buffer size: %d", odom_size, cloud_size);
        if (odom_buffer_.empty() || cloud_buffer_.empty()) {
            ROS_INFO("No data to update");
            return;
        }
        const auto& odom = odom_buffer_.front();
        const auto& cloud = cloud_buffer_.front();
        simple_lio_localization::Pose3d lio_pose = simple_lio_localization::Pose3d::Identity();
        lio_pose.translation() << odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z;
        lio_pose.rotate(Eigen::Quaterniond(odom.pose.pose.orientation.w, odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z));
        loc_.update(*cloud, lio_pose);
        Eigen::Isometry3d pose = loc_.getPose();
        ROS_INFO("Pose: x=%f, y=%f, z=%f", pose.translation().x(), pose.translation().y(), pose.translation().z());
        odom_buffer_.pop_front();
        cloud_buffer_.pop_front();

        publish_estimated_pose(odom.header.stamp, pose);
        publish_transform(odom.header.stamp, loc_.getLIOToMap());
    }

    void publish_estimated_pose(const ros::Time &stamp, const simple_lio_localization::Pose3d &pose) {
        geometry_msgs::PoseStamped pose_msg;
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
        pose_pub_.publish(pose_msg);
    }

    void publish_transform(const ros::Time &stamp, const simple_lio_localization::Pose3d &lio_to_map) {
        geometry_msgs::TransformStamped transformStamped;

        transformStamped.header.stamp = stamp;
        transformStamped.header.frame_id = "map";
        transformStamped.child_frame_id = "camera_init";

        transformStamped.transform.translation.x = lio_to_map.translation().x();
        transformStamped.transform.translation.y = lio_to_map.translation().y();
        transformStamped.transform.translation.z = lio_to_map.translation().z();
        geometry_msgs::Quaternion orientation;
        Eigen::Quaterniond q(lio_to_map.rotation());
        orientation.x = q.x();
        orientation.y = q.y();
        orientation.z = q.z();
        orientation.w = q.w();
        transformStamped.transform.rotation = orientation;

        tf_broadcaster_.sendTransform(transformStamped);
    }

private:
    simple_lio_localization::SimpleLIOLoc loc_;
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::Subscriber cloud_sub_;
    ros::Publisher pose_pub_;
    ros::Publisher map_pub_;
    ros::Publisher registration_pub_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    boost::circular_buffer<nav_msgs::Odometry> odom_buffer_;
    boost::circular_buffer<simple_lio_localization::PointCloudPCL::Ptr> cloud_buffer_;
    std::mutex registered_cloud_mutex_;
    simple_lio_localization::PointCloudPCL loc_registered_cloud_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fast_lio_handler_node");

    FastLIOHandler handler;
    ros::spin();
    handler.terminate();
    std::cout << "terminated" << std::endl;

    return 0;
}

