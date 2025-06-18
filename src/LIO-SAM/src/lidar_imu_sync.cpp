#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>

ros::Time latestLidarTime;
bool lidarTimeAvailable = false;

ros::Publisher syncedImuPub;

void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    latestLidarTime = msg->header.stamp;
    lidarTimeAvailable = true;
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    if (!lidarTimeAvailable) {
        ROS_WARN_THROTTLE(5.0, "No LiDAR timestamp received yet. Skipping IMU sync.");
        return;
    }

    sensor_msgs::Imu syncedMsg = *msg;
    syncedMsg.header.stamp = latestLidarTime;

    syncedImuPub.publish(syncedMsg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_imu_sync");
    ros::NodeHandle nh;

    ros::Subscriber lidarSub = nh.subscribe("/points_filtered", 10, lidarCallback);
    ros::Subscriber imuSub = nh.subscribe("/imu/data", 100, imuCallback);

    syncedImuPub = nh.advertise<sensor_msgs::Imu>("/imu/data_synced", 100);

    ROS_INFO("LiDAR-IMU sync node started.");
    ros::spin();
    return 0;
}

