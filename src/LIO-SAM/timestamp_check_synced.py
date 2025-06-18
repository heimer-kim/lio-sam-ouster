#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu, PointCloud2

imu_time = None
lidar_time = None

def imu_callback(msg):
    global imu_time
    imu_time = msg.header.stamp.to_sec()
    check_diff()

def lidar_callback(msg):
    global lidar_time
    lidar_time = msg.header.stamp.to_sec()
    check_diff()

def check_diff():
    if imu_time is not None and lidar_time is not None:
        diff = abs(imu_time - lidar_time)
        print(f"[Check] Time difference: {diff:.6f} seconds")
        if diff < 0.1:
            print("✅ Time difference OK (< 0.1s)")
        else:
            print("❌ Time difference too large!")

if __name__ == '__main__':
    rospy.init_node('timestamp_diff_checker_synced')
    rospy.Subscriber("/imu/data_synced", Imu, imu_callback)
    rospy.Subscriber("/points_synced", PointCloud2, lidar_callback)
    rospy.spin()
