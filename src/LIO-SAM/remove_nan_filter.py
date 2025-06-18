#!/usr/bin/env python3
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2

def callback(msg):
    # NaN 제거
    points = [p for p in pc2.read_points(msg, skip_nans=True)]

    # 기존 header를 그대로 사용 (stamp 덮어쓰기 ❌)
    filtered = pc2.create_cloud(msg.header, msg.fields, points)
    filtered.is_dense = True

    pub.publish(filtered)

if __name__ == '__main__':
    rospy.init_node('remove_nan_filter')
    pub = rospy.Publisher('/points_filtered', PointCloud2, queue_size=1)
    rospy.Subscriber('/ouster/points', PointCloud2, callback)
    rospy.spin()

