#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
SHIFT = 0.0031802757  # seconds, make IMU earlier
pub = None
def cb(msg):
    msg.header.stamp = rospy.Time.from_sec(msg.header.stamp.to_sec() - SHIFT)
    pub.publish(msg)
if __name__ == "__main__":
    rospy.init_node("imu_timeshift")
    pub = rospy.Publisher("/imu_shifted", Imu, queue_size=1000)
    rospy.Subscriber("/davis_left/imu", Imu, cb, queue_size=1000)
    rospy.spin()
