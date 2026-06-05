#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Header

def main():
    rospy.init_node('imu_time_shifter')
    offset = rospy.get_param('~offset_sec', 0.0031802757)  # 默认 3.180 ms
    in_topic  = rospy.get_param('~in_topic',  '/davis_left/imu')
    out_topic = rospy.get_param('~out_topic', '/davis_left/imu_shifted')

    pub = rospy.Publisher(out_topic, Imu, queue_size=200)

    def cb(msg):
        # 复制并平移时间戳
        out = Imu()
        out.header = Header()
        out.header.seq   = msg.header.seq
        out.header.frame_id = msg.header.frame_id
        out.header.stamp = msg.header.stamp + rospy.Duration.from_sec(offset)
        out.orientation      = msg.orientation
        out.orientation_covariance = msg.orientation_covariance
        out.angular_velocity = msg.angular_velocity
        out.angular_velocity_covariance = msg.angular_velocity_covariance
        out.linear_acceleration = msg.linear_acceleration
        out.linear_acceleration_covariance = msg.linear_acceleration_covariance
        pub.publish(out)

    rospy.Subscriber(in_topic, Imu, cb, queue_size=200)
    rospy.loginfo("imu_time_shifter: shifting %s -> %s by %+fs", in_topic, out_topic, offset)
    rospy.spin()

if __name__ == '__main__':
    main()
