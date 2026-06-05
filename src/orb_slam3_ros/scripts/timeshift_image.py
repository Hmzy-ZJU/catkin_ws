#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Header

def main():
    rospy.init_node('timeshift_right_image')
    shift = rospy.get_param('~shift_sec', -2.587292)  # 右图减去 2.587292 秒
    pub = rospy.Publisher('/davis_right/image_raw_shifted', Image, queue_size=10)

    def cb(msg):
        msg.header = Header(stamp=msg.header.stamp + rospy.Duration.from_sec(shift),
                            frame_id=msg.header.frame_id,
                            seq=msg.header.seq)
        pub.publish(msg)

    rospy.Subscriber('/davis_right/image_raw', Image, cb, queue_size=50)
    rospy.spin()

if __name__ == '__main__':
    main()
