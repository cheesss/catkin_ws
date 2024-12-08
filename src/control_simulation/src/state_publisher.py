#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray
import math
import random

def main():
    rospy.init_node('state_publisher', anonymous=True)
    pub = rospy.Publisher('/state', Float32MultiArray, queue_size=10)

    rate = rospy.Rate(10)  # 10 Hz
    x, y, theta = 0.0, 0.0, 0.0
    while not rospy.is_shutdown():
        # 간단한 움직임 시뮬레이션 (랜덤 이동)
        x += random.uniform(-0.1, 0.1)
        y += random.uniform(-0.1, 0.1)
        theta += random.uniform(-0.05, 0.05)

        # 데이터 발행
        msg = Float32MultiArray()
        msg.data = [x, y, theta]
        pub.publish(msg)

        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
