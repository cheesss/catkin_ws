#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray
import pandas as pd

def main():
    rospy.init_node('reference_path_publisher', anonymous=True)
    pub = rospy.Publisher('/reference_path', Float32MultiArray, queue_size=10)

    # Load the reference path
    path_file = '/tmp/reference_path.csv'  # CSV 파일 경로
    path = pd.read_csv(path_file)

    rate = rospy.Rate(10)  # 10 Hz
    for _, row in path.iterrows():
        msg = Float32MultiArray()
        msg.data = [row['x'], row['y'], row['z']]
        pub.publish(msg)
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
