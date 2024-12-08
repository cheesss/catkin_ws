#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Path

target_path = []
def path_callback(data):
    global target_path
    target_path = [(pose.pose.position.x, pose.pose.position.y) for pose in data.poses]

def control_callback(data):
    global target_path
    if not target_path:
        return

    current_position = (data.data[0], data.data[1])
    target = target_path[0]

    # 제어값 계산
    control_msg = Vector3Stamped()
    control_msg.header.stamp = rospy.Time.now()
    control_msg.vector.x = 0.5  # throttle
    control_msg.vector.y = (target[0] - current_position[0]) * 0.1  # 조향 각
    control_msg.vector.z = 0.0  # brake

    pub.publish(control_msg)

def controller():
    rospy.init_node('controller_node', anonymous=True)
    rospy.Subscriber('/mobile_system_control/ego_vehicle', Float32MultiArray, control_callback)
    rospy.Subscriber('/optimized_path', Path, path_callback)
    global pub
    pub = rospy.Publisher('/mobile_system_control/control_msg', Vector3Stamped, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException:
        pass
