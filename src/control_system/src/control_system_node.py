#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
import math

# 기존의 클래스 및 함수 정의를 여기에 추가
# 예: angle_confine, Integrator, PIDController, KanayamaController

class ControlSystemNode:
    def __init__(self):
        rospy.init_node('control_system_node', anonymous=True)
        
        # ROS Publisher & Subscriber
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.state_sub = rospy.Subscriber('/state', Float32MultiArray, self.state_callback)
        
        self.controller = KanayamaController(
            Kx=0.5, Ky=0.5, Kw=1.0,
            max_vel=1.0, min_vel=0.1,
            max_ang_vel=1.0, min_ang_vel=-1.0
        )
        
        # 데이터 저장용
        self.x_data = []
        self.y_data = []
        self.v_data = []
        self.w_data = []

    def state_callback(self, msg):
        x, y, theta = msg.data[0], msg.data[1], msg.data[2]
        v, w = self.controller(
            self.x_ref, self.y_ref, self.theta_ref,
            x, y, theta, self.v_ref, self.w_ref
        )
        
        self.x_data.append(x)
        self.y_data.append(y)
        self.v_data.append(v)
        self.w_data.append(w)

        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)

    def save_and_plot(self):

        with open('/tmp/control_system_data.csv', 'w') as f:
            f.write('x,y,v,w\n')
            for x, y, v, w in zip(self.x_data, self.y_data, self.v_data, self.w_data):
                f.write(f'{x},{y},{v},{w}\n')

        plt.figure()
        plt.plot(self.x_data, self.y_data, label='Path')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Path Followed by the Robot')
        plt.legend()
        plt.grid()
        plt.savefig('/tmp/control_system_path.png')
        plt.show()

    def run(self):
        try:
            rospy.spin()
        except KeyboardInterrupt:
            self.save_and_plot()
            rospy.loginfo("Node shutdown and data saved.")

if __name__ == "__main__":
    node = ControlSystemNode()
    node.run()
