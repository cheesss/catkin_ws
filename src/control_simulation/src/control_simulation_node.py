#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import math
import pandas as pd
import matplotlib.pyplot as plt

# Helper function for angle wrapping
def angle_confine(ang):
    ang %= 2 * math.pi
    if ang > math.pi:
        ang -= 2 * math.pi
    return ang

# Kanayama Controller class
class KanayamaController:
    def __init__(self, Kx, Ky, Kw, max_vel, min_vel, max_ang_vel, min_ang_vel):
        self.Kx = Kx
        self.Ky = Ky
        self.Kw = Kw
        self.max_vel = max_vel
        self.min_vel = min_vel
        self.max_ang_vel = max_ang_vel
        self.min_ang_vel = min_ang_vel

    def __call__(self, x_ref, y_ref, theta_ref, x, y, theta, v_ref, w_ref):
        x_sub = x_ref - x
        y_sub = y_ref - y
        x_err = math.cos(theta) * x_sub + math.sin(theta) * y_sub
        y_err = -math.sin(theta) * x_sub + math.cos(theta) * y_sub
        theta_err = angle_confine(theta_ref - theta)
        v = v_ref * math.cos(theta_err) + self.Kx * x_err
        w = w_ref + v_ref * (self.Ky * y_err + self.Kw * math.sin(theta_err))
        v = max(min(v, self.max_vel), self.min_vel)
        w = max(min(w, self.max_ang_vel), self.min_ang_vel)
        return v, w

# ROS Node class
class ControlSimulationNode:
    def __init__(self):
        rospy.init_node('control_simulation_node', anonymous=True)

        # ROS Publishers and Subscribers
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.state_sub = rospy.Subscriber('/state', Float32MultiArray, self.state_callback)
        self.path_sub = rospy.Subscriber('/reference_path', Float32MultiArray, self.path_callback)

        # Controller Initialization
        self.controller = KanayamaController(
            Kx=0.5, Ky=0.5, Kw=1.0,
            max_vel=1.0, min_vel=0.1,
            max_ang_vel=1.0, min_ang_vel=-1.0
        )

        # Reference Values
        self.x_ref, self.y_ref, self.theta_ref = None, None, None
        self.v_ref, self.w_ref = 0.5, 0.0

        # Data for Visualization
        self.x_data = []
        self.y_data = []

    def path_callback(self, msg):
        # Update reference values
        self.x_ref, self.y_ref, self.theta_ref = msg.data

    def state_callback(self, msg):
        if self.x_ref is None or self.y_ref is None or self.theta_ref is None:
            rospy.logwarn("Reference path not received yet.")
            return

        x, y, theta = msg.data[0], msg.data[1], msg.data[2]
        v, w = self.controller(self.x_ref, self.y_ref, self.theta_ref, x, y, theta, self.v_ref, self.w_ref)

        # Save data
        self.x_data.append(x)
        self.y_data.append(y)

        # Publish control command
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)

    def save_and_plot(self):
        if not self.x_data or not self.y_data:
            rospy.logwarn("No data to save or plot.")
            return
        data_file = '/tmp/control_simulation_data.csv'
        df = pd.DataFrame({'x': self.x_data, 'y': self.y_data})
        df.to_csv(data_file, index=False)
        rospy.loginfo(f"Data saved to {data_file}")

        plt.figure()
        plt.plot(self.x_data, self.y_data, label="Path")
        plt.xlabel("X [m]")
        plt.ylabel("Y [m]")
        plt.title("Vehicle Path")
        plt.legend()
        plt.grid()
        plt.savefig('/tmp/control_simulation_path.png')
        rospy.loginfo("Path visualization saved to /tmp/control_simulation_path.png")
        plt.show()


    def run(self):
        try:
            rospy.spin()
        except KeyboardInterrupt:
            self.save_and_plot()
            rospy.loginfo("Node shutdown and data saved.")

if __name__ == "__main__":
    node = ControlSimulationNode()
    node.run()
