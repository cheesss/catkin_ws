#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Vector3Stamped
from carla_msgs.msg import CarlaEgoVehicleControl

def carla_control_callback(data):
    control_msg = CarlaEgoVehicleControl()
    control_msg.throttle = data.vector.x
    control_msg.steer = data.vector.y
    control_msg.brake = data.vector.z
    pub.publish(control_msg)

def carla_integration():
    rospy.init_node('carla_integration_node', anonymous=True)
    rospy.Subscriber('/mobile_system_control/control_msg', Vector3Stamped, carla_control_callback)
    global pub
    pub = rospy.Publisher('/carla/ego_vehicle/vehicle_control_cmd', CarlaEgoVehicleControl, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    try:
        carla_integration()
    except rospy.ROSInterruptException:
        pass
