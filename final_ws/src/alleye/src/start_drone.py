#!/usr/bin/env python3
# TODO: Get rid of airsim references
# from airsim_ros_pkgs.srv import Takeoff, Land, SetLocalPosition
from apriltag_ros.msg import AprilTagDetectionArray
from std_msgs.msg import Empty, Bool
from geometry_msgs.msg import Twist
import rospy

class Drone:
    def __init__(self):
        self.namespace = 'drone'
        self.subTag = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.tagCallback)
        self.pub_takeoff = rospy.Publisher(f"/{self.namespace}/takeoff", Empty, queue_size=10)
        self.pub_land = rospy.Publisher(f"/{self.namespace}/land", Empty, queue_size=10)
        self.pub_posctrl = rospy.Publisher(f"/{self.namespace}/posctrl", Bool, queue_size=10)
        self.pub_cmdvel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        while not self.pub_takeoff.get_num_connections() or not self.pub_land.get_num_connections() \
            or not self.pub_posctrl.get_num_connections or not self.pub_cmdvel.get_num_connections():
            rospy.logwarn_throttle(1.0, "Waiting for publishers to connect")

    def start(self):
        self.takeoff()
        # Move to point above area we want to monitor
        self.set_local_position(0.0, 0.0, 1.0, 0.0)

    def takeoff(self):
        self.pub_takeoff.publish()
        rospy.loginfo("Took off")

    def land(self):
        self.pub_land.publish()

    def set_local_position(self, x, y, z, yaw, vehicle_name=''):
        """
        Sets local position in XYZ frame.
        """
        self.pub_posctrl.publish(True)
        twist = Twist()
        twist.linear.x = x
        twist.linear.y = y
        twist.linear.z = z
        self.pub_cmdvel.publish(twist)
        rospy.loginfo(f"Sent waypoint x: {x} y: {y} z: {z} yaw: {yaw}")

    def tagCallback(self, data):
        pass


def main():
    rospy.init_node("alleye_drone")
    drone = Drone()
    drone.start()
    rospy.spin()

if __name__=="__main__":
    main()
