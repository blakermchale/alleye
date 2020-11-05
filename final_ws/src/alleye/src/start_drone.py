#!/usr/bin/env python3
from airsim_ros_pkgs.srv import Takeoff, Land, SetLocalPosition
from apriltag_ros.msg import AprilTagDetectionArray
import rospy

class Drone:
    def __init__(self):
        self.namespace = '/airsim_node/drone_1'
        rospy.wait_for_service(self.namespace + '/takeoff')
        rospy.wait_for_service(self.namespace + '/land')

        # Setup services
        self.srv_takeoff = rospy.ServiceProxy(self.namespace + '/takeoff', Takeoff)
        self.srv_land = rospy.ServiceProxy(self.namespace + '/land', Land)
        self.srv_local_goal = rospy.ServiceProxy('/airsim_node/local_position_goal/override', SetLocalPosition)
        self.subTag = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.tagCallback)

    def start(self):
        # Initialize and takeoff drone
        self.takeoff()

        # Move to point above area we want to monitor
        self.set_local_position(0.0, 0.0, -10.0, 0.0)

    def takeoff(self):
        try:
            resp = self.srv_takeoff(False)
            return resp.success 
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def land(self):
        try:
            resp = self.srv_land(False)
            return resp.success 
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def set_local_position(self, x, y, z, yaw, vehicle_name=''):
        """
        Sets local position in NED frame.
        """
        try:
            resp = self.srv_local_goal(x, y, z, yaw, vehicle_name)
            return resp.success 
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def tagCallback(self, data):
        pass


def main():
    rospy.init_node("alleye_drone")
    drone = Drone()
    drone.start()
    rospy.spin()

if __name__=="__main__":
    main()
