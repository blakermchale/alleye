#!/usr/bin/env python3
from apriltag_ros.msg import AprilTagDetectionArray
from std_msgs.msg import Empty, Bool, Header
from geometry_msgs.msg import Twist, Pose, TransformStamped, Vector3, Quaternion
import tf2_ros
import math
import PyKDL
import rospy
import yaml

class Drone:
    def __init__(self):
        self.namespace = 'drone'
        self.subTag = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.tagCallback)
        self.pub_takeoff = rospy.Publisher(f"/{self.namespace}/takeoff", Empty, queue_size=10)
        self.pub_land = rospy.Publisher(f"/{self.namespace}/land", Empty, queue_size=10)
        self.pub_posctrl = rospy.Publisher(f"/{self.namespace}/posctrl", Bool, queue_size=10)
        self.pub_cmdvel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.sub_Pose = rospy.Subscriber(f"/{self.namespace}/gt_pose", Pose, self.handle_drone_pose, queue_size=10)

        self.static_br = tf2_ros.StaticTransformBroadcaster()
        self.br = tf2_ros.TransformBroadcaster()

        self.camera_tf = TransformStamped()
        realsense_translation = Vector3(x=0., y=0., z=0.)  # 0.052 0 -0.0186
        x, y, z, w = PyKDL.Rotation().RPY(0, math.pi, math.pi/2).GetQuaternion()
        realsense_quat = Quaternion(x=x, y=y, z=z, w=w)
        self.camera_tf.transform.translation = realsense_translation
        self.camera_tf.transform.rotation = realsense_quat
        self.handle_fixed_sensors()

        while not self.pub_takeoff.get_num_connections() or not self.pub_land.get_num_connections() \
            or not self.pub_posctrl.get_num_connections or not self.pub_cmdvel.get_num_connections():
            rospy.logwarn_throttle(1.0, "Waiting for publishers to connect")

    def start(self):
        self.takeoff()
        # Move to point above area we want to monitor
        self.set_local_position(-9.0, 0.0, 7.0, 0.0)

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
        
    def handle_fixed_sensors(self):
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = self.namespace
        self.camera_tf.header = header
        self.camera_tf.child_frame_id = "camera_link"
        self.static_br.sendTransform(self.camera_tf)

    def handle_drone_pose(self, msg):
        """
        ROS subscriber to republish drone's position in TF.

        :param msg: Pose message.
        """
        tfmsg = TransformStamped()
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "world"

        translation = Vector3(x=msg.position.x, y=msg.position.y, z=msg.position.z)
        tfmsg.transform.translation = translation
        tfmsg.transform.rotation = msg.orientation
        tfmsg.child_frame_id = self.namespace
        tfmsg.header = header

        self.br.sendTransform(tfmsg)


def main():
    rospy.init_node("alleye_drone")
    drone = Drone()
    drone.start()
    rospy.spin()

    #init world
    # Input: Boundaries(-9.2, 6.0), (1.6, 6.0), (1.6, -4.5), (-9.2, -4.5), Obstacles [(x,y), ...], size input ex. (20,20), apriltag config to bounding box
    # output: Binary world [[]]

    # apriltag config to goal/start positions

    # run a_star.py


if __name__=="__main__":
    main()
