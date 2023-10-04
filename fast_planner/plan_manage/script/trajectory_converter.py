#!/usr/bin/env python

import rospy
from quadrotor_msgs.msg import PositionCommand # for Fast-Planner
from mavros_msgs.msg import PositionTarget
from geometry_msgs.msg import PoseStamped

class MessageConverter:
    def __init__(self):
        rospy.init_node('trajectory_msg_converter')

        self.currentPose = PoseStamped()
        self.checkPose = 0

        self.pose = PositionTarget()
        self.pose.coordinate_frame = 1
        self.pose.position.x = 0
        self.pose.position.y = 0
        self.pose.position.z = 3
        self.pose.yaw = 0

        self.traj_pub = rospy.Publisher('fastplanner/waypoint', PositionTarget, queue_size=1)
        rospy.Subscriber('planning/pos_cmd', PositionCommand, self.fastPlannerTrajCallback)
        rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.poseCallback)

    def fastPlannerTrajCallback(self, msg):
        # position and yaw
        
        self.pose.position.x = msg.position.x
        self.pose.position.y = msg.position.y
        self.pose.position.z = msg.position.z
        
        self.pose.velocity.x = msg.velocity.x
        self.pose.velocity.y = msg.velocity.y
        self.pose.velocity.z = msg.velocity.z

        self.pose.acceleration_or_force.x = msg.acceleration.x
        self.pose.acceleration_or_force.y = msg.acceleration.y
        self.pose.acceleration_or_force.z = msg.acceleration.z

        self.pose.yaw = msg.yaw

    def poseCallback(self, msg):
        self.currentPose = msg
        self.checkPose = self.checkPose + 1
        if self.checkPose >= 3:
            self.checkPose = 3



    def run(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():

            if self.checkPose == 1 or self.checkPose == 2:
                self.pose.position.x = self.currentPose.pose.position.x
                self.pose.position.y = self.currentPose.pose.position.y
                self.pose.position.z = 2
            
            self.traj_pub.publish(self.pose)
            rate.sleep()

if __name__ == '__main__':
    obj = MessageConverter()
    obj.run()