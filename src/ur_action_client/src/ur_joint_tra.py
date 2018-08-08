#!/usr/bin/env python
# -*- coding:utf-8 -*-
import time
import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import *
from std_msgs.msg import *


JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

class JointTra:
    #第一个点必须是当前点的位置
    q1 = [] #当前位置
    q2 = [] # 下一个目标点
    client = None
    sub1 = None
    sub2 = None

    def __init__(self):
        # rospy.Time.now()
        self.client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
        self.sub1 = rospy.Subscriber('joint_states',JointState,self.currentJoint)
        self.sub2 = rospy.Subscriber('joint_to_move',Float32MultiArray,self.currentJoint)#下一个路径点
        print "Waiting for server..."
        self.client.wait_for_server()
        print "Connected to server"
        while not rospy.is_shutdown():
            self.move1()
          
    def targetJoint(self, msg):
      if len(self.q2)==0:
          self.self.q2.extend(msg.data)
          print(self.q2)

    def currentJoint(self,msg):
        if len(self.q1)==0:
            self.q1.extend(msg.position)
            # print(q1)

    def move1(self):
        g = FollowJointTrajectoryGoal()
        g.trajectory = JointTrajectory()
        g.trajectory.joint_names = JOINT_NAMES
        g.trajectory.points = [
            JointTrajectoryPoint(positions=self.q1, velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
            JointTrajectoryPoint(positions=self.q2, velocities=[0]*6, time_from_start=rospy.Duration(5.0))]
        self.client.send_goal(g)
        try:
            self.client.wait_for_result()
        except KeyboardInterrupt:
            self.client.cancel_goal()
            raise

if __name__ == '__main__':
    rospy.init_node("ur_joint_tra", anonymous=True, disable_signals=True)
    ic = JointTra()
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        rate.sleep()
