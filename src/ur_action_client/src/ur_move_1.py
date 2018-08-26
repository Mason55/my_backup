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
from point_msgs.msg import Point

import numpy as np
from numpy import linalg
from numpy  import *

import cmath
import math
from math import cos as cos
from math import sin as sin
from math import atan2 as atan2
from math import acos as acos
from math import asin as asin
from math import sqrt as sqrt
from math import pi as pi

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

mat = np.matrix
d1 =  0.089159
a2 = -0.425
a3 = -0.39225
d4 =  0.10915
d5 =  0.09465
d6 =  0.0823
d = mat([0.089159, 0, 0, 0.10915, 0.09465, 0.0823]) #ur5
# d = mat([0.1273, 0, 0, 0.163941, 0.1157, 0.0922])#ur10 mm
a =mat([0 ,-0.425 ,-0.39225 ,0 ,0 ,0]) #ur5
# a =mat([0 ,-0.612 ,-0.5723 ,0 ,0 ,0])#ur10 mm
alph = mat([math.pi/2, 0, 0, math.pi/2, -math.pi/2, 0 ])  #ur5
# global init_time = 0
# alph = mat([pi/2, 0, 0, pi/2, -pi/2, 0 ]) # ur10

class UrMove:
    #第一个点必须是当前点的位置
    # Q1 = [] #当前的关节角
    Q2 = [] #目标的关节角
    Q3 = [] #插值的关节角
    quat_time_in = []#七元树加上时间(最后一位是时间)
    quat_in = [] # 目标位置的四元数,后四位不要动
    joint_to_choose = [] #计算出来的下一组关节角
    sub1 = None
    client = None
    dcm=np.zeros((4,4))

    def __init__(self):
        self.client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
        self.sub1 = rospy.Subscriber('/ur/move/tra',Point,self.currentQua)#接受一个四元数这里需要修改
        print "Waiting for server..."
        self.client.wait_for_server()
        print "Connected to server"        

    def currentQua(self,msg):
        self.quat_time_in=msg.data
        self.quat_in = self.quat_time_in[0:7]
        print(self.quat_in[0],self.quat_in[1],self.quat_in[2])

    def cal3JiTraje(self,orig,goal,freq,time):
        ref = goal
        time_n = freq*time
        if time_n <= 1:
            ref = orig + (goal-orig)*time_n*time_n*(3.0-2*time_n);
        return ref;

           
    def moveOnce(self):
        joint_states = rospy.wait_for_message("joint_states", JointState)
        joints_pos = joint_states.position#当前位置的6个关节角
        self.dcm = self.quatToDcm(self.quat_in,self.dcm)
        self.joint_to_choose = self.invKine(self.dcm)
        self.Q2 = self.findRightJoint(joints_pos,self.joint_to_choose)
        g = FollowJointTrajectoryGoal()
        g.trajectory = JointTrajectory()
        g.trajectory.joint_names = JOINT_NAMES

        d=self.quat_time_in[7]#总时间
      
        g.trajectory.points = [
            JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0))
            ]

        g.trajectory.points.append(
            JointTrajectoryPoint(positions=self.Q2, velocities=[0]*6, time_from_start=rospy.Duration(d)))
  
        self.client.send_goal(g)
        
        try:
            self.client.wait_for_result()
        except KeyboardInterrupt:
            self.client.cancel_goal()
            raise
        # g.trajectory.points.clear()

# ************************************************** FORWARD KINEMATICS

    def AH(self, n,th,c  ):

      T_a = mat(np.identity(4), copy=False)
      T_a[0,3] = a[0,n-1]
      T_d = mat(np.identity(4), copy=False)
      T_d[2,3] = d[0,n-1]

      Rzt = mat([[cos(th[n-1,c]), -sin(th[n-1,c]), 0 ,0],
    	         [sin(th[n-1,c]),  cos(th[n-1,c]), 0, 0],
    	         [0,               0,              1, 0],
    	         [0,               0,              0, 1]],copy=False)


      Rxa = mat([[1, 0,                 0,                  0],
    			 [0, cos(alph[0,n-1]), -sin(alph[0,n-1]),   0],
    			 [0, sin(alph[0,n-1]),  cos(alph[0,n-1]),   0],
    			 [0, 0,                 0,                  1]],copy=False)

      A_i = T_d * Rzt * T_a * Rxa


      return A_i

    def HTrans(self,th,c ):
      A_1=self.AH( 1,th,c  )
      A_2=self.AH( 2,th,c  )
      A_3=self.AH( 3,th,c  )
      A_4=self.AH( 4,th,c  )
      A_5=self.AH( 5,th,c  )
      A_6=self.AH( 6,th,c  )

      T_06=A_1*A_2*A_3*A_4*A_5*A_6

      return T_06

# ************************************************** INVERSE KINEMATICS

    def invKine(self,desired_pos):# T60
      th = mat(np.zeros((6, 8)))
      P_05 = (desired_pos * mat([0,0, -d6, 1]).T-mat([0,0,0,1 ]).T)

      # **** theta1 ****

      psi = atan2(P_05[2-1,0], P_05[1-1,0])
      phi = acos(d4 /sqrt(P_05[2-1,0]*P_05[2-1,0] + P_05[1-1,0]*P_05[1-1,0]))
      #The two solutions for theta1 correspond to the shoulder
      #being either left or right
      th[0, 0:4] = pi/2 + psi + phi
      th[0, 4:8] = pi/2 + psi - phi
      th = th.real

      # **** theta5 ****

      cl = [0, 4]# wrist up or down
      for i in range(0,len(cl)):
    	      c = cl[i]
    	      T_10 = linalg.inv(self.AH(1,th,c))
    	      T_16 = T_10 * desired_pos
    	      th[4, c:c+2] = + acos((T_16[2,3]-d4)/d6);
    	      th[4, c+2:c+4] = - acos((T_16[2,3]-d4)/d6);

      th = th.real

      # **** theta6 ****
      # theta6 is not well-defined when sin(theta5) = 0 or when T16(1,3), T16(2,3) = 0.

      cl = [0, 2, 4, 6]
      for i in range(0,len(cl)):
    	      c = cl[i]
    	      T_10 = linalg.inv(self.AH(1,th,c))
    	      T_16 = linalg.inv( T_10 * desired_pos )
    	      th[5, c:c+2] = atan2((-T_16[1,2]/sin(th[4, c])),(T_16[0,2]/sin(th[4, c])))

      th = th.real

      # **** theta3 ****
      cl = [0, 2, 4, 6]
      for i in range(0,len(cl)):
    	      c = cl[i]
    	      T_10 = linalg.inv(self.AH(1,th,c))
    	      T_65 = self.AH( 6,th,c)
    	      T_54 = self.AH( 5,th,c)
    	      T_14 = ( T_10 * desired_pos) * linalg.inv(T_54 * T_65)
    	      P_13 = T_14 * mat([0, -d4, 0, 1]).T - mat([0,0,0,1]).T
    	      t3 = cmath.acos((linalg.norm(P_13)**2 - a2**2 - a3**2 )/(2 * a2 * a3)) # norm ?
    	      th[2, c] = t3.real
    	      th[2, c+1] = -t3.real

      # **** theta2 and theta 4 ****

      cl = [0, 1, 2, 3, 4, 5, 6, 7]
      for i in range(0,len(cl)):
    	      c = cl[i]
    	      T_10 = linalg.inv(self.AH( 1,th,c ))
    	      T_65 = linalg.inv(self.AH( 6,th,c))
    	      T_54 = linalg.inv(self.AH( 5,th,c))
    	      T_14 = (T_10 * desired_pos) * T_65 * T_54
    	      P_13 = T_14 * mat([0, -d4, 0, 1]).T - mat([0,0,0,1]).T

    	      # theta 2
    	      th[1, c] = -atan2(P_13[1], -P_13[0]) + asin(a3* sin(th[2,c])/linalg.norm(P_13))
    	      # theta 4
    	      T_32 = linalg.inv(self.AH( 3,th,c))
    	      T_21 = linalg.inv(self.AH( 2,th,c))
    	      T_34 = T_32 * T_21 * T_14
    	      th[3, c] = atan2(T_34[1,0], T_34[0,0])
      th = th.real

      return th

    #获得两个list之间各个元素差值的绝对值的和
    def calDistance(self,list_in,list_re_out):
      dis_sum = 0
      for i in range(0,6):
        dis_sum += abs(list_re_out[i]-list_in[i])
      return dis_sum

    #cur_joint 6*1的list inv_joint_list 8*6 的逆运动学可能解
    def findRightJoint(self,cur_joint,inv_joint_list):
      inv_joint=inv_joint_list.T#转化为6*8
      dis=[]#存储比较结果的list
      list_out=[]#存储各组解
      index =0
      for item in inv_joint:
        list1=item.tolist()#转化为list
        list_out.append(list1[0])#降维
        dis.append(self.calDistance(cur_joint,list1[0]))#存入各组比较值
      index=dis.index(min(dis))#返回最小值下标
      # print(list_out[index])
      return list_out[index]#返回最小值小标对应的一组关节角

    def quatToDcm(self,quaternion_in,dcm):
      dcm[0,0] = quaternion_in[6]*quaternion_in[6] + quaternion_in[3]*quaternion_in[3] - quaternion_in[4]*quaternion_in[4] - quaternion_in[5]*quaternion_in[5]
      dcm[0,1] = 2*(quaternion_in[3]*quaternion_in[4] + quaternion_in[6]*quaternion_in[5])
      dcm[0,2] = 2*(quaternion_in[3]*quaternion_in[5] - quaternion_in[6]*quaternion_in[4])
      dcm[1,0] = 2*(quaternion_in[3]*quaternion_in[4] - quaternion_in[6]*quaternion_in[5])
      dcm[1,1] = quaternion_in[6]*quaternion_in[6] - quaternion_in[3]*quaternion_in[3] + quaternion_in[4]*quaternion_in[4] - quaternion_in[5]*quaternion_in[5]
      dcm[1,2] = 2*(quaternion_in[4]*quaternion_in[5] + quaternion_in[6]*quaternion_in[3])
      dcm[2,0] = 2*(quaternion_in[3]*quaternion_in[5] + quaternion_in[6]*quaternion_in[4])
      dcm[2,1] = 2*(quaternion_in[4]*quaternion_in[5] - quaternion_in[6]*quaternion_in[3])
      dcm[2,2] = quaternion_in[6]*quaternion_in[6] - quaternion_in[3]*quaternion_in[3] - quaternion_in[4]*quaternion_in[4] + quaternion_in[5]*quaternion_in[5]
      dcm[0,3] = quaternion_in[0]
      dcm[1,3] = quaternion_in[1]
      dcm[2,3] = quaternion_in[2]
      dcm[3,3] = 1.0
      return dcm

   


if __name__ == '__main__':
    rospy.init_node("ur_tra_ik", anonymous=True, disable_signals=True)
    ik=UrMove()
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        ik.moveOnce()
        rate.sleep()