#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:


        ### Your FK code here
        # Create symbols
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6, = symbols('alpha0:7')

        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')

       
	#
	#
	# Create Modified DH parameters
        DH_table = {alpha0 :0    , a0 : 0, d1 : 0.75, q1 : q1,
                    alpha1 :-pi/2, a1 : 0.35, d2 : 0, q2 : q2-pi/2,
                    alpha2 :0    , a2 : 1.25, d3 : 0, q3 : q3,
                    alpha3 :-pi/2, a3 : -0.054, d4 : 1.5, q4 : q4,
                    alpha4 :pi/2 , a4 : 0, d5 : 0, q5 : q5,
                    alpha5 :-pi/2, a5 : 0, d6 : 0, q6 : q6,
                    alpha6 : 0   , a6 : 0, d7 : 0.303, q7 : 0}
	#
	#
	# Define Modified DH Transformation matrix
        def TF_matrix(alpha, a, d, q):
 	    TF=Matrix([[cos(q)           , -sin(q)          ,  0          , a           ],
 		       [sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha) ,-sin(alpha)*d],
 		       [sin(q)*sin(alpha), cos(q)*sin(alpha), cos(alpha)  ,cos(alpha)*d ],
 		       [0                , 0                , 0           , 1           ]])
 	    return TF
	#
	#
	# Create individual transformation matrices
        TF_1 = TF_matrix(alpha0, a0, d1, q1).subs(DH_table)
        TF_2 = TF_matrix(alpha1, a1, d2, q2).subs(DH_table)
        TF_3 = TF_matrix(alpha2, a2, d3, q3).subs(DH_table)
        TF_4 = TF_matrix(alpha3, a3, d4, q4).subs(DH_table)
        TF_5 = TF_matrix(alpha4, a4, d5, q5).subs(DH_table)
        TF_6 = TF_matrix(alpha5, a5, d6, q6).subs(DH_table)
        TF_7 = TF_matrix(alpha6, a6, d7, q7).subs(DH_table)
   
        TF1_E = TF_1*TF_2*TF_3*TF_4*TF_5*TF_6*TF_7


	#
	#
	# Extract rotation matrices from the transformation matrices
        
	R0_6 = TF1_E[0:3,0:3]


        r, p, y = symbols('r p y')
    	ROT_x = Matrix([[1,  0     ,        0],
                        [0,  cos(r),  -sin(r)],
                        [0,  sin(r),   cos(r)]])

        ROT_y = Matrix([[cos(p) , 0 , sin(p)],
                        [0      , 1 ,      0],
                        [-sin(p), 0 , cos(p)]])

        ROT_z = Matrix([[cos(r),  -sin(r), 0],
                        [sin(r),   cos(r), 0],
                        [0     ,   0     , 1]])

        ROT_EE = simplify(ROT_x*ROT_y*ROT_z)
        rot_error = ROT_z.subs(y, pi)*ROT_y.subs(p, pi/2)

        ROT_EE = ROT_EE*rot_error

    
       
	#
	#
        ###

        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

	    # Extract end-effector position and orientation from request
	    # px,py,pz = end-effector position
	    # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])
            
	   

             ### Your IK code here
	    # Compensate for rotation discrepancy between DH parameters and Gazebo
	    
    	    
            ROT_EE = ROT_EE.subs({'r' : roll, 'p' : pitch, 'y' : yaw})

            EE = Matrix([[px],
                         [py],
                         [pz]])

            WC = EE - (0.303) * ROT_EE[:,2]
            
	    side_a = 1.5
            side_b = sqrt(pow((sqrt(WC[0]*WC[0]+WC[1]*WC[1])-0.35), 2) + pow((WC[2]-0.75),2))
            side_c = 1.25

            ang_a = acos((side_b * side_b + side_c * side_c - side_a * side_a) / (2 * side_b * side_c))
            ang_b = acos((side_a * side_a + side_c * side_c - side_b * side_b) / (2 * side_a * side_c))
            ang_c = acos((side_b * side_b + side_a * side_a - side_c * side_c) / (2 * side_b * side_a))

	    
	    theta1 = atan2(WC[1],WC[0])
	    theta2 = pi/2 - ang_a - atan2((WC[2]-0.75),(sqrt(WC[0]*WC[0]+WC[1]*WC[1])-0.35))
            theta3 = pi/2 - (ang_b+0.036)

	
            TF0_3 = TF_1*TF_2*TF_3
            rot0_3 = TF0_3[0:3,0:3]
            rot0_3 = rot0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})

            rot3_6 = rot0_3.transpose() * ROT_EE
	    #
	    #
	    # Calculate joint angles using Geometric IK method

	    
	    theta4 = atan2(rot3_6[2,2], -rot3_6[0,2])
            theta5 = atan2(sqrt(rot3_6[0,2]*rot3_6[0,2]+rot3_6[2,2]*rot3_6[2,2]), rot3_6[1,2])
            theta6 = atan2(-rot3_6[1,1], rot3_6[1,0])


	    #
	    #
            ###

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
	    joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()

