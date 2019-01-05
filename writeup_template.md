## Project: Kinematics Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc3.png
[image3]: ./misc_images/misc2.png
[image4]: ./misc_images/misc4.png
[image5]: ./misc_images/misc5.png
[image6]: ./misc_images/misc6.png
[image7]: ./misc_images/misc7.png
[image8]: ./misc_images/misc8.png
[image9]: ./misc_images/misc9.png
[image10]: ./misc_images/misc10.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

Shows an example of the Kuka KR210 6DOF arm used for this project.

![alt text][image1]

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
  --- | --- | --- | --- | ---
0->1  | 0      | 0 | 0.75 | q1
1->2  | - pi/2 | 0.35 | 0 | -pi/2 + q2
2->3  | 0      | 1.25 | 0 | q3
3->4  | - pi/2 | -0.054 | 1.24 | q4
4->5  |   pi/2 | 0    | 0 | q5
5->6  | - pi/2  | 0   | 0 | q6
6->EE | 0      | 0    | 0.303 | 0

The above parameters have been obtained by the following method.
a(i) is the distance from Z(i-1) to Z(i) measured along X(i-1).
alpha(i) is the angle from Z(i-1) to Z(i) measured about X(i-1).
d(i) is the distance from X(i-1) to X(i) measured along Z(i).
theta(i) is the angle from X(i-1) to X(i) measured about Z(i).

The Tranformation matrix is given as follows:
#### Homogeneous Transforms
TF = Matrix([[             cos(q1),            -sin(q1),            0,              a0],
               [ sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1],
               [ sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),  cos(alpha0),  cos(alpha0)*d1],
               [                   0,                   0,            0,               1]])
The individual transformation matrix for each link, considering the DH Parameters is given below:

![alt text][image4]

To Determine the complete homogeneous transform between the base link and the gripper link (end-effector) using just the end-effector pose 

Firstly, From the Roll, pitch and yaw angle of end effector position of object to be picked up from shelf, Calculate the individual rotational matrix for rotation in x-axis(roll), y-axis(pitch), yaw(z-axis) 

Then take the product of individual rotation matrix to get the rotation matrix of end effector position.
The final matrix for the tranformation of end effector is given below 
![alt text][image7]


![alt text][image5]

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

And here's where you can draw out and show your math for the derivation of your theta angles. 

For the manipulator, we can split the inverse kinematics problem into two parts- 
1. Inverse position kinematics : In this the position of wrist centre to calculate the three joints angles(base to link 2).
2. Inverse Orientation kinematics : In this the orientation of wrist is used to calculate the remaining three joint angles(link3 to link6)
![alt text][image2]
Step to calculate the all individual joint angles:
Step 1: First, Assume 6 Degree of freedom, the last three interects at Oc(centre of wrist centre). 
![alt text][image6]
Use position of wrist centre(WC) to determine first three joint angles. 
Now, origin of tool frame, O6, is a distance d6 translated along z5. 
Thus, the third column of R0_6 is in the direction z6 and we can write the equation to calculate the postion of wrist centre by using the equation below
![alt text][image8]
Step 2 : Position wrist centre gives a vector is used to calculate the first three joint angles. By using the forward kinematics, derive the kinematics equation of the 3 link RRR system. 
Theta1: It can be calculated by projecting onto x-y plane and solve trignometric problem by taking the tan of projection.

Step 3: Calculate the remaining two joint angles using the cosine law
Consider a tringle having sides as link 2, link3 and wrist centre for applying the cosine law
    side_a = 1.5
    side_b = sqrt(pow((sqrt(w_c[0]*w_c[0] + w_c[1]*w_c[1])- 0.35),2) + pow((w_c[2] - 0.75), 2))
    side_c = 1.25 
calculate the value of the interior angles of the triangle
    ang_a = acos((side_b*side_b + side_c*side_c - side_a*side_a)/(2*side_b*side_c))
    ang_b = acos((side_a*side_a + side_c*side_c - side_b*side_b)/(2*side_a*side_c))
    ang_c = 180 - (ang_a + ang_b)
Step 4 :calculate the theta 2 and theta 3 
    theta2 =  pi/2 - ang_a - atan2((w_c[2]- 0.75), sqrt(w_c[0]*w_c[0]+ w_c[1]*w_c[1] - 0.35))
    theta3 = pi/2 - (ang_b + 0.036)
Step 5 : Now, we can use the desired orientation of end effector to solve for the last three joint angles. For finding a set of euler angles corresponding to a desired rotation matrix R.

To calculate the rotation matrix for link from base link to link 2, Substitute the value of theta1, theta2 and theta3 in the product of the transformation matrices from 0 to 3. The resulting transformation matrix contain rotation matrix and position vector as shown in figure.
Pick the rotaion matrix from the transformation matrix.

Step 6: The rotation matrix thus obatined is the rotation from joint 0 to joint 3. As we know the rotation matrix for rotation from joint 0 to joint6, Use the rotation matrix to calculate the rotation matrix from joint 3 to joint6 as give below.
![alt text][image9]
(I used transpose in place of inverse, as the rotational matrix is orthogonal)
Step 7: Rotational matrix from joint 3 to joint 6 will give the remaining joint angles.

theta4 = atan2(R3_6[2,2], -R3_6[0,2])
theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]), R3_6[1,2])
theta6 = atan2(-R3_6[1,1], R3_6[1,0])

Note : There is error in the definition of the end effector orientation between the URDF and DH parameters. This error should be multiplied in the calculation of rotation matrix in step 6. In case of wrist centre, we are already taking the desired orientation and position so error need not be multiplied there.  

![alt text][image3]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  

The project was tricky one. I tested it on the IK_debug file for error. When errors were minimum, I took next step of running it with Gazebo and Rviz. 

Few Problems I faced in implimentation are:

1. Use of simplify function really slow down the speed of computation. I didn't use it for debug but in final code I used it, as it gives less erratic inverse kinematics. The average cycle time for my pick and place operation is 5 minutes(I am using windows machine with 8 GB RAM and CORE 5i processor and I allocated around 6 GB RAM and 3 core processor to VM machine)

2. The Gripper leave the target quiet frequently. While using the next button, if you overwrite the command, this can skip the pick or release function of the arm. While using the continuue on Rviz,I found out that the sleep time was not enough for the gripper to hold the target, I changed(by the help from Udacity community) the sleep time in src/trajectory_sampler.cpp.(Always run catkin make after you change anything in cpp file).

3. I used rotation error for rotation matrix from joint 3 to joint 6. Which is a error in orientation of gripper as defined in URDF file of gazebo and DH parameters.

4. To reduce the cycle time I did all the calculation for tranforamtion matrices, rotation matrices outside the for loop. 
And just for fun, another example image:
![alt text][image3]

The image show the top view of the environment. The KUKA arm successfully picked and placed 8 cylindrical blocks. (The extra one shelf is for the next cycle)
![alt text][image10]

