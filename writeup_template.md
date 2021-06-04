s## Project: Kinematics Pick & Place
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


[image1]: ./misc_images/misc4.PNG
[image2]: ./misc_images/misc8.PNG
[image3]: ./misc_images/misc5.PNG
[image4]: ./misc_images/misc7.PNG
[image5]: ./misc_images/misc6.PNG
[image6]: ./misc_images/misc9.PNG

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points 
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  
The writeup / README includes a statement and supporting figures / images that explain how each rubric item was addressed, and specifically where in the code each step was handled.

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.


Run roslaunch kuka_arm forward_kinematics.launchere:![alt text][image1]


![alt text][image4]

Modified DH Parameters:

Joint | alpha(i-1) | a(i-1) | d(i) | theta(i)
--- | --- | --- | --- | ---
1 | 0 | 0 | 0.75 | q1
2 | - pi/2 | 0.35 | 0 | -pi/2 + q2
3 | 0 | 1.25 | 0 | q3
4 | - pi/2 | -0.054 | 0 | q4
5 | pi/2 | 0 | 1.5 | q5
6 | - pi/2 | 0 | 0 | q6
gripper | 0 | 0 | 0.303 | q7:0
 
alpha(i-1): twist angle, angle between axis Z(i-1) and Z(i) measured about axis X(i-1)

a(i-1): link length, distance from axis Z(i-1) to Z(i) measured along axis X(i-1)

d(i): link offset, distance from axis X(i-1) to X(i) measured along axis Z(i)

theta(i): joint angle, angle between axis X(i-1) and X(i) measured about axis Z(i)
 
![alt text][image5]
#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.


```sh
Modified DH parameters
        s = {alpha0:        0, a0:      0, d1:  0.75, q1: q1,
             alpha1: rad(-90), a1:   0.35, d2:     0, q2: q2-rad(90),
             alpha2:        0, a2:   1.25, d3:     0, q3: q3,
             alpha3: rad(-90), a3: -0.054, d4:  1.50, q4: q4,
             alpha4:  rad(90), a4:      0, d5:     0, q5: q5,
             alpha5: rad(-90), a5:      0, d6:     0, q6: q6,
             alpha6:        0, a6:      0, d7: 0.303, q7: 0
        }
```

```sh
trans_matrix(alpha, a, d, q):
    T = Matrix([[            cos(q),           -sin(q),           0,             a],
                [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                [                 0,                 0,           0,             1]])
```

```sh
individual transformation matrices:
        T0_1 = trans_matrix(alpha0, a0, d1, q1).subs(s)
        T1_2 = trans_matrix(alpha1, a1, d2, q2).subs(s)
        T2_3 = trans_matrix(alpha2, a2, d3, q3).subs(s)
        T3_4 = trans_matrix(alpha3, a3, d4, q4).subs(s)
        T4_5 = trans_matrix(alpha4, a4, d5, q5).subs(s)
        T5_6 = trans_matrix(alpha5, a5, d6, q6).subs(s)
        T6_EE = trans_matrix(alpha6, a6, d7, q7).subs(s)

        T0_EE = simplify(T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE)
```

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.
The location of the wrist center (WC) and end effector (EE) relative to the base frame "0" is given by, 0rWC/0 and 0rEE/0 , respectively. 
The location of the EE relative to the WC is given by, 0rEE/WC .

Once the first three joint variables are known, we calculate 03R via application of homogeneous transforms up to the WC.

![alt text][image6]

![alt text][image2]

### Project Implementation
#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 



I talk about the code as follows. My code guides the robot to successfully complete 9/10 pick and place cycles.  

```sh
rot_x(q):
    R_x = Matrix([[ 1,              0,        0],
                  [ 0,         cos(q),  -sin(q)],
                  [ 0,         sin(q),  cos(q)]])
    
rot_y(q):              
    R_y = Matrix([[ cos(q),        0,  sin(q)],
                  [      0,        1,       0],
                  [-sin(q),        0, cos(q)]])

rot_z(q):    
    R_z = Matrix([[ cos(q),  -sin(q),       0],
                  [ sin(q),   cos(q),       0],
                  [      0,        0,       1]])
```

```sh
R_EE, px, py, pz, roll, pitch, yaw:
    Rot_err = rot_z(rad(180)) * rot_y(rad(-90))
    R_EE = R_EE * Rot_err
    R_EE = R_EE.subs({'r': roll, 'p': pitch, 'y': yaw})

    G = Matrix([[px], [py], [pz]])
    WC = G - (0.303) * R_EE[:, 2]
    theta1 = atan2(WC[1], WC[0])

    a = 1.501 # Found by using "measure" tool in RViz.
    b = sqrt(pow((sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35), 2) + \
        pow((WC[2] - 0.75), 2))
    c = 1.25 # Length of joint 1 to 2.
    alpha = acos((b*b + c*c - a*a) / (2*b*c))
    beta = acos((a*a + c*c - b*b) / (2*a*c))
    delta = atan2(WC[2] - 0.75, sqrt(WC[0]*WC[0] + WC[1]*WC[1]) - 0.35)
    theta2 = pi/2 - alpha - delta

    epsilon = 0.036 
    theta3 = pi/2 - (beta + epsilon)
```

```sh
r, p, y = symbols('r p y')
            R_x = rot_x(r)
            R_y = rot_y(p)
            R_z = rot_z(y)

            R_EE = R_z * R_y * R_x
            R_EE, WC, theta1, theta2, theta3 = calculate_123(R_EE, px, py, pz, roll, pitch, yaw)

            R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]
            R0_3 = R0_3.evalf(subs={q1:theta1, q2:theta2, q3:theta3})
            
            R3_6 = R0_3.inv("LU") * R_EE

            theta6 = atan2(-R3_6[1,1], R3_6[1,0])# +0.45370228
            sq5 = -R3_6[1,1]/sin(theta6)
            cq5 = R3_6[1,2]
            theta5 = atan2(sq5, cq5)
            sq4 = R3_6[2,2]/sin(theta5)
            cq4 = -R3_6[0,2]/sin(theta5)
            theta4 = atan2(sq4, cq4)
```

![alt text][image3]


