# 3R Planar Robot For 2D Shape Drawing 
# Project Overview
The project serves as a demonstration of [Newton-Raphson inverse kinematics algorithm](https://en.wikipedia.org/wiki/Newton%27s_method). In this project, any shape within its workspace of the 3R arm can be drawn. An example curve, [the heart curve](http://mathworld.wolfram.com/HeartCurve.html), is stored in the parameter server. You can put in any parametric functions
of a single continuous shape, please see section **Changing the Shape To Draw** for more details.
![Screenshot from 2019-12-31 14-09-40](https://user-images.githubusercontent.com/39393023/71637676-7efe9800-2c0e-11ea-93e2-1d50e41b6b7a.png)


**The design of the robot**

![Screenshot from 2019-12-30 21-18-44](https://user-images.githubusercontent.com/39393023/71608911-0c2be900-2b4a-11ea-8b23-0e264bd6ae2c.png)


**The workspace of the robot**


![Screenshot from 2019-12-30 16-21-04](https://user-images.githubusercontent.com/39393023/71603023-952e2a80-2b20-11ea-8494-e804d4c4b106.png)


## Demonstration
Please checkout my youtube channel for more details!

[![Screenshot from 2019-12-31 22-37-02](https://user-images.githubusercontent.com/39393023/71638164-20411a80-2c1e-11ea-85ce-6b6454e343a2.png)](https://youtu.be/TwYZKQe96Wo)


## How to Use This Code
- To install this package, on your local computer, Run 
```
$ mkdir -p robot_arm_simulations/src
$ cd robot_arm_simulations/src
$ git clone git@github.com:RicoJia/Robot_Arm_Simulations.git
$ cd ..
$ catkin_make 
```
-  To see visualize the trajectory on Rviz using marker, run

```
$ roslaunch robot_arm_simulation robot_arms_marker_included.launch
```

For showing the robot's movement only, run
```
$ roslaunch robot_arm_simulation robot_arms_marker_included.launch marker_status:=false
```


## Code Overview
##### 2D Newton Raphson Inverse Kinematics Algorithm

This part was built for 2D planar inverse kinematics problem, using the [screw](https://en.wikipedia.org/wiki/Screw_theory) representation of robot joints. 
The inputs to this part are:
1. M - Home position of the robot (when all joint angles are zero)
2. blist - Joint Screw axes in the end effector body frame
3. Tsd - Desired end effector transformation in Lie Group SE(2)
4. thetalist0 - Current joint angles
5. e_theta - Orientation error tolerance between the end result and the desired end effector transformation Tsd. Default value is 0.01rad
6. e_v - Distance Error tolerance between the end result and the end effector transformation Tsd. The default value is 0.001

 the board) is set by using a simple P controller as follows:

The outputs from this part are:
1. success - True if a solution is found, otherwise false
2. thetalist_new - Joint angles from the furthest joint to the closest joint to the end effector

##### Changing the Shape to Draw

To change the shape to draw, simply go to ```robot_arm_simulation``` file under
```src``` directory. Find **TRAJECTORY_X** and **TRAJECTORY_Y**, and put your full list of trajectory
points as a numpy array there. 
