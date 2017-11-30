# i5 robot

[ROS-Industrial](http://wiki.ros.org/Industrial) i5 robot metapackage.

this repository contains ROS packages which are fully compatible with the [ROS-Industrial](http://wiki.ros.org/Industrial) spec.

Now, we only tested on ubuntu 16.04(Xenial) with ros kinetic.

**i5_driver** is a package that contains some i5 specified ros msgs and srvs.
**i5_driver** is a package that communicate to the controller and convert to ros standard communication.
**i5_resources** is a package that defines some command urdf macro
**i5_a3_support** is a package that contains a3 robot(an 3 kilogram 6 joint industrial manipulator) urdf and mesh.
**i5_a3_ik_fast** is a package that contains ik fast kinematics algorithm for a3 robot.
**i5_a3_moveit_config** is a package that contains moveit config for a3 robot.


## How to install
As a ros industrial package, you need install ROS && MoveIt && ROS Industrial firstly.

Install Steps:

1. Install ROS follow [this](http://wiki.ros.org/kinetic/Installation/Ubuntu) page

2. Install MoveIt
`sudo apt install ros-kinetic-moveit*`
3. Install ROS Industrial
`sudo apt install ros-kinetic-industrial*`	
4. Install i5_robot
To install i5_robot packages, you need firstly create a catkine workspace.You can do it follow [this](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment) page.
Once you create a cakin workspace(named *catkin_ws*), you can checkout the i5_robot src and compile it.
```
cd ~/catkin_ws/src
git clone git@github.com:i5cnc/i5_robot.git -b kinetic-devel
cd ~/catkin_ws
catkin_make
```
Note: remember source your catkin_ws setup file before you compile:
```
source ~/catkin_ws/devel/setup.bash
```



## How to run (use a3 robot as an example)

***Respect Robot Safety Practices***

***WARNING: INDUSTRIAL ROBOTS ARE VERY DANGEROUS AND CAN SERIOUSLY INJURE OR KILL. THE ROS-INDUSTRIAL SOFTWARE IS PROVIDED UNDER THE TERMS OF THE BSD LICENSE (I.E. AS-IS AND WITH NO WARRANTY). WHEN OPERATING AN INDUSTRIAL ROBOT UNDER ROS-INDUSTRIAL CONTROL, MAKE CERTAIN THAT NO ONE IS WITHIN THE ROBOT WORKSPACE AND THE E-STOP IS UNDER OPERATOR CONTROL.***

Steps:

1. Power on and enable the robot.
you can power on the robot and enable it by the teach pandent(following the operation manual),
or from ROS side (refer to the i5_driver README)

2. In ros pc, run (take moveit as an example):
`roslaunch i5_a3_moveit_config moveit_planning_execution.launch sim:=false robot_ip:=your_robot_ip`

3. Change the robot to`playback`or`remote`mode by robot teach pandent or from ROS side (refer to the i5_driver README). **(Remember you must switch the robot to `playback` or `remote` mode not 'teach' mode, because teach is used for manual jog)**

4. Then, you can try to plan and execute in rviz and robot will move follow the trajectory.


Note: 

1. Remember to source your catkin_ws setup file before you run roslaunch:
```
source ~/catkin_ws/devel/setup.bash
```
2. Do not try to manipulate the robot from teach pendant and ros pc simultaneously (for example, sending trajcectory from ros and execute a program from teach pandent simultaneously).

