# i5 driver
Driver for robot controller and ros connection.
Connection services are classified into four types:
- **motion** provide motion control interface
- **io** provide io control interface 
- **system** provide some system level interface.(enable, disable, etc.)
- **state** provide the robot and joint state interface.

# motion
the node are **motion_streaming_interface**
Based on the ros industrial spec, two types motion control interface are provided, **stream** and **download**.
**download** will send the trajectory point-by-point to the controller, then execute all at once. If a new download is sending to the controller while an existing downloaded motion is running in the controller,the existing motion will be cancelled and the new motion will be executed.
**stream** will send individual point to the controller and execute immediately. Sending more points will be buffered in the controller.

Note: now for i5 robot controller, **download** is not supported.

For the motion interface, the **time parameterization** is an important concept.
Time parameterization usually means adding time information associated with the path.


i5 robot controller support the following time parameterization.
- **PT** set the joint position and time duration.(the velocity and acceleration will be figured out by the controller itself)
- **PVAT** set the joint position,velocity,acceleration and time duration.

The ROS Industrial simple message protocol has two build-in msg type:**JointTrajPtMessage** and **JointTrajPtFullMessage**

The **JointTrajPtMessage** contains time,position and velocity data.
The **JointTrajPtFullMessage** contains time,position,velocity and acceleration data.

the node **motion_streaming_interface** in the **industrial_robot_client** package use **JointTrajPtMessage** to communicate with robot. it internally implmented a simple time parameterization of path.

the node **motion_streaming_interface** in the **i5_driver** package use **JointTrajPtFullMessage** to communicate with robot. no time parameterization is implemented in this node. It only receives **trajectory_msgs/JointTrajectory** and fullfill **JointTrajPtFullMessage** with its data. 

So user must implement the time parameterization by themselves.

The launch file **motion_streaming_interface.launch** has an arg called **use_pt_full**, if this arg set to true, the node  **motion_streaming_interface** in the **i5_driver** package will be used, otherwise the node **motion_streaming_interface** in the **industrial_robot_client** package will be used.

For example, MoveIt implemented its own time parameterization.So when user use MoveIt with i5 robot, you can directly use **motion_streaming_interface** in the **i5_driver** package.

But for some applications, if user only knows the path points and duration time,then only **motion_streaming_interface** in the **industrial_robot_client** package can be used.

# io
the node is **io_service**
Currently, io provides two services:
- WriteDigitalOutput
- ReadDigitalInput


# system
the node is **system_service**
Currently, system provides one service:
- ExecuteApplicationCmd
| comand | action |
|--------|--------|
| 0 | servo on the robot |
| 1 | enable robot |
| 2 | disable robot |
| 3 | change the robot mode to `playback` |
| 4 | change the robot mode to `teach` |
| 5 | change the robot mode to `remote` |
| 6 | stop current motion |
| 7 | do a reset, used for error reset |


# state
the node is **robot_state**
According to ros indsutrial spec, all the following topics are provided
- feedback_states
- joint_states
- robot_status












