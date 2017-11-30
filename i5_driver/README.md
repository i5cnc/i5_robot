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
Note: **download** is not implemented yet.

#io
the node is **io_service**
Currently, io provides two services:
- WriteDigitalOutput
- ReadDigitalInput

#system
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




#state
the node is **robot_state**
According to ros indsutrial spec, all the following topics are provided
- feedback_states
- joint_states
- robot_status












