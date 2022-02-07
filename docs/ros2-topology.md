# ROS2 topology

In this file is descripted the topology of the ROS2 nodes and the topics involved in the project.

### Naming convention

**[NODE]** *node_name*: description
- **pub/sub** *ROS2Interface* ```topic_name```

**[INTERFACE]** *InterfaceName*: description
- *param_type* ```param_name```

**[LAUNCH]** *launch_file*

# Graph overview

The following graph shows a general overview of the ROS2 **nodes** (circled) and **topics** (boxed).

![image](ros-topology-graph.svg)


## [PKG] alphabot2

**[NODE]** *motion_driver*: motors and movement management
- **sub** *Twist* ```cmd_vel```
- **sub** *Obstacle* ```obstacles```

**[NODE]** *IR_obstacle_sensors*: IR sensors management
- **pub** *Obstacle* ```obstacles```

**[NODE]** *virtual_odometer*: fake speed information (the one published on cmd_vel)
- **sub** *Twist* ```cmd_vel```
- **sub** *Obstacle* ```obstacles```
- **pub** *Twist* ```virtual_odometry```

**[NODE]** *QR_detector*: QR detection
- **sub** *CompressedImage* ```image_raw/compressed```
- **pub** *String* ```qr_codes```

**[LAUNCH]** *alphabot2_launch*

## [PKG] alphabot2_interfaces

**[MSG]** *Obstacle*: IR obstacle sensors status
- *bool* ```left_obstacle```
- *bool* ```right_obstacle```


# TODO nodes/interfaces

**[NODE]** *IR_line_tracking_sensor*: IR line tracking sensors management
- **pub** *LineTrackingStatus* ```line_tracking```

**[NODE]** *joystick*: onboard 4-way joystick management
- **pub** *String* ```joystick_position```

**[NODE]** *joystick_teleop*: onboard 4-way joystick robot controller
- **sub** *String* ```joystick_position```
- **pub** *Twist* ```cmd_vel```

**[NODE]** *IR_remote*: IR remote management
- **pub** *String* ```pressed_key```

**[NODE]** *IR_remote_teleop*: IR remote robot controller
- **sub** *String* ```pressed_key```
- **pub** *Twist* ```cmd_vel``` 
- should control also the cam pan/tilt

**[NODE]** *rgb_leds*: LEDs management
- **sub** *RGBLedsController* ```rgb_leds_controller```

**[NODE]** *rgb_leds_controller* LEDs controller
- **pub** *RGBLedsController* ```rgb_leds_controller```

**[NODE]** *speaker*: speaker management
- **sub** *Sound* ```play_sound```

**[NODE]** *speaker_controller*: speaker controller
- **pub** *Sound* ```play_sound```

**[MSG]** *LineFollow*: IR line sensors status
- *int32* ```left_outer```
- *int32* ```left_inner```
- *int32* ```centre```
- *int32* ```right_inner```
- *int32* ```right_outer```

**[MSG]** *PanTilt*: camera pan/tilt status
- *float32* ```pan_angle``` (-180 < angle < +180)
- *float32* ```tilt_angle```


