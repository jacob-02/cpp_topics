
# QR Code Based Odometry Correction

The package helps in correction of the bot's calculated error from odometry.
It uses the pose that the QR gives to correct the errors fed to the controller.
For now the QR data is just a bool data that is generated every 1 metre if the
bot is in a 5 cm vicinity of the QR code. This can changed as the users requirement.




## Features

- Multi robot capability
- QR Code based Odometry correction
- ROS2


## Installation

Create a ROS2 workspace 

```bash
cd
cd ${WORKSPACE}/src
git clone https://github.com/jacob-02/cpp_topics/settings
cd ..
colcon build
```

## Deployment

To deploy this project run

```bash
cd 
cd ${WORKSPACE_NAME}
. install/setup.bash
roslaunch cpp_topics main.launch
```
There are multiple different nodes and their respective parameters that I have
added in the launch file. They are commented at the moment of uploading this 
package. They can be uncommented and the parameters changed as per requirement.

The `params.launch.py` and `rotate.cpp` can be ignored.

`goalNode.cpp` is used to generate the goal (x,y) positions

`linear_vb.cpp` , `new.cpp` and `old.cpp` are different editions of `virtual_bot.cpp`
They can also be ignored.

`square.cpp` is the velocity generator. It generated the velocities for both the ground
Truth Odometry and the bot's position based velocity. The bot's position based odometry
depicts the correction of the odometry after the correction from the QR code pose.



## Screenshots

The following image shows the result of the QR code based error correction. It helps in comparing the generated Odometry with the True motion of the robot.

![App Screenshot](https://github.com/jacob-02/cpp_topics/blob/master/output/image_fused.png)

## Subscribed Topics

Do ensure that the prefix is appropriately added. The parameter is `tf_prefix`

```bash
/cmd_vel
/cmd_vel_error
/qr
/x_goal
/y_goal
```

## Published Topics

```bash
/robot
/pose_updated
```

## Parameters


```bash
tf_prefix := Helps in identifying the bot in case of multi robot systems
x_pos := Initial x position of the bot
y_pos := Initial y position of the bot
a_pos := Initial a position of the bot
x_goal := Default x goal position of the bot
y_goal := Default y goal position of the bot
```


## Acknowledgements

 - [ROS2 Wiki](https://docs.ros.org/en/galactic/index.html)

