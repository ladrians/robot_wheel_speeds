# ROS Wheel Speeds
ROS package originally based on [Jonny Dark's](https://github.com/jonnydark/ros_wheel_speeds) work.

It reads rotary encoders and converting to wheel speeds for a Raspberry Pi based differential drive robot.

Depends on [wiringPi](http://wiringpi.com/) to build

Publishes a message containing individual wheel speeds as well as Unicycle Model linear and angular velocities

Tested with ROS Indigo and Kinetic.

## Sample usage

```
<node pkg="robot_wheel_speeds" type="robot_wheel_speeds" name="wheel_speeds" respawn="true" output="screen">
  <param name="test_mode" value="$(arg test_mode)" />
  <rosparam file="$(find robot_wheel_speeds)/param/wheel_params.yaml" command="load" />
</node>
```

Check the `wheel_params.yaml` file to tune your robot.

## Troubleshooting

When building, an error detailing that the `WheelVelocities.h` is missing may appear.
Compile first this package and then the whole workspace.


```
catkin_make --pkg robot_wheel_speeds
```
