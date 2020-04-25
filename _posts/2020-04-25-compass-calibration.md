---
layout: post
title: Compass Calibration
description: I'm making progress on my outdoor robomagellan robot.
date: '2020-04-25T09:30:00.00-04:00'
author: Michael Ferguson
tags:
- robomagellan
- calibration
- robots
- ros
---

<center>
<img src="/{{ site.baseurl }}assets/images/2020-04-25-rviz.png" width="90%" />
</center>

I've made quite a bit of progress on the robomagellan robot recently. Over the past week
I've installed the Realsense D435 camera, setup a URDF, and made some serious
progress towards global localization.

##### Absolute Heading Reference
I'm using an EKF from the
[robot_localization](http://docs.ros.org/melodic/api/robot_localization/html/index.html) package
for outdoor localization. This package can merge the wheel odometry, IMU and GPS data together
into a cohesive global pose and odometry. I'll post more details about how I'm using
_robot_localization_ in a future post once I finish tuning things.

Since GPS data doesn't include heading, _robot_localization_ requires an IMU or odometry source
with absolute heading information. While some higher end IMUs may put this information out
directly, none of the IMUs I have offer a reliable absolute heading.

Today's IMUs typically include a three-axis accelerometer, gyro and magnetometer.
Most robotics people are pretty familiar with gyros and accelerometers, but less so with
magnetometers, especially if they have primarily worked on indoor mobile robots. Magnetometers
measure the geomagnetic field. If there were no disturbances and our magnetometer was
accurately calibrated, then plotting the magnitude vector while rotating the sensor around would
form a sphere with a radius equal to the local field strength. A projection of the magnitude
vector into the horizontal plane gives us the equivelent of a compass bearing.

The <code>imu_filter_madgwick</code> ROS package is used to merge the accelerometer, gyro, and
magnetometer readings into an orientation estimate which we can feed into the EKF. Unfortunately,
the geomagnetic field is not very strong so calibration is quite important. This filter is
able to take in a magnetometer bias, which corresponds to what is often called the
_hard iron_ offset [^1]. To get this bias vector I wrote a <code>magnetometer_calibration</code>
node in the [robot_calibration](https://github.com/mikeferguson/robot_calibration) package.

This calibration node rotates the robot around slowly while recording the magnetometer
values. It then tries to fit them to a sphere. It outputs the center of that sphere (which
corresponds to our _bias_ or _hard iron_ offset). It doesn't currently do the more complex
_soft iron_ calibration since there isn't a good way to merge that into the pipeline anyways.
I'm planning to implement both the _soft iron_ calibration, and the extensions to the filter
node so it can use the _soft iron_ calibration soon.

The output of the node looks something like this:

```
$ rosrun robot_calibration magnetometer_calibration
[ INFO]: Subscribing to /imu/mag
[ INFO]: Publishing to /cmd_vel
[ INFO]: Rotating robot for 60 seconds.
[ INFO]: Done rotating robot.
[ INFO]: Saving bagfile with 1337 poses.
[ INFO]: Ceres Solver Report: <truncated>
[ INFO]: mag_bias_x: 0.042075341
[ INFO]: mag_bias_y: -0.175981837
[ INFO]: mag_bias_z: -1.057499311
```

The <code>mag_bias</code> parameters can then be passed to the <code>imu_filter_madgwick</code>
as parameters (along with <code>use_mag</code> parameter being set to true). I still need to add
documentation on the new magnetometer calibration node, but the basics are:

 * Topics (remap these to your actual topics):
   * /imu/mag - the magnetometer data. Should be a sensor_msgs/MagneticField message.
   * /cmd_vel - the commands to the mobile base. geometry_msgs/Twist.
 * Parameters
   * rotation_velocity - this is the angular velocity to command. Should be relatively
     low, but has to be high enough that your robot actually rotates. Defaults to 0.2 rad/s.
   * rotation_duration - the robot has to make at least one full revolution in place. If
     your robot is really slow, this might need to be longer. Defaults to 60 seconds.

The node also has some other modes, such as manual rotation (in case your IMU isn't actually
mounted in a robot yet). Right now the best source of documentation is the code itself. I plan
to add documentation to the robot_calibration README later this week and then get a new release
pushed to debians.

##### Next Steps
I'm still working on tuning the EKF parameters. I've also started to write some GPS-waypoint
navigation code. Both of these topics will be the subject of upcoming posts.

[^1]: [NXP Application Note AN4246](https://www.nxp.com/docs/en/application-note/AN4246.pdf)
