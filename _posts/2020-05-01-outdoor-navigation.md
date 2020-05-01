---
layout: post
title: Outdoor Global Localization
description: I've managed to get some decent results for outdoor localization of the robomagellan robot.
date: '2020-05-01T9:00:00.00-04:00'
author: Michael Ferguson
tags:
- robomagellan
- robots
- ros
---

I've been making steady progress on the robomagellan robot. In my previous post, I detailed
how I calibrated the magnetometer on the robot in preparation for using the _robot_localization_
package to fuse the wheel odometry, IMU, and GPS data. This post discusses the actual use of
_robot_localization_.

###### Coordinate Frames
Before diving into how the robomagellan robot is localizing, we should explore what the
coordinate frames look like. [REP 105](https://www.ros.org/reps/rep-0105.html) specifies
the standard conventions for mobile robot coordinate frames in ROS.

For an indoor robot, the coordinate frames are pretty straight-forward. The "center" of the robot
is _base_link_. Your odometry source (typically wheel encoders, often merged with an IMU) is used
to generate an _odom_ frame. Then you have a map of the building and a program such as
[AMCL](http://wiki.ros.org/amcl) can use laser scan data to align the robot with the map,
publishing a correction in the form of a transformation from _odom_ to _map_ frame. The _map_ frame
is arbitrary, but fixed in reference to the map of the building and set at the time the map was
built. Goal poses are typically specified in the _map_ frame, since it is consistent over time
and over reboots of the robot.

For an outdoor robot, it ends up being more complex. The _base_link_ and _odom_ frames are the
same as they were indoors. The _map_ frame origin is less well defined. REP-105 states:

```
Map coordinate frames can either be referenced globally
or to an application specific position.
```

With _robot_localization_, the _map_ frame origin is wherever the robot odometry started. That
is a bit different for those of us who mainly use indoor robots where the _map_ frame is
consistent from reboot to reboot.

A pair of ROS services are offered which can convert between GPS coordinates and _map_ coordinates.
Internally, these services track the location of the _map_ frame in UTM coordinates. The
[Universal Transverse Mercator (UTM)](https://en.wikipedia.org/wiki/Universal_Transverse_Mercator_coordinate_system)
system splits the world up into a series of coordinate grids. These grids are very large and
so you don't often want to do your calculations in UTM coordinates, hence the local _map_ frame
which is located where ever the robot started up. There is an option to publish the _utm_ frame,
but it appears rarely used.

###### Global Localization
"Global" localization is pretty easy with indoor mobile robots based on ROS since it really
just consists of finding the robot pose in the _map_ frame. You simply merge an
odometry source with your laser scan and a map using _AMCL_. There are quite a few parameters,
but the defaults work pretty well out of the box. Things get a bit more complicated when you go
outdoors.

People often improve their odometry source by merging the wheel encoder odometry with an IMU.
The [robot_localization](http://docs.ros.org/melodic/api/robot_localization/html/index.html) package
offers an Extended Kalman Filter (EKF) that can do this for both indoor and outdoor robots.

While the EKF does not take GPS data directly, the _robot_localization_ package also offers
the <code>navsat_transform_node</code> that can convert GPS data into an odometry stream.
The node subscribes to your _gps/fix_ topic and then outputs an odometry source that encodes
the robot pose in the _map_ frame. Internally, it tracks the transformation between the UTM
coordinates and the _map_ frame.

###### Navsat Transform Node
The <code>navsat_transform_node</code> also subscribes to two other topics, your IMU and the
odometry output from the EKF. The node only needs this data until it gets a first GPS fix.
The IMU part is easy to understand - GPS does not contain heading information and so the
node needs to get that from the IMU in order to determine the transformation between UTM and
_map_.

The circular subscription (that the transform node is subscribing to the odometry output by
the EKF, and the EKF is subscribing to the odometry output by the transform node) is probably
the least understood aspect in _robot_localization_ -- probably half of all the related
questions on [answers.ros.org](http://answers.ros.org) are about this. The reasoning is as such:
since the _map_ frame is located where the robot _started_, we need to know how far we have
traveled from the start when we finally get a first GPS fix. If you don't move the robot at
all before you get a valid fix, you really wouldn't need this odometry source in
<code>navsat_transform_node</code>.

In setting up the launch files for the EKF, I specifically annotated the somewhat confusing
subscription and publication topics. My
[launch file](https://github.com/mikeferguson/robomagellan/blob/master/launch/compute/ekf_global.launch.xml)
for the EKF pipeline basically looks like this:

{% highlight xml %}
  <!-- Global frame localization -->
  <node name="ekf_global_odom" pkg="robot_localization" type="ekf_localization_node" >
    <rosparam command="load" file="$(find robomagellan)/config/ekf_global.yaml" />

    <!-- Subscriptions (in yaml)
      odom0: odometry/gps
      odom1: base_controller/odom
      imu0:  imu/data
    -->

    <!-- Publications -->
    <remap from="odometry/filtered" to="odometry/global" />
  </node>

  <!-- Integrating GPS -->
  <node name="navsat_transform_node" pkg="robot_localization" type="navsat_transform_node" output="screen" >
    <!-- Parameters truncated - see full file on GitHub -->

    <!-- Subscriptions -->
    <remap from="imu/data" to="imu/data" />
    <remap from="gps/fix" to="gps/fix" />
    <remap from="odometry/filtered" to="odometry/global" />

    <!-- Publications -->
    <remap from="gps/filtered" to="gps/filtered" />
    <remap from="odometry/gps" to="odometry/gps" />
  </node>
{% endhighlight %}

##### Setting Up Robot Localization
Parts of the EKF setup are pretty straight forward. The _odometry/gps_ source that comes from
the <code>navsat_transform_node</code> gives us an absolute estimate of _x_ and _y_ coordinates
derived from the GPS data. The _base_controller/odom_ source gives us differential estimates of
_x_, _y_ and _yaw_ derived from the wheel encoders.

How to fuse the IMU data is less intuitive. Since the IMU is processed by the
<code>imu_filter_madgwick</code> node, we know it can give an absolute estimate of _roll_, _pitch_,
_yaw_. The absolute orientation is quite important, since no other sensor in our system can
give us this important information. The filter also removes bias from the gyro and accelerometers,
so we can use the IMU data for differential estimates of _roll_, _pitch_, and _yaw_ as well as
accelerations of _x_, _y_, and _z_. I chose not to use the accelerations though, since it seems
to give better results.

There are numerous notes and warnings that if you are merging two sources of rotation/orientation
data you have to make sure the covariances are well set. While I'm not 100% confident in the
covariance of the wheel encoder odometry source, it seems to be merging fine with the IMU.

<center>
<img src="/{{ site.baseurl }}assets/images/2020-05-01-map.png" width="90%" />
</center>

The above image shows localization during a test run. The red dots are the raw GPS data. The blue
dots are the output of the EKF. I'm downsampling here to 1 meter spacing between dots.
It's worth noting that the robot actually stayed on the gravel-colored paths, so the raw GPS is
pretty far off (especially the track on the left that swings way out into the grass). The
datasheet for the MTK3339-based GPS states that the positional accuracy is 2.5 meters (50% CEP),
which is quite a bit worse than the Garmin 18x referenced in the robot_localization paper.

At this point, I have decent global localization for how bad the GPS signal is.
The next step is going to be replacing the GPS module with one that supports Real Time Kinematics
(RTK). The cost of these modules have come down a great deal and so it makes total sense to upgrade
to this. There really aren't any publicly accessible RTK base stations here in New Hampshire, so
I'll also be setting up a base station.

Even while I wait for the new GPS modules to show up, I plan to make some progress on navigation
since the localization is still pretty decent within the map frame, and I can temporarily shift
the goal poses for testing.
