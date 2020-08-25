---
layout: post
title: Porting EtherbotiX to ROS2
description: Maxwell and the Robomagellan are making progress.
date: '2020-08-25T09:00:00.00-04:00'
author: Michael Ferguson
tags:
- maxwell
- robomagellan
- robots
- ros2
---

<center>
<img src="/{{ site.baseurl }}assets/images/port_all_the_robots.jpg" width="70%" />
</center>

Now that the UBR-1 is running pretty well under ROS2, I have started to also port
<a href="/{{ site.baseurl }}tag/maxwell/">Maxwell</a>
and the <a href="/{{ site.baseurl }}tag/robomagellan/">Robomagellan robot</a>
to ROS2. Both depend on my
[etherbotix drivers](https://github.com/mikeferguson/etherbotix) package,
so that is the first package to port.

## Etherbotix in ROS2

In ROS1, the <code>etherbotix_python</code> package was written in Python and
leveraged some code from the <code>arbotix_ros</code> package (mainly for the
controllers). I decided while porting things to ROS2 that I would migrate to using
C++ and leverage the [robot_controllers](http://github.com/fetchrobotics/robot_controllers)
which I had [recently ported]({% post_url 2020-06-11-ubr1-on-ros2 %}) to ROS2.

Since this was effectively a new package, I used the
[ros2 pkg create](https://index.ros.org/doc/ros2/Tutorials/Creating-Your-First-ROS2-Package/#create-a-package)
command to setup the CMakeLists.txt, package.xml and other boiler plate stuff.

I then started to setup the node based off my very old
[asio_ros](https://github.com/mikeferguson/sandbox/tree/hydro-devel/asio_ros) example.
At some point I should probably setup a test to see how accurate ROS2 timers are,
but I knew for sure that this code would work so I stuck with <code>boost::asio</code>.

## Python Wrappers

In ROS1 I had a number of scripts for interacting with the Etherbotix. For some
of these, such as <code>read_etherbotix</code>, it was easy to port them to C++.
For others, such as my motor_trace script which uses matplotlib, I really wanted
to keep the majority of the script in Python. To accomplish this, I wrapped my
underlying C++ drivers using Boost Python.

It required a bit of CMake:

```cmake
find_package(PythonLibs REQUIRED)
find_package(Boost REQUIRED python)

ament_python_install_package(${PROJECT_NAME})

add_library(etherbotix_py SHARED ...)
set_target_properties(etherbotix_py PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}
  PREFIX "")
target_link_libraries(etherbotix_py
  ${Boost_LIBRARIES}
  ${PYTHON_LIBRARIES}
)
ament_target_dependencies(etherbotix_py ...)

install(
  TARGETS etherbotix_py
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
)
```

I then created a file <code>etherbotix/__init__.py</code>:

```python
from etherbotix.etherbotix_py import Etherbotix
```

This allowed me to import my C++ class Etherbotix in my scripts with:

```python
from etherbotix import Etherbotix
```

Why Boost Python? Aren't there newer things out there? Yes, there are newer things
out there, and I spent a while trying out <code>pybind11</code> but I just couldn't
get it to work and reverted to what I knew already. 

## Exporting Libraries

Most of the ROS2 code I've ported has thus far been various nodes, rather than a
library that will be used by other packages (the one exception being
<code>robot_controllers_interface</code>). I hadn't previously paid super close
attention to how things are exported. There are a few declarations that get
placed at the end of your <code>CMakeLists.txt</code>:

```cmake
# Tell downstream packages where to find our headers
ament_export_include_directories(include)
# Tell downstream packages our libraries to link against
ament_export_libraries(my_library)
# Help downstream packages to find transitive dependencies
ament_export_dependencies(
  rclcpp
)
ament_package()
```

This code snippet is from my
[ros2_cookbook](https://github.com/mikeferguson/ros2_cookbook/blob/main/pages/cmake.md#ament)
project on GitHub. You can also find the
[full commit](https://github.com/mikeferguson/etherbotix/commit/2c31281bbc1c0ba5014ee30fddf98d05599baa2b)
for enabling downstream packages to build upon the library exported by the
Etherbotix library.

## Maxwell Bringup

So why did I have to export the Etherbotix libraries? I had to write a custom controller
for the torso on Maxwell, and that controller had to access an Etherbotix instance. This
involved a bit of custom code to add both a custom _JointHandle_ and custom _Controller_.
The controller also automatically loads the custom _JointHandle_.

One of the advantages of using <code>robot_controllers</code> is that several controllers
worked out of the box. I had never actually updated the old Python code to work with
Maxwell's parallel jaw gripper, but with the _ParallelGripperController_ and a
_ScaledMimicController_ for the second finger, everything worked out of the box.

The controllers for Maxwell are now all setup. You can see the configuration
[here](https://github.com/mikeferguson/maxwell/blob/ros2/maxwell/config/etherbotix.yaml).

## Some Fancy-ish C++

Throughout the driver code we have to construct buffers of various bytes to send
to the hardware - often involving arrays of bytes of varying length. This is
generally really clean in Python, but in C++ usually results in something like this:

```cpp
uint8_t len = 0;
buffer[len++] = 0xff;
buffer[len++] = 0xff;
buffer[len++] = Etherbotix::ETHERBOTIX_ID;
buffer[len++] = 5;  // Length of remaining packet
buffer[len++] = dynamixel::WRITE_DATA;
if (motor_idx == 1)
{
  buffer[len++] = Etherbotix::REG_MOTOR1_VEL;
}
else
{
  buffer[len++] = Etherbotix::REG_MOTOR2_VEL;
}
buffer[len++] = (desired_velocity_ & 0xff);
buffer[len++] = (desired_velocity_ >> 8);
buffer[len++] = dynamixel::compute_checksum(buffer, 9);
```

I decided to come with a cleaner approach, since there are quite a few
instances of this throughout the code base. I ended up creating a
<code>get_write_packet</code> function like I had in the Python code, with this
signature:

```cpp
inline uint8_t get_write_packet(
   uint8_t* buffer,
   uint8_t device_id,
   uint8_t address,
   std::vector<uint8_t> params)
```

And then using an _initializer list_ to create the variable-size buffers of
bytes to send:

```cpp
uint8_t len = dynamixel::get_write_packet(
  buffer,
  Etherbotix::ETHERBOTIX_ID,
  (motor_idx == 1) ? Etherbotix::REG_MOTOR1_VEL : Etherbotix::REG_MOTOR2_VEL,
  {static_cast<uint8_t>(desired_velocity_ & 0xff),
   static_cast<uint8_t>(desired_velocity_ >> 8)}
);
```

Yes, this probably is not the fastest code (since it passes the vector of
bytes by-value), but I like how it cleans up the code and none of these
vectors are all that large. You can see the
[full commit on GitHub](https://github.com/mikeferguson/etherbotix/commit/f814e60cdd3e3e7f2794dbd27d18531325a2e600)

## Diagnostics

The final part of my <code>etherbotix</code> port was to
[add diagnostics](https://github.com/mikeferguson/etherbotix/commit/9e23386123fc0034ef24c4cedc21a4a6eb93623e)
back into the node. <code>diagnostic_msgs</code> are an underused feature in ROS.
They offer a common way to send information about the status of things, mostly
hardware. Drivers like [urg_node](https://github.com/ros-drivers/urg_node), the
[joy node](https://github.com/ros-drivers/joystick_drivers), and even higher level
filters like [robot_localization](https://github.com/cra-ros-pkg/robot_localization)
publish diagnostics.

While diagnostics can be passed through analyzers to output a <code>diagnostics_agg</code>
topic, I often just use the <code>rqt_runtime_monitor</code> which access the raw
diagnostics topic. I found it was missing in ROS2 - but there is a port, which
hasn't been merged yet. You can
[find that port here](https://github.com/ros-visualization/rqt_runtime_monitor/pull/5).

## Next Steps

I've made some major progress on running navigation2 on the UBR-1 and that will be the
subject of a post next week. After that I'll be continuing on bringup of the robomagellan
robot, including integrating <code>robot_localization</code> in ROS2.
