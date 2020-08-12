---
layout: post
title: Some Notes on Cyclone DDS
description: I've been working with Cyclone DDS lately.
date: '2020-08-12T12:00:00.00-04:00'
author: Michael Ferguson
tags:
- robots
- ros2
---

One of the biggest differences between ROS1 and ROS2 is the
replacement of the single middleware with a plugin-based architecture.
This allows ROS2 to use various Robotic Middle Ware (RMW) implementations.
All these RMW implementations are currently based on DDS. You can read all about
the details in the
[ROS2 Design Docs](https://design.ros2.org/articles/ros_middleware_interface.html).

Over time, the supported RMW implementations have shifted and new ones have
been introduced. The default is currently <code>FastRTPS</code> (which
apparently has been renamed to FastDDS, but after the Foxy release).
The newest option is <code>CycloneDDS</code> which uses
[Eclipse Cyclone DDS](https://projects.eclipse.org/projects/iot.cyclonedds).
Cyclone DDS has gotten a lot of
[praise](https://rosindustrial.org/news/2020/7/8/developing-a-ros2-collaborative-industrial-scan-n-plan-application)
lately, so let's take a closer look.

<center>
<img src="/{{ site.baseurl }}assets/images/2020-08-12-cyclone.png" width="40%" />
</center>

## RMW Implementations

Choosing between RMW implementations is still a bit of a challenge since ROS2
is still very much under active development. There are
[multiple](https://github.com/ros-planning/navigation2/issues/1772)
[tickets](https://github.com/ros2/rmw_fastrtps/issues/392)
[about](https://github.com/ros2/ros2/issues/931)
FastDDS service discovery issues. CycloneDDS is less than two years old,
which means it is still under very active development and might not be
fully featured, but it is supposed to be really highly performant.
Mixing multiple implementations at runtime has
[noted caveats](https://index.ros.org/doc/ros2/Tutorials/Working-with-multiple-RMW-implementations/#multiple-rmw-implementations).

Luckily, it's very easy to switch between implementations by simply setting
the <code>RMW_IMPLEMENTATION</code> environment variable (assuming the selected
implementation is built/installed).

When switching between implementations, be sure to stop the
<code>ros2 dameon</code> so that it gets restarted with the proper RMW
implementation:

    ros2 daemon stop

First you've heard of the ROS2 daemon? Check out
[this ROS Answers post](https://answers.ros.org/question/327348/what-is-ros2-daemon/)
which contains the best description I've seen.

## Debugging Issues

While FastDDS was mostly working out of the box, the whole service problem
was wreaking havoc on setting/getting parameters -- and I've been tuning
parameters frequently. I went ahead and set the RMW_IMPLEMENTATION to
<code>rmw_cyclonedds_cpp</code>, or so I thought.

I noticed that service discovery wasn't much better. Then I noticed on the
robot I had set RMW_IMPLEMTATION - so I fixed the spelling mistake. Now
everything should totally work great!

Wrong.

On the robot, discovery worked fine and services worked great - but half
or more of the nodes couldn't be seen by my laptop. Restarting launch
files resulted in different nodes often missing!

I started to debug and came across the <code>ddsperf</code> tool. If you're
using [ROS2 on MacOSX]({% post_url 2020-08-10-rviz2-on-mac %})
you'll want to check out this issue on
[how to install ddsperf](https://github.com/ros2/rmw_cyclonedds/issues/216).

## Multiple Network Interfaces

Running <code>ddsperf sanity</code> gave an interesting warning on the robot:

```
ddsperf: using network interface enp3s0 (udp/10.42.0.1) selected arbitrarily from: enp3s0, wlp2s0
```

The UBR-1 has two network interfaces: wlp2s0 is a wifi connection to the
outside world and enp3s0 is an internal ethernet port which only talks to
the robot hardware. Apparently, my nodes were frequently using the wrong
network interface. The upstream Cyclone DDS [README](https://github.com/eclipse-cyclonedds/cyclonedds)
does mention, way down the page, that "proper use of multiple network interfaces
simultaneously will come, but is not there yet."

The configuration guide states that the selection of network adapter prefers
non-link-local interfaces, but apparently something is tripping it up in detecting
that the ethernet interface is configured that way.

The work around is to set a <code>NetworkInterfaceAddress</code> in the
<code>CYCLONEDDS_URI</code> environment variable:

```
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><NetworkInterfaceAddress>wlp2s0</NetworkInterfaceAddress></General></Domain></CycloneDDS>'
```

If you're prone to typos, and want to make sure you're actually running the
expected RMW interface, I'd recommend this command:

```
ros2 doctor --report | grep middleware
```

After a few seconds, you should see:

```
middleware name    : rmw_cyclonedds_cpp
```

I actually setup an alias in my bashrc so that <code>which_rmw</code> runs
that command. Once I settled on using Cyclone DDS as my new default, I also
added the <code>RMW_IMPLEMENTATION</code> and <code>CYCLONEDDS_URI</code>
settings to the bashrc on the robot.

## Final Thoughts

Once I worked through the configuration issue, CycloneDDS appears to be the
most stable of the few RMW implementations I've tried. I haven't actually
tested the performance head-to-head, but
[others](https://github.com/eclipse-cyclonedds/cyclonedds#performance)
[have]().

I would recommend looking at the
[Configuration](https://github.com/eclipse-cyclonedds/cyclonedds#configuration)
section of the upstream Eclipse CycloneDDS project. This contains a bunch
of useful information about what you can specify in the CYCLONEDDS_URI. The
[Guide to Configuring](https://github.com/eclipse-cyclonedds/cyclonedds/blob/master/docs/manual/config.rst)
is also very worth reading. It's honestly a great resource for simply understanding
all those things you hoped you'd never need to learn about DDS.
