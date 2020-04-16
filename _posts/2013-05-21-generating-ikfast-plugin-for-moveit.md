---
layout: post
title: Generating an IKFast Plugin for MoveIt
description: How to setup ikfast for a better inverse kinematics on your robot.
date: '2013-05-21T17:37:00.001-04:00'
author: Michael Ferguson
tags:
- moveit
- maxwell
- robots
- ros
modified_time: '2013-05-30T14:51:12.534-04:00'
blogger_id: tag:blogger.com,1999:blog-8995060597167955455.post-7602906199008063605
blogger_orig_url: http://www.showusyoursensors.com/2013/05/generating-ikfast-plugin-for-moveit.html
---

Over the past few weeks I've been using MoveIt quite a bit -- I was actually using the Pick and
Place actions for Maxwell's chess playing last weekend at Maker Faire (an updated on that
shortly).

I recently changed over to using IKFast for IK under MoveIt. This fixes a number of problems
with the default IK plugin. I'm using ROS Groovy under Ubuntu Precise, and ran into several
issues which I thought I would mention here.

First off, I start with the excellent [moveit_ikfast_generator](https://github.com/ros-planning/moveit_ikfast)
from Dave Coleman. You should run his instructions for _"Create Collada File for Use with
OpenRave"_. Once you have a collada file, here is what I did to make the ikfast generator
work:

```
sudo apt-add-repository ppa:openrave/release
sudo apt-get update
sudo apt-get install openrave0.8-dp-ikfast
```

At this point, I had to edit a _/usr/lib/python2.7/dist-packages/openravepy/\_\_init\_\_.py_ to
add the following line just after the copyright:

```
__openravepy_version__ = "0.8"
```

Without this, nothing seems to run. Once the code is updated, the tools listed in
[section 5 of the ROS Industrial tutorial](http://www.ros.org/wiki/Industrial/Tutorials/Create_a_Fast_IK_Solution)
work as indicated.

Then it's back to the README in moveit_ikfast_generator, where the instructions will
walk you through generating the ikfast plugin. One note there is also a command for:

```
openrave-robot.py my_robot.dae --info joints
```

which is very helpful if you have a 7-dof actuator and need to fill in the --freeindex
parameter.

I'd like to take one quick moment to thank Dave and the ROS Industrial team for their
documentation, and Ioan Sucan and the rest of the MoveIt team for the great platform
they've provided.
