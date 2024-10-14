---
layout: post
title: A New Local Planner
description: Implementing the graceful_controller package.
date: '2024-10-14T9:30:00.00-04:00'
author: Michael Ferguson
tags:
- robots
- ros
- ros2
---

This is another blog post that has sat in draft for 2+ years. I'm pushing it out ahead
of ROSCon this year where I'll be talking about migrating the UBR-1 mobile manipulator
to ROS2.

For a long time I've wanted a better open-sourced local controller for ROS navigation.
It's finally implemented, and has been running on the robot fleet at Cobalt Robotics
for about two years now. The full code of the graceful_controller is available on
[GitHub](https://github.com/mikeferguson/graceful_controller).

## Motivation

There are a number of local controllers in the ROS1 and ROS2 navigation stacks, but most
of them are based on either the
[Dynamic Window Approach](https://www.ri.cmu.edu/pub_files/pub1/fox_dieter_1997_1/fox_dieter_1997_1.pdf),
[Trajectory Rollout](https://citeseerx.ist.psu.edu/document?repid=rep1&type=pdf&doi=dabdbb636f02d3cff3d546bd1bdae96a058ba4bc), or
optimization approaches such as 
Time Elastic Bands.

Both DWA and Trajectory Rollout suffer from a series of challenges:

 - All of these controllers have many parameters, but the ones for DWA are especially
   difficult to tune because they are highly interrelated (for instance the goal, path,
   and obstacle biases). This is marginally improved by the [DWB]() controller.
 - The <code>sim_time</code> parameter is especially difficult to tune for all operating
   environments. Too long of a simulation time will make it difficult to enter tight
   spaces (except at extremely slow speeds), while too low of a value will cause
   instability
 - DWA requires that users either have very good odometry response, or falsify their
   acceleration limits to actually get robots moving. This is a reason so many robots
   with low acceleration limits still use the older Trajectory Rollout controller.
 - Even if these acceleration limits are defined properly, the way the forward simulations
   work are pretty crufty.

For this reason, many people replace the default controllers - [especially corporate
users of ROS](https://vimeo.com/142624685). Sadly most of these improved controllers
don't get released into the open source.

## Goals

The goals for the new controller were quite simple:

 - Accurately model the robot limitations (acceleration, deceleration, etc).
 - Accurately forward simulate where the robot will end up.
 - Produce repeatable results that look smooth and intelligent.

## Underlying Algorithms

A number of years ago at Fetch Robotics, I wrote a package for our robots to autonomously
connect to their charge dock. Sometime later, Fetch open sourced the package under an LGPL
license. I used this same underlying control law for the first part of the new controller.

This underlying control law is based on a paper called  [A Smooth Control Law for Graceful Motion of Differential Wheeled Mobile Robots in 2D Environments](http://www-personal.umich.edu/~jongjinp/papers/Park-icra-11.pdf) by Park and Kuipers, which was presented at ICRA 2011.

This control law implements a closed-form solution to find a kinematically feasible linear
and angular velocity for a differential drive robot to approach a pose, based on just a few
parameters that are actually fairly robot-independent. The image below shows some example
trajectories:

<center>
<img src="/{{ site.baseurl }}assets/images/2024-10-14-control-law.png" width="60%" />
</center>

The underlying control law has a number of nice features:

 - It automatically slows down for highly curved paths.
 - It automatically slows to zero as the target pose is reached.

The underlying control law really only works for approaching a single pose - which we call
the <code>target pose</code>. This is exactly what docking with a charger entails. For
something like navigation, we typically need to approach a series of poses in succession
(since following the control law directly towards the final pose in the path is usually not 
collision free). Section IV.B of the paper describes a fairly complex approach to do this,
however, our controller takes a simpler approach.

Our controller attempts to use as the <code>target pose</code> the farthest pose in the
path which is both A) less than some maximum <code>lookahead</code> distance away from
the robot, and B) reachable with our current control law parameters without collision.

The reachability portion is implemented through a forward simulation. Unlike DWA, the
forward simulation is not time-based, but rather terminates when we have reached the
<code>target pose</code>. Additionally, since the target pose is the farthest that we
have collision checked, we will actually forward simulate based on stopping at the target
pose.

The new controller is built out of two ROS packages. The first package includes the
underlying control law, which is licensed under the LGPL. The second package implements
a ROS1 or ROS2 plugin for navigation, and is licensed under BSD since it is loosely
based on the DWA codebase.

## Additional Features

While the basic control law with target pose simulation works fairly well, there are
a number of additional improvements implemented:

 - <b>Optional initial rotation</b> - since the control law can cause the robot to take
   large sweeping arcs when the target pose is behind or to the side of the robot, we
   implement an optional initial in-place rotation that points the robot towards the
   target pose. If in-place rotation is not possible (due to collision), the regular
   control law will be applied.
 - <b>Optional final rotation</b> - similarly, for the final pose, it is possibly preferred
   to approach the final pose without considering the heading of the pose. Once at the
   final pose, the robot can rotate to the final goal orientation.
 - <b>Footprint Scaling at Speed</b> - this feature inflates the robot footprint
   at higher speeds. This allows the robot to move quickly in more open spaces but
   naturally slow down when the environment gets tighter.
 - <b>Orientation filter</b> - can be used to smooth out the orientations of the
   poses in the global plan.

## ROS2 Support

As it turns out, porting to ROS2 was fairly straight forward. Improvements to the controller
API in ROS2 mean that the following features are far easier to implement:

 - The controller command velocity generation API natively passes the current robot speed,
   which negates the need for additional boilerplate that existed in the ROS1 controller
   to listen to the odometry feedback.
 - The notion of variable speed limits is native to Navigation2, and the controller has
   an API for it.
 - The goal checking is split out to a different plugin.

## Future Work

There have been quite a few developments since I first started working on this controller
in early 2021.

The Nav2 project added the MPPI controller, which addresses many of my concerns with
both DWA and Trajectory Rollout. My only issue with this controller is that it uses
special SSE/NEON instructions for parallelization, and so it doesn't work on all CPUs
(in particular, I found it wouldn't work on the very low end Celeron processor in
my FireBot).

An alternate implementation of the control law was also added to Nav2 as part of the
docking server project. It's not quite ready to be a full controller yet, but at some
point I expect that version to supersede the ROS 2 implementation of my graceful controller
package.

## Kudos

This work was supported in part by Cobalt Robotics.
