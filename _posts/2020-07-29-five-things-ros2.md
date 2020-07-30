---
layout: post
title: 5 Things ROS2 Needs in 2020
description: Take only what you need to survive.
date: '2020-07-29T14:00:00.00-04:00'
author: Michael Ferguson
tags:
- robots
- ros2
---

<center>
<img src="/{{ site.baseurl }}assets/images/2020-07-29-survive.jpg" width="90%" />
</center>

I've been using ROS2 quite a bit over the past several months. As
I've previously mentioned, it would appear there aren't too many real robots running
ROS2 yet. We have a bit of a chicken-and-egg problem where the tools are not yet
fully ready for real robots, but until people start using ROS2 on real robots nobody
knows the real pain points.

There are many, many things that could be done in ROS2. But there is limited time to
implement them all - so we need to focus on those that enable robots and their
developers to "survive".

I often get asked if ROS2 is ready for prime time. My answer for
a long time was "no". I'm going to upgrade it to "maybe, depends on what you're
doing" at this point. This post describes five things that I think would make the
answer "hell yes" for most roboticists. I actually hope this post ages poorly and
that all these things come to happen in ROS2.

## Automatic QoS for RVIZ, rcl2cli

Quality of Service (QoS) is probably the biggest change between ROS1 and ROS2 - it's also
the one that causes the most headaches from what I can tell.
The ROS2 Foxy release [adds](https://github.com/ros2/ros2cli/pull/385) a
<code>--verbose</code> option to the <code>ros2 topic info</code> command which is a huge
step in the right direction. This lets you quickly diagnose when a publisher and subscriber
are using incompatible QoS.

<code>rosbag2</code> got a [huge upgrade in ROS2 Foxy](https://github.com/ros2/rosbag2/issues/125):
it automatically determines the proper settings for Quality of Service (QoS) so that it always
connects to the publisher you're trying to record (note: if multiple publishers are publishing
to the same topic with different QoS it may not work - but really, who does that?).

Now we need that feature in RVIZ2 and the command line utilities (CLI).
These are debugging tools, so they need to be able to "just work" in most scenarios.

Since most of the time you're using RVIZ2 to connect to sensor data, which is often published
with a non-default QoS (the sensor data profile), it's absolutely bonkers that RVIZ uses
the default QoS on everything
([which is incompatible with sensor profile](https://index.ros.org/doc/ros2/Concepts/About-Quality-of-Service-Settings/)).
Even something as simple as [latched topics](https://answers.ros.org/question/305795/ros2-latching/)
won't work by default.

This is not an easy ask. It will involve significant changes to RVIZ as well as changes to
lower level packages like <code>message_filters</code>, but I'm pretty sure this is the
single biggest bang-for-your-buck improvement that will make ROS2 work better for robot
developers.

## Documentation

Ok, I'm sounding like a broken record (or the squeaky caster on your 8 year old mobile manipulator),
but this is really important.

I'm not just talking about the lack of tutorials here. One of the things that made ROS great
for new developers in the 2011-2014 era (when it experienced huge growth in the community), was
a very polished and up-to-date wiki. If you wanted to find out about a package, you could go to
wiki.ros.org/package_name - and the documentation was right there (or if it wasn't, you had a
pretty good idea this package wasn't ready for prime time). With ROS2, we don't have a centralized
place for documentation yet - and I think that is holding the community growth back.

There is also the issue of "user documentation". Nearly everything for ROS2 is written assuming
an expert programming background (even more so than ROS1 documentation). <i>Reading the source
code is not how you're supposed to learn how to run a ROS driver for a laser scanner</i>.

Building out a community is super important.
The best way to get a bug fixed is to find a developer who needs it fixed. I've only been using
ROS2 on-and-off for a couple months - and in that time I've fixed half a dozen bugs across multiple
ROS2 packages, and even taken on maintaining the ROS2 port of
[urg_node](https://github.com/ros-drivers/urg_node) and the related packages.

## Subscriber Connect Callbacks

Now we'll jump into a super technical issue - but the impact is huge - especially for those
doing perception (which is, you know, generally a big part of robotics).
When creating a publisher in ROS1, you could register a callback which would get called
whenever a subscriber connected or disconnected. This feature doesn't yet exist in ROS2,
but I think it is essential for real robotics systems. Here's why:

Robots can generate lots of sensor data - especially when you add processing pipelines
into the mix. Sometimes you might need a high-resolution point cloud with color and depth
information. Sometimes you need a low-res colorless point cloud.  This is especially true
when the robot system does multiple tasks. For instance, imagine a robot that is both
mobile and a manipulator - for navigating the environment it wants that high frame
rate, low-res point cloud for collision avoidance. When the mobile manipulator gets
to the destination it wants to switch to a high-res point cloud to decide what to grab.

Sometimes you literally cannot be publishing all the data streams possible because
it would overwhelm the hardware (for instance, saturating the USB bus if you were to
pull depth and color and IR from most RGBD sensors at the same time).

In ROS1, you could create "lazy publishers" so that the creators of these intensive
data types would only create and publish the data when someone was listening. They
would be alerted to someone listening by the connect callback. The lack of lazy
publishers throughout various drivers and the <code>image_proc</code> and
<code>depth_image_proc></code> packages is a real challenge to building high performance
perception systems. When people ask me "is ROS2 ready?", my first question these days is
"how much perception/vision are you doing?".

To be clear, there are workarounds available in some cases. If you're creating a
publisher yourself, you can:

 * Create a loop that "polls" whether there are subscribers (using get_subscription_count)
   as I did right now in the <code>openni2_camera</code> package.
 * Use parameters to dynamically reconfigure what is running. While this might
   work in some cases (and maybe even be a preferred solution for some use cases),
   it likely leads to a more brittle system.
 * Re-architect your system never need lazy publishers by hard coding exactly
   what you need for a given robot. While some of this is likely to happen in
   a more production environment, it doesn't lend itself to code reuse and sharing
   which was one of the major selling points of ROS1.

Note that I said, "if you're creating a publisher yourself". There are lots of
packages that are widely relied on in ROS1 whose ROS2 ports are crippled or broken
due to the lack of subscriber connect callbacks:

 * message_filters
 * image_transport
 * image_proc
 * depth_image_proc

Related answers.ros.org:

 * [ROS2 publisher callback on subscription match](https://answers.ros.org/question/351827/ros2-publisher-callback-on-subscription-match/)

## Developer Involvement

<i>Note: in the month that I've been writing this post, a number of questions have been answered, so we're already getting there!</i>

I remember folks joking that ROS Answers was misnamed, because there were no answers there,
just questions. It's actually not true - unless you search for the
[ROS2 tag](https://answers.ros.org/questions/scope:all/sort:activity-desc/tags:ros2/page:1/).

There are a lot of really good questions there. Like, stuff that's not anywhere in the documentation
and is probably quite relevant to a large number of users. Here's a few examples:

 * [Are rmw_serialize and deserialize thread safe?](https://answers.ros.org/question/354484/are-rmw_serialize-and-rmw_deserialize-thread-safe-functions/)
 * [How to unsubscribe from a topic?](https://answers.ros.org/question/354792/rclcpp-how-to-unsubscribe-from-a-topic/)
 * [Why should we use destory_node in Python](https://answers.ros.org/question/354054/why-should-we-use-destroy_node-for-python-nodes-in-ros2/)
 * [What Does set(node_plugins) actually do?](https://answers.ros.org/question/354631/what-does-setnode_plugins-actually-do/)

ROS2 developers, please take note: we've got lots of great features in this system,
please help your users learn to how to actually use them - maybe they'll even help
contribute back!

## Your Robot on ROS2

There's probably a bunch of other bugs/issues/etc hiding in the weeds. Your robot is probably
not exactly the same as mine - and your use cases are going to be different. We need more robots
running ROS2 to dig into things. The good news is: you can install ROS1 and ROS2 on the same system
and switch back and forth pretty easily.
