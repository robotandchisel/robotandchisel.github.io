---
layout: post
title: ROS 2 in Parallels VM on MacOSX
description: How I'm using ROS 2 on my Macbook
date: '2024-01-08T14:00:00.00-04:00'
author: Michael Ferguson
tags:
- robots
- ros2
---

While I sported a Linux laptop throughout grad school, my Willow Garage days,
and my early startup years, I've been using a Macbook Pro as my daily driver
for about a decade now.

That equates to a lot of "ROS on Mac" pain.

Things were supposed to get easier with ROS 2.

Back in 2020, I installed ROS 2 natively on my last Intel-based Macbook.
You can read all about the
[fun of compiling from source]({% post_url 2020-08-10-rviz2-on-mac %}).

Then Apple moved to their new M1 architecture, ROS downgraded OSX to Tier 3
support, and I got older and maybe a bit more grumpy. For all of these reasons,
I decided not to do a native installation for ROS 2 Humble. I tried out a few
approaches as [documented here]({% post_url 2022-06-21-ubr1-on-ros2 %}), but
ended up settling on using a Parallels VM.

Yes, Parallels costs cash money. But, how do you value your time?

## Parallels Installation

My original post had a quick run down of my installation, but here is a recap:

 * M1/2/3 architecture is ARM64 - which is Tier 1 supported!
 * Ubuntu has an ARM64 installer - but only for server, so you have to do a few
   extra steps - but there is a
   [detailed description of the workflow on askubuntu.com](https://askubuntu.com/questions/1405124/install-ubuntu-desktop-22-04-arm64-on-macos-apple-silicon-m1-pro-max-in-parall)

## Issue: RVIZ2

```rviz2``` runs well inside the Parallels VM, especially if you avoid
using "points" as the display type for laser scan and point cloud messages. There
seems to be a bug there that causes frequent crashes - but only for that display type.
I've had great success with ```Flat Squares``` as the rendering type. I've also had no
issue visualizing points in a ```visualization_msgs/Marker```.

## Issue: Bridged Networks

If you just want to develop locally within your VM, then you can keep right on
using the "Shared Network" profile and skip over this issue.

However, if you want to connect to a robot and actually stream data from ROS 2,
you will need to change from "Shared Network" to "Bridged Network". This sounds
easy, however, it appears there are numerous issues with Parallels creating
bridged networks (their support forum was a wasteland of these issues, all unanswered).
I could not get the default network to even come up with a bridged configuration.

The workaround appears to be to create a second network adapter, and make that
one bridged. Since the primary network is still shared and comes up as expected,
your Ubuntu VM will boot, and then you can configure the bridged network within
the VM. I found that using a fixed IP was the most reliable approach:

 * Shut down the VM
 * In the Parallels Configuration screen, add an additional network adapter,
   and select "Bridged Network", and the appropriate adapter on the Mac.
 * Boot the VM
 * I configured the network adapter with a fixed IP and then started up:
 ```
 sudo ip addr add 192.168.0.150/24 dev enp0s6
 sudo ip link set dev enp0s6 up
 ```

## Issue: Disk Size

About a year later, I started to run out of disk space. I had created the VM
with a 64GB drive in the Parallels configuration, but inside the VM it only
reported a 32GB drive. Apparently, this is a side effect of using the server
installation of Ubuntu - the drive won't automatically be fully sized. The
good news here is that we don't need to use tools like ```gparted``` as we
are only adjusting the logical partition. I got my other 30GB with:

```
sudo lvresize —resizes —size +30G ubuntu-vg/ubuntu-lv
```

## Summary

This is a pretty short post - because things are mostly working. I've been using
this setup over the past few weeks to connect RVIZ to my RoboMagellan robot while
it is navigating around outdoors. I'll have some RoboMagellan-specific posts coming
up soon.
