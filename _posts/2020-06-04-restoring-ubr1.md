---
layout: post
title: Restoring a UBR-1
description: I bought a UBR-1 on Craigslist and am working on restoring it.
date: '2020-06-04T12:00:00.00-04:00'
author: Michael Ferguson
tags:
- ubr1
- robots
- ros
---

Some people collect classic cars. I tend to collect classic robots. For a long time I've
preserved many of my robots - or at least the head of the robot if I needed to reuse the
majority of the components. I also bought a [PR2 head]({% post_url 2013-04-29-pr-shelf %}) during my time
at Willow Garage. Recently I added the best item to this collection.

A couple of weeks ago, someone from the [Homebrew Robotics Club](http://www.hbrobotics.org)
posted a link to a Craigslist ad for a UBR-1 robot in Tracy, CA. I immediately reached out to
the seller to find out more. A few days later the robot was on it's way to NH in it's bright
orange case.

<center>
<img src="/{{ site.baseurl }}assets/images/2020-06-04-craigslist.png" width="90%" />
</center>

Upon arrival in NH, I quickly unpacked the robot. The skins were quite dirty, and a number of
fasteners were missing so I quickly stripped all the skins off the robot to check on the insides.

Removing the batteries, I found they were at only 0.5V each. I had figured they would be garbage
since they were eight years old and they had been left plugged in (I'm not sure, but I suspect
there is always a small draw on the batteries when plugged into the main board). New batteries
were ordered and the cabling was moved over. I manually charged the batteries to balance them
and then installed them into the robot.

I unplugged all the computer and motor power cables coming off the mainboard before starting
the robot up. Once satisfied that the power rails were coming up nicely I plugged the computer
back in. And it booted. And gravity compensation on the arm starting working as soon as I
released the runstop.

The joystick batteries are completely dead and unable to recharge, so I switched to keyboard
teleop to test the base and head. I tested the arm with an older controller test script that
was part of the [UBR-1 preview repo](https://github.com/unboundedrobotics/ubr1_preview).

<center>
<iframe width="560" height="315" src="https://www.youtube.com/embed/ih8KzTE_Mb0" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</center>

The scanning laser was also missing on this robot (I vaguely recall we had a loaner laser that
had to go back when Unbounded shut down). Luckily I have some spare lasers here.

A few other things needed to be cleaned up before the skins were put back on. In removing the
scanning laser, lots of tie straps were missing on the cabling near the right side drive
motor (see image below). Thermistors on the base motors had also come undone, but were
quickly fixed with some new kapton tape.

<center>
<img src="/{{ site.baseurl }}assets/images/2020-06-04-guts.jpeg" width="90%" />
</center>

I then backed up the contents of the hard drive and swapped it out for a new drive so I could
updated from 14.04/Indigo. I set up the new drive to dual both both 18.04 and 20.04 since I
wasn't sure how well Noetic was going to run. Surprisingly it wasn't too bad. I got the
drivers all updated and ready to go before World MoveIt Day.

During World MoveIt Day I got MoveIt working on the robot and then moved onto testing
robot_calibration, both packages needed a few updates for changes in underlying dependencies
and the move to Python 3. By mid-day I managed to calibrate the robot:

<center>
<img src="/{{ site.baseurl }}assets/images/2020-06-04-calibrated.jpeg" width="90%" />
</center>

I'm using Ansible to deploy most of the [robot setup](https://github.com/mikeferguson/ubr_reloaded/tree/master/ansible).
I've still got some work to get grasping demos updated and running on the robot, then
I'm hoping to revive the [chess playing demo](https://github.com/mikeferguson/chessbox)
although that code hasn't been run since Hydro.

And maybe add a "Beer Me" demo.

<center>
<img src="/{{ site.baseurl }}assets/images/2020-06-04-beer-me.jpeg" width="90%" />
</center>
