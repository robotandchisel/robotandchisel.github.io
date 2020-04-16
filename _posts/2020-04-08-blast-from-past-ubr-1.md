---
layout: post
title: 'Blast From the Past: UBR-1'
description: A look back at my favorite little orange robot.
date: '2020-04-08T11:26:00.000-04:00'
author: Michael Ferguson
tags:
- blast-from-the-past
- national-robotics-week
- robots
- ros
modified_time: '2020-04-08T11:26:12.697-04:00'
thumbnail: https://3.bp.blogspot.com/-MZ_5KG74JxM/XooWXdnxO-I/AAAAAAAADg0/APJeUbNw_2kNd8Sci38LMx-Pz8hs94PsgCK4BGAYYCw/s72-c/80E1732.jpg
blogger_id: tag:blogger.com,1999:blog-8995060597167955455.post-2439713892085343997
blogger_orig_url: http://www.showusyoursensors.com/2020/04/blast-from-past-ubr-1.html
---

_This is day 4 of my <a href="/{{ site.baseurl }}tag/national-robotics-week/">National Robotics Week</a>
blog marathon - it's halfway over!_

In 2013, Unbounded Robotics was the last spin-off from Willow Garage. Our four person team set out to
build a new robotics platform for the research and development community. The robot would cost a
fraction of what the Willow Garage PR2 cost. We did build three robots and demo them at a number of
events, but Unbounded eventually ran out of money and was unable to secure further funding. In the
summer of 2014 the company shut down.

I wasn't really blogging during this whole time because I was really busy when things were going well,
and then I didn't really want to talk about it while things were going downhill. The whole affair is
now quite a few years ago, so here we go. First, a picture of our little robot friend:

<center>
<img src="/{{ site.baseurl }}assets/images/2020-04-08-ubr.jpg" width="70%" />
<i>UBR-1 by Unbounded Robotics</i>
</center>

The robot had a differential drive base, since that is really the only cost-effective style of robot
base out there. It used an interesting 10:1 worm-gear brushless motor, similar to what had been in the
<a href="https://spectrum.ieee.org/automaton/robotics/industrial-robots/platformbot-willow-garages-secret-robot-prototype">PlatformBot</a>
our team had previously designed at Willow Garage. The brushless motors were really quite efficient and
the whole drive was super quiet, but the worm gear was terribly inefficient (more on that below).
The base was about 20" in diameter - which made the robot much more maneuverable than the 26" square
footprint of the PR2, even though PR2 had holonomic drive.

The 7-DOF arm had similar kinematics to the PR2 robot, but tossed the counterbalancing for simplicity,
lower cost and weight reduction. A parallel jaw gripper replaced the very complex gripper used in the
PR2. A torso lift motor allowed the robot to remain quite short during navigation but rise to interact
with items at table height - it was a little short for typical counter-top heights but still had a
decent workspace if the items weren't too far back on the countertop.

One of the first demos I set up on the UBR-1 was my chess playing demo:

<center>
<img src="/{{ site.baseurl }}assets/images/2020-04-08-ubr.gif" width="70%" />
</center>

##### UBR-1 Software
As with everything that came out of Willow Garage, UBR-1 used the Robot Operating System (ROS).
You can still download the <a href="https://github.com/unboundedrobotics/ubr1_preview">preview repository</a>
for the UBR-1 which included simulation with navigation and grasping demos. As with the ROS legacy of
Willow Garage, the open source software released by Unbounded Robotics continues to live on to this day.
My <a href="https://github.com/mikeferguson/simple_grasping">simple_grasping</a> package is an improved
and robot-agnostic version of some of the grasping demos we created at Unbounded (which were actually
based on some of my earlier grasping demos created for my Maxwell robot). A number of improvements and
bug fixes for ROS Navigation and other packages also came out during this time since I was a primary
maintainer of these packages in the dark days following Willow's demise.

<center>
<img src="/{{ site.baseurl }}assets/images/2020-04-08-rviz.png" width="70%" />
<i>UBR-1 in Gazebo Simulation</i>
</center>

##### Power Usage and Battery Life
Power usage reductions and battery life increases were some of the biggest improvements in the UBR-1
versus the PR2. The PR2 was designed in 2010 and used two computers, each with 24Gb of RAM and 2
quad-core Intel L5520 Nehalem processors. The Nehalem was the first generation of Intel Core processors.
The PR2 batteries were specified to have 1.3kWh of stored energy, but only gave about two hours of
runtime, regardless of whether the robot was even doing anything with the motors. There were other
culprits besides the computers, in particular, the 33 motor controller boards each had two ethernet
PHYs accounting about 60W of power draw. But the computers were the main power draw. This was made
worse by the computers being powered by isolated DC-DC converters that were only about 70% efficient.

<center>
<img src="/{{ site.baseurl }}assets/images/2020-04-08-arm.jpg" width="70%" />
<i>The UBR-1 arm.</i>
</center>

The UBR-1 used a 4th generation Intel Core processor. The gains in just four years were quite
impressive: the computer drew only 30-35W of power, but we were able to run similar demos of navigation
and manipulation that ran on the PR2. Based on the Intel documentation, a large part of that was the
75% power reduction for the same performance from the first to fourth generation chips. A smaller
contributor was improvements in the underlying algorithms and code base.

For dynamic movements, the UBR-1 arm was also significantly more power efficient than the PR2, since
it weighed less than half as much. Gravity compensation of the 25 lb arm required only about 15W in
the worst configurations - this could have been lowered with higher gearing, but would have had adverse
effects on the efficiency when moving and might have jeopardized the back-drivability.

The base motors were still highly inefficient - UBR-1 needed about 250W to drive around on carpet.
Hub motors have become pretty common in the years since and can improve the efficiency of the drivetrain
from a measly 35% to upwards of 85%.

##### Robot Evolution
Processors have gotten more efficient in the years since UBR-1 - but their prices have pretty much
stopped dropping. Motors haven't really gotten any cheaper since the days of the PR2, although the
controls have gotten more sophisticated. Sensors also really haven't gotten much cheaper or better
since the introduction of the Microsoft Kinect. While there has been a lot of money flowing into robotics
over the past decade, we haven't seen the predicted massive uptake in robots. Why is that?

One of my theories around robotics is that we go through a repeated cycle of "hardware limited" and
"software limited" phases. A hardware innovation allows an increased capability and it takes some time
for the software to catch up. At some point the software innovation has increased to a point where the
hardware is now the limiting factor. New hardware innovations come out to address this, and the
process repeats.

Before ROS, we were definitely software-limited. There were a number of fairly mechanically sophisticated
robots in research labs all over the planet, but there were not widely-accepted common frameworks with
which to share software. PhD students would re-invent the wheel for the first several years of their
work before contributing something new, but then have no way to pass that new innovative code on. ROS
changed this significantly.

On the hardware side, there were very few common platforms before the PR2. Even the Mobile Robots
Pioneer wasn't exactly a "common platform" because everyone installed different sensors on them,
so code wasn't very portable. The introduction of PR2 going to a number of top universities, combined
with the Willow Garage intern program, really kickstarted the use of a common platform. The
introduction of the Microsoft Kinect and the advent of low-cost depth sensors also triggered a
huge leap forward in robot capability. I found it amusing at the time to see the several thousand
dollar stereo camera suite on the PR2 pretty much replaced (and outperformed) by a single $150 Kinect.

For a few years there was a huge explosion in the software being passed around, and then we were
hardware-limited again because the PR2 was too expensive for wide adoption. While the UBR-1 never
got to fill that void, there are now a number of lower-cost platforms available with pretty
solid capabilities. We're back to software-limited.

So why are robots still software limited? The world is a challenging environment. The open source
robotics community has made great strides in motion planning, motor control, and navigation. But
perception is still really hard. Today we're mainly seeing commercially deployed robots making
inroads in industries where the environment is pretty well defined - warehouses and office spaces,
for instance. In these environments we can generally get away with just knowing that an "obstacle"
is somewhere - our robot doesn't really care what the obstacle is. We've got pretty good sensors -
although they're still a little pricey - but we generally lack the software to really leverage them.
Even in the ROS ecosystem, there's huge amounts of hardware drivers and motion planning software
(ROS Navigation, MoveIt, OMPL, SBPL, the list goes on), but very little perception code. Perception
is still very dependent on the exact task you're doing, there just isn't a lot of "generic"
perception out there.

There is a certain magic in finding applications where robots can offer enough value to the customer
at the right price point. Today, those tend to be applications where the robot needs limited
understanding of the environment. I look forward to what the applications of tomorrow might be.
