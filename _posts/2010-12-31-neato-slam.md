---
layout: post
title: Neato + SLAM
description: How to build a map with gmapping and a 360 degree laser.
date: '2010-12-31T23:18:00.003-05:00'
author: Michael Ferguson
tags:
- robots
- ros
- slam
modified_time: '2010-12-31T23:30:14.605-05:00'
blogger_id: tag:blogger.com,1999:blog-8995060597167955455.post-8073844081896967124
blogger_orig_url: http://www.showusyoursensors.com/2010/12/neato-slam.html
redirect_from: "/2010/12/neato-slam.html"
---

Here is yet another story for "the power of open source.

I've been spending quite a bit of time working on SLAM with the Neato XV-11 using both
the built in laser and the Hokuyo URG-04LX-UG01. I had pretty much given up on gmapping
working with the Neato -- until earlier today we found an issue with the scan angle increment
computation in gmapping not working with the Neato laser specifications. I probably wouldn't
have found this bug had it not been for a user of the Trossen Robotic Community pointing out
some issues he was having with gmapping, as my version still had some modifications from my
PML work earlier this year.

Anyways, for anyone wanting to use gmapping with the Neato robot, you can apply the
following patch:

```
339c339
- gsp_laser_angle_increment_ = (angle_max - angle_min)/scan.ranges.size();
+ gsp_laser_angle_increment_ = scan.angle_increment;
```

to slam_gmapping.cpp. This uses the angle_increment from the laser scan, rather than the
computed one, which is incorrect for full rotation scans. This will avoid issues with the
scan being improperly inverted, and issues with scan matching.
