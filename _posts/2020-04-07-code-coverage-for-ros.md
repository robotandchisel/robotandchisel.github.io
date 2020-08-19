---
layout: post
title: Code Coverage for ROS
description: Introducing a ROS package I've been working on for a while.
date: '2020-04-07T10:30:00.002-04:00'
author: Michael Ferguson
tags:
- national-robotics-week
- ros
modified_time: '2020-04-07T10:30:37.063-04:00'
thumbnail: https://1.bp.blogspot.com/-caOsU8XEV-0/XoyJaEsaQOI/AAAAAAAADiA/lapq-wHwbH4wHDClG9QioJZQH3h7cjmyACK4BGAYYCw/s72-c/Screen%2BShot%2B2020-04-07%2Bat%2B10.07.10%2BAM.png
blogger_id: tag:blogger.com,1999:blog-8995060597167955455.post-1482389950025892645
blogger_orig_url: http://www.showusyoursensors.com/2020/04/code-coverage-for-ros.html
redirect_from: "/2020/04/code-coverage-for-ros.html"
---

_This is day 3 of my 2020 [National Robotics Week](/{{site.baseurl}}tag/national-robotics-week) blog marathon!_

About two years ago I created a little package called
[code_coverage](https://github.com/mikeferguson/code_coverage). This package is a bit
of CMake which makes it easier to run coverage testing on your ROS packages. Initially it only
supported C++, but recently it has been expanded to cover Python code as well.

##### What is Code Coverage?
Before I get into how to use the _code_coverage_ package, let's discuss what coverage
testing is all about. We all know it is important to have tests for your code so that it does
not break as you implement new features and inevitably refactor code. Coverage testing tells
you what parts of your code your tests actually test. This can help you find branch paths or
even entire modules of the code that are not properly tested. It can also help you know if new
code is actually getting tested.

The output of a coverage test is generally some really nice webpages that show you line-by-line
what code is getting executed during the test:

<center>
<img src="/{{ site.baseurl }}assets/images/2020-04-07-coverage.png" width="95%" />
</center>

##### Using code_coverage for C++
We will start by discussing the usage of code_coverage with C++ code first, because it is
actually quite a bit simpler. C++ coverage can be done almost entirely in CMake.

First, update your <code>package.xml</code> to have a <code>test_depend</code> on
_code_coverage_ package.

Next, we need to update two places in the <code>CMakeLists.txt</code> file. The first change
should be right after you call to _catkin_package_. The second change is where you define your
test targets. You need to define a new target, which we will typically call
<code>{package_name}_coverage_report<code>


```cmake
# After catkin_package()

if(CATKIN_ENABLE_TESTING AND ENABLE_COVERAGE_TESTING)
  find_package(code_coverage REQUIRED)
  # Add compiler flags for coverage instrumentation before defining any targets
  APPEND_COVERAGE_COMPILER_FLAGS()
endif()

# Add your targets here

if (CATKIN_ENABLE_TESTING)
  # Add your tests here

  # Create a target ${PROJECT_NAME}_coverage_report
  if(ENABLE_COVERAGE_TESTING)
    set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test*" "*/${PROJECT_NAME}/other_dir_i_dont_care_about*")
    add_code_coverage(
      NAME ${PROJECT_NAME}_coverage_report
      DEPENDENCIES tests
    )
  endif()
endif()
```

That's the configuration needed. Now we can compile the code (with coverage turned on)
and run the coverage report (which in turn will run the tests):

```
catkin_make -DENABLE_COVERAGE_TESTING=ON -DCMAKE_BUILD_TYPE=Debug PACKAGE_NAME_coverage_report
```

You can find these same instructions (and how to use catkin tools) in the
<a href="https://github.com/mikeferguson/code_coverage/blob/master/README.md">code_coverage README</a>.

##### Using code_coverage for Python
Python unit tests will automatically get coverage turned on just with the CMake configuration
shown above, but Python-based rostests (those that are launched in a launch file) need some
extra configuration.

First, we need to turn on coverage testing in each node using the launch-prefix. You can decide
on a node-by-node basis which nodes should actually generate coverage information:

```xml
<launch>

    <!-- Add an argument to the launch file to turn on coverage -->
    <arg name="coverage" default="false"/>

    <!-- This fancy line forces nodes to generate coverage -->
    <arg name="pythontest_launch_prefix" value="$(eval 'python-coverage run -p' if arg('coverage') else '')"/>

    <!-- This node will NOT generate coverage information -->
    <node pkg="example_pkg" name="publisher_node" type="publisher_node.py" />

    <!-- But this node WILL generate coverage -->
    <node pkg="example_pkg" name="subscriber_node" type="subscriber_node.py"
          launch-prefix="$(arg pythontest_launch_prefix)" />

    <!-- The test can also generate coverage information if you include the launch-prefix -->
    <test time-limit="10" test-name="sample_rostest" pkg="example_pkg" type="sample_rostest.py"
          launch-prefix="$(arg pythontest_launch_prefix)" />

</launch>
```

Then we turn on coverage by adding the argument in our CMakeLists.txt:

```cmake
add_rostest(example_rostest.test ARGS coverage:=ENABLE_COVERAGE_TESTING)
```

You can find this full Python example from my co-worker Survy Vaish on
<a href="https://github.com/SarvagyaVaish/ros_ci_and_coverage">GitHub</a>.

##### Using codecov.io For Visualization
codecov.io is a cloud-based solution for visualizing the output of your coverage testing.
It can combine all of the reports from individual packages, as well as the C++ and Python
reports into some nice graphs and track results over multiple commits:

<center>
<img src="/{{ site.baseurl }}assets/images/2020-04-07-codecov.png" width="90%" />
<i>codecov.io dashboard for robot_calibration</i>
</center>

##### A Full Working Example
The <a href="https://github.com/mikeferguson/robot_calibration">robot_calibration</a> package use
<code>code_coverage</code>, codecov.io, and Travis-CI to run code coverage testing on every pull
request and commit to master branch. It uses the popular
<a href="https://github.com/ros-industrial/industrial_ci/tree/master">industrial-ci</a> package as
the base line and then the following changes are made:

 * I set the CMAKE_ARGS in the travis.yml so that coverage is turned on, and the build type is debug.
 * I created a _.coverage.sh_ script which runs as the AFTER_SCRIPT in Industrial-CI. This script runs
   the coverage report target and then calls the codecov.io
   <a href="https://docs.codecov.io/docs/about-the-codecov-bash-uploader">bash uploader</a>.
 * Since Industrial-CI runs in a docker, I introduced a _.codecov.sh_ script which exports the
   required environment variables into the docker. This uses the
   <a href="https://docs.codecov.io/docs/testing-with-docker">env script from codecov.io</a>.
