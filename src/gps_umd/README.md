gps_umd [![Build Status](https://travis-ci.org/swri-robotics/gps_umd.svg?branch=master)](https://travis-ci.org/swri-robotics/gps_umd)
=======

This package is a space to stage messages and common GPS-processing routines.  The previous maintainer has not released these packages since ROS Indigo; this fork was created in order to fix them up and release them for ROS Jade and Kinetic.

One change of note is that in the version of libgps in Ubuntu 16.04, the `STATUS_DGPS_FIX` flag was removed, so the `gpsd_client` package will be unable to indicate whether DGPS was used in a fix or not.

Otherwise, the API is unchanged; see http://wiki.ros.org/gps_common .
