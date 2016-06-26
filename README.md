# `people_detection_vision`

![foo](doc/divulgacion.png)

User detection, recognition and tracking is at the heart of Human Robot
Interaction, and yet, to date, no universal robust method exists for being
aware of the people in a robot surroundings. The presented work aims at
importing into existing social robotics platforms different techniques, some
of them classical, and other novel, for detecting, recognizing and tracking
human users. These algorithms are based on a variety of sensors, mainly
cameras and depth imaging devices, but also lasers and microphones. The
results of these parallel algorithms are then merged so as to obtain a
modular, expandable and fast architecture. This results in a local user
mapping thanks to multi-modal fusion.

This package gathers all the different people detection algorithms that were integrated:

* a 3D improvement of the **Viola-Jones face detection**
[Viola and Jones, 2001]

* a 3D improvement of the **Histogram of Oriented Gradients (HOG)**
[Dalal and Triggs, 2005]

* the **Polar-Perspective Map (PPM)**,
[Howard and Matthies, 2007].
used in autonomous driving for
pedestrian detection
It uses a polar transformation and an accumulation process on the ground plane.

* the **NiTE** algorithm, the Kinect middleware
[Berliner and Hendel, 2007]

* the **tabletop** algorithm :
[Blodow and Rusu, 2009]
uses point cloud manipulation techniques, detecting users as
it would for objects on a table.

For more information, check out
[Arnaud Ramey's PhD](https://sites.google.com/site/rameyarnaud/research/phd).

How to install
==============

## 1. Dependencies included in the Ubuntu packages

Please run the [rosdep](http://docs.ros.org/independent/api/rosdep/html/) utility:

```bash
$ sudo apt-get install python-rosdep
$ sudo rosdep init
$ rosdep install vision_utils
```

## 2. Dependencies from sources

Dependencies handling is based on the [wstool](http://wiki.ros.org/wstool) tool.
Run the following instructions:

```bash
$ sudo apt-get install python-wstool
$ sudo wstool init
$ wstool merge `rospack find people_detection_vision`/dependencies.rosinstall
$ roscd ; cd src
$ wstool update
```

How to cite this work
=====================

Use the following BiB entry

```bib
@phdthesis{
 title = {Local user mapping via multi-modal fusion for social robots},
 type = {phdthesis},
 year = {2015},
 pages = {274},
 websites = {https://sites.google.com/site/rameyarnaud/research/phd},
 institution = {Robotics Lab, Universidad Carlos III, Madrid, Spain},
 id = {f328e23d-c158-3598-a30f-30301cb0087f},
 created = {2015-06-19T17:09:06.000Z},
 file_attached = {false},
 profile_id = {b7924f35-7a80-333d-a823-0bc412c499bd},
 last_modified = {2015-06-22T09:27:23.000Z},
 tags = {Data fusion,Depth image,Image Processing,Kinect,ROS,Robotics,User Awareness},
 authored = {true},
 hidden = {false},
 bibtype = {phdthesis},
 author = {Ramey, Arnaud}
}
```

References
==========

* [Berliner and Hendel, 2007]
Berliner, T. and Hendel, Z. (2007). Modeling Of Humanoid Forms From Depth
Maps. United States Patent Application 20100034457.

* [Blodow and Rusu, 2009]
Blodow, N. and Rusu, R. (2009). Partial view modeling and validation in 3D
laser scans for grasping. Humanoid Robots, 2009.

* [Dalal and Triggs, 2005]
Dalal, N. and Triggs, B. (2005). Histograms of
oriented gradients for human detection. Computer Vision and Pattern
Recognition, 2005. CVPR 2005. IEEE Computer Society Conference on
(Volume:1 ).

* [Howard and Matthies, 2007].
Howard, A. and Matthies, L. (2007). Detect- ing pedestrians with stereo
vision: safe operation of autonomous ground vehicles in dynamic
environments. In Kaneko, Makoto; Nakamura, Y., editor, Proceedings of the
13th Int. Symp. of Robotics Research, Hiroshima, Japan.

* [Viola and Jones, 2001]
Viola, P. and Jones, M. (2001). Rapid object detection using a boosted
cascade of simple features. In Computer Vision and Pattern Recognition, IEEE
Computer Society Conference on
