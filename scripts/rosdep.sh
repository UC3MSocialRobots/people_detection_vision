#!/bin/sh
# External, ROS and system package dependencies

PACKAGES="ros-indigo-openni-camera
          ros-indigo-openni-tracker
          ros-indigo-openni-launch
          libgmp-dev
          ros-indigo-resource-retriever
          freeglut3
          freeglut3-dev
	  binutils-gold
          libgstreamer0.10-dev
          libv4l-dev"

sudo apt-get install $PACKAGES

# ar_pose
if [ -z $ROS_WORKSPACE ]; then
    echo ""
    echo "###########################################################"
    echo 'ERROR. $ROS_WORKSPACE missing, check .bashrc file. Exiting.'
    echo "###########################################################"
    echo ""
else
    cd $ROS_WORKSPACE
    #git clone https://github.com/LucidOne/ar_tools
    #indigo fork
    git clone -b catkin https://github.com/xqms/ar_tools.git    
    cd ..
    catkin_make --pkg ar_tools
fi
