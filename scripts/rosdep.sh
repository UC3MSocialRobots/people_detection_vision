#!/bin/sh
# External, ROS and system package dependencies

PACKAGES="ros-groovy-openni-camera
          ros-groovy-openni-tracker
          ros-groovy-openni-launch
          libgmp-dev
          ros-groovy-resource-retriever
          freeglut3
          freeglut3-dev"

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
    git clone https://github.com/LucidOne/ar_tools
    rospack profile
    rosmake ar_tools
fi
