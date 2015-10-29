
#!/bin/sh
# External, ROS and system package dependencies

# not compile 'binutils-gold' beacuse cause broken ld program fail

PACKAGES="ros-`rosversion -d`-openni-camera
          ros-`rosversion -d`-openni-launch
          ros-`rosversion -d`-resource-retriever
          libgstreamer0.10-dev
          libv4l-dev"

sudo apt-get install $PACKAGES
#~ exit

# ar_pose
if [ -z $ROS_WORKSPACE ]; then
    echo ""
    echo "###########################################################"
    echo 'ERROR. $ROS_WORKSPACE missing, check .bashrc file. Exiting.'
    echo "###########################################################"
    echo ""
else
    source $ROS_WORKSPACE/devel/setup.bash
    cd $ROS_WORKSPACE/src/
    git clone https://github.com/ar-tools/ar_tools.git
    cd $ROS_WORKSPACE
    catkin_make --only-pkg-with-deps artoolkit
    #~ catkin_make -DCATKIN_WHITELIST_PACKAGES=""
fi
