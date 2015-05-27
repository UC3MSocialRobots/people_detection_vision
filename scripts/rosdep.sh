
#!/bin/sh
# External, ROS and system package dependencies

# not compile 'binutils-gold' beacuse cause broken ld program fail

PACKAGES="ros-`rosversion -d`-openni-camera
          ros-`rosversion -d`-openni-tracker
          ros-`rosversion -d`-openni-launch
          libgmp-dev
          ros-`rosversion -d`-resource-retriever
          freeglut3
          freeglut3-dev
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

  echo ""
    echo "###########################################################"
    echo 'WARNING. This script will delete your build folder'
    echo "###########################################################"
    echo ""

    cd $ROS_WORKSPACE
    git clone -b catkin https://github.com/xqms/ar_tools src/ar_tools
	catkin_make --only-pkg-with-deps artoolkit
	catkin_make --only-pkg-with-deps ar_tools

fi
