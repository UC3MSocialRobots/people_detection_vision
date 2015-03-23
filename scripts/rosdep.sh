
#!/bin/sh
# External, ROS and system package dependencies

PACKAGES="ros-hydro-openni-camera
          ros-hydro-openni-tracker
          ros-hydro-openni-launch
          libgmp-dev
          ros-hydro-resource-retriever
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
