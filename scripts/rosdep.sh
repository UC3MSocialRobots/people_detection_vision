
#!/bin/sh
# External, ROS and system package dependencies

# not compile 'binutils-gold' beacuse cause broken ld program fail

PACKAGES="ros-`rosversion -d`-openni-camera
          ros-`rosversion -d`-openni-launch
          ros-`rosversion -d`-resource-retriever
          libgstreamer0.10-dev
          libv4l-dev"

sudo apt-get install $PACKAGES
exit

# ar_pose
source /opt/ros/indigo/setup.bash
source /home/arnaud/catkin_ws/devel/setup.bash
roscd
cd ../src/
git clone https://github.com/ar-tools/ar_tools.git
cd ..
catkin_make --only-pkg-with-deps artoolkit
#~ catkin_make -DCATKIN_WHITELIST_PACKAGES=""
