FROM ros:noetic-perception-focal

# Change the default shell to Bash
SHELL [ "/bin/bash" , "-c" ]

RUN apt-get -y update

RUN apt-get -y install libyaml-cpp-dev

RUN apt-get -y install libeigen3-dev

# Create a Catkin workspace 
RUN . /opt/ros/noetic/setup.bash \
 &&	mkdir -p catkin_ws/src \
 && cd catkin_ws/src \
 && catkin_init_workspace

# copy package to source
COPY multicamera_calibration /catkin_ws/src/multicamera_calibration

# build package
RUN . /opt/ros/noetic/setup.bash \
 &&  cd /catkin_ws \
 &&	catkin_make

# source ROS for startup
CMD source "/opt/ros/noetic/setup.bash"



