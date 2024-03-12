FROM osrf/ros:melodic-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies
RUN apt update && apt install -y\
 ros-melodic-smach\
 ros-melodic-smach-ros\
 ros-melodic-executive-smach\
 ros-melodic-smach-viewer\
 vim\
 nano\
 python-rosdep\
 python-rosinstall\
 python-rosinstall-generator\
 python-wstool\
 build-essential\
 ros-melodic-catkin\
 python-catkin-tools\
 python-scipy

# Update dependencies
RUN rosdep update
RUN apt-get update && apt-get dist-upgrade -y

# Create a Catkin Workspace
RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws
RUN . /opt/ros/melodic/setup.sh && \
 catkin_make

# Download ROS packages
WORKDIR /root/catkin_ws/src
RUN git clone https://github.com/uuvsimulator/uuv_simulator.git
RUN git clone https://github.com/uuvsimulator/desistek_saga.git
RUN git clone https://github.com/uuvsimulator/rexrov2.git
RUN git clone https://github.com/stereolabs/zed-ros-interfaces.git
RUN git clone https://github.com/Rumarino-Team/Hydrus.git



#  We add this for  solving the following issue:
#https://github.com/uuvsimulator/uuv_simulator/issues/376
# The solution was to add the following line to the package.xml file of uuv_simulation_evaluation
#<build_depend>uuv_simulation_evaluation<\/build_depend>
# https://github.com/uuvsimulator/uuv_simulation_evaluation/pull/54/files

# This issue seems to happen when we install other packages  in the same workspace
# that are not from the uuvsimulator organization.
RUN git clone https://github.com/uuvsimulator/uuv_simulation_evaluation.git && \
sed -i '/<buildtool_depend>catkin<\/buildtool_depend>/a \ \ <build_depend>uuv_simulation_evaluation<\/build_depend>' /root/catkin_ws/src/uuv_simulation_evaluation/uuv_smac_utils/package.xml


# Install packages and their dependencies
WORKDIR /root/catkin_ws
RUN . /opt/ros/melodic/setup.sh && \
 . /root/catkin_ws/devel/setup.sh && \
 catkin_make install

# Add setup.sh for catkin workspace to bashrc
RUN echo "source /root/catkin_ws/devel/setup.sh" >> ~/.bashrc
