FROM osrf/ros:melodic-desktop-full
# RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt-get update && apt-get install -y \
  git 
  # \
  # python3-catkin-tools \
  # python3-osrf-pycommon
#RUN apt-get install -y \
#  ros-noetic-hector-*
#RUN apt-get install -y \
#  ros-noetic-xv-11-laser-driver
RUN apt-get update && apt-get install -y \
  ros-melodic-rosbridge-suite
#RUN apt-get install -y \
#  ros-melodic-rosbridge-suite
RUN git clone https://github.com/RainerKuemmerle/g2o.git
RUN cd g2o && mkdir build && cd build && cmake -j4 .. && make && make install
