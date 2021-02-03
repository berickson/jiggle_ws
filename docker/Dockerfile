#FROM osrf/ros:melodic-desktop-full
FROM ros:melodic-perception
# RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt-get update && apt-get install -y \
  git \
  screen \
  # because DeserializationError cannot deserialize: unknown error handler name 'rosmsg' 
  ros-melodic-genpy \
  ros-melodic-rosbridge-suite \
  ros-melodic-hector-*

RUN apt-get update && apt-get install -y \
  htop \
  nano \
  python-catkin-tools

RUN apt-get update && apt-get install -y \
  python-pip

RUN pip install psutil

#RUN apt-get install -y \
#  ros-noetic-xv-11-laser-driver
  
# commenting out for now because it takes a long time
#RUN git clone https://github.com/RainerKuemmerle/g2o.git
#RUN cd g2o && mkdir build && cd build && cmake -j4 .. && make && make install
