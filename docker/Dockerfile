FROM ros:melodic-robot
RUN apt-get update && apt-get install -y \
  python-catkin-tools \
  ros-melodic-desktop-full 
RUN apt-get install -y \
  ros-melodic-hector-*
RUN apt-get install -y \
  ros-melodic-xv-11-laser-driver
#RUN apt-get install -y \
#  ros-melodic-rosbridge-suite
RUN git clone https://github.com/RainerKuemmerle/g2o.git
RUN cd g2o && mkdir build && cd build && cmake .. && make && make install