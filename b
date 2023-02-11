#/bin/bash
source /opt/ros/humble/setup.bash
colcon build --packages-select jiggle --cmake-args -DCMAKE_BUILD_TYPE=Release