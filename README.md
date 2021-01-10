ROS workspace for building and testing Jiggle SLAM

Docker images are supplied to build and run the projects. Local folders are mapped to expose github project contents within the docker images.

Can either connect to live lidar or test using pre-recorded lidar sessions from ROS bag files.

## Hacking
this project uses git submodules, to clone
```bash
git clone git@github.com:berickson/jiggle_ws.git
cd jiggle_ws
git submodule update --init --recursive
```

build the berickson/ros docker image
```bash
cd docker
./build
```
launch the docker image
```bash
cd docker
./start_no_lidar # or./start if running with lidar
```
from the docker terminal, start roscore
```bash
source devel/setup.bash
roscore
```
from host machine, you can launch more docker terminals to build and run modules
```bash
cd docker
./terminal
```
build from docker terminal
```bash
cd src
catkin build
```
run jiggle slam against back yard bag file (from docker terminal)
```
time rosrun jiggle process_bag data/a3-lab-bar-kitchen-2020-07-14-21-38-09.bag
# results will be in data/a3-back-yard-2020-07-14-21-40-47.bag.out.bag
```
view bag results in webviz (from host machine browser)
- browse to https://webviz.io/app/
- config menu item, paste content of webviz_layout.json
- drag output bag file from above
