#!/bin/bash

ws_dir=$(dirname $0)

catkin build -DCMAKE_BUILD_TYPE=Release
if [ $? -ne 0 ]; then
    echo "build failed"
    exit 0
fi
echo "running process_bag"

#BAGPATH=src/tu-darmstadt-ros-pkg-dataset_backup/
BAGPATH=${ws_dir}/data/

# uncomment one of below to select bag
BAG=a3-lab-bar-kitchen-2020-07-14-21-38-09.bag
#BAG=around-bar-3-x-2020-06-04-16-46-44.bag
#BAG=Team_Hector_MappingBox_Dagstuhl_Neubau.bag
#BAG=Team_Hector_MappingBox_L101_Building.bag
#BAG=Team_Hector_MappingBox_RoboCup_2011_Rescue_Arena.bag
#BAG=Team_Hector_MappingBox_RoboCupGermanOpen2011_Arena.bag

echo running $BAGPATH$BAG
time rosrun jiggle process_bag $BAGPATH$BAG 991000 > out.csv


#echo "done processing bag, launching Calc"
#soffice --calc out.csv &
