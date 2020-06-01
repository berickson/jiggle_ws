#!/bin/bash

catkin build -DCMAKE_BUILD_TYPE=Release
if [ $? -ne 0 ]; then
    echo "build failed"
    exit 0
fi
echo "running process_bag"

BAGPATH=src/tu-darmstadt-ros-pkg-dataset_backup/

# uncomment one of below to select gag

BAG=Team_Hector_MappingBox_Dagstuhl_Neubau.bag
#BAG=Team_Hector_MappingBox_L101_Building.bag
#BAG=Team_Hector_MappingBox_RoboCup_2011_Rescue_Arena.bag
#BAG=Team_Hector_MappingBox_RoboCupGermanOpen2011_Arena.bag

time rosrun jiggle process_bag $BAGPATH$BAG 991000 > out.csv


echo "done processing bag, launching Calc"

soffice --calc out.csv &