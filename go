#!/bin/bash

catkin build -DCMAKE_BUILD_TYPE=Release
if [ $? -ne 0 ]; then
    echo "build failed"
    exit 0
fi
echo "running process_bag"

# uncomment one of below to select gag
BAG=Team_Hector_MappingBox_Dagstuhl_Neubau.bag
#BAG=Team_Hector_MappingBox_L101_Building.bag
#BAG=Team_Hector_MappingBox_RoboCupGermanOpen2011_Arena.bag

time rosrun jiggle process_bag $BAG 991000 > out.csv


echo "done processing bag, launching Calc"

soffice --calc out.csv &