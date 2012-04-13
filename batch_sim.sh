#!/bin/sh

#algo=MinimumLengthNBVAlgorithm
algo=MCDMPrometheeNBVAlgorithm

export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/explore_beego
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/explore_stage2
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/explore2
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/bosch_maps2
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/bosch_worlds2
for i in $(seq 10); do
    roslaunch explore_stage2 explore.launch &
    stagepid=$!
    rosrun explore_beego NextBestViewAlgorithm.py $algo
    kill $stagepid
    wait $stagepid
    echo "resim"
    sleep 2
done

echo "done!"
