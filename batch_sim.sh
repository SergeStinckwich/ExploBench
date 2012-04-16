#!/bin/sh

NSIM=10
ALGOS="GBLNBVAlgorithm MinimumLengthNBVAlgorithm MCDMPrometheeNBVAlgorithm "

sim () {
    algo=$1

    echo "start sim with $algo"
    roslaunch explore_stage2 explore.launch &
    stagepid=$!
    rosrun explore_beego NextBestViewAlgorithm.py $algo
    kill $stagepid
    wait $stagepid
}

export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/explore_beego
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/explore_stage2
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/explore2
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/bosch_maps2
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/bosch_worlds2

for algo in $ALGOS; do
    echo "start $NSIM sim with $algo"
    for i in $(seq $NSIM); do
        sim $algo
        sleep 2
    done
    echo "done $i sim with $algo"
    exit 0 # tmp
done

echo "done!"
