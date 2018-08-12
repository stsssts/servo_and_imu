#!/usr/bin/env bash

pushd ~/test_ws/devel/lib/servo_and_imu # cd to the directory with your node
sudo chown root:root servo_and_imu_node # change ownship to root
sudo chmod a+rx servo_and_imu_node      # set as executable by all
sudo chmod u+s servo_and_imu_node       # set the setuid bit
popd

