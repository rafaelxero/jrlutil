#!/usr/bin/env bash

source config.sh

#pull changes
git config --global url."https://gite.lirmm.fr/".insteadOf git@gite.lirmm.fr:
git config --global credential.helper cache #allows for only a single login
cd $SRC_DIR/Eigen3ToPython
git pull
cd $SRC_DIR/eigen-qld
git pull
cd $SRC_DIR/eigen-lssol #Requires login
git pull
cd $SRC_DIR/tinyxml2 
git pull
cd $SRC_DIR/sch-core-python 
git pull
cd $SRC_DIR/SpaceVecAlg 
git pull
cd $SRC_DIR/RBDyn
git pull
cd $SRC_DIR/mc_rbdyn_urdf
git pull
cd $SRC_DIR/Tasks
git pull

#Pull catkin workspace repos
cd $SRC_DIR/catkin_ws/src/hrp2_drc
git pull
cd $SRC_DIR/catkin_ws/src/hrp5/hrp5_p_description
git pull
#cd $SRC_DIR/catkin_ws/src/hrp4
#git pull
cd $SRC_DIR/catkin_ws/src/mc_rtc_ros_data
git pull
cd $SRC_DIR/catkin_ws/src/mc_rtc_ros
git pull

cd $SRC_DIR/mc_rtc
git pull
cd $SRC_DIR/mc_hrp5_p
git pull
cd $SRC_DIR/mc_openrtm
git pull
