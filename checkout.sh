#!/usr/bin/env bash

RUNNINGSCRIPT="$0"
FILENAME="$(echo $(cd $(dirname "$BASH_SOURCE") && pwd -P)/$(basename "$BASH_SOURCE"))"
err_report() {
    echo "Error on line $2:$1"
    echo "Stopping the script $(basename "$3")."
}
trap 'err_report $LINENO $FILENAME $RUNNINGSCRIPT; exit 1' ERR
set -E -o pipefail 

source config.sh

#pull changes
git config --global url."https://gite.lirmm.fr/".insteadOf git@gite.lirmm.fr:
git config --global credential.helper cache #allows for only a single login

cd $SRC_DIR/Eigen3ToPython
git pull
git submodule update --recursive
cd $SRC_DIR/eigen-qld
git pull
git submodule update --recursive
cd $SRC_DIR/eigen-lssol #Requires login
git pull
git submodule update --recursive
cd $SRC_DIR/tinyxml2 
git pull
git submodule update --recursive
cd $SRC_DIR/sch-core-python 
git pull
git submodule update --recursive

cd $SRC_DIR/SpaceVecAlg 
git pull
git submodule update --recursive
cd $SRC_DIR/RBDyn
git pull
git submodule update --recursive
cd $SRC_DIR/mc_rbdyn_urdf
git pull
git submodule update --recursive
cd $SRC_DIR/Tasks
git pull
git submodule update --recursive
cd $SRC_DIR/hpp-spline
git pull
git submodule update --recursive

cd $SRC_DIR/hrp5_p_description
git pull
git submodule update --recursive
cd $SRC_DIR/mc_rtc_ros_data
git pull
git submodule update --recursive

cd $SRC_DIR/mc_rtc
git pull
git submodule update --recursive
cd $SRC_DIR/mc_hrp5_p
git pull
git submodule update --recursive
cd $SRC_DIR/mc_openrtm
git pull
git submodule update --recursive
