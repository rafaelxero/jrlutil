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
scriptDirectory="$(pwd)"

makeLevel="sudo make"
if [[ $INSTALL_DIR == $HOME* ]]
	then 
		makeLevel="make"
fi

#Build 
cd $SRC_DIR/Eigen3ToPython/build
make -j $NUBMBER_OF_CORES
$makeLevel -j$NUBMBER_OF_CORES install
cd $SRC_DIR/eigen-qld/build
make -j $NUBMBER_OF_CORES
$makeLevel -j$NUBMBER_OF_CORES install
cd $SRC_DIR/eigen-lssol/build
make -j $NUBMBER_OF_CORES
$makeLevel -j$NUBMBER_OF_CORES install
cd $SRC_DIR/tinyxml2/build
make -j $NUBMBER_OF_CORES
$makeLevel -j$NUBMBER_OF_CORES install
cd $SRC_DIR/SpaceVecAlg/build
make -j $NUBMBER_OF_CORES
$makeLevel -j$NUBMBER_OF_CORES install
cd $SRC_DIR/sch-core-python/build
make -j $NUBMBER_OF_CORES
$makeLevel -j$NUBMBER_OF_CORES install
cd $SRC_DIR/RBDyn/build
make -j $NUBMBER_OF_CORES
$makeLevel -j$NUBMBER_OF_CORES install
cd $SRC_DIR/mc_rbdyn_urdf/build
make -j $NUBMBER_OF_CORES
$makeLevel -j$NUBMBER_OF_CORES install
cd $SRC_DIR/Tasks/build
make -j $NUBMBER_OF_CORES
$makeLevel -j$NUBMBER_OF_CORES install

cd $SRC_DIR/catkin_ws/
catkin_make

cd $SRC_DIR/mc_rtc/build
make -j $NUBMBER_OF_CORES
$makeLevel -j$NUBMBER_OF_CORES install
cd $SRC_DIR/mc_hrp5_p/build
make -j $NUBMBER_OF_CORES
$makeLevel -j$NUBMBER_OF_CORES install
cd $SRC_DIR/mc_openrtm/build
make -j $NUBMBER_OF_CORES
$makeLevel -j$NUBMBER_OF_CORES install

if [ $REBUILD_HMC != 0 ]; then 
    cd $scriptDirectory
    ./build_hmc.sh    
fi