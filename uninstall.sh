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


#Build 

#cd $SRC_DIR/Eigen3ToPython/$BUILD_SUBDIR
#make -j $NUMBER_OF_CORES
#$makeLevel -j$NUMBER_OF_CORES uninstall


cd $SRC_DIR/spdlog/$BUILD_SUBDIR
$makeLevel -j$NUMBER_OF_CORES uninstall

cd $SRC_DIR/mc_openrtm/$BUILD_SUBDIR
$makeLevel -j$NUMBER_OF_CORES uninstall

#cd $SRC_DIR/mc_hrp5_p/$BUILD_SUBDIR
#$makeLevel -j$NUMBER_OF_CORES uninstall

cd $SRC_DIR/mc_rtc/$BUILD_SUBDIR
$makeLevel -j$NUMBER_OF_CORES uninstall

cd $SRC_DIR/hpp-spline/$BUILD_SUBDIR
$makeLevel -j$NUMBER_OF_CORES uninstall

cd $SRC_DIR/Tasks/$BUILD_SUBDIR
$makeLevel -j$NUMBER_OF_CORES uninstall

cd $SRC_DIR/mc_rbdyn_urdf/$BUILD_SUBDIR
$makeLevel -j$NUMBER_OF_CORES uninstall

cd $SRC_DIR/RBDyn/$BUILD_SUBDIR
$makeLevel -j$NUMBER_OF_CORES uninstall

#cd $SRC_DIR/sch-core-python/$BUILD_SUBDIR
#$makeLevel -j$NUMBER_OF_CORES uninstall

cd $SRC_DIR/SpaceVecAlg/$BUILD_SUBDIR
$makeLevel -j$NUMBER_OF_CORES uninstall

cd $SRC_DIR/tinyxml2/$BUILD_SUBDIR
$makeLevel -j$NUMBER_OF_CORES uninstall

cd $SRC_DIR/eigen-lssol/$BUILD_SUBDIR
$makeLevel -j$NUMBER_OF_CORES uninstall

cd $SRC_DIR/eigen-qld/$BUILD_SUBDIR
$makeLevel -j$NUMBER_OF_CORES uninstall


if [ $REBUILD_HMC != 0 ]; then 
    cd $scriptDirectory
    ./build_hmc.sh    
fi
