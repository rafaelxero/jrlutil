#!/usr/bin/env bash

source config.sh
source $DRCUTIL_DIR/setup.bash
source $SRC_DIR/catkin_ws/devel/setup.bash

RUNNINGSCRIPT="$0"
FILENAME="$(echo $(cd $(dirname "$BASH_SOURCE") && pwd -P)/$(basename "$BASH_SOURCE"))"
err_report() {
    echo "Error on line $2:$1"
    echo "Stopping the script $(basename "$3")."
}
trap 'err_report $LINENO $FILENAME $RUNNINGSCRIPT; exit 1' ERR
set -E -o pipefail 


if [ $REBUILD_HMC != 0 ]; then 
    CMAKE_OPT="-DBUILD_MULTI_CONTACT_MOTION_SOLVER=ON"
    export CMAKE_ADDITIONAL_OPTIONS=$CMAKE_OPT
    export CXXFLAGS="$CXXFLAGS -std=c++11"
    echo "entering $DRCUTIL_DIR"
    cd $DRCUTIL_DIR
    source config.sh
    if [[ "${CMAKE_ADDITIONAL_OPTIONS[@]}" =~ $CMAKE_OPT ]]; then #checks if the options are taken into account (temporary code)
        unset CMAKE_ADDITIONAL_OPTIONS
        export CMAKE_ADDITIONAL_OPTIONS=$CMAKE_OPT
        ./install.sh hmc2
    else
        echo "Error in $RUNNINGSCRIPT"
        echo "Please check that the value of CMAKE_ADDITIONAL_OPTIONS is not errased in $DRCUTIL_DIR/config.sh"
        echo "After fixing the issue. YOu may directly run this script ./build_hmc.sh"
    fi
    
fi
