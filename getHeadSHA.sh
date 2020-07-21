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

echo "jrlutil headSHA" | tee $SRC_DIR/jrlutilHeadSHA.log

getSHA() {
    dir_name=$1
    package_name=$2
    if [ -e $dir_name ]; then
        cd "$dir_name/$package_name"
	echo -n "$package_name ... " | tee -a $SRC_DIR/jrlutilHeadSHA.log
	sha=$(git show -s --format=%H)
	 
	echo -n "$sha" | tee -a $SRC_DIR/jrlutilHeadSHA.log
	
	if [[ $(git diff --stat) != '' ]]; then
          echo -n ' ... dirty' | tee -a $SRC_DIR/jrlutilHeadSHA.log
        fi
        #insert new line
        echo  | tee -a $SRC_DIR/jrlutilHeadSHA.log 
        cd ../
    fi
}


keLevel="sudo make"
if [[ $INSTALL_DIR == $HOME* ]]
	then 
		makeLevel="make"
fi

#Build 


getSHA $SRC_DIR/ spdlog
getSHA $SRC_DIR/ Eigen3ToPython
getSHA $SRC_DIR/ eigen-qld
getSHA $SRC_DIR/ eigen-quadprog
getSHA $SRC_DIR/ eigen-lssol
getSHA $SRC_DIR/ tinyxml2
getSHA $SRC_DIR/ SpaceVecAlg
getSHA $SRC_DIR/ sch-core-python
getSHA $SRC_DIR/ RBDyn
getSHA $SRC_DIR/ mc_rbdyn_urdf
getSHA $SRC_DIR/ Tasks
getSHA $SRC_DIR/ hpp-spline
getSHA $SRC_DIR/ hrp5_p_description
getSHA $SRC_DIR/ mc_rtc_data
getSHA $SRC_DIR/ mc_rtc
getSHA $SRC_DIR/ mc_hrp5_p
getSHA $SRC_DIR/ mc_openrtm

