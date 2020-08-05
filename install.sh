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
source $DRCUTIL_DIR/setup.bash

git config --global credential.helper cache #allows for only a single login
git config --global url."https://gite.lirmm.fr/".insteadOf git@gite.lirmm.fr:

soft_mkcd() 
{
    dirname="$1"
    if [ ! -d "$dirname" ]; then
        mkdir -p $dirname
    fi
    cd $dirname;
}

scriptDirectory="$(pwd)"
soft_mkcd $SRC_DIR

echo "Now in $scriptDirectory"

clone_repo()
{
    repoaddress="$1"
    reponame="$2"
    repoaddress="$1"
    reponame="$2"
    all=( ${@} )
    IFS=' '
    repooptions="${all[*]:3}"
    
    if [ ! -d $reponame ]; then 
        git clone --recursive $repoptions $repoaddress$reponame
    else
        echo "Directory $reponame exists, leaving"
    fi
    
}

cond_checkout()
{
    localbranch="$1"
    trackedbranch="$2"
    git checkout -b $localbranch --track $trackedbranch ||  git checkout  $localbranch   
}

failsafe_cmd()
{
 $( $@ ) || echo "Error occured, but moving on..."
}

echo "Clone repos"

clone_repo https://github.com/gabime/ spdlog -b v1.6.1 
clone_repo https://github.com/jrl-umi3218/ Eigen3ToPython
clone_repo https://github.com/jrl-umi3218/ eigen-qld
clone_repo https://github.com/jrl-umi3218/ eigen-quadprog
clone_repo https://gite.lirmm.fr/multi-contact/ eigen-lssol #Requires login
clone_repo https://github.com/leethomason/ tinyxml2 

clone_repo https://github.com/jrl-umi3218/ sch-core-python
cd sch-core-python/
failsafe_cmd git remote add rafaelxero https://github.com/rafaelxero/sch-core-python
git fetch rafaelxero
cond_checkout topic/HRG rafaelxero/topic/HRG
cd $SRC_DIR

clone_repo https://github.com/jrl-umi3218/ SpaceVecAlg

clone_repo https://github.com/jrl-umi3218/ RBDyn
cd RBDyn/
failsafe_cmd git remote add rafaelxero https://github.com/rafaelxero/RBDyn 
git fetch rafaelxero
failsafe_cmd cond_checkout topic/HRG rafaelxero/topic/HRG
cd $SRC_DIR

clone_repo https://github.com/jrl-umi3218/ mc_rbdyn_urdf
clone_repo https://github.com/jrl-umi3218/ Tasks
cd Tasks/
failsafe_cmd git remote add rafaelxero    https://github.com/rafaelxero/Tasks
git fetch rafaelxero
failsafe_cmd cond_checkout topic/HRG rafaelxero/topic/HRG

cd $SRC_DIR
clone_repo https://github.com/humanoid-path-planner/ hpp-spline

cd $SRC_DIR
clone_repo https://gite.lirmm.fr/mc-hrp5/ hrp5_p_description
cd $SRC_DIR
clone_repo https://github.com/jrl-umi3218/ mc_rtc_data

cd $SRC_DIR/
git config --global url."https://gite.lirmm.fr/".insteadOf git@gite.lirmm.fr:
clone_repo  https://gite.lirmm.fr/multi-contact/ mc_rtc
cd mc_rtc/
git checkout topic/HRG
cd ..
clone_repo https://gite.lirmm.fr/mc-hrp5/ mc_hrp5_p
cd mc_hrp5_p/
git checkout topic/HRG
cd ..

clone_repo  https://github.com/jrl-umi3218/ mc_openrtm

cd $SRC_DIR

echo "Build"

sudo apt-get install libgeos++-dev python-pip libyaml-cpp-dev gfortran
pip install nose
pip install Cython 
pip install coverage


export PBUI=OFF


cd $SRC_DIR
wget https://github.com/nanomsg/nanomsg/archive/1.1.5.zip
unzip 1.1.5.zip
#From nanomsg-1.1.5 README’s Quick Installation.
cd nanomsg-1.1.5/
soft_mkcd $BUILD_SUBDIR
cmake ..
cmake -D CMAKE_INSTALL_PREFIX="$INSTALL_DIR" .
cmake --build .
ctest .
echo "$SUDO cmake --build . --target install"
$SUDO cmake --build . --target install


cd $SRC_DIR/spdlog
soft_mkcd $BUILD_SUBDIR
cmake -D CMAKE_INSTALL_PREFIX="$INSTALL_DIR" -DSPDLOG_BUILD_EXAMPLE:BOOL=OFF -DSPDLOG_BUILD_SHARED:BOOL=ON -DCMAKE_BUILD_TYPE=$BUILD_TYPE ..
make -j $NUMBER_OF_CORES
$makeLevel -j$NUMBER_OF_CORES install

cd $SRC_DIR/Eigen3ToPython/
soft_mkcd $BUILD_SUBDIR
cmake ..
cmake -D CMAKE_INSTALL_PREFIX="$INSTALL_DIR" .
cmake -D PYTHON_BINDING_USER_INSTALL=$PBUI .
cmake -D CMAKE_BUILD_TYPE=$BUILD_TYPE .

make -j $NUMBER_OF_CORES
$makeLevel -j$NUMBER_OF_CORES install

cd $SRC_DIR/eigen-qld/
soft_mkcd $BUILD_SUBDIR
cmake ..
cmake -D CMAKE_INSTALL_PREFIX="$INSTALL_DIR" .
cmake -D PYTHON_BINDING_USER_INSTALL=$PBUI .
cmake -D CMAKE_BUILD_TYPE=$BUILD_TYPE .
make -j $NUMBER_OF_CORES
$makeLevel -j$NUMBER_OF_CORES install

cd $SRC_DIR/eigen-quadprog/
soft_mkcd $BUILD_SUBDIR
cmake ..
cmake -D CMAKE_INSTALL_PREFIX="$INSTALL_DIR" .
cmake -D CMAKE_BUILD_TYPE=$BUILD_TYPE .
make -j $NUMBER_OF_CORES
$makeLevel -j$NUMBER_OF_CORES install

cd $SRC_DIR/eigen-lssol/
soft_mkcd $BUILD_SUBDIR
cmake ..
cmake -D CMAKE_INSTALL_PREFIX="$INSTALL_DIR" .
cmake -D PYTHON_BINDING_USER_INSTALL=$PBUI .
cmake -D CMAKE_BUILD_TYPE=$BUILD_TYPE .
make -j $NUMBER_OF_CORES
$makeLevel -j$NUMBER_OF_CORES install

cd $SRC_DIR//tinyxml2/
soft_mkcd $BUILD_SUBDIR
cmake ..
cmake -D CMAKE_INSTALL_PREFIX="$INSTALL_DIR" .
cmake -D PYTHON_BINDING_USER_INSTALL=$PBUI .
cmake -D CMAKE_BUILD_TYPE=$BUILD_TYPE .
make -j $NUMBER_OF_CORES
$makeLevel -j$NUMBER_OF_CORES install

cd $SRC_DIR/SpaceVecAlg/
soft_mkcd $BUILD_SUBDIR
cmake -B. -H..
cmake -D CMAKE_INSTALL_PREFIX="$INSTALL_DIR" .
cmake -D PYTHON_BINDING_USER_INSTALL=$PBUI .
cmake -D CMAKE_BUILD_TYPE=$BUILD_TYPE .
make -j $NUMBER_OF_CORES
$makeLevel -j$NUMBER_OF_CORES install

cd $SRC_DIR/sch-core-python/
soft_mkcd $BUILD_SUBDIR
cmake -B. -H..
cmake -D CMAKE_INSTALL_PREFIX="$INSTALL_DIR" .
cmake -D PYTHON_BINDING_USER_INSTALL=$PBUI .
cmake -D CMAKE_BUILD_TYPE=$BUILD_TYPE .
make -j $NUMBER_OF_CORES
$makeLevel -j$NUMBER_OF_CORES install

cd $SRC_DIR/RBDyn/
soft_mkcd $BUILD_SUBDIR
cmake -B. -H..
cmake -D CMAKE_INSTALL_PREFIX="$INSTALL_DIR" .
cmake -D PYTHON_BINDING_USER_INSTALL=$PBUI .
cmake -D CMAKE_BUILD_TYPE=$BUILD_TYPE .
make -j $NUMBER_OF_CORES
$makeLevel -j$NUMBER_OF_CORES install

cd $SRC_DIR/mc_rbdyn_urdf/
soft_mkcd $BUILD_SUBDIR
cmake -B. -H..
cmake -D CMAKE_INSTALL_PREFIX="$INSTALL_DIR" .
cmake -D PYTHON_BINDING_USER_INSTALL=$PBUI .
cmake -D CMAKE_BUILD_TYPE=$BUILD_TYPE .
make -j $NUMBER_OF_CORES
$makeLevel -j$NUMBER_OF_CORES install

cd $SRC_DIR/Tasks/
soft_mkcd $BUILD_SUBDIR
cmake -B. -H..
cmake -D CMAKE_INSTALL_PREFIX="$INSTALL_DIR" .
cmake -D PYTHON_BINDING_USER_INSTALL=$PBUI .
cmake -D CMAKE_BUILD_TYPE=$BUILD_TYPE .
make -j $NUMBER_OF_CORES
$makeLevel -j$NUMBER_OF_CORES install

cd $SRC_DIR/hpp-spline/
soft_mkcd $BUILD_SUBDIR
cmake -D BUILD_PYTHON_INTERFACE=OFF -B. -H..
cmake -D CMAKE_INSTALL_PREFIX="$INSTALL_DIR" .
cmake -D PYTHON_BINDING_USER_INSTALL=$PBUI .
cmake -D CMAKE_BUILD_TYPE=$BUILD_TYPE .
make -j $NUMBER_OF_CORES
$makeLevel -j$NUMBER_OF_CORES install

cd $SRC_DIR/hrp5_p_description/
soft_mkcd $BUILD_SUBDIR
cmake -B. -H..
cmake -D DISABLE_ROS="ON" .
cmake -D CMAKE_INSTALL_PREFIX="$INSTALL_DIR" .
cmake -D CMAKE_BUILD_TYPE=$BUILD_TYPE .
make -j $NUMBER_OF_CORES
$makeLevel -j$NUMBER_OF_CORES install

cd $SRC_DIR/mc_rtc_data/
soft_mkcd $BUILD_SUBDIR
cmake -B. -H..
cmake -D DISABLE_ROS="ON" .
cmake -D CMAKE_INSTALL_PREFIX="$INSTALL_DIR" .
cmake -D CMAKE_BUILD_TYPE=$BUILD_TYPE .
make -j $NUMBER_OF_CORES
$makeLevel -j$NUMBER_OF_CORES install

echo "building mc_rtc"
cd $SRC_DIR/mc_rtc/
soft_mkcd $BUILD_SUBDIR
cmake -B. -H..
cmake -D DISABLE_ROS="ON" .
cmake -D CMAKE_INSTALL_PREFIX="$INSTALL_DIR" .
cmake -D PYTHON_BINDING_USER_INSTALL=$PBUI .
cmake -D CMAKE_BUILD_TYPE=$BUILD_TYPE .
make -j $NUMBER_OF_CORES
$makeLevel -j$NUMBER_OF_CORES install

cd $SRC_DIR/mc_hrp5_p/
soft_mkcd $BUILD_SUBDIR
cmake -B. -H..
cmake -D CMAKE_INSTALL_PREFIX="$INSTALL_DIR" .
cmake -D PYTHON_BINDING_USER_INSTALL=$PBUI .
cmake -D CMAKE_BUILD_TYPE=$BUILD_TYPE .
make -j $NUMBER_OF_CORES
$makeLevel -j$NUMBER_OF_CORES install

cd $SRC_DIR/mc_openrtm/
soft_mkcd $BUILD_SUBDIR
cmake -B. -H..
cmake -D CMAKE_INSTALL_PREFIX="$INSTALL_DIR" .
cmake -D PYTHON_BINDING_USER_INSTALL=$PBUI .
cmake -D CMAKE_BUILD_TYPE=$BUILD_TYPE .
make -j $NUMBER_OF_CORES
$makeLevel -j$NUMBER_OF_CORES install

if [ $REBUILD_HMC != 0 ]; then 
    cd $scriptDirectory
    ./build_hmc.sh    
fi
