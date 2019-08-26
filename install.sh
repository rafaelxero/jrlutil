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

git config --global credential.helper cache #allows for only a single login

soft_mkcd() 
{
    dirname="$1"
    if [ ! -d "$dirname" ]; then
        mkdir -p $dirname
    fi
    cd $dirname;
}



soft_mkcd $SRC_DIR

scriptDirectory="$(pwd)"
echo "Now in $scriptDirectory"

clone_repo()
{
    repoaddress="$1"
    reponame="$2"
    
    if [ ! -d $reponame ]; then 
        git clone --recursive $repoaddress$reponame
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
clone_repo https://github.com/jrl-umi3218/ Eigen3ToPython
clone_repo https://github.com/jrl-umi3218/ eigen-qld

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
cond_checkout topic/HRG rafaelxero/topic/HRG
cd $SRC_DIR

clone_repo https://github.com/jrl-umi3218/ mc_rbdyn_urdf
clone_repo https://github.com/jrl-umi3218/ Tasks
cd Tasks/
failsafe_cmd git remote add rafaelxero    https://github.com/rafaelxero/Tasks
git fetch rafaelxero
cond_checkout topic/HRG rafaelxero/topic/HRG


cd $SRC_DIR
if [ "$(rosversion -d)" != "kinetic" ]
	then
		echo "Install ROS"
		sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
		wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
		failsafe_cmd sudo apt update
		sudo apt install ros-kinetic-desktop-full python-rosinstall
		failsafe_cmd sudo rosdep init
		rosdep update
                echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
                source /opt/ros/kinetic/setup.bash            
fi

echo "remove mc_rtc_ros if it exists"
if [ -d "$SRC_DIR/catkin_ws/src/mc_rtc_ros" ]
then
	sudo rm -r $SRC_DIR/catkin_ws/src/mc_rtc_ros
fi

echo "Create Catkin Workspace"
soft_mkcd catkin_ws/src
failsafe_cmd catkin_init_workspace
cd ..
catkin_make
echo "source $SRC_DIR/catkin_ws/devel/setup.bash" >> ~/.bashrc
source $SRC_DIR/catkin_ws/devel/setup.bash

echo "Clone catkin workspace repos"
cd $SRC_DIR/catkin_ws/src/
clone_repo https://gite.lirmm.fr/mc-hrp2/ hrp2_drc
soft_mkcd hrp5/
clone_repo https://gite.lirmm.fr/mc-hrp5/ hrp5_p_description
#cd $SRC_DIR/catkin_ws/src/
#clone_repo --recursive https://gite.lirmm.fr/mc-hrp4/ hrp4
cd $SRC_DIR/catkin_ws/src/
clone_repo https://gite.lirmm.fr/multi-contact/ mc_rtc_ros_data


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
clone_repo  https://gite.lirmm.fr/multi-contact/ mc_openrtm
cd mc_openrtm/
git checkout topic/HRG

echo "Build"

sudo apt-get install libgeos++-dev

export PBUI=OFF
makeLevel="sudo make"
if [[ $INSTALL_DIR == $HOME* ]]
	then 
		export PBUI=ON
		makeLevel="make"
fi

cd $SRC_DIR
wget https://github.com/nanomsg/nanomsg/archive/1.1.5.zip
unzip 1.1.5.zip
#From nanomsg-1.1.5 README’s Quick Installation.
cd nanomsg-1.1.5/
soft_mkcd build
cmake ..
cmake -D CMAKE_INSTALL_PREFIX="$INSTALL_DIR" .
cmake --build .
ctest .
echo "$makeLevel cmake --build . --target install"
cmake --build . --target install

cd $SRC_DIR/Eigen3ToPython/
soft_mkcd build
cmake ..
cmake -D CMAKE_INSTALL_PREFIX="$INSTALL_DIR" .
cmake -D PYTHON_BINDING_USER_INSTALL=$PBUI .
cmake -D CMAKE_BUILD_TYPE=$BUILD_TYPE .
sudo apt-get install python-pip
pip install Cython 
pip install coverage
make -j $NUBMBER_OF_CORES
$makeLevel -j$NUBMBER_OF_CORES install

cd $SRC_DIR/eigen-qld/
soft_mkcd build
cmake ..
cmake -D CMAKE_INSTALL_PREFIX="$INSTALL_DIR" .
cmake -D PYTHON_BINDING_USER_INSTALL=$PBUI .
cmake -D CMAKE_BUILD_TYPE=$BUILD_TYPE .
make -j $NUBMBER_OF_CORES
$makeLevel -j$NUBMBER_OF_CORES install

cd $SRC_DIR/eigen-lssol/
soft_mkcd build
cmake ..
cmake -D CMAKE_INSTALL_PREFIX="$INSTALL_DIR" .
cmake -D PYTHON_BINDING_USER_INSTALL=$PBUI .
cmake -D CMAKE_BUILD_TYPE=$BUILD_TYPE .
make -j $NUBMBER_OF_CORES
$makeLevel -j$NUBMBER_OF_CORES install

cd $SRC_DIR//tinyxml2/
soft_mkcd build
cmake ..
cmake -D CMAKE_INSTALL_PREFIX="$INSTALL_DIR" .
cmake -D PYTHON_BINDING_USER_INSTALL=$PBUI .
cmake -D CMAKE_BUILD_TYPE=$BUILD_TYPE .
make -j $NUBMBER_OF_CORES
$makeLevel -j$NUBMBER_OF_CORES install

cd $SRC_DIR/SpaceVecAlg/
soft_mkcd build
cmake -B. -H..
cmake -D CMAKE_INSTALL_PREFIX="$INSTALL_DIR" .
cmake -D PYTHON_BINDING_USER_INSTALL=$PBUI .
cmake -D CMAKE_BUILD_TYPE=$BUILD_TYPE .
make -j $NUBMBER_OF_CORES
$makeLevel -j$NUBMBER_OF_CORES install

cd $SRC_DIR/sch-core-python/
soft_mkcd build
cmake -B. -H..
cmake -D CMAKE_INSTALL_PREFIX="$INSTALL_DIR" .
cmake -D PYTHON_BINDING_USER_INSTALL=$PBUI .
cmake -D CMAKE_BUILD_TYPE=$BUILD_TYPE .
make -j $NUBMBER_OF_CORES
$makeLevel -j$NUBMBER_OF_CORES install

cd $SRC_DIR/RBDyn/
soft_mkcd build
cmake -B. -H..
cmake -D CMAKE_INSTALL_PREFIX="$INSTALL_DIR" .
cmake -D PYTHON_BINDING_USER_INSTALL=$PBUI .
cmake -D CMAKE_BUILD_TYPE=$BUILD_TYPE .
make -j $NUBMBER_OF_CORES
$makeLevel -j$NUBMBER_OF_CORES install

cd $SRC_DIR/mc_rbdyn_urdf/
soft_mkcd build
cmake -B. -H..
cmake -D CMAKE_INSTALL_PREFIX="$INSTALL_DIR" .
cmake -D PYTHON_BINDING_USER_INSTALL=$PBUI .
cmake -D CMAKE_BUILD_TYPE=$BUILD_TYPE .
make -j $NUBMBER_OF_CORES
$makeLevel -j$NUBMBER_OF_CORES install

cd $SRC_DIR/Tasks/
soft_mkcd build
cmake -B. -H..
cmake -D CMAKE_INSTALL_PREFIX="$INSTALL_DIR" .
cmake -D PYTHON_BINDING_USER_INSTALL=$PBUI .
cmake -D CMAKE_BUILD_TYPE=$BUILD_TYPE .
make -j $NUBMBER_OF_CORES
$makeLevel -j$NUBMBER_OF_CORES install

echo "building catkin workspace"
cd $SRC_DIR/catkin_ws/
catkin_make

echo "building mc_rtc"
cd $SRC_DIR/mc_rtc/
soft_mkcd build
cmake -B. -H..
cmake -D CMAKE_INSTALL_PREFIX="$INSTALL_DIR" .
cmake -D PYTHON_BINDING_USER_INSTALL=$PBUI .
cmake -D CMAKE_BUILD_TYPE=$BUILD_TYPE .
make -j $NUBMBER_OF_CORES
$makeLevel -j$NUBMBER_OF_CORES install

cd $SRC_DIR/mc_hrp5_p/
soft_mkcd build
cmake -B. -H..
cmake -D CMAKE_INSTALL_PREFIX="$INSTALL_DIR" .
cmake -D PYTHON_BINDING_USER_INSTALL=$PBUI .
cmake -D CMAKE_BUILD_TYPE=$BUILD_TYPE .
make -j $NUBMBER_OF_CORES
$makeLevel -j$NUBMBER_OF_CORES install

cd $SRC_DIR/mc_openrtm/
soft_mkcd build
cmake -B. -H..
cmake -D CMAKE_INSTALL_PREFIX="$INSTALL_DIR" .
cmake -D PYTHON_BINDING_USER_INSTALL=$PBUI .
cmake -D CMAKE_BUILD_TYPE=$BUILD_TYPE .
make -j $NUBMBER_OF_CORES
$makeLevel -j$NUBMBER_OF_CORES install

echo "cloning mc_rtc_ros"
cd $SRC_DIR/catkin_ws/src
clone_repo https://gite.lirmm.fr/multi-contact/ mc_rtc_ros

echo "building catkin workspace"
cd $SRC_DIR/catkin_ws
catkin_make


cd $scriptDirectory
./build_hmc.sh
