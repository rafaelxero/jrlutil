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
git clone --recursive https://github.com/jrl-umi3218/Eigen3ToPython
git clone --recursive  https://github.com/jrl-umi3218/eigen-qld
git clone --recursive https://gite.lirmm.fr/multi-contact/eigen-lssol.git #Requires login

cd $SRC_DIR
git clone --recursive  https://github.com/leethomason/tinyxml2 
git clone --recursive https://github.com/jrl-umi3218/sch-core-python
cd sch-core-python/
git remote add rafaelxero https://github.com/rafaelxero/sch-core-python
git fetch rafaelxero
git checkout -b topic/HRG --track rafaelxero/topic/HRG
cd $SRC_DIR
git clone --recursive https://github.com/jrl-umi3218/SpaceVecAlg
git clone --recursive https://github.com/jrl-umi3218/RBDyn
cd RBDyn/
git remote add rafaelxero https://github.com/rafaelxero/RBDyn
git fetch rafaelxero
git checkout -b topic/HRG --track rafaelxero/topic/HRG
cd $SRC_DIR
git clone --recursive https://github.com/jrl-umi3218/mc_rbdyn_urdf.git
git clone --recursive  https://github.com/jrl-umi3218/Tasks.git
cd Tasks/
git remote add rafaelxero    https://github.com/rafaelxero/Tasks
git fetch rafaelxero
git checkout -b topic/HRG --track rafaelxero/topic/HRG


cd $SRC_DIR
if [ "$(rosversion -d)" != "kinetic" ]
	then
		echo "Install ROS"
		sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
		wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
		sudo apt update
		sudo apt install ros-kinetic-desktop-full python-rosinstall
		sudo rosdep init
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
mkdir -p catkin_ws/src
cd catkin_ws/src
catkin_init_workspace
cd ..
catkin_make
echo "source $SRC_DIR/catkin_ws/devel/setup.bash" >> ~/.bashrc
source $SRC_DIR/catkin_ws/devel/setup.bash

echo "Clone catkin workspace repos"
cd $SRC_DIR/catkin_ws/src/
git clone --recursive https://gite.lirmm.fr/mc-hrp2/hrp2_drc
mkdir hrp5
cd hrp5/
git clone --recursive https://gite.lirmm.fr/mc-hrp5/hrp5_p_description.git
#cd $SRC_DIR/catkin_ws/src/
#git clone --recursive https://gite.lirmm.fr/mc-hrp4/hrp4
cd $SRC_DIR/catkin_ws/src/
git clone --recursive https://gite.lirmm.fr/multi-contact/mc_rtc_ros_data


cd $SRC_DIR/
git config --global url."https://gite.lirmm.fr/".insteadOf git@gite.lirmm.fr:
git clone --recursive https://gite.lirmm.fr/multi-contact/mc_rtc.git
cd mc_rtc/
git checkout topic/HRG
cd ..
git clone --recursive https://gite.lirmm.fr/mc-hrp5/mc_hrp5_p.git
cd mc_hrp5_p/
git checkout topic/HRG
cd ..
git clone --recursive https://gite.lirmm.fr/multi-contact/mc_openrtm.git
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
mkdir build
cd build
cmake ..
cmake -D CMAKE_INSTALL_PREFIX="$INSTALL_DIR" .
cmake --build .
ctest .
$makeLevel cmake --build . --target install

cd $SRC_DIR/Eigen3ToPython/
mkdir build
cd build/
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
mkdir build
cd build/
cmake ..
cmake -D CMAKE_INSTALL_PREFIX="$INSTALL_DIR" .
cmake -D PYTHON_BINDING_USER_INSTALL=$PBUI .
cmake -D CMAKE_BUILD_TYPE=$BUILD_TYPE .
make -j $NUBMBER_OF_CORES
$makeLevel -j$NUBMBER_OF_CORES install

cd $SRC_DIR/eigen-lssol/
mkdir build
cd build/
cmake ..
cmake -D CMAKE_INSTALL_PREFIX="$INSTALL_DIR" .
cmake -D PYTHON_BINDING_USER_INSTALL=$PBUI .
cmake -D CMAKE_BUILD_TYPE=$BUILD_TYPE .
make -j $NUBMBER_OF_CORES
$makeLevel -j$NUBMBER_OF_CORES install

cd $SRC_DIR//tinyxml2/
mkdir build
cd build/
cmake ..
cmake -D CMAKE_INSTALL_PREFIX="$INSTALL_DIR" .
cmake -D PYTHON_BINDING_USER_INSTALL=$PBUI .
cmake -D CMAKE_BUILD_TYPE=$BUILD_TYPE .
make -j $NUBMBER_OF_CORES
$makeLevel -j$NUBMBER_OF_CORES install

cd $SRC_DIR/SpaceVecAlg/
mkdir build
cd build/
cmake -B. -H..
cmake -D CMAKE_INSTALL_PREFIX="$INSTALL_DIR" .
cmake -D PYTHON_BINDING_USER_INSTALL=$PBUI .
cmake -D CMAKE_BUILD_TYPE=$BUILD_TYPE .
make -j $NUBMBER_OF_CORES
$makeLevel -j$NUBMBER_OF_CORES install

cd $SRC_DIR/sch-core-python/
mkdir build
cd build/
cmake -B. -H..
cmake -D CMAKE_INSTALL_PREFIX="$INSTALL_DIR" .
cmake -D PYTHON_BINDING_USER_INSTALL=$PBUI .
cmake -D CMAKE_BUILD_TYPE=$BUILD_TYPE .
make -j $NUBMBER_OF_CORES
$makeLevel -j$NUBMBER_OF_CORES install

cd $SRC_DIR/RBDyn/
mkdir build
cd build/
cmake -B. -H..
cmake -D CMAKE_INSTALL_PREFIX="$INSTALL_DIR" .
cmake -D PYTHON_BINDING_USER_INSTALL=$PBUI .
cmake -D CMAKE_BUILD_TYPE=$BUILD_TYPE .
make -j $NUBMBER_OF_CORES
$makeLevel -j$NUBMBER_OF_CORES install

cd $SRC_DIR/mc_rbdyn_urdf/
mkdir build
cd build/
cmake -B. -H..
cmake -D CMAKE_INSTALL_PREFIX="$INSTALL_DIR" .
cmake -D PYTHON_BINDING_USER_INSTALL=$PBUI .
cmake -D CMAKE_BUILD_TYPE=$BUILD_TYPE .
make -j $NUBMBER_OF_CORES
$makeLevel -j$NUBMBER_OF_CORES install

cd $SRC_DIR/Tasks/
mkdir build
cd build/
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
mkdir build
cd build/
cmake -B. -H..
cmake -D CMAKE_INSTALL_PREFIX="$INSTALL_DIR" .
cmake -D PYTHON_BINDING_USER_INSTALL=$PBUI .
cmake -D CMAKE_BUILD_TYPE=$BUILD_TYPE .
make -j $NUBMBER_OF_CORES
$makeLevel -j$NUBMBER_OF_CORES install

cd $SRC_DIR/mc_hrp5_p/
mkdir build
cd build/
cmake -B. -H..
cmake -D CMAKE_INSTALL_PREFIX="$INSTALL_DIR" .
cmake -D PYTHON_BINDING_USER_INSTALL=$PBUI .
cmake -D CMAKE_BUILD_TYPE=$BUILD_TYPE .
make -j $NUBMBER_OF_CORES
$makeLevel -j$NUBMBER_OF_CORES install

cd $SRC_DIR/mc_openrtm/
mkdir build
cd build/
cmake -B. -H..
cmake -D CMAKE_INSTALL_PREFIX="$INSTALL_DIR" .
cmake -D PYTHON_BINDING_USER_INSTALL=$PBUI .
cmake -D CMAKE_BUILD_TYPE=$BUILD_TYPE .
make -j $NUBMBER_OF_CORES
$makeLevel -j$NUBMBER_OF_CORES install

echo "cloning mc_rtc_ros"
cd $SRC_DIR/catkin_ws/src
git clone --recursive https://gite.lirmm.fr/multi-contact/mc_rtc_ros

echo "building catkin workspace"
cd $SRC_DIR/catkin_ws
catkin_make


cd $scriptDirectory
./build_hmc.sh
