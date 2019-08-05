./config.sh
source config.sh

mkdir $SRC_DIR
cd $SRC_DIR
scriptDirectory="$(pwd)"

#Clone repos
git clone --recursive https://github.com/jrl-umi3218/Eigen3ToPython
git clone --recursive  https://github.com/jrl-umi3218/eigen-qld
git config --global credential.helper cache #allows for only a single login
git clone --recursive https://gite.lirmm.fr/multi-contact/eigen-lssol.git #Requires login

wget https://github.com/nanomsg/nanomsg/archive/1.1.5.zip
unzip 1.1.5.zip
#From nanomsg-1.1.5 README’s Quick Installation.
cd nanomsg-1.1.5/
mkdir build
cd build
cmake ..
cmake --build .
ctest .
sudo cmake --build . --target install

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
		#Install ROS
		sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
		wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
		sudo apt update
		sudo apt install ros-kinetic-desktop-full python-rosinstall
		sudo rosdep init
		rosdep update
                echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
                source /opt/ros/kinetic/setup.bash            
fi

#Create Catkin Workspace
mkdir -p catkin_ws/src
cd catkin_ws/src
catkin_init_workspace
cd ..
catkin_make
echo "source $SRC_DIR/catkin_ws/devel/setup.bash" >> ~/.bashrc
source $SRC_DIR/catkin_ws/devel/setup.bash

#Clone catkin workspace repos
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

#Build 
export PBUI=OFF
makeLevel="sudo make"
if [[ $INSTALL_DIR == $HOME* ]]
	then 
		export PBUI=ON
		makeLevel="make"
fi

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

cd $SRC_DIR/catkin_ws/
catkin_make

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

cd $SRC_DIR/catkin_ws/src
git clone --recursive https://gite.lirmm.fr/multi-contact/mc_rtc_ros
cd $SRC_DIR/catkin_ws
catkin_make

cd $scriptDirectory
./build_hmc.sh
