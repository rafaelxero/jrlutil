./config.sh
source config.sh

makeLevel="sudo make"
if [ $INSTALL_DIR == $HOME/openrtp ]
	then 
		makeLevel="make"
fi

#Build 
cd /$SRC_DIR/Eigen3ToPython/build
make -j $NUBMBER_OF_CORES
$makeLevel -j$NUBMBER_OF_CORES install
cd /$SRC_DIR/eigen-qld/build
make -j $NUBMBER_OF_CORES
$makeLevel -j$NUBMBER_OF_CORES install
cd /$SRC_DIR/eigen-lssol/build
make -j $NUBMBER_OF_CORES
$makeLevel -j$NUBMBER_OF_CORES install
cd /$SRC_DIR/tinyxml2/build
make -j $NUBMBER_OF_CORES
$makeLevel -j$NUBMBER_OF_CORES install
cd /$SRC_DIR/SpaceVecAlg/build
make -j $NUBMBER_OF_CORES
$makeLevel -j$NUBMBER_OF_CORES install
cd /$SRC_DIR/sch-core-python/build
make -j $NUBMBER_OF_CORES
$makeLevel -j$NUBMBER_OF_CORES install
cd /$SRC_DIR/RBDyn/build
make -j $NUBMBER_OF_CORES
$makeLevel -j$NUBMBER_OF_CORES install
cd /$SRC_DIR/mc_rbdyn_urdf/build
make -j $NUBMBER_OF_CORES
$makeLevel -j$NUBMBER_OF_CORES install
cd /$SRC_DIR/Tasks/build
make -j $NUBMBER_OF_CORES
$makeLevel -j$NUBMBER_OF_CORES install

cd /$SRC_DIR/catkin_ws/
catkin_make

cd /$SRC_DIR/mc_rtc/build
make -j $NUBMBER_OF_CORES
$makeLevel -j$NUBMBER_OF_CORES install
cd /$SRC_DIR/mc_hrp5_p/build
make -j $NUBMBER_OF_CORES
$makeLevel -j$NUBMBER_OF_CORES install
cd /$SRC_DIR/mc_openrtm/build
make -j $NUBMBER_OF_CORES
$makeLevel -j$NUBMBER_OF_CORES install

./build_hmc.sh
