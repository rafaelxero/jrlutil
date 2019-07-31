./config.sh
source config.sh
echo $HMC_BUILD_DIR
if [ -n $HMC_BUILD_DIR ]
	then 
		cd $HMC_BUILD_DIR
		cmake -D BUILD_MULTI_CONTACT_MOTION_SOL=ON .
		cmake -D CMAKE_CXX_FLAGS="-g -std=c++11" . 	
		make -j $NUBMBER_OF_CORES
		make -j$NUBMBER_OF_CORES install
fi
