./config.sh
source config.sh
echo $HMC_BUILD_DIR
if [ -n $HMC_BUILD_DIR ]
	then 
		cd $HMC_BUILD_DIR
		make -j $NUBMBER_OF_CORES
		make -j$NUBMBER_OF_CORES install
fi
