source config.sh


containsElement () {

if [ $REBUILD_HMC != 0 ] then 
    cd $DRCUTIL_DIR
    export CMAKE_ADDITIONAL_OPTIONS=($CMAKE_ADDITIONAL_OPTIONS -D BUILD_MULTI_CONTACT_MOTION_SOL=ON -D CMAKE_CXX_FLAGS="$CMAKE_CXX_FLAGS -std=c++11")
    source config.sh
    if [[ "${CMAKE_ADDITIONAL_OPTIONS[@]}" =~ "c++11" ]]; then #checks if the options are taken into account
        ./install hmc2
    else
        echo "Error in build_hmc.sh"
        echo "Please check that the value of CMAKE_ADDITIONAL_OPTIONS is not errased in $DRCUTIL_DIR/config.sh"
        echo "After fixing the issue. YOu may directly run this script ./build_hmc.sh"
    fi
    
fi
