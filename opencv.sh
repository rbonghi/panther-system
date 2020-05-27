#!/bin/bash
# Copyright (C) 2020, Raffaello Bonghi <raffaello@rnext.it>
# All rights reserved
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 1. Redistributions of source code must retain the above copyright 
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its 
#    contributors may be used to endorse or promote products derived 
#    from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND 
# CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, 
# BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
# OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
# EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

bold=`tput bold`
red=`tput setaf 1`
green=`tput setaf 2`
yellow=`tput setaf 3`
reset=`tput sgr0`


INSTALL_OPENCV_CONTRIB="YES"
INSTALL_OPENCV_EXTRA="YES"


opencv_installer()
{
    local NUM_CPU=$(nproc)
    # Local variables
    local LOCAL_FOLDER=$(pwd)
    local OPENCV_VERSION=$1
    local BUILD_FOLDER=$2
    
    echo "Install dependencies for $OPENCV_VERSION"
    # Install dependencies
    sudo apt-get install -y cmake pkg-config \
        libavcodec-dev libavformat-dev libavutil-dev libeigen3-dev \
        libglew-dev libgtk2.0-dev libgtk-3-dev libjpeg-dev libpng-dev \
        libpostproc-dev libswscale-dev libtbb-dev libtiff5-dev \
        libv4l-dev libxvidcore-dev libx264-dev qt5-default zlib1g-dev \
        libgl1 libglvnd-dev

    ### Download last stable opencv source code
    echo "${green}Download OpenCV $OPENCV_VERSION source code in $opencv_source_path${reset}"

    mkdir -p $BUILD_FOLDER
    cd $BUILD_FOLDER
    git clone https://github.com/opencv/opencv.git
    cd opencv
    git checkout -b v${OPENCV_VERSION} ${OPENCV_VERSION}
    
    if [ $INSTALL_OPENCV_CONTRIB == "YES" ] ; then
        echo "Installing opencv_contrib"
        git clone https://github.com/opencv/opencv_contrib.git
        cd opencv_contrib
        git checkout -b v${OPENCV_VERSION} ${OPENCV_VERSION}
        cd ..
    fi
    if [ $INSTALL_OPENCV_EXTRA == "YES" ] ; then
        echo "Installing opencv_extras"
        git clone https://github.com/opencv/opencv_extra.git
        cd opencv_extra
        git checkout -b v${OPENCV_VERSION} ${OPENCV_VERSION}
        cd ..
    fi
        
    echo "Build openCV $JETSON_DESCRIPTION for $JETSON_DESCRIPTION ${bold}($JETSON_CUDA_ARCH_BIN)${reset}"

    cd $BUILD_FOLDER/opencv
    
    if [ -d "build" ] ; then
        echo "${yellow}Clean old Build folder${reset}"
        rm -R build
    fi
    mkdir build 
    cd build

    # Add extra options for OpenCV contrib
    local opencv_contrib_option=""
    if [ $INSTALL_OPENCV_CONTRIB == "YES" ] ; then
        opencv_contrib_option="-D OPENCV_EXTRA_MODULES_PATH=../opencv_contrib/modules"
    fi
    
    # Add extra options for OpenCV extras
    local opencv_extras_option=""
    if [ $INSTALL_OPENCV_EXTRA == "YES" ] ; then
        opencv_extras_option="-D INSTALL_TESTS=ON -D OPENCV_TEST_DATA_PATH=../opencv_extra/testdata"
    fi

    # CMAKE
    time cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=$PATCH_OPENCV_INSTALL_PATH \
          -D WITH_CUDA=ON -D CUDA_ARCH_BIN=$JETSON_CUDA_ARCH_BIN -D CUDA_ARCH_PTX="" \
          -D WITH_CUBLAS=ON -D ENABLE_FAST_MATH=ON -D CUDA_FAST_MATH=ON \
          -D ENABLE_NEON=ON -D WITH_LIBV4L=ON -D BUILD_TESTS=OFF \
          -D BUILD_PERF_TESTS=OFF -D BUILD_EXAMPLES=OFF \
          -D WITH_GSTREAMER=ON -D WITH_GSTREAMER_0_10=OFF \
          -D BUILD_opencv_python2=ON -D BUILD_opencv_python3=ON \
          -D WITH_QT=ON -D WITH_OPENGL=ON $opencv_xavier_option $opencv_contrib_option $opencv_extras_option ..

    if [ $? -eq 0 ] ; then
        echo "CMake configuration make successful"
    else
      # Try to make again
      echo "${red}CMake issues " >&2
      echo "Please check the configuration being used${reset}"
      exit 1
    fi
    
    echo "Make openCV $OPENCV_VERSION with $NUM_CPU CPU"
    time make -j$(($NUM_CPU - 1))
    
    echo "Make install openCV $OPENCV_VERSION"
    sudo make install
    
    echo "ldconfig"
    sudo ldconfig
    
    echo "Remove OpenCV $OPENCV_VERSION source code in $opencv_source_path"
    cd $PATCH_OPENCV_SOURCE_PATH
    sudo rm -R opencv

    # Restore previuous folder
    cd $LOCAL_FOLDER
}


usage()
{
	if [ "$1" != "" ]; then
		echo "${red}$1${reset}"
	fi
	
    echo "opencv. Install OpenCV on a NVIDIA Jetson with CUDA"
    echo "Usage:"
    echo "$0 [options]"
    echo "options,"
    echo "   -h|--help                   | This help"
    echo "   -s|--silent                 | Run this script silent"
    echo "   -opv|--op-version [VERSION] | Version OpenCV to install"
    echo "   --folder [FOLDER]           | OpenCV build folder"
}


main()
{
    local OPENCV_VERSION=$JETSON_OPENCV
    local SILENT=false
    local NOCHECK=false
    local BUILD_FOLDER="/tmp"
	# Decode all information from startup
    while [ -n "$1" ]; do
        case "$1" in
            -h|--help) # Load help
                usage
                exit 0
                ;;
            -s|--silent)
                SILENT=true
                ;;
            -ocv|--ocv-version)
                OPENCV_VERSION=$2
                shift 1
                ;;
            --folder)
                BUILD_FOLDER=$2
                shift 1
                ;;
            *)
                usage "[ERROR] Unknown option: $1"
                exit 1
                ;;
        esac
            shift 1
    done

    # Check Jetson board
    if [ -z $JETSON_DESCRIPTION ] ; then
        echo "${red}jetson-stats not installed${reset}"
        exit 1
    fi
    if [ $JETSON_BOARD == "UNKNOWN" ] ; then
        echo "${red}Please check if jetson-stats is well installed${reset}"
        exit 1
    fi
    # Check CUDA version
    if [ -z $JETSON_CUDA ] ; then
        echo "${red}Please install CUDA${reset}"
        exit 1
    fi

    # Check Opencv version
    if [ -z $JETSON_OPENCV ] ; then
        echo "${red}Please select wich version is needed using option ${bold}-ocv${reset} or ${bold}--ocv-version${reset} ${reset}"
        exit 1
    fi
    # Exit if OpenCV is well installed with CUDA
    if [ $JETSON_OPENCV_CUDA == "YES" ] && [ $JETSON_OPENCV == $OPENCV_VERSION ] ; then
        echo "${green}OpenCV $JETSON_OPENCV is installed with CUDA $JETSON_CUDA${reset}"
        exit 0
    fi

    # Make info table
    echo "----- Installation -----"
    echo "$JETSON_DESCRIPTION [$JETSON_L4T]"
    echo "$JETSON_CUDA (ARCH BIN) $JETSON_CUDA_ARCH_BIN"
    echo "cuDNN $JETSON_CUDNN"
    # OpenCV version
    if [ $JETSON_OPENCV != $OPENCV_VERSION ] ; then
        echo "Change OpenCV from $JETSON_OPENCV -> ${bold}$OPENCV_VERSION${reset}"
    else
        echo "Install OpenCV ${bold}$OPENCV_VERSION${reset}"
    fi
    echo "OpenCV build folder $BUILD_FOLDER"
    echo "------------------------"

    # Ask before start install
    while ! $SILENT; do
        read -p "Do you want install OpenCV ${bold}$OPENCV_VERSION${reset}? [Y/n] " yn
            case $yn in
                [Yy]* ) break;;
                [Nn]* ) exit;;
            * ) echo "Please answer yes or no.";;
        esac
    done



}


main $@
exit 0

# EOF

