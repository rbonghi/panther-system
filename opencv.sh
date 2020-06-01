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

OPENCV_VERSION=$JETSON_OPENCV
INSTALL_OPENCV_PATH="/usr/local"
INSTALL_OPENCV_CONTRIB="YES"
INSTALL_OPENCV_PYTHON2="YES"
INSTALL_OPENCV_PYTHON3="YES"
BUILD_FOLDER=$HOME
FORCE=false


opencv_downloader()
{
    echo "${green}Install dependencies for $OPENCV_VERSION${reset}"
    # Install dependencies
    sudo apt-get install -y build-essential cmake git gfortran pkg-config \
        libatlas-base-dev libavcodec-dev libavformat-dev libavresample-dev libgtk-3-dev \
        libcanberra-gtk3-module libdc1394-22-dev libeigen3-dev libglew-dev \
        libgstreamer-plugins-base1.0-dev libgstreamer-plugins-good1.0-dev libgstreamer1.0-dev \
        libjpeg-dev libjpeg8-dev libjpeg-turbo8-dev liblapack-dev liblapacke-dev \
        libopenblas-dev libpng-dev libpostproc-dev libswscale-dev libtbb-dev libtbb2 \
        libtesseract-dev libtiff-dev libv4l-dev libxine2-dev libxvidcore-dev libx264-dev \
        qv4l2 v4l-utils v4l2ucp zlib1g-dev
    if [ $INSTALL_OPENCV_PYTHON2 == "YES" ] ; then
        sudo apt-get install -y python-dev python-numpy
    fi
    if [ $INSTALL_OPENCV_PYTHON3 == "YES" ] ; then
        sudo apt-get install -y python3-dev python3-numpy python3-matplotlib
    fi
    # Check if exists sources
    if [ -d "$BUILD_FOLDER/opencv" ]; then
        if ! $FORCE ; then
            echo "${red}Folder $BUILD_FOLDER/opencv already exist${reset}"
            exit 1
        else
            echo "${yellow}Remove old source OpenCV folder${reset}"
            sudo rm -R "$BUILD_FOLDER/opencv"
        fi
    fi
    # Download last stable opencv source code
    echo "${green}Download OpenCV $OPENCV_VERSION source code in $BUILD_FOLDER${reset}"
    # Make folder and clone
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
}


opencv_build()
{
    OPENCV_VER_MAJOR=${OPENCV_VERSION%.*}
    OPENCV_VER_MAJOR=${OPENCV_VER_MAJOR%.*}

    echo "Build openCV ${bold}$OPENCV_VERSION${reset} for $HARDWARE_NAME ${bold}($CUDA_ARCH_BIN)${reset}"
    if [ -d "$BUILD_FOLDER/opencv/build" ] ; then
        echo "${yellow}Clean old Build folder${reset}"
        sudo rm -R "$BUILD_FOLDER/opencv/build"
    fi
    mkdir -p "$BUILD_FOLDER/opencv/build" 
    cd "$BUILD_FOLDER/opencv/build"

    # CMAKE
    local CMAKEFLAGS="
-D CMAKE_BUILD_TYPE=RELEASE
-D CMAKE_INSTALL_PREFIX=$INSTALL_OPENCV_PATH
-D WITH_CUDA=ON
-D CUDA_ARCH_BIN=$CUDA_ARCH_BIN
-D CUDA_ARCH_PTX=''
-D WITH_CUBLAS=ON
-D ENABLE_FAST_MATH=ON
-D CUDA_FAST_MATH=ON
-D ENABLE_NEON=ON
-D WITH_LIBV4L=ON
-D BUILD_TESTS=OFF
-D BUILD_PERF_TESTS=OFF
-D BUILD_EXAMPLES=OFF
-D WITH_GSTREAMER=ON
-D WITH_GSTREAMER_0_10=OFF
-D WITH_QT=ON
-D WITH_OPENGL=ON
-D EIGEN_INCLUDE_PATH=/usr/include/eigen3
-D WITH_GSTREAMER=ON
-D OPENCV_GENERATE_PKGCONFIG=ON
-D OPENCV_ENABLE_NONFREE=ON" # for SLAM alghoritms SIFT/SURF
        if [ $INSTALL_OPENCV_CONTRIB == "YES" ] ; then
            CMAKEFLAGS="$CMAKEFLAGS
-D OPENCV_EXTRA_MODULES_PATH=$BUILD_FOLDER/opencv/opencv_contrib/modules"
        fi
        if [ $CUDNN_VERSION != "NOT_INSTALLED" ] ; then
            CUDNN_VERSION_MAJOR=${CUDNN_VERSION%.*}
            CUDNN_VERSION_MAJOR=${CUDNN_VERSION%.*}
            CMAKEFLAGS="$CMAKEFLAGS
-D WITH_CUDNN=ON
-D OPENCV_DNN_CUDA=ON
-D CUDNN_VERSION=$CUDNN_VERSION_MAJOR"
        fi
        if [ $INSTALL_OPENCV_PYTHON2 == "YES" ] ; then
            CMAKEFLAGS="$CMAKEFLAGS
-D BUILD_opencv_python2=ON"
        fi
        if [ $INSTALL_OPENCV_PYTHON3 == "YES" ] ; then
            CMAKEFLAGS="$CMAKEFLAGS
-D BUILD_opencv_python3=ON"
        fi
        # Remove DNN for old releases
        if [ $OPENCV_VER_MAJOR == "4" ] && [ $CUDA_ARCH_BIN == "5.2" ] || [ $CUDA_ARCH_BIN == "5.3" ] ; then
            CMAKEFLAGS="$CMAKEFLAGS
-D OPENCV_DNN_CUDA=OFF"
        fi
        # https://github.com/opencv/opencv_contrib/issues/1786 for opencv 3.4.3
        if [ $OPENCV_VER_MAJOR == "3" ] && [ ${CUDA_VERSION%.*} == "10.2" ] ; then
            CMAKEFLAGS="$CMAKEFLAGS
-D ENABLE_PRECOMPILED_HEADERS=OFF
-D BUILD_opencv_cudacodec=OFF" 
        fi

    time cmake $CMAKEFLAGS ..

    if [ $? -eq 0 ] ; then
        echo "${green}CMake configuration make successful${reset}"
    else
        # Try to make again
        echo "${red}CMake issues"
        echo "Please check the configuration being used${reset}"
        echo $CMAKEFLAGS
        exit 1
    fi
}


opencv_make_install()
{
    local NUM_CPU=$(nproc)

    cd "$BUILD_FOLDER/opencv/build"

    echo "${bold}Make${reset} openCV $OPENCV_VERSION with $NUM_CPU CPU"
    time make -j$(($NUM_CPU - 1))
    
    if [ $? -eq 0 ] ; then
        echo "${green}Make successful${reset}"
    else
        # Try to make again
        echo "${red}Make issues"
        echo "Please check the configuration being used${reset}"
        exit 1
    fi

    echo "${bold}Test${reset} openCV make"
    time make test
    
    echo "${bold}Make${reset} install openCV $OPENCV_VERSION"
    sudo make install
    
    echo "${bold}ldconfig${reset}"
    sudo ldconfig
}


opencv_remove_sources()
{
    echo "${green}Remove OpenCV sources${reset} from $BUILD_FOLDER/opencv"
    sudo rm -R "$BUILD_FOLDER/opencv"
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
    echo "   -y|--yes                    | Automatic install"
    echo "   -s|--silent                 | Run this script silent"
    echo "   -ocv|--oc-version [VERSION] | Version OpenCV to install"
    echo "   --folder [FOLDER]           | OpenCV build folder"
    echo "   --force                     | Force install"
    echo "   --skip-download             | Skip download opencv"
    echo "   --pc [CUDA_ARCH_BIN]        | Install on a Desktop PC."
    echo "                               | Find on https://en.wikipedia.org/wiki/CUDA"
    echo "                               | and use nvidia-smi to detect your board"
}


main()
{
    # Load variables
    CUDA_VERSION=$JETSON_CUDA
    CUDNN_VERSION=$JETSON_CUDNN
    OPENCV_VERSION_CUDA=$JETSON_OPENCV_CUDA
    OPENCV_STATUS=$JETSON_OPENCV
    HARDWARE_NAME="$JETSON_MACHINE [$JETSON_L4T]"
    CUDA_ARCH_BIN=$JETSON_CUDA_ARCH_BIN
    
    local SILENT=false
    local CLEAN_SOURCES=false
    local NO_ASK=false
    local INSTALL_PC=false
    local SKIP_DOWNLOAD=false
	# Decode all information from startup
    while [ -n "$1" ]; do
        case "$1" in
            -h|--help) # Load help
                usage
                exit 0
                ;;
            -y|--yes)
                NO_ASK=true
                ;;
            -s|--silent)
                SILENT=true
                ;;
            --pc)
                INSTALL_PC=true
                if [ -z $2 ] ; then
                    usage "[ERROR] Unknown option: $1"
                    exit 1
                fi
                CUDA_ARCH_BIN=$2
                shift 1
                ;;
            --force)
                FORCE=true
                ;;
            --skip-download)
                SKIP_DOWNLOAD=true
                ;;
            -ocv|--oc-version)
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

    if ! $INSTALL_PC ; then
        # Check Jetson board
        if [ -z "$JETSON_MACHINE" ] ; then
            echo "${red}jetson-stats not installed${reset}"
            exit 1
        fi
        if [ $JETSON_BOARD == "UNKNOWN" ] ; then
            echo "${red}Please check if jetson-stats is well installed${reset}"
            exit 1
        fi
        # Check Opencv version
        if [ -z $JETSON_OPENCV ] ; then
            echo "${red}Please select wich version is needed using option ${bold}-ocv${reset} or ${bold}--ocv-version${reset} ${reset}"
            exit 1
        fi
    fi
    

    # Override for PC version
    if $INSTALL_PC ; then
        HARDWARE_NAME="PC desktop"
        # CUDA search
        CUDA_VERSION=$(cat /usr/local/cuda/version.txt | sed 's/\CUDA Version //g')
        #CUDNN search
        CUDNN_VERSION=$(dpkg -l 2>/dev/null | grep -m1 "libcudnn")
        if [ ! -z "$CUDNN_VERSION" ] ; then
            CUDNN_VERSION=$(echo $CUDNN_VERSION | sed 's/.*libcudnn[0-9] \([^ ]*\).*/\1/' | cut -d '-' -f1 )
        else
            CUDNN_VERSION="NOT_INSTALLED"
        fi
        # Detect opencv
        if hash opencv_version 2>/dev/null; then
            OPENCV_STATUS="$(opencv_version)"
        else
            OPENCV_STATUS="NOT_INSTALLED"
            OPENCV_VERSION_CUDA="NO"
        fi
    fi
    
    # Check CUDA version
    if [ -z $CUDA_VERSION ] ; then
        echo "${red}Please install CUDA${reset}"
        exit 1
    fi

    # Exit if OpenCV is well installed with CUDA
    if [ $OPENCV_VERSION_CUDA == "YES" ] && [ $OPENCV_STATUS == $OPENCV_VERSION ] && ! $FORCE ; then
        echo "${green}OpenCV $JETSON_OPENCV is installed with CUDA $CUDA_VERSION${reset}"
        exit 0
    fi

    # Make info table
    if ! $SILENT ; then
        echo "----- Installation -----"
        if $FORCE ; then
            echo "${red}FORCE MODE ON${reset}"
        fi
        echo "$HARDWARE_NAME"
        echo "${green}CUDA${reset} $CUDA_VERSION (ARCH BIN) $CUDA_ARCH_BIN"
        echo "${green}cuDNN${reset} $CUDNN_VERSION"
        # OpenCV version
        if [ $OPENCV_STATUS != $OPENCV_VERSION ] ; then
            echo "Change ${green}OpenCV${reset} from $OPENCV_STATUS -> ${bold}$OPENCV_VERSION${reset}"
        else
            echo "Install OpenCV ${bold}$OPENCV_VERSION${reset}"
        fi
        echo "OpenCV build ${green}folder${reset} ${bold}$BUILD_FOLDER${reset}"
        echo " - ${green}contrib${reset} $INSTALL_OPENCV_CONTRIB"
        echo " - ${green}python2${reset} $INSTALL_OPENCV_PYTHON2"
        echo " - ${green}python3${reset} $INSTALL_OPENCV_PYTHON3"
        echo "Clean sources: ${bold}$CLEAN_SOURCES${reset}"
        echo "------------------------"
    fi
    # Ask before start install
    while ! $NO_ASK; do
        read -p "Do you want install OpenCV ${bold}$OPENCV_VERSION${reset}? [Y/n] " yn
            case $yn in
                [Yy]* ) break;;
                [Nn]* ) exit 1;;
            * ) echo "Please answer yes or no.";;
        esac
    done

    # Local variables
    local LOCAL_FOLDER=$(pwd)
    # Download and install all dependencies
    if ! $SKIP_DOWNLOAD ; then
        opencv_downloader
    fi
    # Build opencv
    opencv_build
    # Make and install
    opencv_make_install
    # remove source folder
    if $CLEAN_SOURCES ; then
        opencv_remove_sources
    fi
    # Restore previuous folder
    cd $LOCAL_FOLDER
}


main $@
exit 0

# EOF

