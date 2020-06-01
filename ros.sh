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

# variables
DISTRO="melodic"
CUSTOM_WS_NAME="ros_custom_ws"
ROS_WS_NAME="ros_panther_ws"

# Installer RTABmap
# https://github.com/introlab/rtabmap
# https://github.com/introlab/rtabmap/wiki/Installation
# Install for NVIDIA Jetson
# Jetpack 4 https://github.com/introlab/rtabmap/issues/427#issuecomment-516914817
# https://github.com/introlab/rtabmap_ros#build-from-source-for-nvidia-jetson
rtabmap_install()
{
    local NUM_CPU=$(nproc)
    local rtabmap_folder=$HOME/rtabmap
    
    echo "Remove rtabmap packages if installed"
    sudo apt remove -y ros-$DISTRO-rtabmap-ros
    sudo apt remove -y ros-$DISTRO-rtabmap
   
    # rtabmap folder
    echo "Make $HOME/rtabmap folder"
    mkdir -p $rtabmap_folder
    cd $rtabmap_folder
    
    if [ "$PANTHER_TYPE" = "robot" ] ; then
        # Install VTK
        echo "Install ${green}VTK${reset}"
        cd git clone https://github.com/Kitware/VTK.git
        cd VTK
        git checkout v6.3.0
        mkdir build
        cd build
        cmake -DVTK_Group_Qt=ON -DVTK_QT_VERSION=4 -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release ..
        time make -j$(($NUM_CPU - 1))
        
        if [ $? -eq 0 ] ; then
            echo "${green}CMake configuration make successful${reset}"
        else
            # Try to make again
            echo "${red}CMake issues"
            echo "Please check the configuration being used${reset}"
            echo $CMAKEFLAGS
            exit 1
        fi

        # Remove Gt5 related VTK libraries
        sudo rm /usr/lib/aarch64-linux-gnu/libvtkGUISupportQt*
        sudo rm /usr/lib/aarch64-linux-gnu/libvtkRenderingQt*
        sudo rm /usr/lib/aarch64-linux-gnu/libvtkViewsQt*
        sudo rm /usr/lib/cmake/vtk-6.3/Modules/vtkGUISupportQtWebkit.cmake
        # Copy the newly compiled ones with Qt4 support
        cd $rtabmap_folder/VTK/build/lib
        sudo cp libvtkGUISupportQt* /usr/lib/aarch64-linux-gnu/.
        sudo cp libvtkRenderingQt* /usr/lib/aarch64-linux-gnu/.
        sudo cp libvtkGUISupportQtSQL* /usr/lib/aarch64-linux-gnu/.
        sudo cp libvtkViewsQt* /usr/lib/aarch64-linux-gnu/.
        # Copy cmake modules
        sudo cp vtkGUISupportQt.cmake /usr/lib/cmake/vtk-6.3/Modules/.
        sudo cp vtkGUISupportQtOpenGL.cmake /usr/lib/cmake/vtk-6.3/Modules/.
        sudo cp vtkGUISupportQtSQL.cmake /usr/lib/cmake/vtk-6.3/Modules/. 
        sudo cp vtkRenderingQt.cmake /usr/lib/cmake/vtk-6.3/Modules/.
        sudo cp vtkViewsQt.cmake /usr/lib/cmake/vtk-6.3/Modules/.
        # remove Qt5 stuffs
        sudo rm /usr/lib/cmake/vtk-6.3/VTKTargets.cmake
        sudo rm /usr/lib/cmake/vtk-6.3/VTKTargets-none.cmake

        # Create symbolic links to match binaries version
        cd /usr/lib/aarch64-linux-gnu
        sudo ln -s  libvtkGUISupportQtOpenGL-6.3.so.1 libvtkGUISupportQtOpenGL-6.3.so.6.3.0
        sudo ln -s  libvtkGUISupportQt-6.3.so.1 libvtkGUISupportQt-6.3.so.6.3.0
        sudo ln -s  libvtkRenderingQt-6.3.so.1 libvtkRenderingQt-6.3.so.6.3.0
        sudo ln -s  libvtkGUISupportQtSQL-6.3.so.1 libvtkGUISupportQtSQL-6.3.so.6.3.0
        sudo ln -s  libvtkViewsQt-6.3.so.1 libvtkViewsQt-6.3.so.6.3.0
        sudo ln -s libvtkInteractionStyle-6.3.so.6.3.0 libvtkInteractionStyle-6.3.so.1
        sudo ln -s libvtkRenderingOpenGL-6.3.so.6.3.0 libvtkRenderingOpenGL-6.3.so.1 
        sudo ln -s libvtkRenderingCore-6.3.so.6.3.0 libvtkRenderingCore-6.3.so.1
        sudo ln -s libvtkFiltersExtraction-6.3.so.6.3.0 libvtkFiltersExtraction-6.3.so.1
        sudo ln -s libvtkCommonDataModel-6.3.so.6.3.0 libvtkCommonDataModel-6.3.so.1
        sudo ln -s libvtkCommonCore-6.3.so.6.3.0 libvtkCommonCore-6.3.so.1
    fi
    # Download GTSAM
    # https://github.com/borglab/gtsam
    # TODO: Check to add


    # Make Custom workspace before build
    echo "Make ROS custom workspace ${bold}$HOME/$CUSTOM_WS_NAME${reset}"
    mkdir -p $HOME/$CUSTOM_WS_NAME/src
    cd $HOME/$CUSTOM_WS_NAME
    catkin_make

    # Clone, make and install rtabmap
    echo "Clone and install ${green}rtabmap${reset}"
    cd $rtabmap_folder
    git clone https://github.com/introlab/rtabmap.git
    cd rtabmap/build
    cmake -DRTABMAP_QT_VERSION=4 -DCMAKE_INSTALL_PREFIX=~/$CUSTOM_WS_NAME/devel ..
    time make -j$(($NUM_CPU - 1))
    
    if [ $? -eq 0 ] ; then
        echo "${green}CMake configuration make successful${reset}"
    else
        # Try to make again
        echo "${red}CMake issues"
        echo "Please check the configuration being used${reset}"
        echo $CMAKEFLAGS
        exit 1
    fi
    
    make install # Sudo is not needed
}


ros_ws()
{
    local ros_ws=$1
    local rosinstall=$2
    local THIS="$(pwd)"
    echo " * ROS Install on ${green}$HOME${reset}"
    # Install wstool
    sudo apt-get install python-rosinstall -y
    echo "   - Make workspace ${green}$HOME${reset}"
    mkdir -p $HOME/$ros_ws/src
    # Copy panther wstool and run
    echo "   - Initialization rosinstall"
    # Move to catkin_ws folder
    cd $HOME/$ros_ws/
    # Initialize wstool
    # https://www.systutorials.com/docs/linux/man/1-wstool/
    if [ ! -f $HOME/$ros_ws/src/.rosinstall ] ; then
        wstool init src
    fi
    wstool merge -t src $rosinstall
    # Update workspace
    wstool update -t src
    echo "   - Install all dependencies and catkin_make"
    # Install all dependencies
    # http://wiki.ros.org/rosdep
    rosdep install --from-paths src --ignore-src -r -y
    # Catkin make all workspace
    catkin_make
    # Add environment variables on bashrc
    # --extend reference:
    # https://stackoverflow.com/questions/26410578/number-of-catkin-directory-in-ros
    # https://answers.ros.org/question/206876/how-often-do-i-need-to-source-setupbash/
    if ! grep -Fxq "source $HOME/$ros_ws/devel/setup.bash --extend" $HOME/.bashrc ; then
        echo "   - Add workspace ${green}$ROS_WS_NAME${reset} on .bashrc"
        echo "source $HOME/$ros_ws/devel/setup.bash --extend" >> $HOME/.bashrc
    fi
    # Load locally on this script the source workspace
    source $HOME/$ros_ws/devel/setup.bash
    # Return to home folder
    cd $THIS
}


ros_ws_status()
{
    local ros_ws=$1
    if [ -d "$HOME/$ros_ws" ] ; then
        echo "OK"
        return
    fi
    # Return not installed
    echo "NOT_INSTALLED"
}


ros()
{
    # Install ROS
    if [ "$PANTHER_TYPE" = "sim" ] ; then
        sudo apt-get install ros-melodic-desktop-full
    elif [ "$PANTHER_TYPE" = "robot" ] ; then
        sudo apt-get install ros-melodic-robot
    else
        usage "[ERROR] Unknown config: $PANTHER_TYPE"
        exit 1
    fi

    # Add environment variables on bashrc
    if ! grep -Fxq "source /opt/ros/$DISTRO/setup.bash" $HOME/.bashrc ; then
        echo "   - Add ROS $DISTRO source to ${green}.bashrc${reset}"
        echo "source /opt/ros/$DISTRO/setup.bash" >> $HOME/.bashrc
    fi
    if ! grep -Fxq "export EDITOR='nano -w'" $HOME/.bashrc ; then
        echo "   - Add ${green}EDITOR nano${reset} nn .bashrc"
        echo "export EDITOR='nano -w'" >> $HOME/.bashrc
    fi
    # Add var enviroments
    if [ "$PANTHER_TYPE" = "robot" ] ; then
        if ! grep -Fxq "export ROS_MASTER_URI=http://$HOSTNAME.local:11311/" $HOME/.bashrc ; then
            echo "   - Add ${green}ROS_MASTER_URI=http://$HOSTNAME.local:11311/${reset} on .bashrc"
            echo "export ROS_MASTER_URI=http://$HOSTNAME.local:11311/" >> $HOME/.bashrc
        fi
        if ! grep -Fxq "export ROS_HOSTNAME=$HOSTNAME.local" $HOME/.bashrc ; then
            echo "   - Add ${green}ROS_HOSTNAME=$HOSTNAME.local${reset} on .bashrc"
            echo "export ROS_HOSTNAME=$HOSTNAME.local" >> $HOME/.bashrc
        fi
    fi
    # Load locally on this script the source workspace
    source /opt/ros/$DISTRO/setup.bash
    # TODO: Ask reload bashrc
}



ros_status()
{
    if [ ! -z "ROS_DISTRO" ] ; then
        echo $ROS_DISTRO
        return
    fi
    # Return not installed
    echo "NOT_INSTALLED"
}


extra_scripts()
{
    # Add environment variables on bashrc
    if ! grep -Fxq "export PATH=$(pwd)/bin\${PATH:+:\${PATH}}" $HOME/.bashrc ; then
        echo "NOT_INSTALLED"
        return
    fi
    if ! grep -Fxq "export PANTHER_TYPE='$PANTHER_TYPE'" $HOME/.bashrc ; then
        echo "NOT_INSTALLED"
        return
    fi
    if ! grep -Fxq "export PANTHER_WS='$ROS_WS_NAME'" $HOME/.bashrc ; then
        echo "NOT_INSTALLED"
        return
    fi
    # Return not installed
    echo "OK"
}

recap()
{
    local status=0
    # Panther scripts
    if [ $(extra_scripts) = "NOT_INSTALLED" ] ; then
        echo " 1. ${yellow}Install${reset} Panther scripts"
        status=1
    else
        echo " 1. Panther scripts installed"
    fi
    # ROS
    if [ $(ros_status) = "NOT_INSTALLED" ] ; then
        echo " 2. ${yellow}Install${reset} ROS $DISTRO"
        status=1
    else
        echo " 2. ROS ${green}$(ros_status)${reset} installed"
    fi
    # Rtabmap
    if [ "A" != "B" ] ; then
        echo " 3. ${yellow}Install${reset} RTABmap"
        status=1
    else
        echo " 3. ${green}RTABmap${reset} is installed"
    fi    
    # ROS workspace
    if [ $(ros_ws_status $CUSTOM_WS_NAME) = "NOT_INSTALLED" ] ; then
        echo " 4. ${yellow}Install${reset} ROS workspace ${green}$CUSTOM_WS_NAME${reset}"
        status=1
    else
        echo " 4. ROS workspace ${green}$CUSTOM_WS_NAME${reset} installed"
    fi
    # ROS workspace
    if [ $(ros_ws_status $ROS_WS_NAME) = "NOT_INSTALLED" ] ; then
        echo " 5. ${yellow}Install${reset} ROS workspace ${green}$ROS_WS_NAME${reset}"
        status=1
    else
        echo " 5. ROS workspace ${green}$ROS_WS_NAME${reset} installed"
    fi
    return $status
}


usage()
{
	if [ "$1" != "" ]; then
		echo "${red}$1${reset}"
	fi
	
    echo "Panther installer. This script install all dependencies and ROS packages"
    echo "Usage:"
    echo "$0 [options]"
    echo "options,"
    echo "   -h|--help            | This help"
    echo "   -s|--silent          | Run this script silent"
    echo "   -f|--file [FILE]     | File to read"
    echo "   -c|--config [TYPE]   | Define Panther type ${yellow}{sim, robot}${reset}"
    echo "   -d|--distro [DISTRO] | Define ROS distribution [Default: ${green}$DISTRO${reset}]"
}


main()
{
    local rosinstall_file=""
    local SILENT=false
    local INSTALL_ALL=false
	# Decode all information from startup
    while [ -n "$1" ]; do
        case "$1" in
            -h|--help) # Load help
                usage
                exit 0
                ;;
            -f|--file)
                rosinstall_file=$2
                shift 1
                ;;
            -s|--silent)
                SILENT=true
                ;;
            -c|--config)
                PANTHER_TYPE=$2
                shift 1
                ;;
            -d|--distro)
                DISTRO=$2
                shift 1
                ;;
            *)
                usage "[ERROR] Unknown option: $1"
                exit 1
            ;;
        esac
            shift 1
    done

    # Check config option
    if [ "$PANTHER_TYPE" != "sim" ] && [ "$PANTHER_TYPE" != "robot" ] ; then
        usage "[ERROR] Unknown config: $PANTHER_TYPE"
        exit 1
    fi

	# Check run in sudo
    if [[ `id -u` -eq 0 ]] ; then 
        echo "${red}Please don't run as root${reset}"
        exit 1
    fi

    # Recap installatation
    echo "------ Configuration ------"
    echo " - ${bold}Hostname:${reset} ${green}$HOSTNAME${reset}"
    echo " - ${bold}User:${reset} ${green}$USER${reset}"
    echo " - ${bold}Home:${reset} ${green}$HOME${reset}"
    echo " - ${bold}Type:${reset} ${green}${PANTHER_TYPE^^}${reset}"
    if [ $(ros_status) != $DISTRO ] ; then 
        echo " - ${bold}ROS Distro:${reset} ${yellow}$(ros_status)${reset} -> ${green}$DISTRO${reset}"
    else
        echo " - ${bold}ROS Distro:${reset} ${green}$(ros_status)${reset}"
    fi
    echo "------ Install status -----"
    recap
    local recap_status=$?
    echo "---------------------------"

    if [ $recap_status -eq 0 ] ; then
        echo "${green}Panther ROS installed. Nothing to do!${reset}"
        exit 0
    fi

    # Ask before start install
    echo "Numbers [0-9], [a/A] to install All or press [q/Q] to quit"
    while ! $SILENT; do
        read -p "What do you want to do?[0-9aAqQ] " input
        if [[ $input ]] && [ $input -eq $input 2>/dev/null ] ; then
            break
        elif [ ${input,,} = "a" ] ; then
            INSTALL_ALL=true
            break
        elif [ ${input,,} = "q" ] ; then
            exit 0
        else
            echo "Please write a number [0-9], [a/A] to install All or press [q/Q] to quit"
        fi
    done

    # Request sudo password
    sudo -v

    # Install Panther scripts
    if [ "NOT_INSTALLED" = "NOT_INSTALLED" ] && ( [ "$input" = "1" ] || $INSTALL_ALL ] ) ; then
        # Add this folder in bashrc
        if ! grep -Fxq "export PATH=$(pwd)/bin\${PATH:+:\${PATH}}" $HOME/.bashrc ; then
            echo "   - Add PATH=$(pwd)/bin\${PATH:+:\${PATH}} on .bashrc"
            echo "export PATH=$(pwd)/bin\${PATH:+:\${PATH}}" >> $HOME/.bashrc
        fi
        # Type configuration
        if ! grep -Fxq "export PANTHER_TYPE='$PANTHER_TYPE'" $HOME/.bashrc ; then
            echo "   - Add PANTHER_TYPE='$PANTHER_TYPE' on .bashrc"
            echo "export PANTHER_TYPE='$PANTHER_TYPE'" >> $HOME/.bashrc
        fi
        # Catkin workspace reference
        if ! grep -Fxq "export PANTHER_WS='$ROS_WS_NAME'" $HOME/.bashrc ; then
            echo "   - Add PANTHER_WS='$ROS_WS_NAME' on .bashrc"
            echo "export PANTHER_WS='$ROS_WS_NAME'" >> $HOME/.bashrc
        fi
    fi
    
    # Install ROS
    if [ $(ros_status) = "NOT_INSTALLED" ] && ( [ "$input" = "2" ] || $INSTALL_ALL ] ) ; then
        echo "Install ROS"
        ros
    fi

    # Detect opencv
    if hash opencv_version 2>/dev/null; then
        OPENCV_STATUS="$(opencv_version)"
    else
        OPENCV_STATUS="NOT_INSTALLED"
        OPENCV_VERSION_CUDA="NO"
    fi
    # Get OpenCV major version
    OPENCV_MAJOR=${OPENCV_STATUS%.*}
    OPENCV_MAJOR=${OPENCV_MAJOR%.*}

    # Install ROS customization
    if [ $(ros_ws_status $CUSTOM_WS_NAME) = "NOT_INSTALLED" ] && ( [ "$input" = "4" ] || $INSTALL_ALL ] ) ; then
        local rosinstall_uri="https://raw.githubusercontent.com/rpanther/panther/master/${DISTRO}_cv$OPENCV_MAJOR.rosinstall"
        # Run rosinstall uri
        echo "Install ${green}custom ROS${reset} workspace from ${bold}$rosinstall_uri${reset}"
        ros_ws $CUSTOM_WS_NAME $rosinstall_uri
    fi

    # Install ROS workspace
    if [ $(ros_ws_status $ROS_WS_NAME) = "NOT_INSTALLED" ] && ( [ "$input" = "5" ] || $INSTALL_ALL ] ) ; then
        # Check if rosinstall file is not empty
        if [ ! -z $rosinstall_file ] ; then
            rosinstall_uri=$rosinstall_file
        # Extract rosinstall uri
        elif [ "$PANTHER_TYPE" = "sim" ] ; then
            rosinstall_uri="https://raw.githubusercontent.com/rpanther/panther_simulation/master/simulation.rosinstall"
        elif [ "$PANTHER_TYPE" = "robot" ] ; then
            rosinstall_uri="https://raw.githubusercontent.com/rpanther/panther_robot/master/robot.rosinstall"
        else
            usage "[ERROR] Unknown config: $PANTHER_TYPE"
            exit 1
        fi
        # Run rosinstall uri
        echo "Install ROS workspace from ${bold}$rosinstall_uri${reset}"
        ros_ws $ROS_WS_NAME $rosinstall_uri
    fi
    
    if [ -f /var/run/reboot-required ] ; then
        # After install require reboot
        echo "${red}*** System Restart Required ***${reset}"
    fi
}


main $@
exit 0

# EOF

