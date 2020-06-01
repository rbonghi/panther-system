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
CUSTOM_WS_NAME="custom_ws"
ROS_WS_NAME="catkin_ws"

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
    if ! grep -Fxq "source $HOME/$ros_ws/devel/setup.bash" $HOME/.bashrc ; then
        echo "   - Add workspace ${green}$ROS_WS_NAME${reset} on .bashrc"
        echo "source $HOME/$ros_ws/devel/setup.bash" >> $HOME/.bashrc
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
    if ! grep -Fxq "export PANTHER_TYPE=$PANTHER_TYPE" $HOME/.bashrc ; then
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
        echo " - ${yellow}Install${reset} Panther scripts"
        status=1
    else
        echo " - Panther scripts installed"
    fi
    # ROS
    if [ $(ros_status) = "NOT_INSTALLED" ] ; then
        echo " - ${yellow}Install${reset} ROS $DISTRO"
        status=1
    else
        echo " - ROS ${green}$(ros_status)${reset} installed"
    fi
    # ROS workspace
    if [ $(ros_ws_status $CUSTOM_WS_NAME) = "NOT_INSTALLED" ] ; then
        echo " - ${yellow}Install${reset} ROS workspace ${green}$CUSTOM_WS_NAME${reset}"
        status=1
    else
        echo " - ROS workspace ${green}$CUSTOM_WS_NAME${reset} installed"
    fi
    # ROS workspace
    if [ $(ros_ws_status $ROS_WS_NAME) = "NOT_INSTALLED" ] ; then
        echo " - ${yellow}Install${reset} ROS workspace ${green}$ROS_WS_NAME${reset}"
        status=1
    else
        echo " - ROS workspace ${green}$ROS_WS_NAME${reset} installed"
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
    while ! $SILENT; do
        read -p "Do you want install panther-system? [Y/n] " yn
            case $yn in
                [Yy]* ) break;;
                [Nn]* ) exit;;
            * ) echo "Please answer yes or no.";;
        esac
    done

    # Request sudo password
    sudo -v

    # Install Panther scripts
    if [ $(extra_scripts) = "NOT_INSTALLED" ] ; then
        # Add this folder in bashrc
        if ! grep -Fxq "export PATH=$(pwd)/bin\${PATH:+:\${PATH}}" $HOME/.bashrc ; then
            echo "   - Add PATH=$(pwd)/bin\${PATH:+:\${PATH}} on .bashrc"
            echo "export PATH=$(pwd)/bin\${PATH:+:\${PATH}}" >> $HOME/.bashrc
        fi

        if ! grep -Fxq "export PANTHER_TYPE=$PANTHER_TYPE" $HOME/.bashrc ; then
            echo "   - Add PANTHER_TYPE=$PANTHER_TYPE on .bashrc"
            echo "export PANTHER_TYPE=$PANTHER_TYPE" >> $HOME/.bashrc
        fi
    fi
    
    # Install ROS
    if [ $(ros_status) = "NOT_INSTALLED" ] ; then
        echo "Install ROS"
        ros
    fi

    # Install ROS customization
    if [ $(ros_ws_status $CUSTOM_WS_NAME) = "NOT_INSTALLED" ] ; then
        local rosinstall_uri="https://raw.githubusercontent.com/rpanther/panther/master/$DISTRO.rosinstall"
        # Run rosinstall uri
        echo "Install ${green}custom ROS${reset} workspace from ${bold}$rosinstall_uri${reset}"
        ros_ws $CUSTOM_WS_NAME $rosinstall_uri
    fi

    # Install ROS workspace
    if [ $(ros_ws_status $ROS_WS_NAME) = "NOT_INSTALLED" ] ; then
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

