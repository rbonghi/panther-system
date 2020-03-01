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

red=`tput setaf 1`
green=`tput setaf 2`
reset=`tput sgr0`

# Variable to show a message if is require a reboot
REQUIRE_REBOOT=false

ros()
{
    local THIS="$(pwd)"
    echo " * ROS Install on ${green}$HOME${reset}"
    # Install wstool
    sudo apt-get install python-rosinstall -y
    echo "   - Make workspace ${green}$HOME${reset}"
    mkdir -p $HOME/catkin_ws/src
    # Copy panther wstool and run
    echo "   - Initialization rosinstall"
    # Move to catkin_ws folder
    cd $HOME/catkin_ws/
    # Initialize wstool
    # https://www.systutorials.com/docs/linux/man/1-wstool/
    if [ ! -f $HOME/catkin_ws/src/.rosinstall ] ; then
        wstool init src
    fi
    wstool merge -t src $THIS/panther.rosinstall
    # Update workspace
    wstool update -t src
    echo "   - Install all dependencies and catkin_make"
    # Install all dependencies
    # http://wiki.ros.org/rosdep
    rosdep install --from-paths src --ignore-src -r -y
    # Catkin make all workspace
    catkin_make
    # Return to home folder
    cd $THIS
}

udev()
{
    local UDEV="/etc/udev/rules.d/"
    echo " * Setup UDEV"
    echo "   - set dialout to $USER"
    sudo adduser $USER dialout
    echo "   - Install all rules in ${green}$UDEV${reset}"
    # https://unix.stackexchange.com/questions/66901/how-to-bind-usb-device-under-a-static-name
    sudo cp rules/* $UDEV
    # Reload all rules and trigger
    # https://superuser.com/questions/677106/how-to-check-if-a-udev-rule-fired
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    # Require reboot
    REQUIRE_REBOOT=true
}

usage()
{
	if [ "$1" != "" ]; then
		echo "${red}$1${reset}"
	fi
	
    echo "Panther installer. (Require SUDO) This script install all dependencies and ROS packages"
    echo "Usage:"
    echo "$0 [options]"
    echo "options,"
    echo "   -h|--help      | This help"
    echo "   -s|--silent    | Run this script silent"
    echo "   --all          | Install all parts"
    echo "   --udev         | Install UDEV rules"
    echo "   --ros          | Install ROS packages"
}

main()
{
    local SILENT=false
    local ALL=false
    local UDEV=false
    local ROS=false
    local noflag=true
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
            --all)
                ALL=true
                noflag=false
                ;;
            --udev)
                UDEV=true
                noflag=false
                ;;
            --ros)
                ROS=true
                noflag=false
                ;;
            *)
                usage "[ERROR] Unknown option: $1"
                exit 1
            ;;
        esac
            shift 1
    done
    
	# Check if run in sudo
    if [[ `id -u` -eq 0 ]] ; then 
        echo "${red}Please don't run as root${reset}"
        exit 1
    fi
    
    # Check if 
    if $noflag ; then
        echo "${red}Please select one or more options!${reset}"
        exit 1
    fi

    while ! $SILENT; do
        read -p "Do you want install panther-system for user:${green}$USER${reset}? [Y/n] " yn
            case $yn in
                [Yy]* ) break;;
                [Nn]* ) exit;;
            * ) echo "Please answer yes or no.";;
        esac
    done

    # Request sudo password
    sudo -v
    # Recap installatation
    echo "--- Board configuration ---"
    echo "- User: $USER"
    echo "---------------------------"
    # Install UDEV
    if $UDEV || $ALL ; then
        udev
    fi
    # Install ROS
    if $ROS || $ALL ; then
        ros
    fi
    
    if $REQUIRE_REBOOT ; then
        # After install require reboot
        echo "${red}Require reboot${reset}"
    fi
}


main $@
exit 0

# EOF

