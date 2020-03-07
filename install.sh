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
DISTRO="melodic"
# Components
ALL=false
UDEV=false
ROS=false
ROS_WS=false

ros_ws()
{
    local THIS="$(pwd)"
    local DISTRO=$1
    local ROS_WS_NAME="catkin_ws"
    echo " * ROS Install on ${green}$HOME${reset}"
    # Install wstool
    sudo apt-get install python-rosinstall -y
    echo "   - Make workspace ${green}$HOME${reset}"
    mkdir -p $HOME/$ROS_WS_NAME/src
    # Copy panther wstool and run
    echo "   - Initialization rosinstall"
    # Move to catkin_ws folder
    cd $HOME/$ROS_WS_NAME/
    # Initialize wstool
    # https://www.systutorials.com/docs/linux/man/1-wstool/
    if [ ! -f $HOME/$ROS_WS_NAME/src/.rosinstall ] ; then
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
    # Add environment variables on bashrc
    if ! grep -Fxq "source $HOME/$ROS_WS_NAME/devel/setup.bash" $HOME/.bashrc ; then
        echo "   - Add workspace ${green}$ROS_WS_NAME${reset} on .bashrc"
        echo "source $HOME/$ROS_WS_NAME/devel/setup.bash" >> $HOME/.bashrc
    fi
    # Return to home folder
    cd $THIS
}


ros()
{
    # Add environment variables on bashrc
    if ! grep -Fxq "source /opt/ros/$DISTRO/setup.bash" $HOME/.bashrc ; then
        echo "   - Add ROS $DISTRO source to ${green}.bashrc${reset}"
        echo "source /opt/ros/$DISTRO/setup.bash" >> $HOME/.bashrc
    fi
    # Add var enviroments
    if ! grep -Fxq "export ROS_MASTER_URI=http://$HOSTNAME.local:11311/" $HOME/.bashrc ; then
        echo "   - Add ${green}ROS_MASTER_URI=http://$HOSTNAME.local:11311/${reset} on .bashrc"
        echo "export ROS_MASTER_URI=http://$HOSTNAME.local:11311/" >> $HOME/.bashrc
    fi
    if ! grep -Fxq "export ROS_HOSTNAME=$HOSTNAME.local" $HOME/.bashrc ; then
        echo "   - Add ${green}ROS_HOSTNAME=$HOSTNAME.local${reset} on .bashrc"
        echo "export ROS_HOSTNAME=$HOSTNAME.local" >> $HOME/.bashrc
    fi
    
    # TODO: Write at end install to reload bashrc
}


versioning()
{
    # 1. Set git user and email
    # 2. Setup key for project
    
    # Add github in known_hosts
    # https://github.com/ome/devspace/issues/38
    # ssh-keyscan github.com >> ~/.ssh/known_hosts
    
    # Fix this warning
    # Warning: Permanently added the RSA host key for IP address '140.82.118.3' to the list of known hosts.
    # https://community.atlassian.com/t5/Bitbucket-questions/quot-Warning-Permanently-added-the-RSA-host-key-for-IP-address/qaq-p/28906
    echo "Versioning"
}


udev()
{
    local UDEV_FOLDER="/etc/udev/rules.d/"
    echo " * Setup UDEV"
    echo "   - set dialout to $USER"
    sudo adduser $USER dialout
    echo "   - Install all rules in ${green}$UDEV_FOLDER${reset}"
    # https://unix.stackexchange.com/questions/66901/how-to-bind-usb-device-under-a-static-name
    sudo cp rules/* $UDEV_FOLDER
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
    echo "   -d|--distro    | Define ROS distribution Default:${green}$DISTRO${reset}"
    echo "   --all          | Install all parts"
    echo "   --udev         | Install UDEV rules"
    echo "   --ros          | Install ROS ${green}$DISTRO${reset}"
    echo "   --ros-ws       | Install Panther ROS workspace"
}


list_components()
{
    if $UDEV || $ALL ; then
        echo " * udev"
    fi
    if $ROS || $ALL ; then
        echo " * ros"
    fi
    if $ROS_WS || $ALL ; then
        echo " * ros-ws"
    fi
}


main()
{
    local SILENT=false
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
            -d|--distro)
                DISTRO=$2
                shift 1
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
            --ros-ws)
                ROS_WS=true
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
    # Check if there are not options selected
    if $noflag ; then
        echo "${red}Please select one or more options!${reset}"
        usage
        exit 1
    fi

    # Recap installatation
    echo "--- Board configuration ---"
    echo " - Hostname: ${green}$HOSTNAME${reset}"
    echo " - User: ${green}$USER${reset}"
    echo " - Home: ${green}$HOME${reset}"
    echo " - ROS Distro: ${green}$DISTRO${reset}"
    echo "---------------------------"
    echo " Installing list:"
    list_components
    echo "---------------------------"
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
    # Install UDEV
    if $UDEV || $ALL ; then
        udev
    fi
    # Install ROS
    if $ROS || $ALL ; then
        ros $DISTRO
    fi
    # Install Panther ROS workspace
    if $ROS_WS || $ALL ; then
        ros_ws $DISTRO
    fi
    
    echo "---------------------------"
    echo " Installed:"
    list_components
    if $REQUIRE_REBOOT ; then
        # After install require reboot
        echo "${red}Require reboot${reset}"
    fi
}


main $@
exit 0

# EOF

