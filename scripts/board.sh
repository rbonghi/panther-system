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
reset=`tput sgr0`

ZED_VERSION="3.1"


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


zed()
{
    local THIS="$(pwd)"
    # Version link
    # https://download.stereolabs.com/zedsdk/3.1/jp43/jetsons
    local JP="jp${JETSON_JETPACK//./}"
    local LINK="https://download.stereolabs.com/zedsdk/$ZED_VERSION/$JP/jetsons"
    local DOWNLOAD_FILE="zed_driver_${ZED_VERSION//./}.run"
    
    echo " * Install ZED SDK $ZED_VERSION"
    echo "   - Download from $LINK"

    cd "$HOME/Downloads"
    if [ ! -f $DOWNLOAD_FILE ] ; then
        # Download ZED drivers
        wget --output-document $DOWNLOAD_FILE $LINK
        # Set executable launcher
        chmod +x $DOWNLOAD_FILE
        # Launch zed_driver in silent mode
        ./$DOWNLOAD_FILE --quiet -- "silent"
    fi
    # Return to home folder
    cd $THIS
}


zed_status()
{
    # Extract version
    if [ -f "/usr/local/zed/zed-config-version.cmake" ] ; then
        local version="$(cat //usr/local/zed/zed-config-version.cmake | head -1)"
        version="$(echo $version | grep -oP 'PACKAGE_VERSION \"\K[^\"]+')"
        echo "$version"
        return
    fi
    # Return not installed
    echo "NOT_INSTALLED"
}


udev()
{
    local UDEV_FOLDER="/etc/udev/rules.d/"
    echo " * Setup UDEV"
    echo "   - set ${green}[dialout, input]${reset} to $USER"
    sudo adduser $USER dialout
    sudo adduser $USER input
    echo "   - Install all rules in ${green}$UDEV_FOLDER${reset}"
    # https://unix.stackexchange.com/questions/66901/how-to-bind-usb-device-under-a-static-name
    sudo cp rules/* $UDEV_FOLDER
    # Reload all rules and trigger
    # https://superuser.com/questions/677106/how-to-check-if-a-udev-rule-fired
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    # Require reboot
    if [ ! -f /var/run/reboot-required ] ; then
        sudo sh -c 'echo "*** System Restart Required ***" > /var/run/reboot-required'
    fi
}

sudoers_rules()
{
    echo " * Install sudo-ers rules"
    # Backup of sudoers file and change the backup file.
    sudo cp /etc/sudoers /tmp/sudoers.bak
    echo "$USER ALL=(ALL) NOPASSWD:/sbin/shutdown, /sbin/reboot" | sudo tee -a /tmp/sudoers.bak > /dev/null

    # Check syntax of the backup file to make sure it is correct.
    sudo visudo -cf /tmp/sudoers.bak > /dev/null
    if [ $? -eq 0 ]; then
      # Replace the sudoers file with the new only if syntax is correct.
      sudo cp /tmp/sudoers.bak /etc/sudoers
      echo "   - ${green}suoders rules added for /sbin/shutdown, /sbin/reboot${reset}"
    else
      echo "${red}Could not modify /etc/sudoers file. Please do this manually.${reset}"
    fi
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
}


main()
{
    local SILENT=false
    local INSTALL_ALL=false
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
    
    # Recap installatation
    echo "--- Board configuration ---"
    echo " - Hostname: ${green}$HOSTNAME${reset}"
    echo " - User: ${green}$USER${reset}"
    echo " - Home: ${green}$HOME${reset}"
    echo "-------Install ------------"
    echo " 1. Sudoers"
    echo " 2. UDEV"
    echo " 3. ZED SDK ${green}$(zed_status)${reset} - $ZED_VERSION"
    echo "---------------------------"

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
    # Install SUDOERS
    if [ "$input" = "1" ] || $INSTALL_ALL ] ; then
        sudoers_rules
    fi
    # Install UDEV
    if [ "$input" = "2" ] || $INSTALL_ALL ] ; then
        udev
    fi
    # Install ZED
    if [ "$input" = "3" ] || $INSTALL_ALL ] ; then
        zed
    fi

    if [ -f /var/run/reboot-required ] ; then
        # After install require reboot
        echo "${red}*** System Restart Required ***${reset}"
    fi
}


main $@
exit 0

# EOF

