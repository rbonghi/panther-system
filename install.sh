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
    local UDEV="/etc/udev/rules.d/"
	# Decode all information from startup
    while [ -n "$1" ]; do
        case "$1" in
            -h|--help)
                # Load help
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

    while ! $SILENT; do
        read -p "Do you want install panther-system for user:${green}$USER${reset}? [Y/n] " yn
            case $yn in
                [Yy]* ) # Break and install jetson_stats 
                        break;;
                [Nn]* ) exit;;
            * ) echo "Please answer yes or no.";;
        esac
    done

    # Request sudo password
    sudo -v

    echo "-Panther installer for user:$USER"
    echo "  * set dialout to $USER"
    sudo adduser $USER dialout
    sudo cp rules/* $UDEV
    # Reload all rules and trigger
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    echo "  * Install all rules in ${green}$UDEV${reset}"


    echo "${red}Require reboot${reset}"
}


main $@
exit 0

# EOF

