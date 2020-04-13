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

# Catkin main package
PANTHER_REPO="$HOME/panther-system"

update()
{
    local ROS_WS_NAME=$1
    local THIS="$(pwd)"
    local path="$HOME/$ROS_WS_NAME"
    # Merge if there are update of repositories
    wstool merge -t "$path/src" $PANTHER_REPO/panther.rosinstall
    # Update workspace
    wstool update -t "$path/src"
    # Install all dependencies
    # http://wiki.ros.org/rosdep
    rosdep install --from-paths "$path/src" --ignore-src -r -y
    # Go to catkin path
    cd $path
    # Catkin make all workspace
    catkin_make
    # Return to main path
    cd $THIS
}

usage()
{
	if [ "$1" != "" ]; then
		echo "${red}$1${reset}"
	fi
	
    echo "updater. Update workspace, repositories and dependencies"
    echo "Usage:"
    echo "$0 [options]"
    echo "options,"
    echo "   -h|--help      | This help"
}

main()
{
    local ROS_WS_NAME="catkin_ws"
	# Decode all information from startup
    while [ -n "$1" ]; do
        case "$1" in
            -h|--help) # Load help
                usage
                exit 0
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
    # Request sudo password
    sudo -v
    # Run update function    
    update $ROS_WS_NAME
}

main $@
exit 0

# EOF

