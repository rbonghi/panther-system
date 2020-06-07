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


usage()
{
	if [ "$1" != "" ]; then
		echo "${red}$1${reset}"
	fi
	
    local name=$(basename ${0})
    echo "Robot installer. Install all scripts and configurations"
    echo ""
    echo "Commands:"
    echo "        $name board      Install all board drivers"
    echo "        $name opencv     Install OpenCV"
    echo "        $name ros        Install ROS and workspaces"
}


main()
{
    local option=$1
    # Catkin main package
    local ROS_WS_NAME=$PANTHER_WS

    # Check if option is in list
    local options=("board" "opencv" "ros")
    local error=true
    for item in ${options[@]} ; do
        if [ "$item" == "$option" ]; then
            error=false
            break
        fi
    done
    if $error ; then
        if [ -z $option ] ; then
            usage
        else
            usage "[ERROR] Unknown option: $option"
        fi
        exit 1
    fi
    
    
    # Load all arguments except the first one
    local arguments=${@:2}
    # Options
    if [ $option = "board" ] ; then
        bash scripts/board.sh $arguments
    elif [ $option = "opencv" ] ; then
        bash scripts/opencv.sh $arguments
    elif [ $option = "ros" ] ; then
        bash scripts/ros.sh $arguments
    fi
}


main $@
exit 0

# EOF
