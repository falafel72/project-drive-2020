#!/bin/bash
set -e
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
# get UBUNTU_CODENAME, ROS_DISTRO, REPO_DIR, CATKIN_DIR
source $SCRIPT_DIR/identify_environment.bash

if [ "$(ls $HOME/catkin_ws/src/)" ]; then
    echo "git repo already installed"
else
    echo "repo not detected"
    cd "$HOME/catkin_ws/src"
    git clone https://github.com/falafel72/project-drive-2020.git .
#    cd "$HOME/catkin_ws"
#    catkin build --no-status
#    echo "Package built successfully"
fi

# Add scripts to build each package here


# Build the whole workspace
echo "building all packages"
cd $HOME/catkin_ws
catkin_make
echo "built all packages"

# Source the workspace
source $HOME/catkin_ws/devel/setup.bash
