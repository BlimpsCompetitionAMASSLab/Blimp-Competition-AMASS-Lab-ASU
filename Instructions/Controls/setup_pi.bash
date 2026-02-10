#!/bin/bash

# Function to run commands and auto-confirm with "Y"
run_command() {
    echo "Running: $1"
    echo "$1" | sudo -S sh -c "$(cat); echo y"
}

# Getting user input
read -p "Enter your email address: " user_email

# Run each command
run_command "apt upgrade -y && apt install locales -y"
run_command "locale-gen en_US en_US.UTF-8"
run_command "update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8"
export LANG=en_US.UTF-8
run_command "apt install software-properties-common -y"
run_command "add-apt-repository universe"
run_command "apt update && apt install curl -y"
run_command "curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg"
run_command "sh -c 'echo \"deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main\" > /etc/apt/sources.list.d/ros2.list'"
run_command "apt update"
run_command "apt upgrade -y"
run_command "apt install ros-humble-ros-base -y"
# run_command "apt install ros-humble-desktop -y"
echo "source /opt/ros/humble/setup.bash" | tee -a ~/.bashrc
run_command "apt install python3 -y"
run_command "apt install python3-colcon-common-extensions -y"
echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" | tee -a ~/.bashrc
run_command "apt install libraspberrypi-bin v4l-utils ros-humble-v4l2-camera -y"
run_command "apt install ros-humble-image-transport-plugins -y"
run_command "apt-get install raspi-config -y"
run_command "apt install python3-pip -y"
run_command "pip3 install Adafruit-circuitpython-lis3dh"
run_command "pip3 install adafruit-circuitpython-bno055"
run_command "pip3 install adafruit-circuitpython-bmp3xx"
run_command "pip3 install RPi.GPIO"
run_command "apt install ros-humble-joy -y"
run_command "wget https://github.com/joan2937/pigpio/archive/master.zip"
run_command "apt install unzip -y"
unzip master.zip
cd pigpio-master
make
sudo make install
cd ..
run_command "pip3 install scipy"

# ADDING GIT TO PROJECT
ssh-keygen -t ed25519 -C "$user_email" -f ~/.ssh/github_ed25519 -N ""
public_key_file=~/github_ssh_key.txt
cat ~/.ssh/github_ed25519.pub > "$public_key_file"
echo "Your SSH public key has been saved to $public_key_file. Add this key to your GitHub account."
eval "$(ssh-agent -s)"
echo "eval "$(ssh-agent -s)"" | tee -a ~/.bashrc
