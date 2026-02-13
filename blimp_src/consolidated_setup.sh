#!/bin/bash

##############################################################################
# BLIMP Consolidated Setup Script
# 
# This script combines the best practices from:
# - INSTALLATION.md (official installation guide)
# - setup_pi.bash (automated setup from team)
#
# Features:
# - Automated system configuration (locale, SSH, raspi-config)
# - ROS 2 Humble installation (ros-humble-desktop)
# - All sensor drivers and dependencies
# - Workspace initialization
# - Optional interactive configuration
#
# Usage:
#   chmod +x consolidated_setup.sh
#   ./consolidated_setup.sh
#
# Note: This script should be run on a fresh Ubuntu 22.04 LTS installation
#       on Raspberry Pi 4B with root/sudo privileges
##############################################################################

set -e  # Exit on any error

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Functions for colored output
print_header() {
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}$1${NC}"
    echo -e "${BLUE}========================================${NC}"
}

print_success() {
    echo -e "${GREEN}✓ $1${NC}"
}

print_warning() {
    echo -e "${YELLOW}⚠ $1${NC}"
}

print_error() {
    echo -e "${RED}✗ $1${NC}"
}

# Check if running as root or with sudo
if [[ $EUID -ne 0 ]]; then
    print_error "This script must be run as root or with sudo"
    exit 1
fi

print_header "BLIMP Consolidated Setup Script"

##############################################################################
# Phase 1: System Configuration
##############################################################################

print_header "Phase 1: System Configuration"

echo "Step 1.1: Update system packages..."
apt update
apt upgrade -y
apt install curl wget git build-essential python3-dev python3-pip -y
print_success "System packages updated"

echo ""
echo "Step 1.2: Configure system locale (UTF-8)..."
apt install locales -y
locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
export LC_ALL=en_US.UTF-8
print_success "System locale configured to en_US.UTF-8"

echo ""
echo "Step 1.3: Install raspi-config (interactive hardware setup)..."
apt install raspi-config -y
print_success "raspi-config installed"

##############################################################################
# Phase 2: GitHub SSH Setup (Optional - Recommended)
##############################################################################

print_header "Phase 2: GitHub SSH Setup (Optional)"

read -p "Do you want to configure GitHub SSH access? (y/n) " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    read -p "Enter your GitHub email address: " github_email
    
    if [ ! -d ~/.ssh ]; then
        mkdir -p ~/.ssh
        chmod 700 ~/.ssh
    fi
    
    echo "Generating SSH key for GitHub..."
    ssh-keygen -t ed25519 -C "$github_email" -f ~/.ssh/github_ed25519 -N ""
    print_success "SSH key generated at ~/.ssh/github_ed25519"
    
    echo ""
    print_warning "IMPORTANT: Add this public key to GitHub"
    echo "Public key:"
    echo "---"
    cat ~/.ssh/github_ed25519.pub
    echo "---"
    echo "Go to: https://github.com/settings/keys"
    echo "Click 'New SSH key' and paste the above key"
    echo ""
    read -p "Press Enter when you've added the key to GitHub..."
    
    # Test connection
    echo "Testing SSH connection to GitHub..."
    ssh-keyscan -H github.com >> ~/.ssh/known_hosts 2>/dev/null || true
    if ssh -T git@github.com &>/dev/null; then
        print_success "GitHub SSH connection verified"
    else
        print_warning "GitHub SSH test failed - you may need to add the key manually"
    fi
else
    print_warning "Skipping GitHub SSH setup - you can set it up later manually"
fi

##############################################################################
# Phase 3: ROS 2 Humble Installation
##############################################################################

print_header "Phase 3: ROS 2 Humble Installation"

echo "Step 3.1: Add ROS 2 repository..."
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
apt update
print_success "ROS 2 repository added"

echo ""
echo "Step 3.2: Install ROS 2 Humble..."
apt install ros-humble-ros-base ros-humble-joy ros-humble-rqt-gui ros-humble-rqt-console -y
print_success "ROS 2 Humble installed"

echo ""
echo "Step 3.3: Install rosdep (dependency manager)..."
apt install python3-rosdep -y
rosdep init || true
rosdep update
print_success "rosdep configured"

echo ""
echo "Step 3.4: Configure ROS 2 environment..."
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
export LANG=en_US.UTF-8
export LC_ALL=en_US.UTF-8
print_success "ROS 2 environment configured in ~/.bashrc"

##############################################################################
# Phase 4: BLIMP Workspace Setup
##############################################################################

print_header "Phase 4: BLIMP Workspace Setup"

echo "Step 4.1: Create workspace..."
mkdir -p ~/blimp_ws/src
cd ~/blimp_ws
print_success "Workspace created at ~/blimp_ws"

echo ""
echo "Step 4.2: Clone BLIMP repository..."
cd ~/blimp_ws/src

# Determine clone method based on SSH setup
if [ -f ~/.ssh/github_ed25519 ]; then
    print_warning "Using SSH to clone repository..."
    git clone git@github.com:BlimpsCompetitionAMASSLab/Blimp-Competition-AMASS-Lab-ASU.git
else
    print_warning "Using HTTPS to clone repository (SSH not configured)..."
    git clone https://github.com/BlimpsCompetitionAMASSLab/Blimp-Competition-AMASS-Lab-ASU.git
fi

# Verify clone
if [ -d "Blimp-Competition-AMASS-Lab-ASU" ]; then
    print_success "Repository cloned successfully"
else
    print_error "Failed to clone repository"
    exit 1
fi

echo ""
echo "Step 4.3: Initialize colcon workspace..."
cd ~/blimp_ws
colcon build --symlink-install
print_success "Workspace initialized"

##############################################################################
# Phase 5: Hardware and Sensor Drivers
##############################################################################

print_header "Phase 5: Hardware and Sensor Drivers"

echo "Step 5.1: Install hardware interface tools..."
apt install pigpio pigpiod i2c-tools -y
print_success "Hardware tools installed"

echo ""
echo "Step 5.2: Install sensor driver libraries..."
pip3 install adafruit-circuitpython-bno055 adafruit-circuitpython-bmp3xx RPi.GPIO scipy numpy opencv-python -y
print_success "Sensor libraries installed"

echo ""
echo "Step 5.3: Enable I2C interface..."
# Check if I2C is already in config
if ! grep -q "dtparam=i2c_arm=on" /boot/firmware/config.txt; then
    echo "dtparam=i2c_arm=on" >> /boot/firmware/config.txt
    echo "dtparam=i2c1=on" >> /boot/firmware/config.txt
    print_success "I2C enabled in /boot/firmware/config.txt"
else
    print_warning "I2C already configured"
fi

echo ""
echo "Step 5.4: Enable camera interface..."
if ! grep -q "start_x=1" /boot/firmware/config.txt; then
    echo "start_x=1" >> /boot/firmware/config.txt
    echo "gpu_mem=256" >> /boot/firmware/config.txt
    echo "camera_auto_focus=1" >> /boot/firmware/config.txt
    print_success "Camera enabled in /boot/firmware/config.txt"
else
    print_warning "Camera already configured"
fi

##############################################################################
# Phase 6: Build BLIMP Packages
##############################################################################

print_header "Phase 6: Build BLIMP Packages"

echo "Step 6.1: Building blimp_interfaces (must be first)..."
cd ~/blimp_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select blimp_interfaces
print_success "blimp_interfaces built"

echo ""
echo "Step 6.2: Building all remaining packages..."
colcon build
print_success "All packages built successfully"

echo ""
echo "Step 6.3: Adding workspace sourcing to ~/.bashrc..."
if ! grep -q "source ~/blimp_ws/install/setup.bash" ~/.bashrc; then
    echo "source ~/blimp_ws/install/setup.bash" >> ~/.bashrc
    print_success "Workspace source added to ~/.bashrc"
else
    print_warning "Workspace source already in ~/.bashrc"
fi

##############################################################################
# Phase 7: Enable Services
##############################################################################

print_header "Phase 7: Enable Services"

echo "Step 7.1: Enable pigpio daemon..."
systemctl enable pigpiod
systemctl start pigpiod
print_success "pigpio daemon enabled"

echo ""
echo "Step 7.2: Configure bash shell..."
source ~/.bashrc
print_success "Shell configuration reloaded"

##############################################################################
# Post-Installation Steps
##############################################################################

print_header "Post-Installation Steps"

print_warning "Hardware configuration changes detected!"
print_warning "A REBOOT is required to activate:"
print_warning "  - I2C interface"
print_warning "  - Camera interface"
print_warning "  - SPI interface (if enabled)"

echo ""
read -p "Do you want to reboot now? (y/n) " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    print_success "Rebooting in 10 seconds... (Ctrl+C to cancel)"
    sleep 10
    sudo reboot
else
    print_warning "Remember to REBOOT before running BLIMP software"
    print_warning "Reboot with: sudo reboot"
fi

##############################################################################
# Verification Commands (Optional)
##############################################################################

echo ""
echo "After reboot, verify setup with these commands:"
echo ""
echo "1. Verify ROS 2:"
echo "   ros2 pkg list | grep blimp"
echo ""
echo "2. Verify I2C sensors:"
echo "   i2cdetect -y 1"
echo ""
echo "3. Test pigpio:"
echo "   pigs -v"
echo ""
echo "4. Check Python packages:"
echo "   python3 -c 'import adafruit_circuitpython_bno055; print(\"BNO055 OK\")'"
echo ""

print_success "Setup script completed!"
print_warning "Remember to REBOOT before using BLIMP hardware"
