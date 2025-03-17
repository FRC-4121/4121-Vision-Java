#!/bin/bash
# This script is made to quickly get everything set up on a Pi
# It expects to be run from the project root, cloned to /home/team4121/4121-Vision-Java
# It does require internet
# It will ask for a password for sudo

sudo systemctl stop 4121-vision

# Remove old python code
sudo chmod -x /etc/init.d/4121-vision.sh 2> /dev/null
sudo mv /etc/init.d/4121-vision.sh /etc/init.d/4121-vision.sh.disabled 2> /dev/null
sudo unlink /etc/rc3.d/S024121-vision 2> /dev/null
sudo unlink /etc/rc3.d/K024121-vision 2> /dev/null
sudo unlink /etc/rc5.d/S024121-vision 2> /dev/null
sudo unlink /etc/rc5.d/K024121-vision 2> /dev/null

# Create the new system
sudo cp misc/4121-vision.service /etc/systemd/system/
sudo mkdir -p /mnt/data/4121-Vision/logs/run /mnt/data/4121-Vision/logs/cam
sudo chmod 777 /mnt/data/4121-Vision/logs/run /mnt/data/4121-Vision/logs/cam
sudo systemctl daemon-reload
sudo systemctl enable 4121-vision

mkdir build 2> /dev/null
./gradlew compileJava setupRunScript
