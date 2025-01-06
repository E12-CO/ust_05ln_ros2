#!/bin/bash

g++ hokuyo_detector.cpp -o hokuyo_detector
sudo cp hokuyo_detector /usr/sbin
sudo cp 15-ust05ln.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
echo "setup the Hokuyo UST LiDAR detector succesfully"
