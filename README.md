# ust_05ln_ros2

ROS2 node for Hokuyo UST-05LN and UST-08NL-11 LiDAR sensor.
Ported from ROS1. [Originally by BasB1.](https://github.com/BasB1/hokuyo_ust/tree/master)

Build :
-
```colcon build --symlink-install --packages-select ust_05ln_ros2```

Udev rule:
-
Single LiDAR
```
sudo cp 15-ust05ln.rules /etc/udev/rule.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```
running ```ls /dev | grep hokuyo``` should return the grep result with ```hokuyo```

Multi LiDAR
```
cd hokuyo_detector
./install.sh
```

After running the ```install.sh``` script. The symlink in ```/dev``` will be created with the name of ```hokuyo_xxxxxxxx``` where the last eight characters are the serial number of the LiDAR

Run :
-
```ros2 launch ust_05ln_ros2 ust_05ln.launch.py```

I implemented the auto identification feature to identify LiDAR type using the product name.  
This code is currently support 
- UST-05LN - 5 meter 541 Data points with 0.5 degree resolution
- UST-08LN-11 - 8 meter 1081 Data points with 0.25 degree resolution (similar to UST-10LN)
- UST-08LNR-11*
- UST-07LNR-02*  
*untested as I don't own these sensors, I have a plan to buy the UST-08LNR-11 very soon to test

# TODO
- Get my hand on UST-08LNR11, UST-07LNR-02 and if possible UST-08LN-21 and UST-08LNR-21
