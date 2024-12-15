# ust_05ln_ros2

ROS2 node for Hokuyo UST-05LN LiDAR sensor.  
Ported from ROS1. [Originally by BasB1.](https://github.com/BasB1/hokuyo_ust/tree/master)

Build :  
```colcon build --symlink-install --packages-select ust_05ln_ros2```  
Udev rule:  
```
sudo cp 15-ust05ln.rules /etc/udev/rule.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```
running ```ls /dev | grep hokuyo``` should return the grep result with ```hokuyo``` 

# Tuning guide
edit the ```ust_05ln_ros2/urg_node.py``` and edit the ```offset``` variable to tune the angle offset (unit in radiant)

# TODO
- C++ node instead of Python
- laser frame and offset configure via parameter yaml file
