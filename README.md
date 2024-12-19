# ust_05ln_ros2

ROS2 node for Hokuyo UST-05LN LiDAR sensor.
Ported from ROS1. [Originally by BasB1.](https://github.com/BasB1/hokuyo_ust/tree/master)

Build :
-
```colcon build --symlink-install --packages-select ust_05ln_ros2```

Udev rule:
-
```
sudo cp 15-ust05ln.rules /etc/udev/rule.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```
running ```ls /dev | grep hokuyo``` should return the grep result with ```hokuyo```  

Run :
-
```ros2 launch ust_05ln_ros2 ust_05ln.launch.py```

# Tuning guide
Inside the ```params/ust.yaml```, edit the ```angle_offset``` variable to tune the angle offset (unit in radiant)

# TODO
- None
