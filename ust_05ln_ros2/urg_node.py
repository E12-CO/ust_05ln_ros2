#!/usr/bin/env python

# Port to ROS2 from https://github.com/BasB1/hokuyo_ust/tree/master
# By TinLethax (RB26)

from serial import Serial
import time
from collections import namedtuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan as LaserScanMsg

import os
import sys

class urg_node(Node):
    
    Command = namedtuple('Command',['command', 'answer_expected', 'answer'])

    START_RANGING = Command('#GT15466', False, '')
    STOP_RANGING = Command('#ST5297', True, '#ST00A845')
    ID = Command('#IN0D54', True, '')
    ID2 = Command('#CLC2DD', True, '')

    PD = Command('#PD15F5', True, '#PD0070FA')
    
    PLOP = Command('#GR0EEE1', True, '')
    
    MESURE_LENGHT = 4359

    def __init__(self, port, baudrate=115200):
        super().__init__('urg_node')
        self.ser = Serial(port, baudrate)
        self.stop_ranging()
        self.data_prev = ''
        
        self.pub_scan_msg = self.create_publisher(
            LaserScanMsg,
            '/scan',
            10
        )

    def send_command(self, command, timeout=2):
        self.ser.write(command.command.encode()+b'\n')    # writes command to LIDAR
        self.ser.reset_input_buffer()
        data = b''
        start_time = time.time()
        if command.answer_expected and command.answer != '':    #precise answer expected, search for it !
            while time.time() - start_time < timeout:
                if self.ser.in_waiting:
                    data+=self.ser.read()
                    data = data.split(b'\n')[-1]
                    if command.answer.encode() in data:
                        break
            return data
        elif command.answer_expected:   # answer expected but be don't known which : return the first one (until \n)
            while time.time() - start_time < timeout:
                if self.ser.in_waiting:
                    data+=self.ser.read()
                    if b'\n' in data:
                        data = data.split(b'\n')[0]
                        break
            return data
        else:
            return b''

    def stop_ranging(self):
        self.send_command(self.STOP_RANGING)

    def start_ranging(self):
        self.stop_ranging()
        self.send_command(self.START_RANGING)

    def stop(self):
        self.stop_ranging()
    
    def get_measures(self):
        """
        returns measures under the form (timestamp, [(distance, quality), ...])
        timestamp : time in seconds since the LIDAR startup
        distance range : 0 - 65635
        valur range : 0 - ??? (65635 max)
        eg: (102.123456, [(552, 1244), (646, 1216), (676, 1270), ...])
        """
        raw_bytes = self.ser.read(self.MESURE_LENGHT)
        raw_bytes_str = str(raw_bytes)

        data = raw_bytes_str[28:]
        
        if data.find("#") > 1:
            data = self.data_prev
            self.send_command(self.ID)

        measurements = [(int(data[i:i+4],16), int(data[i+4:i+8],16))  for i in range(0, len(data)-8, 8)]
        self.data_prev = data
        
        return (measurements)

def main(args=None):
    rclpy.init(args=args)
    
    urg_lidar_node = urg_node('/dev/hokuyo')
    urg_lidar_node.get_logger().info('Robot Club KMITL : Starting UST-05LN LiDAR node...')
    
    offset = -1.22173
    
    scan_msg = LaserScanMsg()
    try:
        urg_lidar_node.stop_ranging()
        urg_lidar_node.send_command(urg_lidar_node.ID)

        urg_lidar_node.send_command(urg_lidar_node.PD)
        urg_lidar_node.send_command(urg_lidar_node.ID)
        urg_lidar_node.send_command(urg_lidar_node.START_RANGING)
        while rclpy.ok():
            data = urg_lidar_node.get_measures()
                
            if len(data) == 541 :
                #print("ok at {:.06f} s".format(data[0]))
                laser_data = data
                distance_list = []
                intensity_list = []
                for i in range(541):
                    scan_data = laser_data[i]
                    distance_list.append(scan_data[0] * 0.001)
                    intensity_list.append(scan_data[1] / 65535)
                     
                scan_msg.ranges = distance_list
                scan_msg.intensities = intensity_list
                scan_msg.header.stamp = urg_lidar_node.get_clock().now().to_msg()
                scan_msg.header.frame_id = 'laser_frame'
                scan_msg.angle_min = -1.178097245 + offset;
                scan_msg.angle_max = 1.178097245 + offset;
                scan_msg.angle_increment = 0.008726646 #0,004355258
                scan_msg.scan_time = 1 / 40
                scan_msg.time_increment = (1 / 40) / 541
                
                scan_msg.range_min = 0.0;
                scan_msg.range_max = 5.0;
                
                urg_lidar_node.pub_scan_msg.publish(scan_msg)
                
                rclpy.spin_once(urg_lidar_node, timeout_sec=0.025) # Spin at 40Hz rate
                
    finally:
        urg_lidar_node.stop()

if __name__ == "__main__":
    main()
