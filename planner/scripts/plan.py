#!/usr/bin/env python3
import sys
import argparse
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from nav_msgs.msg import Path

from utils import *
from planner_wrapper import TomogramPlanner

sys.path.append('../')
from config import Config


class PCTPlanner(Node):
    def __init__(self, cfg, tomo_file, start_pos, end_pos):
        super().__init__('pct_planner')
        
        self.cfg = cfg
        self.tomo_file = tomo_file
        self.start_pos = start_pos
        self.end_pos = end_pos
        
        # QoS profile for latched topics (similar to ROS1 latch=True)
        qos_profile = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        
        self.path_pub = self.create_publisher(Path, '/pct_path', qos_profile)
        self.planner = TomogramPlanner(cfg)
        
        self.get_logger().info(f'PCT Planner initialized')
        self.get_logger().info(f'Tomogram file: {tomo_file}')
        self.get_logger().info(f'Start: {start_pos}, End: {end_pos}')
        
        # Run planning after initialization
        self.pct_plan()
    
    def pct_plan(self):
        self.planner.loadTomogram(self.tomo_file)
        
        traj_3d = self.planner.plan(self.start_pos, self.end_pos)
        if traj_3d is not None:
            path_msg = traj2ros(traj_3d, self.get_clock())
            self.path_pub.publish(path_msg)
            self.get_logger().info("Trajectory published")
        else:
            self.get_logger().warn("Planning failed")


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--scene', type=str, default='Spiral', 
                        help='Name of the scene. Available: [\'Spiral\', \'Building\', \'Plaza\']')
    args = parser.parse_args()
    
    cfg = Config()
    
    if args.scene == 'Spiral':
        tomo_file = 'spiral0.3_2'
        start_pos = np.array([-16.0, -6.0], dtype=np.float32)
        end_pos = np.array([-26.0, -5.0], dtype=np.float32)
    elif args.scene == 'Building':
        tomo_file = 'AI_vol5_02_all'
        start_pos = np.array([1.0, 0.0], dtype=np.float32)
        end_pos = np.array([-1.0, 0.5], dtype=np.float32)
    else:
        tomo_file = 'OldTown_all_new1'
        start_pos = np.array([-32.61, -20.24], dtype=np.float32)
        end_pos = np.array([13.39, -20.24], dtype=np.float32)
    
    rclpy.init()
    
    planner_node = PCTPlanner(cfg, tomo_file, start_pos, end_pos)
    
    rclpy.spin(planner_node)
    
    planner_node.destroy_node()
    rclpy.shutdown()