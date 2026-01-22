#!/usr/bin/env python3
import sys
import argparse
import numpy as np
import pickle
import scipy.ndimage as ndimage
import yaml
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray

from utils import *
from planner_wrapper import TomogramPlanner

sys.path.append('../')
from config import Config


class MultiRoutePlanner(Node):
    def __init__(self, cfg, tomo_file, routes_config, execution_mode='independent',
                 cost_threshold=20.0, min_obstacle_dist=1.5, min_spacing=8.0,
                 max_elevation_diff=0.15):
        super().__init__('multi_route_planner')
        
        self.cfg = cfg
        self.tomo_file = tomo_file
        self.routes_config = routes_config
        self.execution_mode = execution_mode
        self.cost_threshold = cost_threshold
        self.min_obstacle_dist = min_obstacle_dist
        self.min_spacing = min_spacing
        self.max_elevation_diff = max_elevation_diff  # 最大高度差，用于过滤人行道
        
        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        
        # 为每条路径创建独立的发布器
        self.path_publishers = {}
        self.waypoint_publishers = {}
        
        for i, route in enumerate(routes_config):
            route_name = route.get('name', f'route_{i}')
            self.path_publishers[route_name] = self.create_publisher(
                Path, f'/pct_path_{route_name}', qos_profile
            )
            self.waypoint_publishers[route_name] = self.create_publisher(
                MarkerArray, f'/pct_waypoints_{route_name}', qos_profile
            )
        
        # 添加完整串联路径的发布器
        self.main_path_pub = self.create_publisher(Path, '/pct_path', qos_profile)
        self.main_waypoints_pub = self.create_publisher(MarkerArray, '/pct_waypoints', qos_profile)
        
        # 添加连接段路径的发布器（用于单独可视化衔接段）
        self.connection_path_pub = self.create_publisher(Path, '/pct_path_connections', qos_profile)
        self.connection_waypoints_pub = self.create_publisher(MarkerArray, '/pct_waypoints_connections', qos_profile)
        
        self.planner = TomogramPlanner(cfg)
        self.cost_map = None
        self.resolution = None
        self.center = None
        self.map_dim = None
        self.valid_region = None  # 缓存连通区域，避免重复计算
        
        # 存储已规划的路径航点
        self.planned_routes = {}
        
        # 存储已规划的完整轨迹
        self.planned_trajectories = {}
        
        self.get_logger().info('=== Multi-Route Planner ===')
        self.get_logger().info(f'Scene: {tomo_file}')
        self.get_logger().info(f'Total routes: {len(routes_config)}')
        
        self.create_timer(1.0, self.run_planning_once)
    
    def load_tomogram_data(self):
        """加载cost地图和高度图"""
        tomo_dir = self.planner.tomo_dir
        tomo_path = tomo_dir + self.tomo_file + '.pickle'
        
        with open(tomo_path, 'rb') as handle:
            data_dict = pickle.load(handle)
            tomogram = np.asarray(data_dict['data'], dtype=np.float32)
            self.resolution = float(data_dict['resolution'])
            self.center = np.asarray(data_dict['center'], dtype=np.float64)
            self.map_dim = [tomogram.shape[2], tomogram.shape[3]]
            
            # 加载cost地图
            self.cost_map = tomogram[0, 0, :, :]
            
            # 加载地面高度图（用于过滤人行道）
            self.elevation_map = tomogram[3, 0, :, :]
            self.elevation_map = np.nan_to_num(self.elevation_map, nan=-100)
        
        self.get_logger().info(f'Map: {self.map_dim[0]}x{self.map_dim[1]}, res={self.resolution}m')
        
        # 分析高度分布
        valid_elevations = self.elevation_map[self.elevation_map > -50]
        if len(valid_elevations) > 0:
            elev_min = np.min(valid_elevations)
            elev_max = np.max(valid_elevations)
            elev_median = np.median(valid_elevations)
            self.get_logger().info(f'Elevation range: min={elev_min:.2f}m, median={elev_median:.2f}m, max={elev_max:.2f}m')
    
    def find_largest_connected_region(self):
        """找到最大的连通低cost区域"""
        low_cost_mask = (self.cost_map < self.cost_threshold) & (~np.isnan(self.cost_map))
        num_low_cost = np.sum(low_cost_mask)
        self.get_logger().info(f'Low-cost cells (cost < {self.cost_threshold}): {num_low_cost}')
        
        labeled_array, num_features = ndimage.label(low_cost_mask)
        self.get_logger().info(f'Found {num_features} connected regions')
        
        region_sizes = []
        for i in range(1, num_features + 1):
            size = np.sum(labeled_array == i)
            region_sizes.append((i, size))
        
        region_sizes.sort(key=lambda x: x[1], reverse=True)
        
        # 打印前3个最大区域
        for i, (label, size) in enumerate(region_sizes[:min(3, len(region_sizes))]):
            self.get_logger().info(f'  Region {i+1}: {size} cells')
        
        largest_label = region_sizes[0][0]
        largest_region_mask = (labeled_array == largest_label)
        
        return largest_region_mask
    
    def sample_waypoints_in_safe_center(self, start_pos, end_pos, num_waypoints):
        """在最大连通区域内采样航点（过滤人行道）"""
        # 如果不需要中间航点，直接返回空数组
        if num_waypoints <= 0:
            self.get_logger().info('num_waypoints <= 0, no intermediate waypoints needed')
            return np.array([])
        
        distance_map = ndimage.distance_transform_edt(self.valid_region) * self.resolution
        
        max_dist = distance_map.max()
        self.get_logger().info(f'Distance to boundary: [0.00, {max_dist:.2f}]m')
        
        safe_center_mask = (distance_map >= self.min_obstacle_dist) & self.valid_region
        num_safe = np.sum(safe_center_mask)
        self.get_logger().info(f'Safe center cells (>{self.min_obstacle_dist}m): {num_safe}')
        
        # === 新增：基于高度过滤人行道 ===
        elevation_mask = None
        if hasattr(self, 'elevation_map') and self.max_elevation_diff > 0:
            # 计算马路基准高度（使用可通行区域的较低高度作为马路高度）
            valid_elevations = self.elevation_map[self.valid_region & (self.elevation_map > -50)]
            if len(valid_elevations) > 0:
                # 使用25%分位数作为马路高度（排除人行道）
                road_height = np.percentile(valid_elevations, 25)
                self.get_logger().info(f'Road reference height: {road_height:.2f}m (filtering sidewalks)')
                
                # 过滤掉高度超过阈值的区域（人行道）
                elevation_mask = np.abs(self.elevation_map - road_height) <= self.max_elevation_diff
                safe_center_mask = safe_center_mask & elevation_mask
                
                num_after_filter = np.sum(safe_center_mask)
                num_filtered_out = num_safe - num_after_filter
                self.get_logger().info(f'After elevation filter (±{self.max_elevation_diff}m): {num_after_filter} cells')
                if num_filtered_out > 0:
                    self.get_logger().info(f'Filtered out {num_filtered_out} sidewalk cells ({100*num_filtered_out/num_safe:.1f}%)')
                num_safe = num_after_filter
        # === 高度过滤结束 ===
        
        # 如果太少，放宽约束
        if num_safe < num_waypoints * 5:
            relaxed_dist = self.min_obstacle_dist * 0.6
            safe_center_mask = (distance_map >= relaxed_dist) & self.valid_region
            
            # 仍然应用高度过滤
            if elevation_mask is not None:
                safe_center_mask = safe_center_mask & elevation_mask
            
            num_safe = np.sum(safe_center_mask)
            self.get_logger().warn(f'Relaxed to {relaxed_dist:.1f}m: {num_safe} cells')
        
        if num_safe == 0:
            self.get_logger().error('No safe center cells!')
            return np.array([])
        
        candidate_indices = np.argwhere(safe_center_mask)
        candidate_distances = distance_map[safe_center_mask]
        sorted_indices = np.argsort(-candidate_distances)
        
        offset = np.array([self.map_dim[0] // 2, self.map_dim[1] // 2])
        candidate_world = []
        for idx in candidate_indices:
            grid_pos = idx - offset
            world_pos = grid_pos * self.resolution + self.center
            candidate_world.append(world_pos)
        candidate_world = np.array(candidate_world)
        
        waypoints = []
        for idx in sorted_indices:
            candidate = candidate_world[idx]
            too_close = False
            
            if np.linalg.norm(candidate - start_pos) < self.min_spacing * 0.7:
                too_close = True
            if np.linalg.norm(candidate - end_pos) < self.min_spacing * 0.7:
                too_close = True
            
            for wp in waypoints:
                if np.linalg.norm(candidate - wp) < self.min_spacing:
                    too_close = True
                    break
            
            if not too_close:
                waypoints.append(candidate)
            
            if len(waypoints) >= num_waypoints:
                break
        
        waypoints = np.array(waypoints, dtype=np.float32)
        
        self.get_logger().info(f'Sampled {len(waypoints)} waypoints')
        for i, wp in enumerate(waypoints):
            self.get_logger().info(f'  WP{i}: [{wp[0]:6.2f}, {wp[1]:6.2f}]')
        
        return waypoints
    
    def sort_waypoints_greedy(self, start_pos, end_pos, waypoints):
        """贪心排序"""
        sorted_wps = [start_pos]
        remaining = list(waypoints)
        current = start_pos
        
        while remaining:
            dists = [np.linalg.norm(current - wp) for wp in remaining]
            nearest_idx = np.argmin(dists)
            nearest = remaining.pop(nearest_idx)
            sorted_wps.append(nearest)
            current = nearest
        
        sorted_wps.append(end_pos)
        return sorted_wps
    
    def plan_multi_segment(self, waypoints):
        """多段规划"""
        self.planner.loadTomogram(self.tomo_file)
        full_traj = []
        
        for i in range(len(waypoints) - 1):
            start, goal = waypoints[i], waypoints[i + 1]
            self.get_logger().info(f'Planning segment {i+1}/{len(waypoints)-1}: [{start[0]:.1f}, {start[1]:.1f}] -> [{goal[0]:.1f}, {goal[1]:.1f}]')
            
            traj = self.planner.plan(start, goal)
            
            if traj is None:
                self.get_logger().error(f'  Segment {i+1} failed!')
                continue
            
            if full_traj:
                full_traj.extend(traj[1:])
            else:
                full_traj.extend(traj)
            
            self.get_logger().info(f'  Segment {i+1} ok: {len(traj)} points')
        
        return np.array(full_traj) if full_traj else None
    
    def publish_waypoints(self, waypoints, publisher, route_name, color):
        """发布航点标记"""
        markers = MarkerArray()
        
        for i, wp in enumerate(waypoints):
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = route_name
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            
            m.pose.position.x, m.pose.position.y, m.pose.position.z = float(wp[0]), float(wp[1]), 0.5
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.8
            
            if i == 0:
                m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 0.0, 1.0
            elif i == len(waypoints) - 1:
                m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.0, 0.0, 1.0
            else:
                m.color.r, m.color.g, m.color.b, m.color.a = color[0], color[1], color[2], 0.9
            
            markers.markers.append(m)
        
        publisher.publish(markers)
    
    def publish_trajectory(self, traj, publisher):
        """发布轨迹"""
        if traj is None or len(traj) == 0:
            return
        
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()
        
        for pt in traj:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = float(pt[0]), float(pt[1]), float(pt[2])
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        
        publisher.publish(path)
    
    def plan_single_route(self, route_config, route_idx):
        """规划单条路径"""
        route_name = route_config.get('name', f'route_{route_idx}')
        self.get_logger().info(f'\n=== Planning Route: {route_name} ===')
        
        # 检查是否是派生路径
        if 'derive_from' in route_config:
            parent_route = route_config['derive_from']
            waypoint_indices = route_config['waypoint_indices']
            
            if parent_route not in self.planned_routes:
                self.get_logger().error(f'Parent route {parent_route} not found!')
                return None
            
            parent_waypoints = self.planned_routes[parent_route]
            
            if max(waypoint_indices) >= len(parent_waypoints):
                self.get_logger().error(f'Invalid waypoint indices: {waypoint_indices}')
                self.get_logger().error(f'Parent has {len(parent_waypoints)} waypoints, requested indices: {waypoint_indices}')
                return None
            
            start_pos = parent_waypoints[waypoint_indices[0]]
            end_pos = parent_waypoints[waypoint_indices[1]]
            
            self.get_logger().info(f'Derived from {parent_route}:')
            self.get_logger().info(f'  Using waypoint {waypoint_indices[0]}: [{start_pos[0]:.2f}, {start_pos[1]:.2f}]')
            self.get_logger().info(f'  Using waypoint {waypoint_indices[1]}: [{end_pos[0]:.2f}, {end_pos[1]:.2f}]')
        else:
            start_pos = np.array(route_config['start_pos'], dtype=np.float32)
            end_pos = np.array(route_config['end_pos'], dtype=np.float32)
            self.get_logger().info(f'Independent route: [{start_pos[0]:.2f}, {start_pos[1]:.2f}] -> [{end_pos[0]:.2f}, {end_pos[1]:.2f}]')
        
        num_waypoints = route_config.get('num_waypoints', 3)
        
        # 采样航点
        waypoints = self.sample_waypoints_in_safe_center(start_pos, end_pos, num_waypoints)
        
        if len(waypoints) == 0:
            if num_waypoints == 0:
                # 直达路径：不需要中间航点
                self.get_logger().info(f'{route_name}: Direct path (no intermediate waypoints)')
                sorted_wps = [start_pos, end_pos]
            else:
                # 采样失败
                self.get_logger().error(f'No waypoints sampled for {route_name}!')
                return None
        else:
            # 排序航点
            sorted_wps = self.sort_waypoints_greedy(start_pos, end_pos, waypoints)
        
        # 保存航点供后续派生路径使用
        self.planned_routes[route_name] = sorted_wps
        
        self.get_logger().info(f'Route {route_name} has {len(sorted_wps)} total waypoints (including start & end)')
        
        # 可视化
        colors = [
            (0.2, 0.6, 1.0),  # 蓝色
            (1.0, 0.5, 0.0),  # 橙色
            (0.8, 0.0, 0.8),  # 紫色
            (0.0, 0.8, 0.8),  # 青色
            (1.0, 0.8, 0.0),  # 黄色
        ]
        color = colors[route_idx % len(colors)]
        
        self.publish_waypoints(sorted_wps, self.waypoint_publishers[route_name], route_name, color)
        
        # 规划轨迹
        self.get_logger().info(f'Starting trajectory planning for {route_name}...')
        traj = self.plan_multi_segment(sorted_wps)
        if traj is not None:
            self.publish_trajectory(traj, self.path_publishers[route_name])
            self.planned_trajectories[route_name] = traj  # 保存轨迹供串联使用
            self.get_logger().info(f'{route_name}: SUCCESS ({len(traj)} trajectory points)')
        else:
            self.get_logger().error(f'{route_name}: FAILED')
        
        return sorted_wps
    
    def plan_connection_segment(self, start_pos, end_pos, segment_name):
        """规划两条路径之间的简单连接段（类似 plan.py 的两点规划）"""
        distance = np.linalg.norm(np.array(end_pos) - np.array(start_pos))
        self.get_logger().info(f'Planning connection: {segment_name}')
        self.get_logger().info(f'  From: [{start_pos[0]:.2f}, {start_pos[1]:.2f}]')
        self.get_logger().info(f'  To: [{end_pos[0]:.2f}, {end_pos[1]:.2f}]')
        self.get_logger().info(f'  Distance: {distance:.2f}m')
        
        # 使用简单的两点规划（与 plan.py 中的方式相同）
        traj = self.planner.plan(start_pos, end_pos)
        
        if traj is not None:
            self.get_logger().info(f'  Connection OK: {len(traj)} points')
        else:
            self.get_logger().error(f'  Connection FAILED')
        
        return traj
    
    def plan_sequential_trajectory(self):
        """规划串联执行的完整轨迹"""
        route_names = list(self.planned_routes.keys())
        
        if len(route_names) < 2:
            self.get_logger().warn('Only one route, no sequential connections needed')
            # 如果只有一条路径，直接发布它
            if len(route_names) == 1:
                single_route = route_names[0]
                if single_route in self.planned_trajectories:
                    traj = self.planned_trajectories[single_route]
                    waypoints = self.planned_routes[single_route]
                    self.publish_trajectory(traj, self.main_path_pub)
                    self.publish_waypoints(waypoints, self.main_waypoints_pub, 
                                          'main', (1.0, 1.0, 0.0))
                    self.get_logger().info(f'Published single route to /pct_path')
            return
        
        self.get_logger().info(f'\n=== Building Sequential Trajectory ===')
        self.get_logger().info(f'Routes to connect: {route_names}')
        
        # 收集所有轨迹段和航点
        full_trajectory = []
        full_waypoints = []
        
        # 收集所有连接段的轨迹和航点（用于单独可视化）
        all_connection_trajectories = []
        all_connection_waypoints = []
        
        for i, route_name in enumerate(route_names):
            if route_name not in self.planned_trajectories:
                self.get_logger().error(f'Route {route_name} has no trajectory!')
                continue
            
            route_traj = self.planned_trajectories[route_name]
            route_waypoints = self.planned_routes[route_name]
            
            if i == 0:
                # 第一条路径：添加完整轨迹和航点
                full_trajectory.extend(route_traj)
                full_waypoints.extend(route_waypoints)
                self.get_logger().info(f'{route_name}: Added {len(route_traj)} trajectory points')
            else:
                # 规划从上一条路径终点到当前路径起点的连接段
                prev_end = full_trajectory[-1][:2]  # 取前两个坐标 (x, y)
                curr_start = route_traj[0][:2]
                
                connection_traj = self.plan_connection_segment(
                    prev_end, curr_start, 
                    f'{route_names[i-1]}_to_{route_name}'
                )
                
                if connection_traj is not None:
                    # 保存连接段用于单独可视化
                    all_connection_trajectories.extend(connection_traj)
                    all_connection_waypoints.append(connection_traj[0])  # 起点
                    all_connection_waypoints.append(connection_traj[-1])  # 终点
                    
                    # 添加连接段（跳过起点以避免重复）
                    full_trajectory.extend(connection_traj[1:])
                    self.get_logger().info(f'  Connection: Added {len(connection_traj)-1} points')
                else:
                    self.get_logger().warn(f'  Connection failed, trajectory may have gap')
                
                # 添加当前路径的轨迹（跳过起点）
                full_trajectory.extend(route_traj[1:])
                full_waypoints.extend(route_waypoints[1:])
                self.get_logger().info(f'{route_name}: Added {len(route_traj)-1} trajectory points')
        
        # 发布完整的串联轨迹到 /pct_path
        if full_trajectory:
            full_trajectory_array = np.array(full_trajectory)
            self.publish_trajectory(full_trajectory_array, self.main_path_pub)
            self.publish_waypoints(full_waypoints, self.main_waypoints_pub, 
                                  'main', (1.0, 1.0, 0.0))
            
            self.get_logger().info(f'\n=== Sequential Trajectory Complete ===')
            self.get_logger().info(f'Total trajectory points: {len(full_trajectory)}')
            self.get_logger().info(f'Total waypoints: {len(full_waypoints)}')
            self.get_logger().info(f'Published to /pct_path')
        else:
            self.get_logger().error('Failed to generate sequential trajectory')
        
        # 发布所有连接段的轨迹（单独可视化）
        if all_connection_trajectories:
            connection_traj_array = np.array(all_connection_trajectories)
            self.publish_trajectory(connection_traj_array, self.connection_path_pub)
            self.publish_waypoints(all_connection_waypoints, self.connection_waypoints_pub,
                                  'connections', (1.0, 0.0, 1.0))  # 紫红色
            
            self.get_logger().info(f'\n=== Connection Segments Visualization ===')
            self.get_logger().info(f'Total connection points: {len(all_connection_trajectories)}')
            self.get_logger().info(f'Connection waypoints: {len(all_connection_waypoints)}')
            self.get_logger().info(f'Published to /pct_path_connections (magenta color)')
        else:
            self.get_logger().info('No connection segments to visualize')
    
    def run_planning_once(self):
        """主流程：按顺序规划所有路径"""
        self.get_logger().info('\n' + '='*50)
        self.get_logger().info('=== Multi-Route Planning Started ===')
        self.get_logger().info('='*50)
        
        self.get_logger().info('\n=== Step 1: Loading Tomogram ===')
        self.load_tomogram_data()
        
        self.get_logger().info('\n=== Step 2: Finding Largest Connected Region ===')
        self.valid_region = self.find_largest_connected_region()
        
        if self.valid_region is None:
            self.get_logger().error('Failed to find valid region!')
            return
        
        # 显示执行模式
        self.get_logger().info(f'\n=== Execution Mode: {self.execution_mode} ===')
        
        # 按顺序规划每条路径
        self.get_logger().info('\n=== Step 3: Planning All Routes ===')
        for i, route_config in enumerate(self.routes_config):
            self.plan_single_route(route_config, i)
        
        # 根据路径数量判断发布策略
        num_routes = len(self.routes_config)
        
        if num_routes == 1:
            # 单路径模式：同时发布到 /pct_path（主接口）
            self.get_logger().info('\n=== Single Route: Publishing to /pct_path ===')
            route_name = list(self.planned_routes.keys())[0]
            
            if route_name in self.planned_trajectories:
                traj = self.planned_trajectories[route_name]
                waypoints = self.planned_routes[route_name]
                
                # 发布到主话题（路径跟踪接口）
                self.publish_trajectory(traj, self.main_path_pub)
                self.publish_waypoints(waypoints, self.main_waypoints_pub, 'main', (1.0, 1.0, 0.0))
                
                self.get_logger().info(f'✓ Published to /pct_path (path tracking interface)')
                self.get_logger().info(f'✓ Also available at /pct_path_{route_name} (visualization)')
        
        elif self.execution_mode == 'sequential':
            # 多路径串联模式：规划连接段并发布完整路径
            self.get_logger().info('\n=== Step 4: Planning Sequential Connections ===')
            self.plan_sequential_trajectory()
        
        else:
            # 多路径但不是 sequential - 不推荐的配置
            self.get_logger().warn('\n=== Warning: Multiple routes without sequential mode ===')
            self.get_logger().warn(f'Multiple routes ({num_routes}) detected but execution_mode is "{self.execution_mode}"')
            self.get_logger().warn('Recommendation: Set execution_mode: "sequential" for multi-route planning')
            self.get_logger().warn('Routes published separately, /pct_path not generated')
        
        self.get_logger().info('\n' + '='*50)
        self.get_logger().info('=== All Routes Completed ===')
        self.get_logger().info(f'Total routes planned: {len(self.planned_routes)}')
        for route_name in self.planned_routes.keys():
            self.get_logger().info(f'  - {route_name}: {len(self.planned_routes[route_name])} waypoints')
        self.get_logger().info('='*50)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Multi-Route Planner: Plan multiple long trajectories with branch routes'
    )
    parser.add_argument('--scene', type=str, default='Spiral',
                        help='Scene name (e.g., Spiral, Plaza, Oldtown, Building)')
    parser.add_argument('--config', type=str, default='scene_configs.yaml',
                        help='Path to scene configuration file')
    parser.add_argument('--cost_threshold', type=float, default=15.0,
                        help='Max cost for traversable region')
    parser.add_argument('--min_obstacle_dist', type=float, default=1.5,
                        help='Min distance to obstacles (meters)')
    parser.add_argument('--min_spacing', type=float, default=8.0,
                        help='Min spacing between waypoints (meters)')
    parser.add_argument('--max_elevation_diff', type=float, default=0.15,
                        help='Max elevation difference from road level to filter sidewalks (meters, 0 to disable)')
    args = parser.parse_args()
    
    # 加载场景配置
    config_path = os.path.join(os.path.dirname(__file__), args.config)
    if not os.path.exists(config_path):
        print(f"错误: 配置文件 {config_path} 不存在!")
        sys.exit(1)
    
    with open(config_path, 'r', encoding='utf-8') as f:
        scene_configs = yaml.safe_load(f)
    
    if args.scene not in scene_configs['scenes']:
        print(f"错误: 场景 '{args.scene}' 不存在!")
        print(f"可用场景: {list(scene_configs['scenes'].keys())}")
        sys.exit(1)
    
    scene_cfg = scene_configs['scenes'][args.scene]
    tomo_file = scene_cfg['tomo_file']
    routes_config = scene_cfg.get('routes', [])
    execution_mode = scene_cfg.get('execution_mode', 'independent')
    
    if not routes_config:
        print(f"错误: 场景 '{args.scene}' 没有定义路径!")
        print(f"请在配置文件中为场景添加 routes 列表")
        sys.exit(1)
    
    # 根据路径数量判断单/多路径
    is_single_route = len(routes_config) == 1
    
    # 单路径时 execution_mode 无意义
    if is_single_route:
        execution_mode = 'independent'
    
    print("="*60)
    print(f"场景: {args.scene}")
    print(f"描述: {scene_cfg.get('description', 'N/A')}")
    print(f"Tomogram: {tomo_file}")
    print(f"路径类型: {'单路径' if is_single_route else '多路径'}")
    print(f"路径数量: {len(routes_config)}")
    if not is_single_route:
        print(f"执行模式: {execution_mode}")
    print("-"*60)
    for i, route in enumerate(routes_config):
        route_name = route.get('name', f'route_{i}')
        if 'derive_from' in route:
            print(f"  {i+1}. {route_name} (派生自 {route['derive_from']})")
            print(f"     - 航点索引: {route['waypoint_indices']}")
            print(f"     - 中间航点数: {route.get('num_waypoints', 3)}")
        else:
            print(f"  {i+1}. {route_name} (独立路径)")
            print(f"     - 起点: {route['start_pos']}")
            print(f"     - 终点: {route['end_pos']}")
            print(f"     - 中间航点数: {route.get('num_waypoints', 3)}")
    print("="*60)
    
    # 显示可视化 topics 信息
    if is_single_route:
        print("\n可视化 Topics:")
        route_name = routes_config[0].get('name', 'route_0')
        print(f"  - /pct_path_{route_name:<15}: 规划路径")
    elif execution_mode == 'sequential':
        print("\n可视化 Topics:")
        print(f"  - /pct_path                  : 完整串联路径（黄色）")
        print(f"  - /pct_path_connections      : 衔接段路径（紫红色）⭐")
        for i, route in enumerate(routes_config):
            route_name = route.get('name', f'route_{i}')
            colors = ['蓝色', '橙色', '紫色', '青色', '黄色']
            color = colors[i % len(colors)]
            print(f"  - /pct_path_{route_name:<15}: 单独路径（{color}）")
    else:
        print("\n可视化 Topics:")
        for i, route in enumerate(routes_config):
            route_name = route.get('name', f'route_{i}')
            colors = ['蓝色', '橙色', '紫色', '青色', '黄色']
            color = colors[i % len(colors)]
            print(f"  - /pct_path_{route_name:<15}: 独立路径（{color}）")
    print()
    
    cfg = Config()
    rclpy.init()
    
    node = MultiRoutePlanner(
        cfg, tomo_file, routes_config,
        execution_mode=execution_mode,
        cost_threshold=args.cost_threshold,
        min_obstacle_dist=args.min_obstacle_dist,
        min_spacing=args.min_spacing,
        max_elevation_diff=args.max_elevation_diff
    )
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

