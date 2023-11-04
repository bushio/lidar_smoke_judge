#! /usr/bin/env python
# -*- coding: utf-8 -*-

import os
import time
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import PointCloud2
import argparse
import matplotlib.pyplot as plt
from .point_cloud_util import point_cloud2_to_array

VIS_SMOKE = False

class LidarSmokeJudge(Node):
    def __init__(self):
        super().__init__('lidar_smoke_judge')

        # Subscriber for point cloud
        #self.create_subscription(PointCloud2, "~/input/velodyne_point_cloud", self.onLidarPointCloud2, 10)
        self.create_subscription(PointCloud2, "/velodyne_points", self.onLidarPointCloud2, 10)

        self.params_is_insmoke = {
            'D': 5.0,
            'thresh_valid': 0.75,
            'thresh_zero': 0.60,
            'max_points_num':28800
            }
        
        self.params_smoke_filt = {
            'D': 12.0,
            'I': 8,
            'Z': 0.5,
            'SetPosX': 0.0,
            'SetPosY': 0.0, 
            'SetPosZ': 2.0,
            }
        
        if VIS_SMOKE:
            z_range=(-2,2)
            self.frame_num = 0
            self.xlim = (-20,20)
            self.ylim = (-20,20)
            self.zlim = z_range
            
            self.fig = plt.figure()

    def onLidarPointCloud2(self, msg: PointCloud2):
        #self.get_logger().info("get PointCloud2 {} {}".format(msg.header.stamp.sec, msg.header.stamp.nanosec))
        cells_lidar = point_cloud2_to_array(msg)
        self.frame_num += 1

        #cells_lidar["Azim"] = np.rad2deg(np.arctan2(-cells_lidar["X"], cells_lidar["Y"]))
        cells_lidar["Azim"] = np.rad2deg(np.arctan2(cells_lidar["Y"], cells_lidar["X"]))

        is_insmoke, valid_idx, valid_ratio, zero_ratio = self.is_in_smoke(cells_lidar)

        self.get_logger().info("is_insmoke, valid_idx, valid_ratio, zero_ratio")
        self.get_logger().info("{}, {}, {}, {}" .format(is_insmoke, valid_idx, valid_ratio, zero_ratio))

        smoke_idx = self.smoke_filter(cells_lidar)
        self.get_logger().info("smoke_idx: {}".format(smoke_idx))

        if VIS_SMOKE:
            self.scatter_graph_smoke(cells_lidar)

    def is_in_smoke(self, cells_lidar):
        # is_in_smoke
        # is_insmoke : (bool) 霧の中にいるか？
        # valid_ratio : (double) 有効な点群の割合 0~1 %
        # zero_ratio : (double) ゼロ埋めされている点群の割合 0~1 %

        # 前方() D以上の点群数が~%以上か？
        filt_dist = cells_lidar['Dist']  > self.params_is_insmoke['D']
        
        # 前方の点を有効な点とする
        # 前方かつDist非ゼロの点をそのフレームの有効な点数とする
        filt_front = np.abs(cells_lidar['Azim']) < 90
        filt_d0 = cells_lidar['Dist'] == 0
        front_points_num = np.count_nonzero(filt_front & ~filt_d0)

        valid_idx = filt_dist & filt_front      # 有効な点
        valid_num = np.count_nonzero(valid_idx)

        # 割合
        valid_ratio = valid_num / front_points_num # 有効な点群の割合 0~1
        # zero_ratio = np.count_nonzero(filt_front & filt_d0) / np.count_nonzero(filt_front) # 前方の点のうちゼロ埋めされている点群の割合 0~1
        # print(~np.isnan(cells_lidar["X"]))
        zero_ratio = np.count_nonzero(~np.isnan(cells_lidar["X"]))/self.params_is_insmoke['max_points_num']

        #zero_ratio = 1-len(cells_lidar['Dist'])/self.params_is_insmoke['max_points_num']
        
        # 判定
        is_insmoke_valid = valid_ratio < self.params_is_insmoke['thresh_valid']
        is_insmoke_zero = zero_ratio > self.params_is_insmoke['thresh_zero']

        is_insmoke = is_insmoke_valid or is_insmoke_zero

        return is_insmoke, valid_idx, valid_ratio, zero_ratio

    def smoke_filter(self, cells_lidar):
        # smoke_filter
        # 霧の可能性がある点群を除去する関数
        
        filt_dist = cells_lidar['Dist']  < self.params_smoke_filt['D']
        filt_I = cells_lidar['I'] < self.params_smoke_filt['I']
        filt_Z = cells_lidar['Z'] + self.params_smoke_filt['SetPosZ'] > self.params_smoke_filt['Z']

        smoke_idx = filt_dist & filt_I[:, 0] & filt_Z
        return smoke_idx
    
    def scatter_graph_smoke(self,cells_lidar):
        
        is_insmoke, valid_idx, valid_ratio, zero_ratio = self.is_in_smoke(cells_lidar)
        smoke_idx = self.smoke_filter(cells_lidar)

        x = cells_lidar['X']
        y = cells_lidar['Y']
        
        frame_num = self.frame_num
        
        # 散布図プロット
        sc = plt.scatter(x, y, c='k', marker='.', label='LiDAR Points with Smoke Judge')
        
        plt.scatter(x[valid_idx], y[valid_idx], c='g',s=10)
        print(smoke_idx)

        plt.scatter(x[smoke_idx], y[smoke_idx], c='r',s=10)
        
        if is_insmoke:
            plt.text(self.xlim[0]+1, self.ylim[1]-1, 'In Smoke', color='r')
        
        plt.xlabel('X')
        plt.ylabel('Y')
        
        ttl_valid_ratio = f"valid-ratio = {valid_ratio*100:.2f}[%]"
        ttl_zero_ratio  = f"zero-ratio = {zero_ratio*100:.2f}[%]"
        ttl_frame       = f'Frame {frame_num}'
        ttl = ttl_frame + "," + ttl_valid_ratio + "," + ttl_zero_ratio
        plt.title(ttl)
        
        plt.grid()
        plt.xlim(self.xlim)
        plt.ylim(self.ylim)
        
        # 軸の比率を1:1に設定
        plt.gca().set_aspect('equal', adjustable='box')

        plt.pause(0.1)
        plt.clf()
        
def main(args=None):
    print('Hi from lidar_smoke_judge')
    rclpy.init(args=None)

    node = LidarSmokeJudge()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()