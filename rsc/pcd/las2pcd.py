#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import os

import numpy as np
import laspy
import open3d as o3d


def main():
    if len(sys.argv) < 2:
        print("用法: python3 las2pcd.py input.las [output.pcd]")
        sys.exit(1)

    in_path = sys.argv[1]
    if not os.path.exists(in_path):
        print(f"[错误] 找不到输入文件: {in_path}")
        sys.exit(1)

    if len(sys.argv) >= 3:
        out_path = sys.argv[2]
    else:
        root, _ = os.path.splitext(in_path)
        out_path = root + ".pcd"

    print(f"[信息] 读取 LAS: {in_path}")
    las = laspy.read(in_path)

    # laspy 会自动处理 scale/offset，这里直接拿真实坐标
    xyz = np.vstack((las.x, las.y, las.z)).T
    print(f"[信息] 点数: {xyz.shape[0]}")

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)

    # 如果 LAS 里有颜色，就一起写进去（CloudCompare 导出的 las 通常可能没有颜色，有就顺带处理）
    has_color = all(hasattr(las, a) for a in ("red", "green", "blue"))
    if has_color:
        rgb = np.vstack((las.red, las.green, las.blue)).T.astype(np.float64)
        # LAS 通常是 0~65535，归一化到 0~1
        rgb /= 65535.0
        pcd.colors = o3d.utility.Vector3dVector(rgb)
        print("[信息] 检测到颜色通道 (RGB)，已写入 point cloud")

    print(f"[信息] 写出 PCD: {out_path}")
    ok = o3d.io.write_point_cloud(out_path, pcd)
    if not ok:
        print("[错误] 保存失败")
        sys.exit(1)

    print("[完成] 转换成功！")


if __name__ == "__main__":
    main()

