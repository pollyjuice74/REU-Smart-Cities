#!/usr/bin/env python3
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
import std_msgs.msg
import numpy as np
import struct

# === UTILITY ===
def pack_rgb(r, g, b):
    return struct.unpack('f', struct.pack('I', (r << 16) | (g << 8) | b))[0]

def generate_half_colored_sphere(radius=1.0, resolution=30, color1=(255, 0, 0), color2=(0, 0, 255), mode="top_half"):
    points = []
    for theta in np.linspace(0, np.pi, resolution):
        for phi in np.linspace(0, 2*np.pi, resolution):
            x = radius * np.sin(theta) * np.cos(phi)
            y = radius * np.sin(theta) * np.sin(phi)
            z = radius * np.cos(theta)

            if mode == "top_half" and z < 0:
                continue
            elif mode == "bottom_half" and z >= 0:
                continue

            # Assign color based on mode
            rgb = pack_rgb(*color1 if mode == "top_half" else color2)
            points.append([x, y, z, rgb])
    return points
