import pandas as pd
import numpy as np
import bpy
import os
import math

outfile = "/home/nicolas/Downloads/ORB_SLAM2/Examples/Monocular/cam.blend"


def cylinder_between(x1, y1, z1, x2, y2, z2, r):
    dx = x2 - x1
    dy = y2 - y1
    dz = z2 - z1
    dist = math.sqrt(dx ** 2 + dy ** 2 + dz ** 2)

    bpy.ops.mesh.primitive_cylinder_add(
        radius=r,
        depth=dist,
        location=(dx / 2 + x1, dy / 2 + y1, dz / 2 + z1)
    )

    phi = math.atan2(dy, dx)
    theta = math.acos(dz / dist)

    bpy.context.object.rotation_euler[1] = theta
    bpy.context.object.rotation_euler[2] = phi


if os.path.isfile(outfile):
    os.remove(outfile)

cam = pd.read_csv("/home/nicolas/Downloads/ORB_SLAM2/Examples/Monocular/cam.txt")

bpy.ops.object.select_all()
bpy.ops.object.delete()

for i, (x, y, z, fn, rx, ry, rz) in cam[['X', 'Y', 'Z', 'id', 'rX', 'rY', 'rZ']].iterrows():
    # print(x, y, z, fn)
    bpy.ops.mesh.primitive_cone_add(location=(x, y, z), rotation=(rx, ry, rz),
                                    radius1=0.05, depth=0.05)

ray = pd.read_csv("/home/nicolas/Downloads/ORB_SLAM2/Examples/Monocular/ray.txt")

for i, (x, y, z, fn, cx, cy, cz) in ray[['X', 'Y', 'Z', 'id', 'cX', 'cY', 'cZ']].iterrows():
    # print(x, y, z, fn)
    a = np.array([x, y, z])
    b = np.array([cx, cy, cz])
    v = b - a
    p1 = a - 5 * v
    p2 = a + 5 * v
    cylinder_between(p1[0], p1[1], p1[2], p2[0], p2[1], p2[2], 0.01)

pos = pd.read_csv("/home/nicolas/Downloads/ORB_SLAM2/Examples/Monocular/pos.txt")

for i, (x, y, z) in pos[['X', 'Y', 'Z']].iterrows():
    bpy.ops.mesh.primitive_ico_sphere_add(location=(x, y, z),
                                          size=0.2)

bpy.ops.wm.save_as_mainfile(filepath=outfile)
