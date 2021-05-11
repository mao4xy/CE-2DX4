#Python 3.88

import math
import xlrd
import open3d as o3d
import numpy as np

f = open("points.xyz", "w")

workbook = xlrd.open_workbook("dataset.xlsx")
sheet = workbook.sheet_by_index(0)
lines = []

# ingesting Excel dataset and processing distance measurement into XYZ format
for x in range(1, 11):
    col = sheet.col_values(x)
    pos = 1
    for dist in col[2:len(col)]:        # measurements start at 3rd element of col
        angle = math.radians((360/512)*8 * pos + 180)  
        y = float(dist) * (math.sin(angle)) 
        z = float(dist) * (math.cos(angle)) 
        f.write("{} {} {}\n".format(col[1], y, z))  # write converted coordinates to .xyz file
        pos += 1

f.close()

po = 0

# linking every point in plane
for x in range(10):
    for pt in range(64):
        lines.append([pt + po, pt + 1 + po])
    po += 64   # 72 measurements per x

po = 0
do = 64    # index of same point on the next plane will be 72 indices from present point

# linking every point in plane to corresponding point in the next plane
for x in range(9):
    for pt in range(64):
        lines.append([pt + po, pt + do + po])
    po += 64

pcd = o3d.io.read_point_cloud("points.xyz", format='xyz')

line_set = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(np.asarray(pcd.points)),
                                lines=o3d.utility.Vector2iVector(lines))
o3d.visualization.draw_geometries([line_set])
