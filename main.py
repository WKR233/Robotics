from math import pi
from spatialmath.pose3d import SE3

import time
import matplotlib.pyplot as plt
import numpy as np
import roboticstoolbox as rtb

import balance
import path

puma = rtb.models.DH.Puma560()
h0 = 1
d3 = 0.3
a2 = 1
d4 = 1
a = 2
m = 0.1

def move(r0, r1, angle, model):
    for i in range(76):
        if i != 0:
            plt.clf()
        ratio = i/75
        T = SE3(np.array([[1, 0, 0, r1[0]*ratio+r0[0]*(1-ratio)],
                [0, 1, 0, r1[1]*ratio+r0[1]*(1-ratio)],
                [0, 0, 1, r1[2]*ratio+r0[2]*(1-ratio)],
                [0, 0, 0, 1]]))
        model.base = T
        model.plot(angle, limits=[-3, 3, -3, 3, 0, 2])
        

puma.base = SE3(np.array([[1, 0, 0, 0],
                [0, 1, 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1]]))
maxheight = puma.fkine([0, pi/2, -pi/2, 0, 0, 0]).t[2]

file_path = input("请输入地图文件位置")
rows = int(input("请输入地图行数"))
cols = int(input("请输入地图列数"))
m = float(input("请输入物品重量"))
limitedheight = float(input("请输入天花板限高"))
x_g, y_g, z_g = map(float, input("请输入起点坐标").split())
x_p, y_p, z_p = map(float, input("请输入终点坐标").split())

if z_g > maxheight or z_p > maxheight:
    print("物品位置过高，无法到达")
    exit(0)



# 起始位置，基座位于原点处，各角度为0
angle_i = [0, 0, 0, 0, 0, 0]
T_i = puma.fkine(angle_i)
puma.plot(angle_i)
time.sleep(2)
plt.close()
input('continue...')

# 移动到第一个地方
move([0, 0, 0], [x_g, y_g, 0], angle_i, puma)
time.sleep(2)
plt.close()
input('continue...')

# 取的位置
puma.base = SE3(np.array([[1, 0, 0, x_g],
                [0, 1, 0, y_g],
                [0, 0, 1, 0],
                [0, 0, 0, 1]]))
T_g = SE3(np.array([[1, 0, 0, x_g],
                [0, 1, 0, y_g],
                [0, 0, 1, z_g],
                [0, 0, 0, 1]]))
angle_g = puma.ikine_LM(T_g).q
q_g = rtb.jtraj(angle_i, angle_g, 50)
puma.plot(q_g.q, backend='pyplot', limits=[-3, 3, -3, 3, 0, 2])
time.sleep(2)
plt.close()
input('continue...')
if(balance.able_to_balance(angle_g[0], angle_g[1], angle_g[2], h0, d3, a2, d4, m, a)):
    print('able to balance\n')


Map = path.loadmap(file_path)
Parent, Cost = path.init(rows, cols)
start_point = (x_g, y_g)
goal_point = (x_p, y_p)
foundpath = path.findpath(start_point, goal_point, Map, rows, cols, Cost, Parent)
print("找到了一条路："+foundpath)

# 移动到第二个地方
move([x_g, y_g, 0], [x_p, y_p, 0], angle_g, puma)
time.sleep(2)
plt.close()
input('continue...')

# 放的位置
T_p = SE3(np.array([[1, 0, 0, x_p],
                [0, 1, 0, y_p],
                [0, 0, 1, z_p],
                [0, 0, 0, 1]]))
angle_p = puma.ikine_LM(T_p).q
q_p = rtb.jtraj(angle_g, angle_p, 50)
puma.plot(q_p.q, backend='pyplot', limits=[-3, 3, -3, 3, 0, 2])
time.sleep(2)
plt.close()
input('continue...')
if(balance.able_to_balance(angle_p[0], angle_p[1], angle_p[2], h0, d3, a2, d4, m, a)):
    print('able to balance\n')