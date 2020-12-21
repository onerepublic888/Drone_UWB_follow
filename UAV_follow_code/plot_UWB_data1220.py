import serial, re, docx
import glob, json, os
import collections
import datetime
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from datetime import datetime
import plotly.express as px
from algorithm_threading import UWB_square_pos


def cal_cdf(pos_error):
    hist, bin_edges = np.histogram(pos_error, bins=20)
    cdf_value = np.cumsum(hist/sum(hist))
    return bin_edges, cdf_value, np.percentile(pos_error,95)


with open("follw_data_11_32_09.txt","r") as f3:
    data3 = json.load(f3)

with open("follw_data_11_32_09.txt","r") as f4:
    data4 = json.load(f4)

'''{"new_ang": 175.83, "dis_to_tag": [7.105, 6.585, 6.323, 7.086], "mov_dis": 0.01, "AHRS": [18.0625, -6.3125, -2.875], "time": "11:32:11"}'''

ti1_ls3, ti2_ls3, rx1_ls3, rx2_ls3 = [], [], [], []
ti1_ls4, ti2_ls4, rx1_ls4, rx2_ls4 = [], [], [], []

new_ang_ls1, dis_to_tag_ls1, mov_dis_ls1, ti_ls1 = [], [], [], []
new_ang_ls2, dis_to_tag_ls2, mov_dis_ls2, ti_ls2 = [], [], [], []
new_ang_ls3, dis_to_tag_ls3, mov_dis_ls3, ti_ls3 = [], [], [], []
new_ang_ls4, dis_to_tag_ls4, mov_dis_ls4, ti_ls4 = [], [], [], []
ori_ang_ls2, ori_ang_ls3, ori_ang_ls4 = [], [], []
ang_dif3, ang_dif4 = [], []
xyz1_ls3, xyz2_ls3, xyz1_ls4, xyz2_ls4= [], [], [], []

md3, md4 = [], []
start = 0
end = 720

start4 = 0
end4 = 195

for i in data3:
    new_ang = round((i['new_ang']), 2)
    md = i['mov_dis']
    dis_to_tag = i['dis_to_tag']
    time = i['time']#.split(':')
    # ti = int(time[1])*60 + int(time[2]) 
    
    ori_ang = UWB_square_pos(dis_to_tag, [0.55, 0.9])
    ori_ang_ls3.append(ori_ang)
    ti_ls3.append(time)
    dis_to_tag_ls3.append(dis_to_tag)
    new_ang_ls3.append(new_ang)
    mov_dis_ls3.append(md)
    ang_dif3.append(ori_ang - new_ang)
    

print('============================================================')

for i in data4:
    new_ang = round((i['new_ang']), 2)
    md = i['mov_dis']
    dis_to_tag = i['dis_to_tag']
    time = i['time']#.split(':')
    # ti = int(time[1])*60 + int(time[2]) 

    ori_ang = UWB_square_pos(dis_to_tag, [0.55, 0.9])
    ori_ang_ls4.append(ori_ang)
    ti_ls4.append(time)
    dis_to_tag_ls4.append(dis_to_tag)
    new_ang_ls4.append(new_ang)
    mov_dis_ls4.append(md)
    ang_dif4.append(ori_ang - new_ang)
    

start = 0
start3 = 0
start4 = 0
# end1 = len(ti_ls1)
end2 = len(ti_ls2)
end3 = len(ti_ls3)
end4 = len(ti_ls4)

dict_3 = {
    'time': ti_ls3[start:end3],
    'distance': dis_to_tag_ls3[start:end3],
    'ori ang': ori_ang_ls3[start:end3],
    'new ang': new_ang_ls3[start:end3],
    'move dis': mov_dis_ls3[start:end3],
    'ang_dif3': ang_dif3[start:end3],
    }
df_3 = pd.DataFrame(dict_3)
pd.set_option('display.max_rows', df_3.shape[0]+1)

dict_4 = {
    'time': ti_ls4[start:end4],
    'distance': dis_to_tag_ls4[start:end4],
    'ori ang': ori_ang_ls4[start:end4],
    'new ang': new_ang_ls4[start:end4],
    'move dis': mov_dis_ls4[start:end4],
    'ang_dif4': ang_dif4[start:end4],
    }
df_4 = pd.DataFrame(dict_4)
pd.set_option('display.max_rows', df_4.shape[0]+1)

# pd.describe_option('display')

# with pd.option_context('display.max_rows', len(ti_ls3), 'display.max_columns', len(ti_ls4)):  # more options can be specified also
pd.options.display.max_rows
pd.set_option('display.max_rows', None)
pd.set_option('display.max_columns', None)
pd.set_option('display.width', None)
pd.set_option('display.max_colwidth', -1)


print(df_3)
# print('==================================')
# print(df_4)

s3 = 0
e3 = 30
s4 = 0
e4 = 30
ini_dis = [7.36, 7.7]

# print(len(ti_ls3))
# print(len(ti_ls4))
# print(len(mov_dis_ls3))
# print(len(mov_dis_ls4))
# 3: 650, 2min
# 4: 530, 1.5min
t3 = np.linspace(0, 110, num=(e3 - s3))
t4 = np.linspace(0, 96, num=(e4 - s4))

plt.figure(0)
plt.plot(t3, np.array(ori_ang_ls3[s3:e3]).reshape((e3 - s3), 1).tolist(), 'b-')
# plt.plot(t3, np.array(new_ang_ls3[s3:e3]).reshape((e3 - s3), 1).tolist(), 'b-')
plt.title('Target angle (Concrete)', fontsize=20)
plt.xlabel('Time [s] ', fontsize=20)
plt.ylabel('Angle [degree]', fontsize=20)
plt.legend(labels = ['Angle'], loc = 'lower left' , fontsize = 20) # 'Angle with KF'
plt.grid(True) 

plt.figure(1)
plt.plot(t4, np.array(ori_ang_ls4[s4:e4]).reshape((e4 - s4), 1).tolist(), 'r--')
plt.plot(t4, np.array(new_ang_ls4[s4:e4]).reshape((e4 - s4), 1).tolist(), 'b-')
plt.title('Target angle (grass)', fontsize=20)
plt.xlabel('Time [s] ', fontsize=20)
plt.ylabel('Angle [degree]', fontsize=20)
plt.legend(labels = ['Angle', 'Angle with KF'], loc = 'lower right' )
plt.grid(True) 

plt.figure(2)
plt.plot(t3, np.array(mov_dis_ls3[s3:e3]).reshape((e3 - s3), 1).tolist(), 'r--')
plt.plot(t4, np.array(mov_dis_ls4[s4:e4]).reshape((e4 - s4), 1).tolist(), 'b-')
plt.title('Distance error', fontsize=20)
plt.xlabel('Time [s] ', fontsize=20)
plt.ylabel('Error [m]', fontsize=20)
plt.legend(labels = ['Concrete', 'Grass'], loc = 'lower right' )
plt.grid(True) 

# plt.figure(3)
# plt.plot(t4, np.array(mov_dis_ls4[s4:e4]).reshape((e4 - s4), 1).tolist(), c='rv-')
# # plt.plot([i for i in range(len(ti_ls4))], np.array(mov_dis_ls4).reshape(len(ti_ls4), 1).tolist(), c='bo-')
# plt.title('Distance error', fontsize=20)
# plt.xlabel('Time [s] ', fontsize=20)
# plt.ylabel('Error [m]', fontsize=20)
# plt.grid(True) 

dis_err3_ls = [abs((i + ini_dis[0]) - ini_dis[0]) for i in mov_dis_ls3[s3:e3]]
dis_err4_ls = [abs((i + ini_dis[1]) - ini_dis[1]) for i in mov_dis_ls4[s4:e4]]
bin_edges3, cdf_value3, cdf95_3 = cal_cdf(dis_err3_ls)
bin_edges4, cdf_value4, cdf95_4 = cal_cdf(dis_err4_ls)
print('cdf95 Concrete, Grass: ', cdf95_3, cdf95_4)

plt.figure(4)
plt.plot(bin_edges3[:-1], cdf_value3,'rv-', linewidth=1, markersize=6)
plt.plot(bin_edges4[:-1], cdf_value4,'bs-', linewidth=1, markersize=6)
plt.title('CDF of tracking error', fontsize=18)
plt.xlabel('Distance error(m)', fontsize=16)
plt.ylabel('CDF', fontsize=18)
plt.legend(labels = ['Concrete', 'Grass'], loc = 'lower right' )
plt.grid(True) 

plt.show()

