import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# 1. 读取 CSV 文件
df = pd.read_csv('./jsbsim/v0_c3104_waypoint.csv')  # 替换为你的 CSV 文件名

# 2. 取经纬度列，并转换为度（可选）
lon = df['position/long-gc-rad'].to_numpy() * 180 / np.pi
lat = df['position/lat-geod-rad'].to_numpy() * 180 / np.pi

# 3. 绘制飞机轨迹
plt.figure(figsize=(8, 6))
plt.plot(lon, lat, label='Aircraft Path', color='blue')

# 4. 设置三个航点（目标点）并转换为度
waypoints = [
    ( -1.6599238, 0.516291656),
    ( -1.6599238, 0.51642159),
    ( -1.6599238, 0.5166114)
]
waypoints_deg = [(lon*180/np.pi, lat*180/np.pi) for lon, lat in waypoints]

# 分别解包
wp_lon, wp_lat = zip(*waypoints_deg)

# 5. 绘制航点并用折线连接
plt.scatter(wp_lon, wp_lat, color='red', zorder=5, label='Waypoints')
plt.plot(wp_lon, wp_lat, color='red', linestyle='--')

# 6. 图例、标签
plt.xlabel('Longitude (deg)')
plt.ylabel('Latitude (deg)')
plt.title('Aircraft Trajectory with Waypoints')
plt.legend()
plt.grid(True)
plt.axis('equal')  # 保持比例
plt.show()
