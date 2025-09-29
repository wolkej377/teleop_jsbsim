import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import
import numpy as np

csv_file = "./jsbsim/c310_aptest.csv"  # 请替换成你的文件名
df = pd.read_csv(csv_file)

# 时间轴
if 'time' in df.columns:
    time = df['time']
else:
    time = np.arange(len(df))

# 1. 速度曲线
plt.figure(figsize=(10,4))
plt.plot(time, df['velocities/u-fps'], label='velocities/u-fps')
plt.xlabel('Time (s)')
plt.ylabel('Velocity (ft/s)')
plt.title('Forward Velocity vs Time')
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()

# 2. 高度曲线
plt.figure(figsize=(10,4))
plt.plot(time, df['position/h-agl-ft'], label='position/h-agl-ft', color='orange')
plt.xlabel('Time (s)')
plt.ylabel('Height (ft)')
plt.title('Height Above Ground Level vs Time')
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()

# 经纬度转换成度
df['lat_deg'] = np.degrees(df['latitude_rad'])
df['lon_deg'] = np.degrees(df['longitude_rad'])

# 3. 三维轨迹
fig = plt.figure(figsize=(10,7))
ax = fig.add_subplot(111, projection='3d')
ax.plot(df['lon_deg'], df['lat_deg'], df['position/h-agl-ft'], label='Flight Trajectory', color='green')
ax.set_xlabel('Longitude (deg)')
ax.set_ylabel('Latitude (deg)')
ax.set_zlabel('Height AGL (ft)')
ax.set_title('Flight Trajectory (Lat, Lon, Height)')
ax.legend()
plt.tight_layout()
plt.show()
