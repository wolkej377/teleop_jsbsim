# import airsim
# from pyproj import Transformer
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import time

class ChartVisualizer:
    def pause(self):
        plt.show(block=True)

    def __init__(self, max_points=200, refresh_rate=10, time_window=5.0):
        """
        max_points: 每条曲线最多显示的点数
        refresh_rate: 刷新频率 Hz
        """
        self.max_points = max_points
        self.refresh_interval = 1.0 / refresh_rate
        self.time_window = time_window  # 新增：显示的时间窗口（秒）

        # 使用 deque 存储时间序列和各状态变量
        self.time_data = deque(maxlen=max_points)
        self.altitude_data = deque(maxlen=max_points)
        self.speed_data = deque(maxlen=max_points)
        self.pitch_data = deque(maxlen=max_points)
        self.roll_data = deque(maxlen=max_points)
        self.yaw_data = deque(maxlen=max_points)

        # 初始化 matplotlib 图表
        self.fig, self.axs = plt.subplots(3, 1, figsize=(8, 6))
        self.fig.tight_layout(pad=3)

        # 线条对象
        self.line_altitude, = self.axs[0].plot([], [], 'r-', label='Altitude (ft)')
        self.line_speed, = self.axs[1].plot([], [], 'g-', label='Speed (kts)')
        self.line_attitude_pitch, = self.axs[2].plot([], [], 'b-', label='Pitch (deg)')
        self.line_attitude_roll, = self.axs[2].plot([], [], 'm-', label='Roll (deg)')
        self.line_attitude_yaw, = self.axs[2].plot([], [], 'c-', label='Yaw (deg)')

        for ax in self.axs:
            ax.grid(True)
        # 图例固定在右上角
        self.axs[0].legend(loc='upper right', bbox_to_anchor=(1, 1), frameon=True)
        self.axs[1].legend(loc='upper right', bbox_to_anchor=(1, 1), frameon=True)
        self.axs[2].legend(loc='upper right', bbox_to_anchor=(1, 1), frameon=True)

        self.anim = animation.FuncAnimation(self.fig, self._update_plot, interval=self.refresh_interval*1000, blit=False)

    def add_data(self, t, altitude=None, speed=None, pitch=None, roll=None, yaw=None):
        """添加一帧数据，并实时刷新图像"""
        self.time_data.append(t)
        if altitude is not None:
            self.altitude_data.append(altitude)
        if speed is not None:
            self.speed_data.append(speed)
        if pitch is not None:
            self.pitch_data.append(pitch)
        if roll is not None:
            self.roll_data.append(roll)
        if yaw is not None:
            self.yaw_data.append(yaw)
        # 实时刷新
        plt.pause(self.refresh_interval)

    def _update_plot(self, frame):
        """更新曲线，x轴区间为time_window"""
        # 只显示最近time_window秒的数据
        if len(self.time_data) > 1:
            t_max = self.time_data[-1]
            t_min = max(self.time_data[0], t_max - self.time_window)
        else:
            t_min, t_max = 0, self.time_window

        # Altitude
        self.line_altitude.set_data(self.time_data, self.altitude_data)
        self.axs[0].set_xlim(t_min, t_max)
        self.axs[0].relim()
        self.axs[0].autoscale_view()

        # Speed
        self.line_speed.set_data(self.time_data, self.speed_data)
        self.axs[1].set_xlim(t_min, t_max)
        self.axs[1].relim()
        self.axs[1].autoscale_view()

        # Attitude
        self.line_attitude_pitch.set_data(self.time_data, self.pitch_data)
        self.line_attitude_roll.set_data(self.time_data, self.roll_data)
        self.line_attitude_yaw.set_data(self.time_data, self.yaw_data)
        self.axs[2].set_xlim(t_min, t_max)
        self.axs[2].relim()
        self.axs[2].autoscale_view()

        return (self.line_altitude, self.line_speed,
                self.line_attitude_pitch, self.line_attitude_roll, self.line_attitude_yaw)


class UEVisualizer:
    def __init__(self, vehicle_name="drone_1", height_offset=0):
        self.vehicle_name = vehicle_name
        self.height_offset = height_offset
        self.client = airsim.VehicleClient()
        self.client.confirmConnection()
        print(f"已连接AirSim: {vehicle_name}")

        # 数据存储
        self.ned_n = []
        self.ned_e = []
        self.ned_d = []
        self.qw = []
        self.qx = []
        self.qy = []
        self.qz = []

        self.ref_x = None
        self.ref_y = None
        self.ref_z = None
        self.ref_lat = None
        self.ref_lon = None

        self.current_index = 0

    # ----------- 坐标转换函数 -------------
    @staticmethod
    def ecef_to_ned(x, y, z, ref_x, ref_y, ref_z, ref_lat, ref_lon):
        dx = x - ref_x
        dy = y - ref_y
        dz = z - ref_z
        phi = np.radians(ref_lat)
        lam = np.radians(ref_lon)
        sin_phi = np.sin(phi)
        cos_phi = np.cos(phi)
        sin_lam = np.sin(lam)
        cos_lam = np.cos(lam)
        R = np.array([
            [-sin_phi * cos_lam, -sin_phi * sin_lam,  cos_phi],
            [        -sin_lam,           cos_lam,       0    ],
            [-cos_phi * cos_lam, -cos_phi * sin_lam, -sin_phi]
        ])
        ned = R @ np.vstack((dx, dy, dz))
        return ned[0], ned[1], ned[2]

    # ----------- CSV加载 -------------
    def load_csv(self, csv_file):
        df = pd.read_csv(csv_file)
        print(f"加载CSV轨迹文件: {csv_file}, 共 {len(df)} 帧")

        # 英尺转米
        ft2m = 0.3048

        # 姿态
        self.qw = df['Q(1)_{LOCAL}'].values
        self.qx = df['Q(2)_{LOCAL}'].values
        self.qy = df['Q(3)_{LOCAL}'].values
        self.qz = df['Q(4)_{LOCAL}'].values

        # ECEF坐标
        X = df['X_{ECEF} (ft)'].values * ft2m
        Y = df['Y_{ECEF} (ft)'].values * ft2m
        Z = df['Z_{ECEF} (ft)'].values * ft2m

        # 参考点
        self.ref_lat = df['Latitude Geodetic (deg)'].iloc[0]
        self.ref_lon = df['Longitude (deg)'].iloc[0]
        self.ref_x = X[0]
        self.ref_y = Y[0]
        self.ref_z = Z[0]

        # NED转换
        ned_n, ned_e, ned_d = self.ecef_to_ned(X, Y, Z, 
                                               self.ref_x, self.ref_y, self.ref_z, 
                                               self.ref_lat, self.ref_lon)

        # 偏移量，使最后一点接近UE原点
        offset_n = -ned_n[-1] + 140
        offset_e = -ned_e[-1]
        offset_d = -ned_d[-1]

        self.ned_n = ned_n + offset_n
        self.ned_e = ned_e + offset_e
        self.ned_d = ned_d + offset_d

        self.current_index = 0
        print(f"轨迹加载完成，偏移量: N={offset_n:.2f}, E={offset_e:.2f}, D={offset_d:.2f}")

    # ----------- 实时添加数据 -------------
    def add_data(self, n, e, d, qw, qx, qy, qz):
        self.ned_n.append(n)
        self.ned_e.append(e)
        self.ned_d.append(d)
        self.qw.append(qw)
        self.qx.append(qx)
        self.qy.append(qy)
        self.qz.append(qz)

    # ----------- 播放轨迹 -------------
    def play(self, dt=0.01):
        total_frames = len(self.ned_n)
        print(f"开始播放轨迹，共 {total_frames} 帧")
        start_time = time.time()
        try:
            while self.current_index < total_frames:
                n = self.ned_n[self.current_index]
                e = self.ned_e[self.current_index]
                d = self.ned_d[self.current_index]
                pose = airsim.Pose(
                    airsim.Vector3r(n, e, d + self.height_offset),
                    airsim.Quaternionr(
                        self.qx[self.current_index],
                        self.qy[self.current_index],
                        self.qz[self.current_index],
                        self.qw[self.current_index]
                    )
                )
                self.client.simSetVehiclePose(pose, ignore_collision=True, vehicle_name=self.vehicle_name)
                self.current_index += 1
                time.sleep(dt)
            elapsed = time.time() - start_time
            print(f"轨迹播放完成! 总时长: {elapsed:.2f}秒")
        except KeyboardInterrupt:
            print("用户中断了轨迹播放")
        finally:
            self.client.simPause(False)
            print("仿真已恢复。")


class CSVVisualizer:
    def __init__(self, csv_file):
        """
        初始化类，加载 CSV 数据
        csv_file: 飞行数据 CSV 文件路径
        """
        self.csv_file = csv_file
        self.df = None
        self._load_csv()
    
    def _load_csv(self):
        """内部方法：读取 CSV 并检查列"""
        self.df = pd.read_csv(self.csv_file)
        expected_columns = ['time','altitude_ft','lat_deg','lon_deg','vc_kts','roll','pitch','yaw']
        for col in expected_columns:
            if col not in self.df.columns:
                raise ValueError(f"CSV 缺少必要列: {col}")
        self.df['time'] = pd.to_numeric(self.df['time'], errors='coerce')
        self.df = self.df.dropna(subset=['time'])
    
    def plot(self, y_columns, x='time', titles=None):
        n = len(y_columns)
        fig, axs = plt.subplots(n, 1, sharex=True)
        
        # 如果只有一个子图，axs不是数组，要转换为数组方便循环
        if n == 1:
            axs = [axs]
        
        for i, col in enumerate(y_columns):
            ax = axs[i]
            ax.plot(self.df[x], self.process_column(col))
            ax.set_ylabel(col)
            if titles and i < len(titles):
                ax.set_title(titles[i])
        
        axs[-1].set_xlabel(x)
        plt.tight_layout()
        plt.show()
        return axs
    
    def plot_combined(self, y_columns, x='time', labels=None, title=None,):
        if isinstance(y_columns, str):
            y_columns = [y_columns]
        
        if labels is None:
            labels = y_columns
        
        fig, ax = plt.subplots()
        for col, label in zip(y_columns, labels):
            ax.plot(self.df[x], self.process_column(col), label=label)
        
        ax.set_xlabel(x)
        ax.set_ylabel(', '.join(y_columns))
        if title:
            ax.set_title(title)
        if len(y_columns) > 1:
            ax.legend()
        plt.show()
        return ax

    def process_column(self, col):
        if col not in self.df.columns:
            raise ValueError(f"列 {col} 不存在")
        
        # 对不同列的处理
        if col == 'yaw':
            self.df[col] = self.df[col].apply(lambda x: x-360 if x>180 else x)
        
        return self.df[col]



if __name__ == "__main__":
    import random

    vis = ChartVisualizer(max_points=100, refresh_rate=10, time_window=1.0)

    t = 0.0
    while t < 20:  # 模拟 20 秒飞行
        # 添加随机数据模拟飞行状态
        vis.add_data(
            t,
            altitude=random.uniform(1000, 5000),
            speed=random.uniform(100, 300),
            pitch=random.uniform(-10, 10),
            roll=random.uniform(-30, 30),
            yaw=random.uniform(0, 360)
        )
        t += 0.1
        time.sleep(0.1)  # 模拟数据更新周期
    vis.pause()