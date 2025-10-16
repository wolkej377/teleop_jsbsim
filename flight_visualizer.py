import errno
import threading
from abc import ABC, abstractmethod
import airsim
from pyproj import Transformer
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from collections import deque
import time
import math
from collections import deque
import socket
import json

class SimDataSender:
    def __init__(self, host='127.0.0.1', port=5555):
        self.addr = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def stop(self):
        self.sock.close()
        print("已关闭UDP发送端")

    def send_udp(self, msg):
        data = (json.dumps(msg) + "\n").encode('utf-8')
        try:
            self.sock.sendto(data, self.addr)
            time.sleep(0.0001)
        except OSError as e:
            # 服务端未启动时静默跳过
            if e.errno in (errno.ECONNREFUSED, 10061, 10054, 111):
                pass
            else:
                print(f"发送异常: {e}")


class VisualizerBase(ABC):
    def __init__(self, host='0.0.0.0', port=5555, buffer_size=20):
        # 数据存储（队列）
        self.trajectory = deque(maxlen=buffer_size)
        self.lock = threading.Lock()
        self.wait_data = True
        self.offline_mode = False

        self.host = host
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.host, self.port))

        self.stop_event = threading.Event()
        self.recv_thread = threading.Thread(target=self.recv_data, daemon=True)

    def start(self, source="udp"):
        if source == "udp":
            self.recv_thread.start()
        else:
            self.offline_mode = True
            self.visualize_from_csv(source)
            print("离线模式，等待加载数据")
        try:
            self.visualize()
        except KeyboardInterrupt:
            if self.recv_thread.is_alive():
                self.stop_event.set()
                self.recv_thread.join(1.0)
            self.sock.close()
            print("可视化服务已关闭，程序退出。")

    @abstractmethod
    def recv_data(self):
        pass


    @abstractmethod
    def visualize(self):
        pass

    @abstractmethod
    def visualize_from_csv(self, csv_file, frequency):
        pass


class UEVisualizer(VisualizerBase):
    def __init__(self, vehicle_name="drone_1", height_offset=-150, time_step=0.0001):
        super().__init__()
        self.vehicle_name = vehicle_name
        self.height_offset = height_offset
        self.time_step = time_step

        self.client = airsim.VehicleClient()
        self.client.confirmConnection()
        print(f"已连接AirSim: {vehicle_name}")

        self.ref_point = {}

    def visualize(self):
        print_flag = True
        has_visualized = 0
        while not self.stop_event.is_set():
            with self.lock:
                # 队空判断
                if not self.trajectory:
                    if self.wait_data and print_flag:
                        print("轨迹数据为空，等待数据...CTRL+C退出")
                        print_flag = False
                    if self.offline_mode and not self.wait_data:
                        print("离线模式运行完毕，退出可视化线程")
                        self.stop_event.set()
                    continue
                total = len(self.trajectory)
                # 弹出一帧数据
                point = self.trajectory.popleft()
                remains = total - 1
            n = point['ned_n']
            e = point['ned_e']
            d = point['ned_d']
            qw = point['qw']
            qx = point['qx']
            qy = point['qy']
            qz = point['qz']

            self.client.simPause(True)
            pose = airsim.Pose(
                airsim.Vector3r(n, e, d + self.height_offset),
                airsim.Quaternionr(qx, qy, qz, qw)
            )
            self.client.simSetVehiclePose(pose, ignore_collision=True, vehicle_name=self.vehicle_name)
            has_visualized += 1
            if self.offline_mode:
                print(f"(UE)余:{remains}, N={n:.2f}, E={e:.2f}, D={d:.2f}, Q=({qw:.3f},{qx:.3f},{qy:.3f},{qz:.3f})")
            else:
                print(f"(UE)成功:{has_visualized}, N={n:.2f}, E={e:.2f}, D={d:.2f}, Q=({qw:.3f},{qx:.3f},{qy:.3f},{qz:.3f})")
            self.client.simPause(False)
            time.sleep(self.time_step)
            print_flag = True

    def recv_data(self):
        print(f"可视化服务器启动：({self.host}, {self.port})")
        recv_num = 0
        while not self.stop_event.is_set():
            try:
                # 默认客户端每次发送一行数据
                data, addr = self.sock.recvfrom(1024)
                msg = json.loads(data.decode('utf-8'))
                parsed = {
                    'longitude': msg.get('longitude', 0.0),
                    'latitude': msg.get('latitude', 0.0),
                    'altitude': msg.get('altitude', 0.0),
                    'roll': msg.get('roll', 0.0),
                    'pitch': msg.get('pitch', 0.0),
                    'yaw': msg.get('yaw', 0.0)
                }
                self.process_data(**parsed)
                recv_num += 1
                print(f"(UE)已接收数据: {recv_num}, 来自: {addr}")
            except Exception as e:
                if e.errno in (errno.EBADF, 10038):
                    break
                print(f"recv_data 异常: {e}")
                break

    @staticmethod
    def euler_to_quaternion(pitch, roll, yaw):
        # 将角度从度转换为弧度
        yaw_rad = math.radians(yaw)
        pitch_rad = math.radians(pitch)
        roll_rad = math.radians(roll)

        # 计算每个角度一半的余弦和正弦值
        cy = math.cos(yaw_rad * 0.5)
        sy = math.sin(yaw_rad * 0.5)
        cp = math.cos(pitch_rad * 0.5)
        sp = math.sin(pitch_rad * 0.5)
        cr = math.cos(roll_rad * 0.5)
        sr = math.sin(roll_rad * 0.5)

        # 根据 ZYX 旋转顺序计算四元数分量
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy

        return w, x, y, z


    def ecef_to_ned(self, X, Y, Z):
        # 支持单点或批量轨迹输入
        X = np.atleast_1d(X).flatten()
        Y = np.atleast_1d(Y).flatten()
        Z = np.atleast_1d(Z).flatten()

        dx = X - self.ref_point['x']
        dy = Y - self.ref_point['y']
        dz = Z - self.ref_point['z']

        phi = np.radians(self.ref_point['lat'])
        lam = np.radians(self.ref_point['lon'])

        sin_phi, cos_phi = np.sin(phi), np.cos(phi)
        sin_lam, cos_lam = np.sin(lam), np.cos(lam)

        # ECEF -> NED旋转矩阵
        R = np.array([
            [-sin_phi * cos_lam, -sin_phi * sin_lam,  cos_phi],
            [        -sin_lam,           cos_lam,       0    ],
            [-cos_phi * cos_lam, -cos_phi * sin_lam, -sin_phi]
        ])
        # shape (3, N)
        ecef = np.stack((dx, dy, dz), axis=0)
        ned = R @ ecef
        if X.size == 1:
            return ned[0].item(), ned[1].item(), ned[2].item()
        else:
            # 若输入为数组，返回3个1D numpy数组
            return ned[0], ned[1], ned[2]

    def set_preference_point(self, longitude, latitude, altitude):
        transformer = Transformer.from_crs("EPSG:4979", "EPSG:4978", always_xy=True)
        x, y, z = transformer.transform(longitude, latitude, altitude * 0.3048)
        self.ref_point['x'] = x
        self.ref_point['y'] = y
        self.ref_point['z'] = z
        self.ref_point['lat'] = latitude
        self.ref_point['lon'] = longitude
        self.ref_point['alt'] = altitude * 0.3048
        print(f"参考点设置: 经度={longitude}, 纬度={latitude}, 高度={altitude}, ECEF=({x:.2f},{y:.2f},{z:.2f})")

    def process_data(self, longitude, latitude, altitude, roll, pitch, yaw, *args, **kwargs):
        # longitude, latitude, roll, pitch, yaw 单位度
        # altitude 单位英尺
        transformer = Transformer.from_crs("EPSG:4979", "EPSG:4978", always_xy=True)
        x, y, z = transformer.transform(longitude, latitude, altitude * 0.3048)
        if not self.ref_point:
            self.set_preference_point(longitude, latitude, altitude)
            return
        qw, qx, qy, qz = self.euler_to_quaternion(pitch, roll, yaw)
        ned_n, ned_e, ned_d = self.ecef_to_ned(x, y, z) #返回的是单个点
        point = {
            "ned_n": ned_n,
            "ned_e": ned_e,
            "ned_d": ned_d,
            "qw": qw,
            "qx": qx,
            "qy": qy,
            "qz": qz
        }
        with self.lock:
            self.trajectory.append(point)
        
    def visualize_from_csv(self, csv_file, frequency=100):
        self.offline_mode = True
        # 清空现有轨迹
        self.trajectory = deque()
        # 设置参考点为最后一个点
        df = pd.read_csv(csv_file)
        last_row = df.iloc[-1]
        self.set_preference_point(
            last_row['lon_deg'],
            last_row['lat_deg'],
            last_row['altitude_ft']
        )

        required = ['time', 'altitude_ft', 'lat_deg', 'lon_deg', 'vc_kts', 'roll', 'pitch', 'yaw']
        for key in required:
            if key not in df.columns:
                raise ValueError(f"CSV 缺少必要列: {key}")
        df['time'] = pd.to_numeric(df['time'], errors='coerce')
        df = df.dropna(subset=['time'])

        last_time = None
        time_interval = 1.0 / frequency
        count = 0
        for _, row in df.iterrows():
            count += 1
            time_val = row['time']
            latitude = row['lat_deg']
            longitude = row['lon_deg']
            altitude = row['altitude_ft']
            roll = row['roll']
            pitch = row['pitch']
            yaw = row['yaw']
            if last_time is None or (time_val - last_time) >= time_interval:
                last_time = time_val
                self.process_data(longitude, latitude, altitude, roll, pitch, yaw)
                print(f"(UE)已加载数据{count}/{len(df)}: time={time_val:.2f}, lat={latitude:.4f}, lon={longitude:.4f}, alt={altitude:.1f}")
        # 后续没有数据添加，队列为空可退出线程
        self.wait_data = False

class PlotVisualizer:
    def __init__(self, csv_file):
        self.csv_file = csv_file
        self.df = None
        self._load_csv()
    
    def _load_csv(self):
        self.df = pd.read_csv(self.csv_file)
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
            ax.plot(self.df[x], self.__process_column(col))
            # 修改标签
            if col == 'vc_kts':
                col = 'vc_fps'
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
            ax.plot(self.df[x], self.__process_column(col), label=label)
        
        ax.set_xlabel(x)
        ax.set_ylabel(', '.join(y_columns))
        if title:
            ax.set_title(title)
        if len(y_columns) > 1:
            ax.legend()
        plt.show()
        return ax

    def __process_column(self, col):
        if col not in self.df.columns:
            raise ValueError(f"列 {col} 不存在")
        
        # 对不同列的数据处理
        if col == 'yaw':
            self.df[col] = self.df[col].apply(lambda x: x-360 if x>180 else x)
            
        if col == 'vc_kts':
            self.df[col] = self.df[col].apply(lambda x: x * 1.68781)
        
        return self.df[col]
    
    

if __name__ == "__main__":
    ue_vis = UEVisualizer()
    # 选择数据源：udp 或 csv 文件路径
    ue_vis.start(source="c310_teleop.csv")
    # ue_vis.start(source="udp")