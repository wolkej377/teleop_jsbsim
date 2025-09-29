
from pyproj import Transformer
import pandas as pd
import airsim
import time
import numpy as np
import os
import pickle


# ECEF坐标转换为NED坐标
def ecef_to_ned(x, y, z, ref_x, ref_y, ref_z, ref_lat, ref_lon):
    """
    将ECEF坐标转换为相对于参考点的NED坐标
    
    参数:
    x, y, z: ECEF坐标 (米)
    ref_x, ref_y, ref_z: 参考点的ECEF坐标 (米)
    ref_lat, ref_lon: 参考点的纬度和经度 (度)
    
    返回:
    north, east, down: NED坐标 (米)
    """
    dx = x - ref_x
    dy = y - ref_y
    dz = z - ref_z

    phi = np.radians(ref_lat)
    lam = np.radians(ref_lon)

    sin_phi = np.sin(phi)
    cos_phi = np.cos(phi)
    sin_lam = np.sin(lam)
    cos_lam = np.cos(lam)

    # ECEF到NED的旋转矩阵
    R = np.array([
        [-sin_phi * cos_lam, -sin_phi * sin_lam,  cos_phi],
        [        -sin_lam,           cos_lam,       0    ],
        [-cos_phi * cos_lam, -cos_phi * sin_lam, -sin_phi]
    ])

    ned = R @ np.vstack((dx, dy, dz))
    return ned[0], ned[1], ned[2]  # N, E, D


def main(time_step=0.0001, height_offset=-150):
    csv_file = "c3104_waypoint.csv"
    vehicle_name = "drone_1"

    print(f"开始加载轨迹文件: {csv_file}")
    
    # 1. 加载数据
    try:
        df = pd.read_csv(csv_file)
        print(f"成功加载数据，共 {len(df)} 条轨迹点")
    except Exception as e:
        print(f"加载数据失败: {e}")
        return
    
    # 英尺到米的转换因子
    ft2m = 0.3048

    # 2. 提取位置和姿态数据
    # 姿态（使用 LOCAL 四元数）
    QW, QX, QY, QZ = df['Q(1)_{LOCAL}'], df['Q(2)_{LOCAL}'], df['Q(3)_{LOCAL}'], df['Q(4)_{LOCAL}']

    # 位置数据
    lat = df['Latitude Geodetic (deg)']
    lon = df['Longitude (deg)']
    alt = df['Altitude ASL (ft)']

    # ECEF坐标
    X, Y, Z = df['X_{ECEF} (ft)'] * ft2m, df['Y_{ECEF} (ft)']* ft2m, df['Z_{ECEF} (ft)'] * ft2m

    # 3. 提取参考点（初始点）
    lat_start = lat.iloc[0]
    lon_start = lon.iloc[0]
    alt_start = alt.iloc[0] * ft2m

    # 参考ECEF坐标
    ref_x, ref_y, ref_z = X.iloc[0], Y.iloc[0], Z.iloc[0]

   
    # 验证坐标转换
    lla2ecef = Transformer.from_crs("EPSG:4979", "EPSG:4978", always_xy=True)
    ref_x_check, ref_y_check, ref_z_check = lla2ecef.transform(lon_start, lat_start, alt_start)
    
    print(f"参考点经度: {lon_start}, 纬度: {lat_start}, 高度: {alt_start}米")
    print(f"参考点ECEF坐标 (CSV): X={ref_x}, Y={ref_y}, Z={ref_z}")
    print(f"参考点ECEF坐标 (转换): X={ref_x_check}, Y={ref_y_check}, Z={ref_z_check}")

    # 4. 将ECEF坐标转换为NED坐标
    ned_n, ned_e, ned_d = ecef_to_ned(X.values, Y.values, Z.values, 
                                      ref_x, ref_y, ref_z, 
                                      lat_start, lon_start)

    # 计算轨迹最后一个点到UE原点的偏移量
    last_index = len(df) - 1
    offset_n = -ned_n[last_index]+140
    offset_e = -ned_e[last_index]
    offset_d = -ned_d[last_index]

    print(f"计算偏移量: North={offset_n:.2f}m, East={offset_e:.2f}m, Down={offset_d:.2f}m")

    # 应用偏移量到所有轨迹点
    ned_n = ned_n + offset_n
    ned_e = ned_e + offset_e
    ned_d = ned_d + offset_d

    # 打印轨迹起始点和结束点的坐标
    print(f"应用偏移后，起始点坐标: North={ned_n[0]:.2f}m, East={ned_e[0]:.2f}m, Down={ned_d[0]:.2f}m")
    print(f"应用偏移后，着陆点坐标: North={ned_n[last_index]:.2f}m, East={ned_e[last_index]:.2f}m, Down={ned_d[last_index]:.2f}m")


    # 5. 连接AirSim
    client = airsim.VehicleClient()
    client.confirmConnection()
    print("已连接AirSim")

    # 6. 播放轨迹
    print(f"开始播放轨迹，使用飞行器: {vehicle_name}")
    
    # 记录开始时间
    start_time = time.time()

    try:
        for i in range(0, len(df), 1):
            # 设置位置和姿态
            position = airsim.Vector3r(ned_n[i], ned_e[i], ned_d[i] + height_offset)
            orientation = airsim.Quaternionr(QX.iloc[i], QY.iloc[i], QZ.iloc[i], QW.iloc[i])

            # 每10帧打印一次状态信息
            # if i % 200 == 0:
            #     print(f"帧 {i}/{len(df)}: 位置={position}")
            
            # 设置飞行器的位置和姿态
            pose = airsim.Pose(position, orientation)
            client.simSetVehiclePose(pose, ignore_collision=True, vehicle_name=vehicle_name)
            
            # 控制播放速度
            time.sleep(0.0001)

        # 计算总播放时间
        elapsed_time = time.time() - start_time
        print(f"轨迹播放完成! 总时长: {elapsed_time:.2f}秒")
        
    except KeyboardInterrupt:
        print("用户中断了轨迹播放")
    except Exception as e:
        print(f"播放轨迹时发生错误: {e}")
    finally:
        # 确保恢复模拟
        client.simPause(False)
        # 这里设置为True会导致运行一次后暂停模拟，后续GNSS和高度计全部失效
        print("仿真已恢复。")

    print("程序结束")

if __name__ == "__main__":
    main(0.0001, 0)
