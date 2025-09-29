import jsbsim
import pandas as pd

def run_c310_script():
    # 创建 FDM 实例
    fdm = jsbsim.FGFDMExec(root_dir=None)  # 如果有 JSBSim data 目录，指定 root_dir
    fdm.load_model("c310")                 # 加载飞机
    fdm.load_ic("./lyj_init.xml", True)                # 加载初始化文件
    fdm.run_ic()                           # 应用初始化
    
    dt = 0.01   # 时间步长
    end_time = 1000.0
    time = 0.0

    # 存储数据
    log_data = []

    while time <= end_time:
        # 阶段0：进入巡航状态
        if time >= 0 and time < 100:
            fdm['fcs/mixture-cmd-norm[0]'] = 1
            fdm['fcs/mixture-cmd-norm[1]'] = 1
            fdm['fcs/advance-cmd-norm[0]'] = 1.0
            fdm['fcs/advance-cmd-norm[1]'] = 1.0
            fdm['propulsion/magneto_cmd'] = 3
            fdm['propulsion/starter_cmd'] = 1

            fdm['fcs/throttle-cmd-norm[0]'] = 0.954
            fdm['fcs/throttle-cmd-norm[1]'] = 0.954

            fdm['ap/altitude_setpoint'] = 550.0
            fdm['ap/altitude_hold'] = 1

            fdm['ap/heading_setpoint'] = 0
            fdm['ap/heading-setpoint-select'] = 0
            fdm['ap/heading_hold'] = 1
            fdm['ap/attitude_hold'] = 0
            fdm['ap/yaw_damper'] = 1

        # 阶段1：100 秒后，开启空速保持 150 ft/s
        if time >= 100 and time < 200:
            fdm['ap/airspeed_setpoint'] = 150.0
            fdm['ap/airspeed_hold'] = 1

        # 阶段2：200 秒后，切换空速保持 200 ft/s
        if time >= 200:
            fdm['ap/airspeed_setpoint'] = 200.0
            fdm['ap/airspeed_hold'] = 1

        # 读取属性
        altitude = fdm['position/h-agl-ft']
        airspeed = fdm['velocities/u-fps']
        # 实时打印
        print(f"t={time:.2f}s | Alt={altitude:.2f} ft | Airspeed={airspeed:.2f} fps | ")

        # 推进一步
        fdm.run()

        # 采集需要的日志
        log_data.append({
            "time": time,
            "altitude_ft": fdm['position/h-agl-ft'],
            "lat_rad": fdm['position/lat-geod-rad'],
            "lon_rad": fdm['position/long-gc-rad'],
            "u_fps": fdm['velocities/u-fps'],
            "v_north_fps": fdm['velocities/v-north-fps'],
            "v_east_fps": fdm['velocities/v-east-fps'],
            # "roll_deg": fdm['attitude/roll-deg'],
            # "pitch_deg": fdm['attitude/pitch-deg'],
            # "yaw_deg": fdm['attitude/yaw-deg'],
            "beta_deg": fdm['aero/beta-deg'],
            "rudder_norm": fdm['fcs/rudder-pos-norm'],
            "airspeed_setpoint": fdm['ap/airspeed_setpoint'],
            "vc_kts": fdm['velocities/vc-kts'],
            "heading_true_deg": fdm['fcs/heading-true-degrees'],
        })

        # 更新时间
        time += dt

    # 保存为 CSV
    df = pd.DataFrame(log_data)
    df.to_csv("./jsbsim/c310_aptest.csv", index=False)

if __name__ == "__main__":
    run_c310_script()
