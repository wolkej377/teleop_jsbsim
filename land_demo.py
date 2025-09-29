import jsbsim
import pandas as pd

# 创建 FDM 实例
fdm = jsbsim.FGFDMExec(root_dir=None)  # 如果有 JSBSim data 目录，指定 root_dir
fdm.load_model("c310")                 # 加载飞机
# fdm.load_ic("./lyj_init.xml", True)                # 加载初始化文件
fdm.run_ic()                           # 应用初始化
    
# ---- 阶段 0: Initial Setup ----
def stage0():
    # 引擎初始化
    fdm["fcs/mixture-cmd-norm[0]"] = 1.0
    fdm["fcs/mixture-cmd-norm[1]"] = 1.0
    fdm["fcs/advance-cmd-norm[0]"] = 1.0
    fdm["fcs/advance-cmd-norm[1]"] = 1.0
    fdm["propulsion/magneto_cmd"] = 3
    fdm["fcs/throttle-cmd-norm[0]"] = 0.85
    fdm["fcs/throttle-cmd-norm[1]"] = 0.85
    fdm["propulsion/starter_cmd"] = 1

    # 自动驾驶设置
    fdm["ap/altitude_setpoint"] = 550.0
    fdm["ap/altitude_hold"] = 1
    fdm["ap/attitude_hold"] = 0
    fdm["guidance/target_wp_latitude_rad"] = 0.5168
    fdm["guidance/target_wp_longitude_rad"] = -1.6599238
    fdm["ap/heading_setpoint"] = 0
    fdm["ap/heading_hold"] = 1
    fdm["ap/heading-setpoint-select"] = 1
    fdm["ap/active-waypoint"] = 0

# ---- 阶段 1: Fly to The Gate (WP1) ----
def stage1():
    fdm["guidance/target_wp_latitude_rad"] = 0.51642159
    fdm["guidance/target_wp_longitude_rad"] = -1.6599238
    fdm["ap/active-waypoint"] = 1

    fdm["gear/gear-cmd-norm"] = 1
    fdm["fcs/flap-cmd-norm"] = 1  # 简化处理，不带 tc
    fdm["fcs/throttle-cmd-norm[0]"] = 0.6
    fdm["fcs/throttle-cmd-norm[1]"] = 0.6
    fdm["ap/altitude_setpoint"] = 380.0

# ---- 阶段 2: Final Approach ----
def stage2():
    fdm["guidance/target_wp_latitude_rad"] = 0.5166114
    fdm["guidance/target_wp_longitude_rad"] = -1.6599238
    fdm["ap/active-waypoint"] = 2
    fdm["ap/altitude_setpoint"] = 30.0
    fdm["fcs/throttle-cmd-norm[0]"] = 0.55
    fdm["fcs/throttle-cmd-norm[1]"] = 0.55

# ---- 阶段 3: Touchdown ----
def stage3():
    fdm["ap/altitude_setpoint"] = 0.0
    fdm["ap/altitude_hold"] = 0
    fdm["fcs/throttle-cmd-norm[0]"] = 0.25
    fdm["fcs/throttle-cmd-norm[1]"] = 0.25
    fdm["fcs/speedbrake-cmd-norm"] = 1.0
    fdm["fcs/left-brake-cmd-norm"] = 1.0
    fdm["fcs/right-brake-cmd-norm"] = 1.0

# ---- 阶段 4: End Simulation ----
def check_terminate():
    h_agl = fdm["position/h-agl-ft"]
    if h_agl < 5.0:
        fdm["simulation/terminate"] = 1
        return True
    return False

# --------------- 仿真循环 ----------------
sim_time = 0.0
dt = 0.01
max_time = 1000.0
# 存储数据
log_data = []

while sim_time < max_time:
    fdm.run()  # 运行一步
    sim_time += dt

    # 根据仿真时间触发事件
    if sim_time >= 0.25:
        stage0()
    if sim_time >= 2.0:
        stage1()                                                                                                                                                                                        
    # WP2 触发条件简化为时间或 waypoint
    if fdm["ap/active-waypoint"] == 1 and fdm["guidance/wp-distance"] < 500:
        stage2()
    # 着陆条件
    if fdm["ap/active-waypoint"] == 2 and fdm["position/h-agl-ft"] < 20.0 and fdm["velocities/vc-kts"] < 80:
        stage3()

    # 终止仿真
    if check_terminate():
        break

    # 可以打印部分输出
    print(f"Time: {sim_time:.2f}, Altitude: {fdm['position/h-agl-ft']:.1f}, Speed: {fdm['velocities/vc-kts']:.1f}")

    # 采集需要的日志
    log_data.append({
        "time": sim_time,
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
        "position/lat-geod-rad":fdm['position/lat-geod-rad'],
        "position/long-gc-rad":fdm['position/long-gc-rad']
    })
    print('hello world')

# 保存为 CSV
df = pd.DataFrame(log_data)
print(log_data)
df.to_csv("c310_land0.csv", index=False)