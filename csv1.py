import jsbsim
import pandas as pd

# ---- 初始化 ----
fdm = jsbsim.FGFDMExec(root_dir=None)
fdm.load_model("c310")
fdm.run_ic()   # 应用初始条件

dt = 0.01       # 仿真步长 (s)
max_time = 10.0 # 仿真 10 秒
log_data = []

# ---- 仿真循环 ----
while fdm.get_sim_time() < max_time:
    fdm.run()  # 用 dt 推进
    sim_time = fdm.get_sim_time()

    # 打印高度和速度
    alt = fdm['position/h-agl-ft']
    vc = fdm['velocities/vc-kts']
    roll = fdm['attitude/phi-deg']
    pitch = fdm['attitude/theta-deg']
    yaw = fdm['attitude/psi-deg']
    lat = fdm['position/lat-geod-deg']
    long = fdm['position/long-gc-deg']
    print(f"t={sim_time:.2f}s  Alt={alt:.1f} ft  Vc={vc:.1f} kts")

    # 保存日志
    log_data.append({
        "time": sim_time,
        "altitude_ft": alt,
        "vc_kts": vc,
        "roll_deg": roll,
        "pitch_deg": pitch,
        "yaw_deg": yaw,
        "lat_deg": lat,
        "lon_deg": long 
    })

# ---- 写入 CSV ----
df = pd.DataFrame(log_data)
df.to_csv("demo_c310.csv", index=False)
print("✅ 仿真完成，数据已保存到 demo_c310.csv")
