import pandas as pd
import matplotlib.pyplot as plt

# 1. 读取 CSV 文件
csv_file = "c310_aptest.csv"  # 替换为你的 CSV 文件路径
csv_file = "./jsbsim/c310_aptest.csv"  # 替换为你的 CSV 文件路径
df = pd.read_csv(csv_file)

# 2. 单位转换
# velocities/vc-kts 从节转为英尺/秒
df['velocities/vc-fps'] = df['velocities/vc-kts'] * 1.68781  # 1 kt = 1.68781 ft/s

# 3. 提取需要绘制的列
time = df.index  # 假设每行是固定时间间隔，可以用索引表示时间
airspeed_setpoint = df['ap/airspeed_setpoint']
airspeed_actual = df['velocities/vc-fps']
airspeed_error = df['ap/airspeed-error-origin']
pid_output = df['ap/airspeed-pi-throttle']
throttle = df['fcs/throttle-cmd-norm[0]']
altitude = df['position/h-agl-ft']

# 4. 绘制图形
fig, axs = plt.subplots(5, 1, figsize=(12, 18), sharex=True)

# 空速设定值和实际空速
axs[0].plot(time, airspeed_setpoint, label='Airspeed Setpoint (ft/s)')
axs[0].plot(time, airspeed_actual, label='Airspeed Actual (ft/s)')
axs[0].set_ylabel('ft/s')
axs[0].legend()
axs[0].grid(True)
axs[0].set_title('Airspeed')

# 空速误差
axs[1].plot(time, airspeed_error, color='orange')
axs[1].set_ylabel('ft/s')
axs[1].grid(True)
axs[1].set_title('Airspeed Error')

# PI 控制器输出
axs[2].plot(time, pid_output, color='green')
axs[2].set_ylabel('PI Output (-0.5~0.5)')
axs[2].grid(True)
axs[2].set_title('Airspeed PI Controller Output')

# 油门
axs[3].plot(time, throttle, color='red')
axs[3].set_ylabel('Throttle (0~1)')
axs[3].grid(True)
axs[3].set_title('Throttle Command')

# 高度
axs[4].plot(time, altitude, color='purple')
axs[4].set_ylabel('Altitude (ft)')
axs[4].set_xlabel('Time (samples)')
axs[4].grid(True)
axs[4].set_title('Aircraft Altitude')

plt.tight_layout()
plt.show()
