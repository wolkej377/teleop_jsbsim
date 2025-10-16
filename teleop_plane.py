import threading
import time
from pynput import keyboard
from fcs_core import AircraftSimulation
from flight_visualizer import UEVisualizer, PlotVisualizer
# 键盘控制说明：
# r 控制自动驾驶速度保持开关
# q e 控制自动驾驶保持速度
# v 控制自动驾驶高度保持开关
# z c 控制自动驾驶保持高度
# w s 控制油门throttle
# a d 控制方向舵rudder
# ↑ ↓ 控制升降舵elevator
# ← → 控制副翼aileron

# 全局变量，存储当前的键盘状态（最新输入）
current_key = None
lock = threading.Lock()

class FlightVariable:
    def __init__(self, simulation, name, min=0.0, max=1.0, step=0.05, initial=0.0):
        self.simulation = simulation
        self.name = name
        self.min = min
        self.max = max
        self.step = step
        self.value = initial
    
    @property
    def value(self):
        return self._value
    
    @value.setter
    def value(self, v):
        self._value = max(self.min, min(self.max, v))
        self.simulation.add_command(self.name, self._value)

     
    def increase(self):
        self.value += self.step

    def decrease(self):
        self.value -= self.step

vis = UEVisualizer()
sim = AircraftSimulation(max_time=100.0, log_csv="c310_teleop.csv", visualizer=vis)

throttle0 = FlightVariable(simulation=sim, name="fcs/throttle-cmd-norm[0]", min=0.0, max=1.0, step=0.01, initial=0.954)
throttle1 = FlightVariable(simulation=sim, name="fcs/throttle-cmd-norm[1]", min=0.0, max=1.0, step=0.01, initial=0.954)
rudder = FlightVariable(simulation=sim, name="fcs/rudder-cmd-norm", min=-1.0, max=1.0, step=0.05, initial=0.0)
elevator = FlightVariable(simulation=sim, name="fcs/elevator-cmd-norm", min=-1.0, max=1.0, step=0.05, initial=0.0)
aileron = FlightVariable(simulation=sim, name="fcs/aileron-cmd-norm", min=-1.0, max=1.0, step=0.05, initial=0.0)

# 自动驾驶相关
airspeed_setpoint = FlightVariable(simulation=sim, name="ap/airspeed_setpoint", min=20.0, max=300.0, step=5.0, initial=200.0)
airspeed_hold = FlightVariable(simulation=sim, name="ap/airspeed_hold", min=0, max=1, step=1, initial=1)
altitude_setpoint = FlightVariable(simulation=sim, name="ap/altitude_setpoint", min=50.0, max=1000.0, step=10.0, initial=550.0)
alltitude_hold = FlightVariable(simulation=sim, name="ap/altitude_hold", min=0, max=1, step=1, initial=1)

def on_press(key):
    global current_key
    try:
        with lock:
            current_key = key.char  # 普通字符键 
    except AttributeError:
        with lock:
            current_key = str(key)  # 特殊键

def on_release(key):
    global current_key
    # 松开键时清空
    with lock:
        current_key = None
    # if key == keyboard.Key.esc:
    #     return False

def keyboard_listener():
    with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
        listener.join()

def set_controls_to_jsbsim():
    global throttle0, throttle1, rudder, elevator, aileron
    while True:
        with lock:
            key = current_key
        if key:
            if key == 'w':
                throttle0.increase()
                throttle1.increase()
                print(f"油门动作：加 当前值:{throttle0.value:.2f}")
            elif key == 's':
                throttle0.decrease()
                throttle1.decrease()
                print(f"油门动作：减 当前值:{throttle0.value:.2f}")
            elif key == 'q':
                altitude_setpoint.increase()
                print(f"高度保持：升高 当前值:{altitude_setpoint.value:.2f}ft")
            elif key == 'e':
                altitude_setpoint.decrease()
                print(f"高度保持：降低 当前值:{altitude_setpoint.value:.2f}ft")
            elif key == 'z':
                airspeed_setpoint.decrease()
                print(f"速度保持：降低 当前值:{airspeed_setpoint.value:.2f}kt")
            elif key == 'c':
                airspeed_setpoint.increase()
                print(f"速度保持：升高 当前值:{airspeed_setpoint.value:.2f}kt")
            elif key == 'a':
                rudder.decrease()
                print(f"方向舵动作：左 当前值:{rudder.value:.2f}")
            elif key == 'd':
                rudder.increase()
                print(f"方向舵动作：右 当前值:{rudder.value:.2f}")
            elif key == 'v':
                if alltitude_hold.value == 0:
                    alltitude_hold.value = 1
                else:
                    alltitude_hold.value = 0
                print(f"高度保持开关：{('关闭' if alltitude_hold.value==0 else '开启')} 当前值:{alltitude_hold.value}")
            elif key == 'r':
                if airspeed_hold.value == 0:
                    airspeed_hold.value = 1
                else:
                    airspeed_hold.value = 0
                print(f"速度保持开关：{('关闭' if airspeed_hold.value==0 else '开启')} 当前值:{airspeed_hold.value}")
            elif key == 'Key.up':
                elevator.increase()
                print(f"升降舵动作：上 当前值:{elevator.value:.2f}")
            elif key == 'Key.down':
                elevator.decrease()
                print(f"升降舵动作：下 当前值:{elevator.value:.2f}")
            elif key == 'Key.left':
                aileron.decrease()
                print(f"副翼动作：左滚转 当前值:{aileron.value:.2f}")
            elif key == 'Key.right':
                aileron.increase()
                print(f"副翼动作：右滚转 当前值:{aileron.value:.2f}")
        # 控制频率20hz
        time.sleep(0.05)

def get_states_from_jsbsim():
    pass

def main():
    key_t = threading.Thread(target=keyboard_listener, daemon=True)
    in_t = threading.Thread(target=set_controls_to_jsbsim, daemon=True)
    out_t = threading.Thread(target=get_states_from_jsbsim, daemon=True)
    key_t.start()
    in_t.start()
    out_t.start()
    sim.run_simulation(initial_work="initial_work1")

if __name__ == "__main__":
    main()
    csv_vis = PlotVisualizer("c310_teleop.csv")
    csv_vis.plot(
    y_columns=['altitude_ft','vc_kts'],
    titles=['Altitude','Velocity'],
    )
    csv_vis.plot_combined(
    y_columns=['roll','pitch','yaw'],
    labels=['Roll','Pitch','Yaw'],
    title='Aircraft Attitude vs Time'
    )
    csv_vis.plot(
    y_columns=['lat_deg','lon_deg'],
    titles=['Latitude','Longitude'],
    )