import jsbsim
import pandas as pd
import queue

class AircraftSimulation:
    def __init__(self, max_time=1000.0, init_xml="./lyj_init.xml", log_csv="c310_demo.csv", broadcaster=None):
        self.fdm = jsbsim.FGFDMExec(root_dir=None)
        self.fdm.load_model("c310")
        self.fdm.load_ic(init_xml, True)
        self.fdm.run_ic()

        self.sim_time = 0.0
        self.max_time = max_time
        self.log_csv = log_csv
        self.broadcaster = broadcaster
        self.main_script = None
        self.print_enable = True

        self.log_data = []
        # 命令队列，外部通过 add_command 添加
        self.command_queue = queue.Queue()

    # 初始化任务
    def initial_work1(self):
        # 动力相关
        self.fdm['fcs/mixture-cmd-norm[0]'] = 1.0
        self.fdm['fcs/mixture-cmd-norm[1]'] = 1.0
        self.fdm['fcs/advance-mocmd-norm[0]'] = 1.0
        self.fdm['fcs/advance-cmd-norm[1]'] = 1.0
        self.fdm['propulsion/magneto_cmd'] = 3
        self.fdm['propulsion/starter_cmd'] = 1

        self.fdm['fcs/throttle-cmd-norm[0]'] = 0.954
        self.fdm['fcs/throttle-cmd-norm[1]'] = 0.954
        # 高度保持
        self.fdm['ap/altitude_setpoint'] = 550.0
        self.fdm['ap/altitude_hold'] = 1
        # 航向保持
        self.fdm['ap/heading_setpoint'] = 0
        self.fdm['ap/heading-setpoint-select'] = 0
        self.fdm['ap/heading_hold'] = 1
        # 姿态保持
        self.fdm['ap/attitude_hold'] = 1    
        # 横滚阻尼
        self.fdm['ap/yaw_damper'] = 1

    def initial_work2(self):
        pass

    def add_command(self, attr_name, value):
        # 在外部线程调用，添加控制命令到队列
        def cmd():
            self.fdm[attr_name] = value
            print(f"[Command executed] {attr_name} = {value}")
        self.command_queue.put((cmd, (), {}))

    def process_commands(self):
        # 每个时间步执行队列中的所有命令
        while not self.command_queue.empty():
            func, args, kwargs = self.command_queue.get()
            func(*args, **kwargs)


    def check_terminate(self):
        h_agl = self.fdm["position/h-agl-ft"]
        if h_agl < 5.0:
            self.fdm["simulation/terminate"] = 1
            return True
        return False


    def log_state(self):
        self.log_data.append({
            "time": self.sim_time,
            "altitude_ft": self.fdm['position/h-agl-ft'],
            "lat_deg": self.fdm['position/lat-geod-deg'],
            "lon_deg": self.fdm['position/long-gc-deg'],
            "vc_kts": self.fdm['velocities/vc-kts'],
            "roll": self.fdm['attitude/phi-deg'],
            "pitch": self.fdm['attitude/theta-deg'],
            "yaw": self.fdm['attitude/psi-deg'],
        })

    def visualize_sync(self):
        # UE可视化
        if self.broadcaster:
            msg = {
                'time': self.sim_time,
                'longitude': self.fdm['position/long-gc-deg'],
                'latitude': self.fdm['position/lat-geod-deg'],
                'altitude': self.fdm['position/h-agl-ft'],
                'speed': self.fdm['velocities/vc-kts'],
                'roll': self.fdm['attitude/phi-deg'],
                'pitch': self.fdm['attitude/theta-deg'],
                'yaw': self.fdm['attitude/psi-deg']
            }
            self.broadcaster.send_udp(msg)
        # 打印状态
        msg = f"(JSBSIM)Time:{self.sim_time:.2f}"
        msg += f", Speed:{self.fdm['velocities/vc-kts']:.1f}"
        msg += f", Altitude:{self.fdm['position/h-agl-ft']:.1f}"
        msg += f", Lat:{self.fdm['position/lat-geod-deg']:.4f}"
        msg += f", Lon:{self.fdm['position/long-gc-deg']:.4f}"
        msg += f", Pitch:{self.fdm['attitude/theta-deg']:.1f}"
        msg += f", Roll:{self.fdm['attitude/phi-deg']:.1f}"
        msg += f", Yaw:{self.fdm['attitude/psi-deg']:.1f}"
        if self.print_enable:
            print(msg)

    # 仿真循环
    def run_simulation(self, initial_work="initial_work1"):
        # 初始化
        if initial_work == "initial_work1":
            self.initial_work1()
        while self.sim_time < self.max_time:
            self.fdm.run()
            self.sim_time = self.fdm.get_sim_time()
            # 执行外部发来的命令
            self.process_commands()

            # 在这里运行脚本控制逻辑
            if self.main_script:
                self.main_script(self)

            # 着陆判断
            if self.check_terminate():
                break
            # 可视化同步
            self.visualize_sync()

            self.log_state()

        # 保存 CSV
        df = pd.DataFrame(self.log_data)
        df.to_csv(self.log_csv, index=False)
        print(f"Simulation finished. Data saved to {self.log_csv}")
        if self.broadcaster:
            self.broadcaster.stop()


if __name__ == "__main__":
    from flight_visualizer import PlotVisualizer, SimDataSender, UEVisualizer
    bro = SimDataSender()
    sim = AircraftSimulation(max_time=100.0, broadcaster=bro)

    def main_script(this):
        # 在这里预设脚本控制逻辑
        # 例如：30秒后改变速度
        # 例如：60秒后改变高度
        if this.sim_time > 30.0:
            this.fdm['ap/airspeed_setpoint'] = 150.0
            this.fdm['ap/airspeed_hold'] = 1
        if this.sim_time > 60.0:
            this.fdm['ap/altitude_setpoint'] = 200.0
            this.fdm['ap/altitude_hold'] = 1

    sim.main_script = main_script
    sim.run_simulation()

    # 仿真结束后执行可视化
    csv_vis = PlotVisualizer("c310_demo.csv")
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
