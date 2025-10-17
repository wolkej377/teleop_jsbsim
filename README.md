## JSBSIM在线轨迹生成
### 功能
该系统基于 JSBSim 飞行动力学仿真引擎 和 Unreal Engine（UE）可视化，通过 Python 脚本实现的主要功能包括：
+ 	JSBSim 仿真环境的初始化与主循环运行，支持条件响应式控制逻辑。
+ 	交互式输入输出接口，可通过键盘或外部程序实时发送控制指令并获取飞行状态。
+ 	数据通信与存储：飞行状态可通过 UDP 实时发送，同时支持离线记录为 CSV 文件。
+ 	实时可视化：基于 UDP 数据在 UE 中展示飞机姿态与飞行轨迹。
+   离线可视化：利用 CSV 数据实现轨迹回放与姿态绘图，同时支持 UE 界面的离线展示。

### 1. 依赖
+ python库
```bash
conda create -n flight python=3.10 -y
conda activate flight
pip install numpy pandas matplotlib
pip install msgpack-rpc-python
pip install airsim
python -c "import airsim; print('AirSim OK')"
pip install jsbsim
python -c "import jsbsim; print('JSBSim OK')"
conda install pyproj
pip install pynput
```
+ 配置文件
```bash
python -c "import sys; print(sys.executable)"

```
输出例如：  
`C:\Users\MSP\.conda\`envs\flight\python.exe   
将 lyj_init.xml、c310ap.xml 放到目录：    
`C:\Users\MSP\.conda\`envs\flight\Lib\site-packages\jsbsim\aircraft\c310


### 附录
1. 控制输出 
```python
fcs/elevator-cmd-norm   
fcs/aileron-cmd-norm    
fcs/rudder-cmd-norm 
fcs/flap-cmd-norm   
fcs/steer-cmd-norm  
fcs/spoiler-cmd-norm

fcs/mixture-cmd-norm[0]
systems/mixture-cmd-norm
fcs/throttle-cmd-norm[0]
fcs/advance-cmd-norm[0]
propulsion/starter_cmd
propulsion/magneto_cmd

fcs/right-brake-cmd-norm
fcs/speedbrake-cmd-norm
fcs/left-brake-cmd-norm
gear/gear-cmd-norm
simulation/terminate
```

2. 状态读取的变量
```python
self.fdm['position/h-agl-ft'] 
self.fdm['position/lat-geod-deg'] 
self.fdm['position/long-gc-deg'] 
self.fdm['velocities/vc-kts'] 
self.fdm['attitude/phi-deg'] 
self.fdm['attitude/theta-deg'] 
self.fdm['attitude/psi-deg'] 
```
