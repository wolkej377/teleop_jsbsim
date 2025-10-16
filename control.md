### 控制量
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

### 环境
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
python -c "import sys; print(sys.executable)"
```

输出例如：C:\Users\MSP\.conda\`envs\flight\python.exe`

将上面两个文件放到目录，选择覆盖文件

C:\Users\MSP\.conda\`envs\flight\Lib\site-packages\jsbsim\aircraft\c310`