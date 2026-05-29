kp由1.5降低至0.5，避免关节抓握物品时因无法到达预设位置而断电

传感器分布图：
[![64b093c7752344e7a8567524fc0c69cd-7680x4320.png](https://i.postimg.cc/CLFN5xdg/64b093c7752344e7a8567524fc0c69cd-7680x4320.png)](https://postimg.cc/s1NS0yp6)

右手状态话题：/dex3/right/state

demo路径：/home/unitree/unitree_dex3_cpp/example

读取并打印传感器数值：
python3 debug_dex3_right.py eth0 (默认读取九个id的数值，如果让终端只打印指定id的数值，可以加上 --id n)

终端以2hz频率打印数值
输出示例：
```
unitree@ubuntu:~/unitree_dex3_cpp/example$ python3 debug_dex3_right.py
Starting Dex-3 right-hand debug on net_if='eth0'...
Mode: tactile-only (read & print pressure at 2 Hz).
Using fixed tactile topic: rt/lf/dex3/right/state
CYCLONEDDS_URI=/home/unitree/cyclonedds_ws/cyclonedds.xml
Dex-3 hand with 7 DOFs.
UnitreeController initialized with network interface: eth0
Odometry enabled, subscribing to sport state topic: rt/odommodestate
G1 type: 5
Subscribed right hand tactile topic: rt/lf/dex3/right/state
Tactile topic ready: rt/lf/dex3/right/state
UnitreeController self-check passed.
Capturing baseline (5 frames)...
Baseline captured:
  ID0: [10.1222, 0.0000, 10.1085, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 10.1082, 0.0000, 10.1088]
  ID1: [0.0000, 0.0000, 0.0000, 10.1016, 0.0000, 0.0000, 10.1059, 0.0000, 10.3045, 0.0000, 0.0000, 0.0000]
  ID2: [10.1115, 0.0000, 10.1046, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 10.1131, 0.0000, 10.1122]
  ID3: [0.0000, 0.0000, 0.0000, 10.1318, 0.0000, 0.0000, 10.3141, 0.0000, 10.3317, 0.0000, 0.0000, 0.0000]
  ID4: [10.1202, 0.0000, 10.1120, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 10.1162, 0.0000, 10.1162]
  ID5: [0.0000, 0.0000, 0.0000, 10.1184, 0.0000, 0.0000, 10.1109, 0.0000, 10.3240, 0.0000, 0.0000, 0.0000]
  ID6: [10.1069, 0.0000, 10.1104, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 10.1070, 0.0000, 10.1091]
  ID7: [10.1088, 0.0000, 10.1118, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 10.1133, 0.0000, 10.1112]
  ID8: [10.1096, 0.0000, 10.1101, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 10.1078, 0.0000, 10.1056]

[Dex3 Right Pressure Diff (current - baseline)]
ID0: [-0.0006, 0.0000, -0.0013, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, -0.0010, 0.0000, 0.0000]
ID1: [0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, -0.0003, 0.0000, 0.0003, 0.0000, 0.0000, 0.0000]
ID2: [-0.0003, 0.0000, 0.0010, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, -0.0003, 0.0000, -0.0010]
ID3: [0.0000, 0.0000, 0.0000, 0.0010, 0.0000, 0.0000, 0.0003, 0.0000, 0.0003, 0.0000, 0.0000, 0.0000]
ID4: [0.0006, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0006, 0.0000, -0.0010]
ID5: [0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0003, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000]
ID6: [0.0003, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, -0.0006, 0.0000, 0.0013]
ID7: [0.0000, 0.0000, -0.0006, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0003, 0.0000, 0.0000]
ID8: [0.0000, 0.0000, 0.0003, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, -0.0006, 0.0000, 0.0000]
```

伸出一根手指：
unitree@ubuntu:~/unitree_dex3_cpp/example$ python3 control_dex3_right_setpoint.py enP8p1s0 0 -1.05 -1.7 1.7 1.8 0 0 
[![wei-xin-tu-pian-20260312173522-108-64.jpg](https://i.postimg.cc/QN2qXgsV/wei-xin-tu-pian-20260312173522-108-64.jpg)](https://postimg.cc/mhS9jHPG)

合上：unitree@ubuntu:~/unitree_dex3_cpp/example$ python3 control_dex3_right_setpoint.py enP8p1s0 0 -1.05 -1.7 1.7 1.8 1.7 1.8

[![wei-xin-tu-pian-20260312173522-109-64.jpg](https://i.postimg.cc/mDcQ4k5Y/wei-xin-tu-pian-20260312173522-109-64.jpg)](https://postimg.cc/PpjLzXhJ)

抓取测试demo：
unitree@ubuntu:~/unitree_dex3_cpp/example$ python3 control_dex3_right_grasp.py