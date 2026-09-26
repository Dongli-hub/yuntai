# rdk_aim —— 机载计算机（地瓜派）视觉瞄准程序

2025 电赛 E 题「简易自行瞄准装置」中，**机载计算机**部分的完整实现。

```
      RCT6 (循迹小车)               地瓜派 (本程序)                 H723 (云台)
   +------------------+        +----------------------+      +------------------+
   | 灰度巡线 + 电机   | UART   | 相机取图             | UART | 双轴稳定 + 速度环 |
   | 拐角/圈数/进度    |<------>| 找靶心 -> 找光斑      |<---->| 姿态IMU + 编码器  |
   +------------------+        | 解算角度偏置 -> 下发  |      | 执行偏置 + 激光IO |
                               +----------------------+      +------------------+
                                        |
                                 USB/MIPI 相机 + 同轴激光
```

## 一句话原理

激光与相机同轴 => **激光光斑在图像中的位置是固定的**（它和相机刚性固连），
转云台时靶心在图像里动、光斑不动。于是

> **激光打中靶心  <==>  光斑像素 == 靶心像素**

误差直接看得见、直接消得掉；而且像素->角度的增益就是焦距 `fx`，
**不需要任何机械标定就能得到正确的开环增益**，闭环只负责修残差。

## 快速开始

### PC 上先跑仿真（不需要任何硬件）

```bash
pip install -r requirements.txt          # numpy / opencv-python / pyserial / PyYAML
python main.py sim --duration 26 --laps 1 --start-car          # 行驶中连续瞄准
python main.py sim --duration 27 --laps 1 --draw --start-car   # 沿 r=6cm 红圈画圆
```

仿真里包含一个完整的合成场景：靶纸、红色同心圆、黑胶带外框、同轴激光光斑、
相机模型、云台一阶执行器、小车沿 1m x 1m 方形轨迹运动。
程序会自己闭环瞄准一遍，并把**每一帧的真值误差**写进 `out/aim_*.csv`
（`sim_miss_mm` = D1 真值，`sim_d2_mm` = D2 真值）。加 `--video` 还会录一段标注视频。

### 真机运行

```bash
python tools/calib_intrinsic.py --source 0            # 1) 相机内参（棋盘格）
python tools/check_laser.py --port /dev/ttyUSB0       # 2) 光斑能不能看见
python tools/calib_boresight.py --port-gimbal /dev/ttyUSB0 --source 0
python tools/view_detect.py --source 0                # 4) 看着画面调检测参数
python main.py run                                    # 5) 正式运行
python main.py unwind                                 # 6) 跑完一轮后解绕（见风险 R2）
```

### 比赛用的「先上电、后触发」口径

H723 上电自检要 3.4s，和「2s 内命中」直接冲突（见风险 R1）。推荐流程：

```bash
# 先给瞄准模块上电，程序会等云台就绪并保持不动
python main.py run --set app.timer_start=trigger --budget 2
# 屏幕打印「云台已就绪，等待触发」后，裁判喊"开始"时按回车（或 touch out/trigger）
```

## 目录

| 路径 | 作用 |
|---|---|
| `docs/00_设计大纲.md` | **先看这个**：完整设计逻辑与代码结构 |
| `docs/01_通信协议.md` | 与 H723 / RCT6 的串口协议逐字节定义 |
| `docs/02_标定与现场调试.md` | 标定流程 + 现场排障顺序 |
| `docs/03_赛题对照与风险对策.md` | 逐条对照赛题指标，列风险与对策 |
| `eaim/` | 程序本体 |
| `tools/calib_intrinsic.py` | 棋盘格标内参 |
| `tools/check_laser.py` | 检查光斑能否检出 + 标光轴点 |
| `tools/calib_boresight.py` | 标光轴点 + 自动判定 yaw/pitch 偏置符号 |
| `tools/view_detect.py` | 实时可视化调检测阈值 |
| `tools/fake_gimbal.py` | 假 H723，PC 上无硬件联调两条链路 |
| `tests/` | 离线自测（合成靶纸检测精度 / 控制环收敛 / 协议往返） |
| `configs/default.yaml` | 全部可调参数，带中文注释 |

## 自测

```bash
python tests/run_all.py     # 21 项，必须全绿
```
