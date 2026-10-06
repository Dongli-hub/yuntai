# k230_aim —— 把地瓜派上的瞄准程序迁移到 K230

## 当前进度（2026-10-06）

* H723 固件：链路已从 **USART1(PA9/PA10) 改到 UART7**（`Core/Src/usart.c`
  + `Core/Src/gimbal_link.c`），并新增：
  - UART7 引脚自动识别（先试 PE7/PE8，再试 PF6/PF7，都不通回落 PE7/PE8）
  - 心跳回 ACK(0x12)，用来分别确认"收"和"发"两个方向
  - ⚠ 需要在 STM32CubeIDE 里重新编译烧写（本机没有 ARM 工具链，没法替你编译）
* K230 端：
  - `tools/k230_link_test.py` —— **只测通信**，不依赖电机反馈（先跑这个）
  - `main.py` —— 完整瞄准程序（引脚配置 + 检测 + 控制 + 状态机），
    拷到 SD 卡根目录即上电自动运行
  - `tools/pc_sim_main.py` —— 电脑上离线跑 main.py 全链路仿真（已通过）

原程序在 `../rdk_aim/`（Ubuntu + OpenCV + numpy + pyserial）。
K230 跑的是 **CanMV MicroPython**：没有 OpenCV / numpy / 多线程 / Linux，
但自带 C 实现的 `image` 模块（OpenMV 风格 API）+ `machine.UART`，算力和延迟反而更适合做实时瞄准。

## 一、迁移后各模块的对应关系

| 地瓜派（rdk_aim） | K230（k230_aim） | 说明 |
|---|---|---|
| `eaim/camera.py`（OpenCV VideoCapture） | `media.sensor` + `sensor.snapshot()` | CanMV 相机通路 |
| `eaim/target.py`（自适应阈值+轮廓+单应） | `image.find_rects()` 找黑胶带框 + 对角线交点 | 靶心=四边形投影中心，不需要单应矩阵 |
| `eaim/laser.py`（颜色优势+连通域） | `image.find_blobs()` + LAB 阈值 + ROI 门控 | 光斑位置刚性固定，ROI 本身就是门控 |
| `eaim/aim.py`（积分视觉伺服） | 同样的数学，纯 Python 重写 | 唯一不能省的核心 |
| `eaim/protocol.py`（struct+CRC16） | 逐字节照抄（`ustruct` + CRC16/Modbus） | 协议不变，H723 不用改 |
| `eaim/state_machine.py`（完整状态机） | 精简版（BOOT_WAIT→SET_ZERO→AIM→LOST） | 先跑通，再补扫描/解绕 |
| 两条串口（云台+小车） | 先只做云台链路（H723），小车链路后加 | 一块 UART 一根线 |

**H723 固件一行都不用改** —— 协议、偏置语义、激光 IO 全部沿用。

## 二、接线（3 根线）

```
   YAHBOOM K230                       STM32H723
   EXPORT 排针                        云台板
   +-------------+
   | 5V          | --（K230 自己供电时可不接，不要两边同时供）-- 5V
   | GND         | ------------------------------------------ GND
   | IO9  (TXD)  | ------------------------------------------ PA10 (USART1_RX)
   | IO10 (RXD)  | ------------------------------------------ PA9  (USART1_TX)
   +-------------+
```

* 115200 8N1，3.3V 电平，**必须共地**
* IO9/IO10 是模块 EXPORT 接口上的丝印（见说明书「12Pin GPIO 介绍」页右侧）
* 备选：12Pin GPIO 的 UART3 = IO32(TXD)/IO33(RXD)，脚本里留了切换开关
* H723 侧的 UART7 接插件有两种可能的引脚对（PE7/PE8 或 PF6/PF7），
  固件会在启动时自动试出来；如果两个都不是，请把接插件旁的丝印告诉我

## 三、分步替换流程（每步都能单独验证）

| 步骤 | 脚本 | 验证什么 | 通过标准 |
|---|---|---|---|
| 0 | `tools/t0_env.py` | 固件、可用模块、UART 通道 | 全部 import OK，UART 能建立 |
| 1 | `tools/k230_link_test.py` | K230 ↔ H723 串口链路（只看通信） | 打印 `双向通信 OK`，`crc_err=0` |
| 2 | `tools/t2_camera.py` | 相机 + 黑框检测 | 画面上黑胶带框被框住，中心十字在靶心附近 |
| 3 | `tools/t3_spot.py` | 激光光斑检测 | 屏幕上光斑被框住，坐标稳定（跑之前先让激光亮） |
| 4 | `main.py` | 完整瞄准闭环 | 误差收敛、偏置下发、丢靶安全 |
| 5 | 脱机运行 | 拷到 SD 卡根目录 | 整体上电自动开始瞄准 |

## 四、在 CanMV IDE 里怎么跑

1. 用 CanMV IDE 打开 `D:\STM32\STM32projects\yuntai2\k230_aim\tools\t0_env.py`
2. 连上 K230（左下角连接图标），点绿色 ▶ 运行
3. 看下方「串行终端」的输出，把内容发给我

每个脚本都是**单文件自包含**的，不依赖其他文件，可以逐个跑。

## 五、参数放哪里

每个脚本顶部都有一块 `===== 参数 =====`，改完直接重新运行即可。
等这些参数在板子上实测稳定后，再合并进最终的 `main.py`（同样单文件，方便脱机保存）。

## 八、电脑上的离线仿真（不接硬件先把逻辑跑通）

```powershell
cd D:\STM32\STM32projects\yuntai2
python k230_aim\tools\pc_sim_main.py          # cv2 方案
python k230_aim\tools\pc_sim_main.py rects    # 原生 find_rects 方案
```

仿真会伪造 K230 环境和一台 H723（含云台一阶模型），完整跑
「WAIT_READY → SET_ZERO → TRACK」并检查闭环是否收敛。
当前结果：两个方案的末端残差都 < 3px。

## 六、K230 固件能力（按你电脑上的官方资料确认）

| 能力 | 模块 | 用途 |
|---|---|---|
| 相机/显示 | `media.sensor` `media.display` `media.media` | `sensor.snapshot()` 取图，`Display.show_image()` 显示 |
| 原生图像处理 | `image`（OpenMV 风格） | `find_rects` / `find_blobs` / `draw_*` |
| **OpenCV 移植版** | `import cv2` | `adaptiveThreshold` / `morphologyEx` / `findContours` / `approxPolyDP` / `warpPerspective` / `getPerspectiveTransform` … |
| 轻量 NumPy | `ulab.numpy as np` | 和 OpenCV 版配合用 |
| 串口 | `machine.UART` / `ybUtils.YbUart` | 115200 与 H723 通信 |

> 结论：地瓜派 `eaim/target.py`（自适应阈值->轮廓->四边形->单应）和 `eaim/laser.py`
> 的算法都可以**基本原样移植**，这是风险最低的路线。`find_rects` 是备选方案。

## 七、PC 端离线自测（不需要 K230，先跑这个）

协议层已经和地瓜派版逐字节比对过：

```powershell
cd D:\STM32\STM32projects\yuntai2
python k230_aim\tools\pc_selftest_proto.py
```

12 项全部 PASS 才说明 K230 发出的每一帧和 H723 期待的一致。
