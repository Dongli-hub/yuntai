#!/usr/bin/env bash
# ============================================================================
# 地瓜派（RDK）一次性环境准备
#
# 用法（在 VSCode 的 SSH 终端里，把整个 rdk_aim 目录拷到板子上之后）：
#     cd ~/rdk_aim/deploy
#     bash setup_on_rdk.sh
#
# 它做四件事：查 Python 依赖 -> 查串口权限 -> 查摄像头 -> 打印下一步命令。
# 全部是"只检查 + 必要时安装"，不会改系统配置（除了往 dialout 组里加人）。
# ============================================================================
set -u

WS="${1:-$HOME/e_aim}"
echo "=============================================================="
echo " rdk_aim 环境准备"
echo "=============================================================="

# ---- 1. Python 依赖 -------------------------------------------------------
echo "[1/4] 检查 Python 依赖..."
python3 - <<'PY'
import sys
mods = [("numpy", "numpy"), ("cv2", "opencv-python"), ("serial", "pyserial")]
missing = []
for mod, pkg in mods:
    try:
        __import__(mod)
        print("  OK   %-12s" % pkg)
    except ImportError:
        missing.append(pkg)
        print("  缺   %-12s" % pkg)
if missing:
    print("\n  请先安装：pip3 install " + " ".join(missing))
    sys.exit(1)
PY
if [ $? -ne 0 ]; then
    echo "依赖不全，先装完再跑本脚本。"
    exit 1
fi

# ---- 2. 串口权限 ----------------------------------------------------------
echo "[2/4] 检查串口权限..."
if id -nG "$USER" | grep -qw dialout; then
    echo "  OK   已在 dialout 组"
else
    echo "  当前用户不在 dialout 组，串口会打不开。执行："
    echo "      sudo usermod -aG dialout $USER"
    echo "  然后【退出 SSH 重新登录】（组权限要重新登录才生效）"
fi
echo "  当前可用的串口设备："
ls /dev/ttyS* /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | sed 's/^/      /' || echo "      没找到"

# ---- 3. 摄像头 ------------------------------------------------------------
echo "[3/4] 检查摄像头..."
if ls /dev/video* >/dev/null 2>&1; then
    ls /dev/video* | sed 's/^/      /'
    echo "  用 v4l2-ctl --list-formats-ext -d /dev/video0 可以看支持的分辨率/帧率"
else
    echo "  没找到 /dev/video*。USB 摄像头插好后再跑一次；"
    echo "  MIPI 摄像头要另配 GStreamer 管道（见 configs/default.yaml 的 camera.source: mipi）"
fi

# ---- 4. 工作区 ------------------------------------------------------------
echo "[4/4] 准备运行目录 $WS ..."
mkdir -p "$WS/out"
echo "  OK   $WS/out"

cat <<EOF

==============================================================
 准备完毕。下一步：

 1) 把程序拷到工作区（在 PC 上执行，<板子IP> 换成实际地址）：
      scp -r rdk_aim sunrise@<板子IP>:~/e_aim

 2) 确认串口设备名（H723 一直在发遥测，接对就能看到）：
      cd ~/e_aim && python3 main.py ports
      python3 main.py ping --port /dev/ttyS1

 3) 标定（第一次必做）：
      python3 tools/check_laser.py --port /dev/ttyS1 --source 0 --save
      python3 tools/calib_boresight.py --port-gimbal /dev/ttyS1 --source 0

 4) 跑瞄准：
      python3 main.py run --duration 20 --budget 4

 5) 跑完一轮后解绕：
      python3 main.py unwind --duration 10

 详细排查顺序见 ~/e_aim/docs/02_标定与现场调试.md，
 H723 侧的联调步骤见 yuntai2/docs/04_H723与地瓜派联调.md。
==============================================================
EOF

