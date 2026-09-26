#!/usr/bin/env bash
# ============================================================================
# 上电后的 30 秒快速体检（在地瓜派上跑）
#
#     bash ~/e_aim/deploy/quick_check.sh
#
# 只做检查、不发运动指令，安全。
# 判断顺序：串口 -> H723 遥测 -> 相机 -> 靶纸检出。
# 哪一步不对，脚本会直接告诉你该跑哪条命令细查。
# ============================================================================
set -u
WS="$(cd "$(dirname "$0")/.." && pwd)"
cd "$WS" || exit 1
PORT="${1:-/dev/ttyS1}"
PASS=0
FAIL=0

ok()  { echo "  [OK]   $1"; PASS=$((PASS+1)); }
bad() { echo "  [FAIL] $1"; FAIL=$((FAIL+1)); }

echo "=============================================================="
echo " 云台-机载计算机快速体检   ($WS)"
echo "=============================================================="

echo "[1/4] 串口设备"
if [ -e "$PORT" ]; then
    ok "$PORT 存在"
else
    bad "$PORT 不存在 —— 先跑：python3 main.py ports"
fi

echo "[2/4] H723 遥测（听 5 秒）"
OUT=$(timeout 14 python3 main.py ping --port "$PORT" --duration 5 2>&1)
if echo "$OUT" | grep -q "链路 OK"; then
    ok "链路通"
    echo "$OUT" | grep "遥测:" | tail -1 | sed 's/^/         /'
    if echo "$OUT" | grep -q "fault=0"; then
        ok "无故障码"
    else
        bad "有故障码 —— 看上面遥测里的 fault 值"
    fi
    if echo "$OUT" | grep -qE "flags=0x1[0-9A-F]"; then
        ok "俯仰编码器在线"
    else
        bad "编码器可能没在线（flags 应有 0x04）"
    fi
else
    bad "收不到 H723 —— 确认它已上电、等 4 秒后手掰云台是硬的"
    echo "$OUT" | tail -3 | sed 's/^/         /'
fi

echo "[3/4] 相机与靶纸（6 秒）"
OUT=$(timeout 25 python3 main.py check --duration 6 --no-gimbal 2>&1)
echo "$OUT" | grep "相机实际生效" | sed 's/^/         /'
if echo "$OUT" | grep -q "相机实际生效: MJPG"; then
    ok "相机格式是 MJPG"
else
    bad "相机不是 MJPG（帧率会差 6 倍）"
fi
RATE=$(echo "$OUT" | grep -oE "靶纸检出率 [0-9]+%" | tail -1 | grep -oE "[0-9]+")
if [ -n "${RATE:-}" ] && [ "${RATE:-0}" -ge 80 ]; then
    ok "靶纸检出率 ${RATE}%"
else
    bad "靶纸检出率 ${RATE:-0}% —— 相机对着靶纸了吗？清晰度够吗？"
    echo "         （跑 tools/focus_check.py 看清晰度）"
fi

echo "[4/4] 结论"
echo "  通过 $PASS 项，失败 $FAIL 项"
if [ "$FAIL" -eq 0 ]; then
    echo "  -> 可以开始标定："
    echo "     python3 tools/calib_boresight.py --port-gimbal $PORT --source 0"
else
    echo "  -> 按上面 [FAIL] 的提示查，常用诊断："
    echo "     tools/find_serial.py      串口找不到 H723"
    echo "     tools/check_downlink.py   指令发不下去"
    echo "     tools/focus_check.py      画面糊 / 镜头被挡"
fi

