#!/usr/bin/env bash
# ============================================================================
# 一键准备（在地瓜派上跑这一条就够）
#
#     cd /root/e_aim && bash deploy/prep.sh          # 快检，需要时才标定
#     cd /root/e_aim && bash deploy/prep.sh --calib   # 强制重新标定
#
# 它做四件事：
#   1) 串口 + H723 链路（几秒）
#   2) 相机 + 靶纸 + 光斑自检（几秒）
#   3) 标定：configs/boresight.yaml 不存在就自动标；存在就跳过（--calib 强制）
#   4) 告诉你下一步跑什么
#
# 为什么要这样分工：标定只要"相机/激光/靶纸的相对位置"没变就一直有效，
# 天天重标是浪费时间；而链路和检出率是"每次上电都可能变"的东西，必须查。
# ============================================================================
set -u
WS="$(cd "$(dirname "$0")/.." && pwd)"
cd "$WS" || exit 1
PORT="${PORT:-/dev/ttyS1}"
FORCE=0
[ "${1:-}" = "--calib" ] && FORCE=1
PASS=0
FAIL=0
ok()  { echo "  [OK]   $1"; PASS=$((PASS+1)); }
bad() { echo "  [FAIL] $1"; FAIL=$((FAIL+1)); }

echo "=============================================================="
echo " 云台准备（$WS）  端口 $PORT"
echo "=============================================================="

echo "[1/4] 串口"
if [ -e "$PORT" ]; then ok "$PORT 存在"; else bad "$PORT 不存在（跑 python3 main.py ports）"; fi

echo "[2/4] H723 链路"
OUT=$(timeout 16 python3 main.py ping --port "$PORT" --duration 4 2>&1)
if echo "$OUT" | grep -q "链路 OK"; then
    ok "链路通"
    echo "$OUT" | grep "遥测:" | tail -1 | sed 's/^/         /'
    echo "$OUT" | grep -q "fault=0" && ok "无故障码" || bad "有故障码（看上面 fault=）"
    echo "$OUT" | grep -qE "flags=0x1[0-9A-F]" && ok "俯仰编码器在线" \
        || bad "编码器可能没在线（flags 应含 0x04）"
else
    bad "收不到 H723 —— 确认已上电、等 4 秒后手掰云台是硬的"
    echo "$OUT" | tail -3 | sed 's/^/         /'
fi

echo "[3/4] 相机 + 靶纸 + 光斑"
OUT=$(timeout 40 python3 main.py check --duration 8 --no-gimbal 2>&1)
echo "$OUT" | grep "相机实际生效" | tail -1 | sed 's/^/         /'
T=$(echo "$OUT" | grep -oE "靶纸检出率 [0-9]+%" | tail -1 | grep -oE "[0-9]+")
S=$(echo "$OUT" | grep -oE "光斑检出率 [0-9]+%" | tail -1 | grep -oE "[0-9]+")
D=$(echo "$OUT" | grep -oE "平均距离 [0-9.]+m" | tail -1)
[ -n "${T:-}" ] && [ "${T:-0}" -ge 80 ] && ok "靶纸检出率 ${T}%（$D）" \
    || bad "靶纸检出率 ${T:-0}% —— 靶纸要在画面中间、别贴边（用 tools/view_detect.py --http 8080 看）"
[ -n "${S:-}" ] && [ "${S:-0}" -ge 80 ] && ok "光斑检出率 ${S}%" \
    || bad "光斑检出率 ${S:-0}% —— 激光没打到纸上？用 tools/probe_spot.py 看颜色"

echo "[4/4] 标定"
if [ "$FORCE" -eq 1 ] || [ ! -f configs/boresight.yaml ]; then
    echo "        没有标定文件（或 --calib）-> 现在标定，云台会小幅转 ±6°"
    OUT=$(timeout 120 python3 tools/calib_boresight.py --port-gimbal "$PORT" --source 0 2>&1)
    echo "$OUT" | grep -E "光轴点 =|du/dyaw|dv/dpitch|sign_yaw|✘" | sed 's/^/         /'
    if echo "$OUT" | grep -q "✘"; then
        bad "标定时云台没真正动（看上面 ✘）—— 结果不可信，先查 H723 状态"
    elif [ -f configs/boresight.yaml ]; then
        ok "标定完成，已写入 configs/boresight.yaml"
    else
        bad "标定没写出文件"
    fi
else
    ok "已有标定，跳过（改过相机/激光/云台安装位置就加 --calib 重标）"
    grep -E "sign_yaw|sign_pitch|boresight_uv" configs/boresight.yaml | sed 's/^/         /'
fi

echo "=============================================================="
echo " 通过 $PASS 项，失败 $FAIL 项"
if [ "$FAIL" -eq 0 ]; then
    echo " 下一步（二选一）："
    echo "   定点打靶（快、稳）: python3 tools/aim_step.py --port $PORT --source 0 --steps 4"
    echo "   连续闭环（带状态机）: python3 main.py run --duration 40 --budget 4 --keep-aim"
    echo "   想看实时画面: python3 tools/view_detect.py --source 0 --no-gimbal --http 8080"
else
    echo " 先按上面 [FAIL] 的提示修，再重跑本脚本。"
fi
echo "=============================================================="
