# -*- coding: utf-8 -*-
"""pc_mask_test.py —— 验证"预览那张融合图"那条分支（_WANT_MASK=True）

    python k230_aim/tools/pc_mask_test.py

为什么要有它：pc_sim_main.py 跑的是**整机** main.py 的语义（_WANT_MASK=False，
不建融合图）；而 tools/vision_check.py 把 _WANT_MASK 打成 True —— 那条分支
（img.copy() -> 反向二值化 -> 闭运算 -> Canny -> 融合 -> 缓存给预览）在 PC 上
一直没人跑过。2026-10-16 现场就是在这条分支上卡了三轮（"看不到融合"、"来回闪"），
所以补一个最小回归：把 _WANT_MASK 打开，跑几帧 detect()，检查
  · 不抛异常；
  · _GY_MASK_OK 置位；
  · _LAST_MASK / _GY_MASK_CACHE[0] 真的拿到了那张图（预览才不会掉回老路）。
返回码：0 = OK，1 = 有问题。
"""
import io
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import pc_sim_main as S          # noqa: E402  （复用它的假模块/假场景）


def main():
    S.install_fake_modules()
    src = io.open(S.MAIN_PY, encoding="utf-8").read()
    src = src.replace('DISPLAY_MODE = "VIRT"', 'DISPLAY_MODE = "OFF"')
    # 这一行就是 vision_check 的 main() 干的事（模拟"调参工具要掩码"）
    src = src.replace("_WANT_MASK = [False]", "_WANT_MASK = [True]")
    assert "_WANT_MASK = [True]" in src, "main.py 里没找到 _WANT_MASK = [False]"
    # 只加载模块、不跑整机主循环：main.py 末尾那行裸的 main() 注掉
    src, _n = re.subn(r"(?m)^main\(\)\s*$", "# main()  # 本测试自己驱动",
                      src)
    assert _n == 1, "main.py 末尾没找到那行 main()（改了？）"

    g = {"__name__": "k230_main_under_test"}
    exec(compile(src, S.MAIN_PY, "exec"), g)      # 只加载，不跑 main()

    from media.sensor import Sensor, CAM_CHN_ID_0
    sensor = Sensor(id=2, width=1280, height=960, fps=90)
    sensor.reset()
    sensor.set_framesize(width=g["IMG_W"], height=g["IMG_H"], chn=CAM_CHN_ID_0)
    sensor.set_pixformat(Sensor.GRAYSCALE, chn=CAM_CHN_ID_0)
    sensor.run()

    det = g["PaperDetector"]()
    n_hit = 0
    for i in range(6):
        img = sensor.snapshot(chn=CAM_CHN_ID_0)
        res = det.detect(img)
        if res is not None:
            n_hit += 1

    ok = True
    if not g["_GY_MASK_OK"][0]:
        print("[FAIL] _GY_MASK_OK 没置位")
        ok = False
    if g["_LAST_MASK"] is None:
        print("[FAIL] _LAST_MASK 是 None（预览会掉回老路掩码）")
        ok = False
    if g["_GY_MASK_CACHE"][0] is None:
        print("[FAIL] _GY_MASK_CACHE 是 None（中间帧没图可复用）")
        ok = False
    print("6 帧里认出靶 %d 帧；_GY_MASK_OK=%s _LAST_MASK=%s cache=%s"
          % (n_hit, g["_GY_MASK_OK"][0],
             "有" if g["_LAST_MASK"] is not None else "无",
             "有" if g["_GY_MASK_CACHE"][0] is not None else "无"))
    print("结果：%s" % ("掩码预览分支 OK" if ok else "掩码预览分支有问题"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
