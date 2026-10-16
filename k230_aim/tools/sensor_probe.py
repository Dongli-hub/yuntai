# -*- coding: utf-8 -*-
"""sensor_probe.py —— 上板验证"关自动曝光(AE) + 手动曝光/增益"到底生不生效

为什么要有它（2026-10-15，翻完固件镜像 + 官方源码之后的结论）：
  · `sensor.set_auto_exposure / set_auto_gain / get_exposure_us` 在本固件里是
    **空壳**（`media/sensor.py` 里就是 `pass` + `@wrap` 打印
    "not support xxx now..."）—— 怎么调都不会有效果；
  · 真正生效的是 VICAP 的 dev attr：`Sensor.run()` 会执行
        kd_mpi_vicap_set_dev_attr(dev_id, self._dev_attr)
    把 `pipe_ctrl.bits.ae_enable` 下发给 ISP。新版固件(v1.8+)官方的
    `sensor.auto_exposure(False)` 改的就是这一位；
  · 曝光时间：v1.8 用 `sensor.exposure(us)`，底层是
    `kd_mpi_sensor_intg_time_set(fd, 秒)`；增益：`sensor.again(gain)`（v1.4 就有）。

这个脚本做三件事，全部打印成 [S1]~[S4] 行：
  S1 关 AE（两种写法都试，回读那一位）
  S2 连续 8 帧画面均值（AE 关掉后应该"钉住"、不再自己飘）
  S3 曝光时间扫描 2/5/10/20/33ms，每档测均值（看哪个接口真能写 + 亮度映射）
  S4 增益扫描（sensor.again），同样测均值
跑法：CanMV IDE 直接运行；把这四段发回来。脚本不写 flash，拔电重启即恢复。
"""
import time
import gc

IMG_W, IMG_H = 640, 480
EXPOSURES = (2000, 5000, 10000, 20000, 33000)
GAINS = (5, 20, 40)


def _safe(fn, *a, **kw):
    try:
        return fn(*a, **kw), None
    except Exception as e:
        return None, "%s: %s" % (type(e).__name__, e)


def mean_luma(img):
    """10x10 网格采样平均亮度（0~255）。"""
    s = 0
    k = 0
    for gy in range(10):
        y = int(IMG_H * (gy + 0.5) / 10)
        for gx in range(10):
            x = int(IMG_W * (gx + 0.5) / 10)
            p, err = _safe(img.get_pixel, x, y)
            if err:
                continue
            try:
                v = ((p[0] + p[1] + p[2]) / 3.0) if isinstance(p, (tuple, list)) else p
            except Exception:
                continue
            s += v
            k += 1
    return (s / k) if k else -1.0


def snap_mean(sensor):
    img = sensor.snapshot(chn=0)
    m = mean_luma(img)
    del img
    return m


def main():
    from media.sensor import Sensor, CAM_CHN_ID_0
    from media.media import MediaManager

    sensor = Sensor(id=2, width=1280, height=960, fps=90)
    sensor.reset()
    time.sleep_ms(100)
    sensor.set_framesize(width=IMG_W, height=IMG_H, chn=CAM_CHN_ID_0)
    sensor.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_0)

    # ---------------- S1: 关自动曝光 ----------------
    how = ""
    _r, _e = _safe(sensor.auto_exposure, False)       # v1.8+ 官方接口
    if _e is None:
        how = "sensor.auto_exposure(False)"
    else:
        _d = getattr(sensor, "_dev_attr", None)
        if _d is not None:
            _r2, _e2 = _safe(setattr, _d.pipe_ctrl.bits, "ae_enable", 0)
            if _e2 is None:
                how = "dev_attr.pipe_ctrl.bits.ae_enable=0"
            else:
                how = "两种都失败: auto_exposure=%s / bits=%s" % (_e, _e2)
        else:
            how = "没有 _dev_attr（auto_exposure=%s）" % _e
    rd = "?"
    try:
        rd = str(int(sensor._dev_attr.pipe_ctrl.bits.ae_enable))
    except Exception as e:
        rd = "读不到(%s)" % e
    print("[S1] 关AE方式 -> %s；run 前回读 ae_enable=%s" % (how, rd))

    MediaManager.init()
    _r3, _e3 = _safe(sensor.run)
    print("[S1] sensor.run() -> %s" % (_e3 if _e3 else "ok"))
    time.sleep_ms(800)

    # ---------------- S2: AE 关掉后画面稳不稳 ----------------
    vals = []
    for i in range(8):
        vals.append(snap_mean(sensor))
        time.sleep_ms(300)
    print("[S2] 连拍均值 %s" % (" ".join("%.1f" % v for v in vals),))
    if vals:
        print("[S2] 均值 min=%.1f max=%.1f 极差=%.1f（越小越说明 AE 真关了）"
              % (min(vals), max(vals), max(vals) - min(vals)))

    # ---------------- S3: 曝光时间扫描 ----------------
    print("[S3] 曝光时间扫描（每个值等 0.6s 再测均值）:")
    for us in EXPOSURES:
        tgt = ""
        _r, _e = _safe(sensor.exposure, us)            # v1.8+
        if _e is None:
            tgt = "sensor.exposure(%d)" % us
        else:
            _rf, _ef = _safe(__import__, "mpp")
            if _ef is None:
                _rf2, _ef2 = _safe(_rf.kd_mpi_sensor_intg_time_set,
                                   sensor.fd, us / 1000000.0)
                tgt = ("kd_mpi_sensor_intg_time_set(%d)" % us) if _ef2 is None \
                    else "失败(%s)" % _ef2
            else:
                tgt = "失败(无 mpp: %s)" % _ef
        time.sleep_ms(600)
        print("[S3] %6dus  %-34s 均=%.1f" % (us, tgt, snap_mean(sensor)))

    # ---------------- S4: 增益扫描 ----------------
    print("[S4] 模拟增益扫描 sensor.again(gain_obj):")
    for g in GAINS:
        _rg, _eg = _safe(__import__, "media.sensor")
        obj = None
        if _eg is None:
            _rf, _ef = _safe(_rg.k_sensor_gain)
            if _ef is None:
                obj = _rf
                try:
                    obj.gain[0] = g
                except Exception:
                    pass
        out = ""
        if obj is not None:
            _rr, _er = _safe(sensor.again, obj)
            out = "sensor.again(gain[0]=%d)" % g if _er is None else "失败(%s)" % _er
        else:
            out = "拿不到 k_sensor_gain(%s)" % _eg
        time.sleep_ms(600)
        print("[S4] gain=%2d  %-34s 均=%.1f" % (g, out, snap_mean(sensor)))

    print("[DONE] 把 [S1]~[S4] 发回；拔电重启即恢复自动曝光")
    try:
        sensor.stop()
    except Exception:
        pass
    try:
        MediaManager.deinit()
    except Exception:
        pass
    gc.collect()


if __name__ == "__main__":
    main()
