#!/usr/bin/env python3
"""可视化调试：实时看靶纸检测、光斑检测、误差。

用法：
    python tools/view_detect.py --source 0                    # 有显示器：实时窗口
    python tools/view_detect.py --source 0 --snapshot out/dbg --interval 1.0
                                                              # 无显示器：存图

按键（窗口模式）：
    q/ESC 退出
    r     切换 target.require_red（用空白 A4 调试时关掉它）
    b     在 laser.b_minus_others 的 20/35/60 之间切换
    s     存一张当前标注帧
"""

import argparse
import os
import sys
import threading
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from eaim import protocol as proto  # noqa: E402
from eaim import viz  # noqa: E402
from eaim.app import AimApp, RunOptions  # noqa: E402
from eaim.config import find_default_config, load_config  # noqa: E402


class MjpegServer:
    """极简 MJPEG 服务器：电脑浏览器打开 http://<板子IP>:<端口>/ 就能看实时画面。

    为什么需要它：地瓜派没接显示器，VSCode SSH 也没有 X11，
    cv2.imshow() 会直接失败。MJPEG 是浏览器原生支持的流，
    不用装任何东西、也不用 CPU 去编 H.264。
    """

    def __init__(self, port: int, quality: int = 70):
        self.port = int(port)
        self.quality = int(quality)
        self.frame = None            # 最新一帧的 JPEG 字节
        self.lock = threading.Lock()
        self._httpd = None

    def put(self, img_bgr) -> None:
        ok, buf = cv2.imencode(".jpg", img_bgr,
                               [int(cv2.IMWRITE_JPEG_QUALITY), self.quality])
        if ok:
            with self.lock:
                self.frame = buf.tobytes()

    def start(self) -> None:
        import http.server
        srv = self

        class Handler(http.server.BaseHTTPRequestHandler):
            protocol_version = "HTTP/1.0"

            def log_message(self, *a):                      # 别刷屏
                pass

            def do_GET(self):
                if self.path in ("/", "/index.html"):
                    html = (b"<html><head><title>eaim view</title></head>"
                            b"<body style='margin:0;background:#111'>"
                            b"<img src='/stream' style='width:100%;display:block'>"
                            b"</body></html>")
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html; charset=utf-8")
                    self.send_header("Content-Length", str(len(html)))
                    self.end_headers()
                    self.wfile.write(html)
                    return
                if self.path.startswith("/stream"):
                    self.send_response(200)
                    self.send_header("Content-Type",
                                     "multipart/x-mixed-replace; boundary=frame")
                    self.end_headers()
                    while True:
                        with srv.lock:
                            data = srv.frame
                        if data is None:
                            time.sleep(0.05)
                            continue
                        try:
                            self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n"
                                             b"Content-Length: %d\r\n\r\n" % len(data))
                            self.wfile.write(data)
                            self.wfile.write(b"\r\n")
                        except Exception:               # 浏览器关掉了这个连接
                            break
                        time.sleep(0.03)                # 约 30fps 上限
                    return
                self.send_error(404)

        class Server(http.server.ThreadingHTTPServer):
            allow_reuse_address = True
            daemon_threads = True

        self._httpd = Server(("0.0.0.0", self.port), Handler)
        threading.Thread(target=self._httpd.serve_forever, daemon=True).start()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", default="0")
    ap.add_argument("--port-gimbal", default=None)
    ap.add_argument("--no-gimbal", action="store_true")
    ap.add_argument("--laser", action="store_true", help="打开激光（测光斑）")
    ap.add_argument("--snapshot", default=None, help="无显示器时存图目录")
    ap.add_argument("--interval", type=float, default=1.0, help="存图间隔 s")
    ap.add_argument("--duration", type=float, default=0.0, help="运行多久后退出，0=一直")
    ap.add_argument("--http", type=int, default=0,
                    help="开一个 MJPEG 网页流（例：--http 8080），"
                         "电脑浏览器打开 http://<板子IP>:8080/ 看实时画面")
    ap.add_argument("--undistort", default="auto", choices=("auto", "on", "off"),
                    help="是否按 calib.dist 做去畸变显示：auto=有畸变参数就开（默认）")
    args = ap.parse_args()

    cfg = load_config(find_default_config())
    if args.port_gimbal:
        cfg.link_gimbal.port = args.port_gimbal
    cfg.link_gimbal.enable = not args.no_gimbal
    cfg.link_car.enable = False
    cfg.app.show_window = (args.snapshot is None) and (args.http == 0)
    cfg.app.record_video = False

    app = AimApp(cfg, RunOptions(laser=args.laser, quiet=True))
    if not app.setup():
        return 1
    if args.snapshot:
        os.makedirs(args.snapshot, exist_ok=True)
    server = None
    # ---- 去畸变显示 ----
    # 画面"球形、矩形边不直"就是它。校正系数来自 calib.dist（棋盘标定或
    # tools/calib_distortion.py 估的），全 0 时这里自动是关的（空操作）。
    dist = tuple(getattr(cfg.calib, "dist", ()) or ())
    has_dist = any(abs(float(v)) > 1e-9 for v in dist)
    do_und = has_dist if args.undistort == "auto" else (args.undistort == "on")
    if do_und and not has_dist:
        print("提示：calib.dist 还是 0，去畸变没有系数可用（跑 tools/calib_intrinsic.py）")
        do_und = False
    if do_und:
        print("已开去畸变显示（k1=%.3f）" % float(dist[0]))
    if args.http:
        server = MjpegServer(args.http)
        server.start()
        print("=" * 64)
        print(" 实时画面已开：在电脑浏览器打开  http://192.168.43.10:%d/" % args.http)
        print(" （VSCode Remote-SSH 一般会自动转发端口；没反应就在端口面板手动加 %d）"
              % args.http)
        print(" 终端里按 Ctrl-C 退出")
        print("=" * 64)
    print("调试中：q/ESC 退出，r 切换 require_red，b 切蓝优势门限，s 存图")
    last_dump = 0.0
    t_end = time.monotonic() + args.duration if args.duration > 0 else None
    show = cfg.app.show_window
    while True:
        if t_end is not None and time.monotonic() >= t_end:
            break
        f, _ = app.camera.read()
        if f is None:
            time.sleep(0.002)
            continue
        img = f.image
        if do_und:
            img = cv2.undistort(img, app.cam_model.K,
                                np.asarray(app.cam_model.dist, dtype=np.float64))
        app._process_vision(img, f.t)
        if app.gimbal_link is not None and args.laser:
            app.gimbal_link.send(
                proto.MsgId.AIM,
                proto.pack_aim(app.aim.yaw_deg, app.aim.pitch_deg,
                               proto.AimFlags.LASER_ON, 0))
        lines = []
        if app.target is not None:
            lines.append("conf=%.2f ring=%.2f dist=%.2fm red=%d"
                         % (app.target.confidence, app.target.ring_score,
                            app.target.distance_m, app.target.red_px))
        else:
            lines.append("靶纸：未检出")
        lines.append("require_red=%s  b_minus=%d"
                     % (cfg.target.require_red, cfg.laser.b_minus_others))
        vis = viz.draw_overlay(img.copy(), target=app.target, spot=app.spot,
                               lines=lines, state="VIEW", fps=app.camera.fps_measured)
        if app.target is not None and app.target.rect is not None:
            rect = cv2.warpPerspective(img, app.target.rect.h_img2rect,
                                       app.target.rect.size)
            vis = viz.draw_rectified_inset(vis, rect, max_w=200)
        if show:
            cv2.imshow("view_detect", vis)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                break
            if key == ord("r"):
                cfg.target.require_red = not cfg.target.require_red
            if key == ord("b"):
                seq = [20, 35, 60]
                i = seq.index(cfg.laser.b_minus_others) if \
                    cfg.laser.b_minus_others in seq else 1
                cfg.laser.b_minus_others = seq[(i + 1) % len(seq)]
            if key == ord("s"):
                cv2.imwrite("view_%d.jpg" % int(time.time()), vis)
        elif args.snapshot and time.monotonic() - last_dump >= args.interval:
            last_dump = time.monotonic()
            p = os.path.join(args.snapshot, "view_%05d.jpg"
                             % int(time.monotonic() - app.t0))
            cv2.imwrite(p, vis)
            print("存图 %s  靶纸=%s 光斑=%s"
                  % (p, "有" if app.target else "无", app.spot.uv if app.spot else "无"))
        if server is not None:
            server.put(vis)
    if show:
        cv2.destroyAllWindows()
    app.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())

