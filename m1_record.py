#!/usr/bin/env python3
"""M1 helper: rekam dan analisis berdiri di tanah dengan effort control (Lite3).

Rekam (butuh ROS 2 sudah di-source, sim sudah jalan):
    python3 m1_record.py record --out m1_run1 --duration 50
  Alur: tunggu >= 5 detik di mode posisi.
        Enter #1 -> segera jalankan `ros2 control switch_controllers ...`
        Enter #2 -> segera jalankan `ros2 param set /gravity_only_controller start_handover true`
  Jangan ubah apa pun lagi sampai --duration habis (>= 40 s dianjurkan).

Analisis:
    python3 m1_record.py analyze --out m1_run1 --kp 40 --kd 1.5
"""
import argparse, csv, math, os, sys, threading
import numpy as np

JOINTS = [f"{leg}_{j}_joint" for leg in ("FL", "FR", "HL", "HR")
          for j in ("HipX", "HipY", "Knee")]
Q_STAND = np.array([-0.02073, -0.67214, 1.32366, 0.01497, -0.67765, 1.33907,
                    -0.02465, -0.64953, 1.32289, 0.01714, -0.65116, 1.33529])


def quat_to_rpy(x, y, z, w):
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


# ---------------------------------------------------------------- record
def record(a):
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import JointState

    os.makedirs(a.out, exist_ok=True)

    class Rec(Node):
        def __init__(self):
            super().__init__("m1_recorder")
            self.t_last = 0.0
            self.t0 = None
            self.fo = open(os.path.join(a.out, "odom.csv"), "w", newline="")
            self.fj = open(os.path.join(a.out, "joints.csv"), "w", newline="")
            self.fm = open(os.path.join(a.out, "marks.csv"), "w", newline="")
            self.wo, self.wj, self.wm = (csv.writer(f) for f in (self.fo, self.fj, self.fm))
            self.wo.writerow(["t", "x", "y", "z", "roll", "pitch", "yaw"])
            self.wj.writerow(["t"] + [f"q_{n}" for n in JOINTS] + [f"dq_{n}" for n in JOINTS]
                             + [f"tau_{n}" for n in JOINTS])
            self.wm.writerow(["t", "label"])
            self.create_subscription(Odometry, a.odom_topic, self.on_odom, 50)
            self.create_subscription(JointState, "/joint_states", self.on_js, 50)

        def stamp(self, h):
            t = h.stamp.sec + h.stamp.nanosec * 1e-9
            if self.t0 is None:
                self.t0 = t
            return t - self.t0

        def on_odom(self, m):
            t = self.stamp(m.header)
            self.t_last = t
            p, o = m.pose.pose.position, m.pose.pose.orientation
            r, pi, ya = quat_to_rpy(o.x, o.y, o.z, o.w)
            self.wo.writerow([t, p.x, p.y, p.z, math.degrees(r), math.degrees(pi), math.degrees(ya)])

        def on_js(self, m):
            t = self.stamp(m.header)
            idx = {n: i for i, n in enumerate(m.name)}
            if not all(n in idx for n in JOINTS):
                return
            g = lambda arr: [arr[idx[n]] if idx[n] < len(arr) else float("nan") for n in JOINTS]
            self.wj.writerow([t] + g(m.position) + g(m.velocity) + g(m.effort))

        def mark(self, label):
            self.wm.writerow([self.t_last, label])
            self.fm.flush()
            print(f"[mark '{label}' pada t={self.t_last:.2f} s]")

    rclpy.init()
    node = Rec()
    labels = iter(["switch_effort", "start_handover"])

    def keys():
        for lab in labels:
            input(f"Tekan Enter untuk menandai '{lab}', lalu langsung jalankan perintahnya... ")
            node.mark(lab)

    threading.Thread(target=keys, daemon=True).start()
    end = None
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if end is None and node.t0 is not None:
                end = node.t_last + a.duration
            if end is not None and node.t_last >= end:
                break
    except KeyboardInterrupt:
        pass
    for f in (node.fo, node.fj, node.fm):
        f.close()
    print("selesai ->", a.out)
    rclpy.shutdown()


# --------------------------------------------------------------- analyze
def load(path):
    with open(path) as f:
        rows = list(csv.reader(f))
    head, data = rows[0], np.array(rows[1:], dtype=float)
    return {h: data[:, i] for i, h in enumerate(head)}


def analyze(a):
    od = load(os.path.join(a.out, "odom.csv"))
    marks = {}
    with open(os.path.join(a.out, "marks.csv")) as f:
        for r in list(csv.reader(f))[1:]:
            marks[r[1]] = float(r[0])
    if "start_handover" not in marks:
        sys.exit("marks.csv tidak punya 'start_handover'; ulangi rekaman dengan dua Enter.")
    th, tend = marks["start_handover"], od["t"][-1]
    pre = (od["t"] > marks.get("switch_effort", th) - 2.0) & (od["t"] <= marks.get("switch_effort", th))
    if pre.sum() == 0:
        pre = (od["t"] > th - 2.0) & (od["t"] <= th)
    z_ref = od["z"][pre].mean()
    win = od["t"] >= tend - a.window
    dur = tend - th
    z, t = od["z"][win], od["t"][win]
    slope = np.polyfit(t, z, 1)[0] * 1000
    sag_cm = (z_ref - z.mean()) * 100
    rmax = np.abs(od["roll"][win]).max()
    pmax = np.abs(od["pitch"][win] - od["pitch"][pre].mean()).max()
    pp_mm = (z.max() - z.min()) * 1000

    ok = {
        f"durasi setelah handover >= {a.min_duration:.0f} s": dur >= a.min_duration,
        "z turun < 1 cm dari acuan sebelum switch": sag_cm < 1.0,
        "|roll| < 2 deg": rmax < 2.0,
        "perubahan pitch < 2 deg": pmax < 2.0,
        "z diam (pk-pk < 3 mm, kemiringan < 0.2 mm/s)": pp_mm < 3.0 and abs(slope) < 0.2,
    }
    print(f"z acuan (sebelum switch)     : {z_ref:.4f} m")
    print(f"z rata-rata {a.window:.0f} s terakhir   : {z.mean():.4f} m  (sag {sag_cm:.2f} cm)")
    print(f"z pk-pk / kemiringan         : {pp_mm:.2f} mm / {slope:.3f} mm/s")
    print(f"|roll| maks / d(pitch) maks  : {rmax:.2f} / {pmax:.2f} deg")
    print(f"durasi setelah handover      : {dur:.1f} s")
    for k, v in ok.items():
        print(("LULUS  " if v else "GAGAL  ") + k)

    jp = os.path.join(a.out, "joints.csv")
    if os.path.exists(jp):
        js = load(jp)
        w = js["t"] >= tend - a.window
        q = np.array([js[f"q_{n}"][w].mean() for n in JOINTS])
        tau = np.array([js[f"tau_{n}"][w].mean() for n in JOINTS])
        e = q - Q_STAND
        print(f"\nPer sendi (rata-rata {a.window:.0f} s terakhir), Kp={a.kp}:")
        print(f"{'sendi':16s}{'e_ss rad':>10s}{'Kp*e Nm':>10s}{'tau aktual':>12s}")
        for n, ei, ti in zip(JOINTS, e, tau):
            print(f"{n:16s}{ei:10.3f}{-a.kp * ei:10.2f}{ti:12.2f}")
        print("Kp*e (tanda minus) = torsi feedforward yang kurang/lebih. Jika besar & konsisten,\n"
              "itu galat model G(q) - J^T F, bukan masalah gain.")


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    sub = p.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("record")
    r.add_argument("--out", default="m1_run")
    r.add_argument("--duration", type=float, default=50.0)
    r.add_argument("--odom-topic", dest="odom_topic", default="/lite3/odometry")
    n = sub.add_parser("analyze")
    n.add_argument("--out", default="m1_run")
    n.add_argument("--kp", type=float, default=40.0)
    n.add_argument("--kd", type=float, default=1.5)
    n.add_argument("--window", type=float, default=10.0)
    n.add_argument("--min-duration", dest="min_duration", type=float, default=10.0)
    a = p.parse_args()
    record(a) if a.cmd == "record" else analyze(a)
