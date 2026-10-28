# =============================================================
# check_moving_base.py — 움직이는 베이스 단독 시험 (PLAN 8단계). 로봇·드론 없이 베이스 (Husky) 만
#
#   M1 궤적 계산 : 왕복 한 주기 끝점·최고 속도·가속도 (sim 없이)
#   M2 위치      : PhysX 가 계산한 차체 위치 = 궤적 (오차 < 1 mm), 높이 그대로
#   M3 속도      : PhysX 가 보고하는 차체 선속도 = 궤적 속도 (kinematic target 이 속도를 전한다 = 순간이동 아님)
#   M4 바퀴      : 바퀴 메시 회전각 (차체 기준, USD 렌더 변환) = 이동 거리 / 반지름
#
# 실행 (레포 최상위):
#   ~/isaacsim/python.sh isaacsim/scripts/check_moving_base.py --headless      # 시험 → PASS / FAIL, 리포트 isaacsim/reports/
#   ~/isaacsim/python.sh isaacsim/scripts/check_moving_base.py                 # GUI 로 보기 (판정도 같이)
#   옵션: --base-config <yaml> (기본 config/base_husky.yaml), --periods N (왕복 횟수, 기본 1)
# =============================================================

import argparse
import datetime
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

parser = argparse.ArgumentParser(description="움직이는 베이스 단독 시험 (PLAN 8단계)")
parser.add_argument("--headless", action="store_true")
parser.add_argument("--base-config", default=None)
parser.add_argument("--periods", type=float, default=1.0, help="왕복 횟수")
parser.add_argument("--report-dir", default=None)
args = parser.parse_args()

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": args.headless})

import isaacsim.core.experimental.utils.app as app_utils  # noqa: E402
import isaacsim.core.experimental.utils.stage as stage_utils  # noqa: E402
import numpy as np  # noqa: E402
from isaacsim.core.experimental.objects import DomeLight, GroundPlane  # noqa: E402
from isaacsim.core.experimental.prims import RigidPrim  # noqa: E402
from isaacsim.core.simulation_manager import SimulationManager  # noqa: E402

import moving_base as mb  # noqa: E402
import test_scene as ts  # noqa: E402

CRIT = {"pos_mm": 1.0, "vel_mm_s": 2.0, "wheel_deg": 0.5, "traj_tol": 1e-9}


class Report:
    def __init__(self):
        self.lines, self.results = [], []

    def log(self, msg=""):
        print(msg, flush=True)
        self.lines.append(msg)

    def section(self, no, title):
        self.log("")
        self.log(f"[{no}] {title}")

    def result(self, no, title, ok, why=""):
        self.results.append((no, title, "PASS" if ok else "FAIL"))
        self.log(f"  => {'PASS' if ok else 'FAIL'}" + (f" ({why})" if why else ""))


R = Report()


def quat_to_R(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def check_trajectory(traj):
    R.section("M1", "궤적 계산 (sim 없이)")
    ts_ = np.arange(0.0, traj.period + 1e-9, 1e-3)
    s, v, a = (np.array(x) for x in zip(*(traj.sample(t) for t in ts_)))
    R.log(f"  거리 {traj.distance} m, 최고 속도 {traj.v_peak:.4f} m/s, 가속 {traj.accel} m/s², 이동 {traj.t_move:.3f} s, "
          f"한 주기 {traj.period:.3f} s")
    ok = (abs(s.min()) < CRIT["traj_tol"] and abs(s.max() - traj.distance) < CRIT["traj_tol"]
          and abs(np.abs(v).max() - traj.v_peak) < 1e-6 and np.abs(a).max() <= traj.accel + 1e-9
          and abs(traj.sample(traj.period)[0]) < CRIT["traj_tol"])
    # 위치를 수치 미분한 속도가 v 와 맞는지 (식 오류 확인)
    dv = np.abs(np.gradient(s, ts_) - v)[2:-2].max()
    R.log(f"  s 범위 {s.min():.6f} ~ {s.max():.6f} m, |v| 최대 {np.abs(v).max():.6f}, |a| 최대 {np.abs(a).max():.4f}, "
          f"ds/dt − v 최대 {dv:.2e}")
    R.result("M1", "궤적 계산", ok and dv < 1e-3)


def build(cfg):
    stage_utils.create_new_stage()
    stage_utils.set_stage_up_axis("Z")
    stage_utils.set_stage_units(meters_per_unit=1.0, kilograms_per_unit=1.0)
    stage = stage_utils.get_current_stage(backend="usd")
    stage_utils.define_prim("/World", "Xform")
    if not SimulationManager.switch_physics_engine("physx"):
        raise RuntimeError("PhysX 로 전환하지 못했습니다")
    SimulationManager.setup_simulation(dt=ts.PHYSICS_DT)
    GroundPlane("/World/GroundPlane")
    if not args.headless:
        DomeLight("/World/DomeLight").set_intensities(1000)
    base = mb.KinematicBase(cfg)
    info = base.add(stage)
    R.log(f"  베이스 {info['path']}: kinematic 강체 {info['kinematic_bodies']}, 끈 관절 {info['joints_disabled']}")
    app_utils.play()
    simulation_app.update()
    return stage, base


def mat_np(m):
    """Gf 행렬 (행벡터 규약) → numpy 3x3 회전 (열벡터 규약)."""
    return np.array([[m[i][j] for j in range(3)] for i in range(3)]).T


def run(stage, base, sec):
    from pxr import Usd, UsdGeom

    rb = RigidPrim([base.body_path])
    body_prim = stage.GetPrimAtPath(base.body_path)
    wheel_prims = [stage.GetPrimAtPath(p) for p in base.wheel_visual_paths]
    base.start()
    rows = []
    t_end = sec
    while base.t < t_end - 1e-9:
        simulation_app.update()
        base.check()
        view = rb._physics_rigid_body_view
        tr = view.get_transforms().numpy().copy()          # (1, 7) xyz + xyzw
        vel = view.get_velocities().numpy().copy()
        p_cmd, v_cmd, _, ang_cmd = base.target(base.t)
        cache = UsdGeom.XformCache(Usd.TimeCode.Default())  # 렌더에 쓰이는 USD 변환
        Rb = mat_np(cache.GetLocalToWorldTransform(body_prim))
        wheel_ang = []
        for wp in wheel_prims:
            Rr = Rb.T @ mat_np(cache.GetLocalToWorldTransform(wp))   # 차체 기준 바퀴 메시 자세 (Y 축 회전)
            wheel_ang.append(math.atan2(Rr[0, 2], Rr[0, 0]))
        rows.append(dict(t=base.t, p=tr[0, :3], p_cmd=p_cmd, v=vel[0, :3], v_cmd=v_cmd, ang=np.array(wheel_ang),
                         ang_cmd=ang_cmd))
    base.stop()
    return rows


def wrap(x):
    return (x + math.pi) % (2 * math.pi) - math.pi


def main():
    cfg = mb.load_base_config(args.base_config)
    R.log(f"[check_moving_base] {datetime.datetime.now():%Y-%m-%d %H:%M:%S}, 베이스 설정 {cfg['path']}")
    base_traj = mb.LinearTrajectory(**{k: cfg["trajectory"][k] for k in ("distance", "max_speed", "accel", "dwell")})
    check_trajectory(base_traj)

    stage, base = build(cfg)
    sec = args.periods * base.traj.period
    R.log(f"  {sec:.2f} s (sim) 동안 궤적 따라 이동")
    rows = run(stage, base, sec)
    t = np.array([r["t"] for r in rows])

    R.section("M2", "차체 위치 = 궤적")
    err = np.array([np.linalg.norm(r["p"] - r["p_cmd"]) for r in rows]) * 1000
    z = np.array([r["p"][2] for r in rows])
    travel = np.array([np.dot(r["p"] - base.p0, base.dir_world) for r in rows])
    R.log(f"  샘플 {len(rows)} 개, 위치 오차 최대 {err.max():.3f} mm, 높이 {z.min():.5f} ~ {z.max():.5f} m (설정 {base.p0[2]}), "
          f"이동 범위 {travel.min():.4f} ~ {travel.max():.4f} m")
    R.result("M2", "위치", err.max() < CRIT["pos_mm"] and np.ptp(z) * 1000 < CRIT["pos_mm"])

    R.section("M3", "PhysX 차체 속도 = 궤적 속도")
    verr = np.array([np.linalg.norm(r["v"] - r["v_cmd"]) for r in rows]) * 1000
    vmax = max(np.linalg.norm(r["v"]) for r in rows)
    R.log(f"  속도 오차 최대 {verr.max():.3f} mm/s (PhysX 는 지난 스텝 평균 속도라 가속 구간에서 a·dt/2 ≈ "
          f"{base.traj.accel * ts.PHYSICS_DT / 2 * 1000:.2f} mm/s 차이 정상), PhysX 속도 최대 {vmax:.4f} m/s (궤적 {base.traj.v_peak:.4f})")
    R.result("M3", "속도", verr.max() < CRIT["vel_mm_s"] and abs(vmax - base.traj.v_peak) < CRIT["vel_mm_s"] / 1000)

    R.section("M4", "바퀴 회전 = 이동 거리 / 반지름")
    werr = max(np.abs(wrap(r["ang"] - r["ang_cmd"])).max() for r in rows)
    R.log(f"  바퀴 4 개 각도 오차 최대 {math.degrees(werr):.4f}°, 바퀴 회전 최대 {math.degrees(max(r['ang_cmd'] for r in rows)):.1f}°")
    R.result("M4", "바퀴", math.degrees(werr) < CRIT["wheel_deg"])

    R.log("")
    R.log("=== 요약 ===")
    for no, title, res in R.results:
        R.log(f"  {no} {title}: {res}")
    n_pass = sum(r[2] == "PASS" for r in R.results)
    R.log(f"  {n_pass}/{len(R.results)} PASS")
    out = args.report_dir or os.path.join(ts.REPORT_DIR, f"check_moving_base_{datetime.datetime.now():%Y%m%d_%H%M%S}")
    os.makedirs(out, exist_ok=True)
    with open(os.path.join(out, "report.txt"), "w", encoding="utf-8") as f:
        f.write("\n".join(R.lines) + "\n")
    np.savez(os.path.join(out, "log.npz"), t=t, p=np.array([r["p"] for r in rows]), p_cmd=np.array([r["p_cmd"] for r in rows]),
             v=np.array([r["v"] for r in rows]), v_cmd=np.array([r["v_cmd"] for r in rows]))
    R.log(f"  리포트: {out}")
    return n_pass == len(R.results)


if __name__ == "__main__":
    ok = False
    try:
        ok = main()
    finally:
        simulation_app.close(exit_code=0 if ok else 1)
