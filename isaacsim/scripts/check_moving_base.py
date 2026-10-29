# =============================================================
# check_moving_base.py — 움직이는 베이스 단독 시험 (PLAN 8단계). 로봇·드론 없이 베이스만. 설정 kind 로 갈림
#
# kind linear (Husky, config/base_husky.yaml):
#   M1 궤적 계산 : 왕복 한 주기 끝점·최고 속도·가속도 (sim 없이)
#   M2 위치      : PhysX 가 계산한 차체 위치 = 궤적 (오차 < 1 mm), 높이 그대로
#   M3 속도      : PhysX 가 보고하는 차체 선속도 = 궤적 속도 (kinematic target 이 속도를 전한다 = 순간이동 아님)
#   M4 바퀴      : 바퀴 메시 회전각 (차체 기준, USD 렌더 변환) = 이동 거리 / 반지름
# kind stewart (config/base_stewart.yaml, --base-motion small|mid|big|lab):
#   S1 궤적 계산 : 시작 위치·속도·가속도 0 (램프), 미분 = 식, 진폭 안 (sim 없이)
#   S2 윗판 자세 : PhysX 윗판 위치·회전 = 궤적 (< 1 mm, 0.05°)
#   S3 속도      : PhysX 선속도 (질량 중심) ·각속도 = 궤적 (지난 스텝 평균이라 a·dt/2 만큼 차이 정상)
#   S4 다리      : 길이가 legs 범위 안 (lab 은 기록만), 로드 끝 = 윗판 부착점·실린더 축 = 다리 방향 (USD 렌더 변환)
#
# 실행 (레포 최상위):
#   ~/isaacsim/python.sh isaacsim/scripts/check_moving_base.py --headless      # 시험 → PASS / FAIL, 리포트 isaacsim/reports/
#   ~/isaacsim/python.sh isaacsim/scripts/check_moving_base.py                 # GUI 로 보기 (판정도 같이)
#   ~/isaacsim/python.sh isaacsim/scripts/check_moving_base.py --headless --base-config isaacsim/config/base_stewart.yaml --base-motion big
#   옵션: --base-config <yaml> (기본 config/base_husky.yaml), --periods N (Husky 왕복 횟수, 기본 1),
#         --base-motion 프리셋 (stewart), --seed N (stewart ω·위상), --seconds S (stewart 시험 길이, 기본 40)
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
parser.add_argument("--base-motion", default=None, help="stewart 움직임 프리셋 (small·mid·big·lab)")
parser.add_argument("--seed", type=int, default=0, help="stewart ω·위상 seed")
parser.add_argument("--seconds", type=float, default=40.0, help="stewart 시험 길이 [sim s] (램프 5 s 포함)")
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

CRIT = {"pos_mm": 1.0, "vel_mm_s": 2.0, "wheel_deg": 0.5, "traj_tol": 1e-9,
        # stewart: 속도는 big 의 최대 가속도 (ω × 1.3 이면 약 0.4 m/s²) · dt/2 ≈ 1.7 mm/s 보다 위
        "rot_deg": 0.05, "svel_mm_s": 3.0, "angvel_deg_s": 0.1, "leg_attach_mm": 1.0, "leg_axis_deg": 0.1}


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
    R.log(f"[check_moving_base] {datetime.datetime.now():%Y-%m-%d %H:%M:%S}, 베이스 설정 {cfg['path']} (kind {cfg['kind']})")
    if cfg["kind"] == "stewart":
        return main_stewart(cfg)
    if args.base_motion not in (None, "static"):
        raise ValueError("--base-motion 은 stewart 베이스에서만 (Husky 궤적은 설정 trajectory 하나)")
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


# ── Stewart (kind stewart) ──
def angle_deg(Ra, Rb):
    return math.degrees(math.acos(np.clip((np.trace(Ra.T @ Rb) - 1) / 2, -1, 1)))


def quat_to_R(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def check_sine(base, sec):
    R.section("S1", "궤적 계산 (sim 없이)")
    tr = base.traj
    t = np.arange(0.0, sec, 1e-3)
    Q = np.array([tr.sample(x) for x in t])
    q, dq, ddq = Q[:, 0], Q[:, 1], Q[:, 2]
    e1 = np.abs(np.gradient(q, t, axis=0) - dq)[5:-5].max()
    e2 = np.abs(np.gradient(dq, t, axis=0) - ddq)[5:-5].max()
    start = max(np.abs(q[0]).max(), np.abs(dq[0]).max(), np.abs(ddq[0]).max())
    over = (np.abs(q) - tr.amp[None]).max()
    p = tr.params()
    R.log(f"  프리셋 {tr.preset} (수집 {'예' if tr.collect else '아니오'}), seed {tr.seed}, 램프 {tr.ramp_time} s")
    R.log(f"  진폭 {p['amplitude']} (m, m, m, rad, rad, rad)")
    R.log(f"  ω {p['omega']} rad/s, 위상 {p['phase']}")
    R.log(f"  시작 위치·속도·가속도 최대 {start:.1e}, dq/dt − q̇ 최대 {e1:.1e}, dq̇/dt − q̈ 최대 {e2:.1e}, 진폭 넘음 {max(over, 0):.1e}")
    R.result("S1", "궤적 계산", start < 1e-12 and e1 < 1e-4 and e2 < 1e-3 and over <= 1e-12)


def build_stewart(cfg):
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
    if args.base_motion is None:
        raise ValueError("stewart 베이스는 --base-motion (small·mid·big·lab) 이 필요")
    base = mb.make_base(cfg, motion=args.base_motion, seed=args.seed)
    info = base.add(stage)
    R.log(f"  베이스 {info['path']}: 윗판 {info['body']}")
    app_utils.play()
    simulation_app.update()
    return stage, base


def run_stewart(stage, base, sec):
    from pxr import Usd, UsdGeom

    rb = RigidPrim([base.body_path])
    rods = [stage.GetPrimAtPath(f"{base.path}/legs/leg_{i + 1}/rod") for i in range(6)]
    cyls = [stage.GetPrimAtPath(f"{base.path}/legs/leg_{i + 1}/cylinder") for i in range(6)]
    B_w = (base.R0 @ base.B.T).T + base.p0
    base.start()
    rows = []
    com = None
    while base.t < sec - 1e-9:
        simulation_app.update()
        base.check()
        view = rb._physics_rigid_body_view
        if com is None:      # PhysX 선속도 = 질량 중심 속도. 윗판 질량 중심은 회전 중심 (윗면) 보다 판 두께 절반 아래
            com = view.get_coms().numpy()[0][:3].astype(float)
            R.log(f"  윗판 질량 중심 (윗판 좌표) {np.round(com, 4).tolist()} m → 기대 선속도 = 회전 중심 속도 + ω × r_com")
        tr = view.get_transforms().numpy()[0].astype(float)
        vel = view.get_velocities().numpy()[0].astype(float)
        p_cmd, v_cmd, _, R_cmd, w_cmd = base.target(base.t)
        v_cmd = v_cmd + np.cross(w_cmd, R_cmd @ com)
        p_m, R_m = tr[:3], quat_to_R(tr[[6, 3, 4, 5]])
        top_w = (R_m @ base.P.T).T + p_m                         # PhysX 윗판 자세로 본 윗판 부착점
        cache = UsdGeom.XformCache(Usd.TimeCode.Default())
        attach, axis, L = [], [], []
        for i in range(6):
            Mr = cache.GetLocalToWorldTransform(rods[i])
            Mc = cache.GetLocalToWorldTransform(cyls[i])
            rod_p = np.array(Mr.ExtractTranslation())
            cyl_p = np.array(Mc.ExtractTranslation())
            cyl_z = np.array([Mc[2][0], Mc[2][1], Mc[2][2]])
            cyl_z /= np.linalg.norm(cyl_z)
            leg = top_w[i] - B_w[i]
            attach.append(max(np.linalg.norm(rod_p - top_w[i]), np.linalg.norm(cyl_p - B_w[i])))
            axis.append(math.degrees(math.acos(np.clip(cyl_z @ leg / np.linalg.norm(leg), -1, 1))))
            L.append(np.linalg.norm(leg))
        rows.append(dict(t=base.t, pos=np.linalg.norm(p_m - p_cmd), rot=angle_deg(R_m, R_cmd), vel=np.linalg.norm(vel[:3] - v_cmd),
                         angvel=np.linalg.norm(vel[3:6] - w_cmd), attach=max(attach), axis=max(axis), L=np.array(L),
                         z=p_m[2], vnorm=np.linalg.norm(vel[:3]), wnorm=np.linalg.norm(vel[3:6])))
    base.stop()
    return rows


def main_stewart(cfg):
    stage, base = build_stewart(cfg)
    sec = float(args.seconds)
    check_sine(base, sec)
    R.log(f"  {sec:.1f} s (sim) 동안 6 축 사인파")
    rows = run_stewart(stage, base, sec)
    a = {k: np.array([r[k] for r in rows]) for k in ("t", "pos", "rot", "vel", "angvel", "attach", "axis", "z", "vnorm", "wnorm")}
    L = np.array([r["L"] for r in rows])
    R.section("S2", "윗판 자세 = 궤적 (PhysX)")
    R.log(f"  샘플 {len(rows)} 개, 위치 오차 최대 {a['pos'].max() * 1000:.3f} mm, 회전 오차 최대 {a['rot'].max():.4f}°, "
          f"윗판 높이 {a['z'].min():.3f} ~ {a['z'].max():.3f} m")
    R.result("S2", "윗판 자세", a["pos"].max() * 1000 < CRIT["pos_mm"] and a["rot"].max() < CRIT["rot_deg"])
    R.section("S3", "PhysX 선속도·각속도 = 궤적")
    R.log(f"  선속도 오차 최대 {a['vel'].max() * 1000:.3f} mm/s (최고 속도 {a['vnorm'].max():.3f} m/s), "
          f"각속도 오차 최대 {math.degrees(a['angvel'].max()):.4f} °/s (최고 {math.degrees(a['wnorm'].max()):.2f} °/s)")
    R.result("S3", "속도", a["vel"].max() * 1000 < CRIT["svel_mm_s"] and math.degrees(a["angvel"].max()) < CRIT["angvel_deg_s"])
    R.section("S4", "다리 (길이·붙어 있음)")
    inside = bool((L.min() >= base.lmin) and (L.max() <= base.lmax))
    R.log(f"  다리 길이 {L.min():.3f} ~ {L.max():.3f} m (범위 {base.lmin} ~ {base.lmax}, 넘은 최대 {base.leg_over * 1000:.1f} mm), "
          f"로드·실린더 끝 ↔ 부착점 최대 {a['attach'].max() * 1000:.3f} mm, 실린더 축 ↔ 다리 방향 최대 {a['axis'].max():.4f}°")
    if not base.traj.collect and not inside:
        R.log(f"  ({base.traj.preset} 는 수집 안 하는 프리셋 → 다리 범위 밖은 경고만)")
    R.result("S4", "다리", (inside or not base.traj.collect) and a["attach"].max() * 1000 < CRIT["leg_attach_mm"]
             and a["axis"].max() < CRIT["leg_axis_deg"])
    R.log("")
    R.log("=== 요약 ===")
    for no, title, res in R.results:
        R.log(f"  {no} {title}: {res}")
    n_pass = sum(r[2] == "PASS" for r in R.results)
    R.log(f"  {n_pass}/{len(R.results)} PASS")
    out = args.report_dir or os.path.join(ts.REPORT_DIR, f"check_moving_base_stewart_{base.traj.preset}_{datetime.datetime.now():%Y%m%d_%H%M%S}")
    os.makedirs(out, exist_ok=True)
    with open(os.path.join(out, "report.txt"), "w", encoding="utf-8") as f:
        f.write("\n".join(R.lines) + "\n")
    np.savez(os.path.join(out, "log.npz"), L=L, **a)
    R.log(f"  리포트: {out}")
    return n_pass == len(R.results)


if __name__ == "__main__":
    ok = False
    try:
        ok = main()
    finally:
        simulation_app.close(exit_code=0 if ok else 1)
