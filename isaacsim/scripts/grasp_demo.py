#!/usr/bin/env python3
# =============================================================
# grasp_demo.py  (docs/PLAN.md 2-4, 2-5)
#
# 스크립트 파지 데모 (텔레옵 없이) + 자동 판정. 씬 = drone_scene.build_scene (ROS 없음)
#   시퀀스 (config/grasp_demo.yaml): 관절 보간 → IK 로 드론 아래 → 수직 상승 → 그리퍼 닫기 → (kill_delay 동안 잡고 있음)
#     → 드론 모터 정지 → 버팀 (끝)
#   드론 제어기 --flight geometric|px4 (PLAN 2단계 2번): px4 면 드론이 테이블 위에서 PX4 로 이륙해 드론 위치에 호버한 뒤 시작,
#     모터 정지 = PX4 kill. 잡은 뒤 kill 까지(S3_hold) 떨림 지표 (TCP 기준 드론 위치·자세 흔들림, 손가락 힘, 모터 ω) 를 리포트에
#     실물: 잡은 뒤 모터를 멈추고 사람이 떼어 감 → 내려놓기는 하지 않음 (착륙 받침대는 팔이 막혀 2026-10-01 제거,
#     내려놓기·충돌 없는 경로는 나중에 MoveIt 등 경로 계획으로)
#   팔 관절 목표는 control_hz 로 보냄 (텔레옵처럼 zero-order hold, 한 주기 변화 ≤ max_dq_per_tick)
#   잡는 높이·닿는 각도 예측: gripper_geom.plan_grasp_from_below (드론 충돌 형상 + 손가락 기구학)
#   판정: grasp_judge.GraspJudge (approach → grasped → held = 성공 / failed + 사유)
#   케이스 (--case): success, offset_y·offset_x(잡는 위치를 닫는 방향 10 mm·몸체 길이 방향 5 mm 어긋나게, 텔레옵처럼),
#     empty(드론 없이 닫기), miss(잡는 높이 4 cm 낮게), drop(모터 정지 순간부터 그리퍼 힘 0.05 Nm), all(넷 다)
#
# 실행:
#   GUI:  ~/isaacsim/python.sh ~/data_collection_ur5_gripper/isaacsim/scripts/grasp_demo.py [--case success] [--prop-spin on]
#         (실제 시간 속도, 'Wrist camera'·'Third view camera' 창을 같이 띄움. 끝나면 --hold 면 창을 닫을 때까지 유지)
#   어긋남 직접 지정: --tcp-offset DX DY DZ [m] (예: --case success --tcp-offset 0 0.015 0)
#   점검: ~/isaacsim/python.sh ~/data_collection_ur5_gripper/isaacsim/scripts/grasp_demo.py --headless --case all
#   결과: isaacsim/reports/grasp_demo_<시각>/ (report.txt, result_<케이스>.json, timeline_<케이스>.csv)
# =============================================================

import argparse
import datetime
import json
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import test_scene as ts  # noqa: E402

CASES = ("success", "offset_y", "offset_x", "empty", "miss", "drop")
EXPECT = {"success": ("held", None), "offset_y": ("held", None), "offset_x": ("held", None), "empty": ("failed", "no_grasp_empty"), "miss": ("failed", "no_grasp"),
          "drop": ("failed", ("slip", "dropped"))}
DEFAULT_DEMO_CONFIG = os.path.join(ts.CONFIG_DIR, "grasp_demo.yaml")

parser = argparse.ArgumentParser(description="스크립트 파지 데모 + 자동 판정 (PLAN 2-4)")
parser.add_argument("--headless", action="store_true")
parser.add_argument("--case", default="success", choices=CASES + ("all",))
parser.add_argument("--demo-config", default=DEFAULT_DEMO_CONFIG)
parser.add_argument("--robot-config", default=None)
parser.add_argument("--scene-config", default=None)
parser.add_argument("--drone-config", default=None)
parser.add_argument("--drone-pos", type=float, nargs=3, default=None, metavar=("X", "Y", "Z"))
parser.add_argument("--mode", default="static", choices=("static", "hover"))
parser.add_argument("--seed", type=int, default=0)
parser.add_argument("--prop-spin", choices=("on", "off"), default="off")
parser.add_argument("--init-pose", type=float, nargs=6, default=None, metavar="Q")
parser.add_argument("--tcp-offset", type=float, nargs=3, default=None, metavar=("DX", "DY", "DZ"),
                    help="잡는 위치를 world 기준으로 어긋나게 [m] (케이스 설정의 tcp_offset 대신). 드론 yaw 0 이면 y = 손가락이 닫히는 방향, "
                         "x = 몸체 길이 방향")
parser.add_argument("--flight", default=None, choices=("geometric", "px4"), help="드론 제어기 (기본: 드론 설정 flight.backend)")
parser.add_argument("--position-source", default=None, choices=("mocap", "flow", "gps"), help="px4 위치 정보 (기본: px4_sitl.yaml)")
parser.add_argument("--kill-delay", type=float, default=None, help="잡은 뒤 드론 모터 정지까지 [s] (기본: 설정 kill_delay)")
parser.add_argument("--px4-param", action="append", default=[], metavar="NAME=VALUE", help="비교용: PX4 파라미터 덮어쓰기")
parser.add_argument("--hold", action="store_true", help="GUI: 끝난 뒤 창을 닫을 때까지 유지")
parser.add_argument("--report-dir", default=ts.REPORT_DIR)
args, _ = parser.parse_known_args()

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": args.headless})

import numpy as np  # noqa: E402
import yaml  # noqa: E402
from scipy.spatial.transform import Rotation, Slerp  # noqa: E402

import collision_geom as cg  # noqa: E402
import drone as dr  # noqa: E402
import drone_scene as ds  # noqa: E402
import gripper_geom as gg  # noqa: E402
from arm_ik import DiffIK  # noqa: E402
from grasp_judge import GraspJudge  # noqa: E402

REALTIME = not args.headless
KILL_DELAY = None                           # main() 에서 설정 (설정 kill_delay 또는 --kill-delay)
CAMERA_WINDOWS = []                         # GUI 카메라 뷰포트 창 (케이스마다 새 씬이라 다시 띄움)


class Abort(Exception):
    pass


def load_demo_config(path):
    with open(path, encoding="utf-8") as f:
        c = yaml.safe_load(f)
    for k in ("control_hz", "max_dq_per_tick", "ik_damping", "seed_pose", "joint_speed", "tool_x_in_drone", "pregrasp_below",
              "approach_speed", "approach_track_until", "move_speed", "reach", "grip_close_timeout", "kill_delay", "hold_time", "end_wait", "judge", "cases"):
        if k not in c:
            raise KeyError(f"{path}: '{k}' 항목이 없습니다")
    return c


class Demo:
    """씬 + IK + 접촉 센서 + 판정 + 제어 주기. 시퀀스 함수가 쓴다."""

    def __init__(self, case, dc):
        self.case, self.dc = case, dc
        cc = dict(dc["cases"].get(case, {}))
        if args.tcp_offset is not None:
            cc["tcp_offset"] = list(args.tcp_offset)
        self.cc = cc
        scene_cfg = ds.load_scene_config(args.scene_config or ds.DEFAULT_SCENE_CONFIG)
        self.nominal = np.asarray(args.drone_pos if args.drone_pos is not None else scene_cfg["drone_pos"], dtype=float)
        spawn = self.nominal + np.asarray(cc.get("drone_offset", [0, 0, 0]), dtype=float)

        def setup(s):
            from pxr import PhysxSchema, UsdPhysics

            # 접촉 보고 (재생 전): 로봇 링크 전부
            links = [p for p in _robot_links(s.stage)]
            for p in links:
                PhysxSchema.PhysxContactReportAPI.Apply(s.stage.GetPrimAtPath(p)).CreateThresholdAttr().Set(0.0)
            if "friction" in cc:   # 손가락·물체 재질 마찰을 바꿈 (합치는 방식 min)
                for mp in (ts.FINGER_MATERIAL_PATH, ts.OBJECT_MATERIAL_PATH):
                    api = UsdPhysics.MaterialAPI(s.stage.GetPrimAtPath(mp))
                    api.GetStaticFrictionAttr().Set(float(cc["friction"]))
                    api.GetDynamicFrictionAttr().Set(float(cc["friction"]))
            return links

        drone_config = args.drone_config
        if args.px4_param:
            drone_config = _drone_config_with_params(args.drone_config, args.px4_param)
        self.s = s = ds.build_scene(args.robot_config, args.scene_config, drone_config, spawn, args.mode, args.seed,
                                    args.prop_spin == "on", "fixed", args.init_pose, extra_setup=setup, flight=args.flight,
                                    position_source=args.position_source, report_dir=args.report_dir, realtime=REALTIME)
        from isaacsim.core.experimental.prims import RigidPrim

        self.link_paths = s.extra_setup
        self.link_names = [p.split("/")[-1] for p in self.link_paths]
        drone_bodies = s.drone_info["bodies"]
        self.robot_sensor = RigidPrim(self.link_paths, contact_filter_paths=drone_bodies, max_contact_count=2048)
        if not args.headless:
            global CAMERA_WINDOWS
            CAMERA_WINDOWS = ds.open_camera_windows(s, CAMERA_WINDOWS)
        self.ik = DiffIK(s.robot, ts.find_link_path(s.stage, ts.PAYLOAD_LINK), ts.PAYLOAD_LINK, s.drive.arm_i,
                         (0.0, 0.0, gg.TCP_Z), damping=dc["ik_damping"])
        self.plan = self._plan_grasp()
        self.judge = GraspJudge(dc["judge"], q_pred=self.plan["q"])
        self.dt_ctrl = 1.0 / dc["control_hz"]
        self.next_tick = 0.0
        self.q_target = s.init_pose.copy()
        self.rows = []
        self.phase = "start"
        self.f_other_max = {}
        self.link_force_max = {}            # 단계 → 링크 → 드론과 접촉력 최대 (> 1 N)
        self.p_close0 = None                # 닫기 시작할 때 드론 위치 (잡는 동안 밀린 거리 기준)
        self.push_max_mm = 0.0
        self.wall0, self.sim0 = time.monotonic(), s.flight.t

    # ── 기하 ──
    def _plan_grasp(self):
        s, dc = self.s, self.dc
        body = s.drone_info["body_path"]
        # body 링크 자신의 좌표계 (재생 직후 이미 몇 mm 떨어진 상태라 최상위 prim 기준이면 높이가 어긋남, 2026-10-01)
        R, t = dr.drone_frame(s.stage, body)
        prims = [p for p in dr.collision_prims(s.stage, s.drone_info["path"]) if dr._owner(p) == body]
        meshes = dr.collision_points(s.stage, prims, R, t)
        pts = np.vstack([m.vertices for m in meshes] + [cg.trimesh.sample.sample_surface(m, 200000)[0] for m in meshes
                                                         if m.faces is not None and len(m.faces)])
        x_g = np.asarray(dc["tool_x_in_drone"], dtype=float)
        z_g = np.array([0.0, 0.0, 1.0])
        y_g = np.cross(z_g, x_g)
        self.R_dg = np.column_stack([x_g, y_g, z_g])                # 드론 좌표계에서 그리퍼 정렬 축
        shapes, _ = cg.collect(s.stage)
        plan = gg.plan_grasp_from_below(shapes, pts @ self.R_dg)
        plan["tcp_in_drone"] = (self.R_dg @ np.array([0.0, plan["center_y"], plan["h"]])).tolist()
        return plan

    def drone_pose(self, nominal=False):
        if nominal:
            return self.nominal.copy(), np.eye(3)
        p, q = self.s.flight._body_pose()
        return p, cg.quat_to_R(q)

    def grasp_target(self):
        """잡는 자세 TCP (world). empty 는 기본 위치를 향함."""
        p, R = self.drone_pose(nominal=self.case == "empty")
        off = np.asarray(self.cc.get("tcp_offset", [0, 0, 0]), dtype=float)
        return p + R @ np.asarray(self.plan["tcp_in_drone"]) + off, R @ self.R_dg

    # ── 한 업데이트 ──
    def update(self, control=None):
        s = self.s
        ds.step(s)
        o = self.observe()
        self.judge.update(o)
        if s.flight.t + 1e-9 >= self.next_tick:
            self.next_tick += self.dt_ctrl
            if control is not None:
                control()
            s.drive.set_arm_targets(self.q_target)
        rel_p = o["tcp_R"].T @ (o["drone_p"] - o["tcp_p"])                      # TCP 좌표계 드론 위치
        rel_rv = Rotation.from_matrix(o["tcp_R"].T @ o["drone_R"]).as_rotvec()
        gnd = getattr(s.flight, "ground", None)
        self.rows.append((o["t"], self.phase, self.judge.state, *o["tcp_p"], *o["drone_p"], math.degrees(o["grip_q"]),
                          o["f_right"], o["f_left"], o["f_other"], int(o["armed"]), *rel_p, *np.degrees(rel_rv),
                          *s.flight.omega, float("nan") if gnd is None else gnd,
                          *np.degrees(getattr(s.flight, "physx_w", np.full(3, np.nan))),
                          *np.degrees(getattr(s.flight, "sensor_w", np.full(3, np.nan)))))
        if self.phase == "S3_close":
            if self.p_close0 is None:
                self.p_close0 = o["drone_p"].copy()
            self.push_max_mm = max(self.push_max_mm, float(np.linalg.norm(o["drone_p"] - self.p_close0)) * 1000)
        if o["f_other"] > 1.0:
            self.f_other_max[self.phase] = max(self.f_other_max.get(self.phase, 0.0), o["f_other"])
        if REALTIME:
            ahead = (s.flight.t - self.sim0) - (time.monotonic() - self.wall0)
            if ahead > 0:
                time.sleep(ahead)
        if not simulation_app.is_running():
            raise Abort("창이 닫힘")

    def observe(self):
        s = self.s
        q = s.robot.get_dof_positions().numpy()[0]
        M = np.linalg.norm(self.robot_sensor.get_contact_force_matrix(dt=ts.PHYSICS_DT).numpy(), axis=2).sum(axis=1)
        f = dict(zip(self.link_names, M))
        for k, v in f.items():
            if v > 1.0:
                ph = self.link_force_max.setdefault(self.phase, {})
                ph[k] = max(ph.get(k, 0.0), float(v))
        right = f["rh_p12_rn_r1"] + f["rh_p12_rn_r2"]
        left = f["rh_p12_rn_l1"] + f["rh_p12_rn_l2"]
        other = float(sum(v for k, v in f.items() if k not in ("rh_p12_rn_r1", "rh_p12_rn_r2", "rh_p12_rn_l1", "rh_p12_rn_l2")))
        dp, dR = self.drone_pose()
        tp, tR = self.ik.tcp_pose()
        return dict(t=s.flight.t, grip_q=float(q[s.drive.grip_i]), grip_closing=s.drive.gripper.goal > 0.5 * ts.GRIPPER_UPPER,
                    armed=s.flight.armed, f_right=float(right), f_left=float(left), f_other=other,
                    drone_p=dp, drone_R=dR, tcp_p=tp, tcp_R=tR)

    def wait(self, sec, until=None):
        t_end = self.s.flight.t + sec
        while self.s.flight.t < t_end:
            self.update()
            if self.judge.state == "failed":
                raise Abort(self.judge.reason)
            if until is not None and until():
                return True
        return False

    # ── 동작 ──
    def joint_move(self, q_goal, speed):
        q0 = self.s.robot.get_dof_positions().numpy()[0][self.s.drive.arm_i].copy()
        q_goal = np.asarray(q_goal, dtype=float)
        T = max(float(np.abs(q_goal - q0).max()) / speed, self.dt_ctrl)
        t0 = self.s.flight.t

        def ctrl():
            u = min((self.s.flight.t - t0) / T, 1.0)
            u = u * u * (3 - 2 * u)
            self.q_target = q0 + u * (q_goal - q0)

        while self.s.flight.t - t0 < T + 0.5:
            self.update(ctrl)
            if self.judge.state == "failed":
                raise Abort(self.judge.reason)
        self.ik.reset(self.q_target)

    def cart_move(self, goal_fn, speed, tol=None):
        """TCP 를 goal_fn() 의 (위치, 자세) 로 직선 이동 (속도 speed, 자세는 slerp). goal_fn 은 매 주기 다시 부름 (드론 따라감)."""
        tol = tol or self.dc["reach"]
        p0, R0 = self.ik.tcp_pose()
        g0, _ = goal_fn()
        T = max(float(np.linalg.norm(g0 - p0)) / speed, 0.2)
        t0 = self.s.flight.t
        st = {"ep": 1.0, "er": 1.0}

        def ctrl():
            g, Rg = goal_fn()
            u = min((self.s.flight.t - t0) / T, 1.0)
            u = u * u * (3 - 2 * u)
            p_t = p0 + u * (g - p0)
            R_t = Slerp([0, 1], Rotation.from_matrix([R0, Rg]))(u).as_matrix()
            max_dq = self.dc["max_dq_per_tick"]
            q, st["ep"], st["er"] = self.ik.step(p_t, R_t, max_dq)
            if u >= 1.0:
                st["ep"], st["er"] = (float(np.linalg.norm(self.ik.error(g, Rg)[:3])),
                                      float(np.linalg.norm(self.ik.error(g, Rg)[3:])))
            self.q_target = q

        while True:
            self.update(ctrl)
            if self.judge.state == "failed":
                raise Abort(self.judge.reason)
            done = self.s.flight.t - t0 >= T
            if done and st["ep"] <= tol["pos_tol"] and math.degrees(st["er"]) <= tol["rot_tol_deg"]:
                return
            if self.s.flight.t - t0 > T + tol["timeout"]:
                raise Abort(f"reach_timeout:{self.phase} (위치 오차 {st['ep'] * 1000:.1f} mm, 자세 {math.degrees(st['er']):.2f}°)")


def _robot_links(stage):
    from pxr import Usd, UsdPhysics

    return [str(p.GetPath()) for p in Usd.PrimRange(stage.GetPrimAtPath(ts.ROBOT_PATH)) if p.HasAPI(UsdPhysics.RigidBodyAPI)]


def sequence(d):
    dc, s = d.dc, d.s
    d.phase = "S1_seed"
    d.joint_move(dc["seed_pose"], dc["joint_speed"])
    d.phase = "S1_pregrasp"
    d.cart_move(lambda: (d.grasp_target()[0] - np.array([0, 0, dc["pregrasp_below"]]), d.grasp_target()[1]), dc["move_speed"])
    # S2: 수직 상승. 잡는 높이까지 approach_track_until 보다 많이 남았으면 지금 드론을 따라가고, 그 아래로 오면 그 순간 목표로 고정
    #   (드론 호버 흔들림 따라가기 + 닿은 뒤 민 드론을 끝없이 쫓아가지 않기, 2026-10-01·02)
    d.phase = "S2_approach"
    frozen = {}

    def approach_goal():
        if "g" in frozen:
            return frozen["g"]
        g = d.grasp_target()
        if g[0][2] - d.ik.tcp_pose()[0][2] <= dc["approach_track_until"]:
            frozen["g"], frozen["t"] = g, s.flight.t
        return g

    d.cart_move(approach_goal, dc["approach_speed"])
    d.phase = "S3_close"
    s.drive.set_gripper_goal(ts.GRIPPER_UPPER)
    d.wait(dc["grip_close_timeout"], until=lambda: d.judge.state != "approach")
    if d.judge.state == "approach":
        raise Abort("grasp_timeout")
    d.phase = "S3_hold"                   # 잡은 채 드론 모터는 아직 돎 (드론 제어기가 그리퍼와 싸움)
    d.wait(KILL_DELAY)
    d.phase = "S4_release"
    s.flight.release()
    if "grip_max_force" in d.cc:   # drop: 모터 정지 순간부터 그리퍼 힘 한계를 낮춤 (잡은 뒤 힘을 잃는 상황, 구동 관절에만)
        s.robot.set_dof_max_efforts(np.array([[float(d.cc["grip_max_force"])]]), dof_indices=np.array([s.drive.grip_i]))
        got = float(s.robot.get_dof_max_efforts().numpy()[0][s.drive.grip_i])
        if abs(got - float(d.cc["grip_max_force"])) > 1e-6:
            raise RuntimeError(f"그리퍼 힘 한계 적용 실패: {got}")
    d.wait(dc["hold_time"] + 0.5, until=lambda: d.judge.state == "held")
    d.phase = "S5_end"                    # 실물은 여기서 사람이 드론을 떼어 감
    d.wait(dc["end_wait"])


ROW_KEYS = ("t", "phase", "state", "tcp_x", "tcp_y", "tcp_z", "drone_x", "drone_y", "drone_z", "grip_deg", "f_right", "f_left",
            "f_other", "armed", "rel_x", "rel_y", "rel_z", "rel_rx_deg", "rel_ry_deg", "rel_rz_deg", "w0", "w1", "w2", "w3", "ground",
            "physx_wx_dps", "physx_wy_dps", "physx_wz_dps", "sensor_wx_dps", "sensor_wy_dps", "sensor_wz_dps")


def hold_metrics(rows, phase="S3_hold"):
    """잡은 채 모터가 도는 구간의 떨림: TCP 기준 드론 위치(mm)·자세(°) 범위·표준편차·주요 주파수, 손가락 힘, 모터 ω."""
    r = [x for x in rows if x[1] == phase]
    if len(r) < 10:
        return None
    a = {k: np.array([x[i] for x in r], dtype=float) for i, k in enumerate(ROW_KEYS) if k not in ("phase", "state")}
    t = a["t"]
    out = {"duration_s": round(float(t[-1] - t[0]), 3), "samples": len(t)}
    rel = np.column_stack([a["rel_x"], a["rel_y"], a["rel_z"]]) * 1000
    # 자세는 구간 첫 샘플 기준 회전 (TCP–드론 상대 자세가 원래 약 180° 라 회전 벡터 성분이 ±180° 에서 뒤집힘)
    Rr = Rotation.from_rotvec(np.radians(np.column_stack([a["rel_rx_deg"], a["rel_ry_deg"], a["rel_rz_deg"]])))
    rot = np.degrees((Rr[0].inv() * Rr).as_rotvec())
    out["rel_pos_ptp_mm"] = np.round(np.ptp(rel, axis=0), 2).tolist()
    out["rel_pos_std_mm"] = np.round(rel.std(axis=0), 3).tolist()
    out["rel_rot_ptp_deg"] = np.round(np.ptp(rot, axis=0), 3).tolist()
    dt = float(np.median(np.diff(t)))
    x = np.linalg.norm(rel - rel.mean(axis=0), axis=1)
    if len(x) > 16 and dt > 0:
        F = np.abs(np.fft.rfft(x - x.mean()))
        fr = np.fft.rfftfreq(len(x), dt)
        out["rel_pos_peak_hz"] = round(float(fr[1:][np.argmax(F[1:])]), 2)
    for k in ("f_right", "f_left"):
        out[f"{k}_mean_N"], out[f"{k}_std_N"] = round(float(a[k].mean()), 2), round(float(a[k].std()), 3)
    w = np.column_stack([a[f"w{i}"] for i in range(4)])
    out["omega_mean"] = np.round(w.mean(axis=0), 1).tolist()
    out["omega_std"] = np.round(w.std(axis=0), 2).tolist()
    if np.isfinite(a["ground"]).any():
        g = a["ground"][np.isfinite(a["ground"])]
        out["ground_min_max_m"] = [round(float(g.min()), 3), round(float(g.max()), 3)]
    return out


def run_case(case, dc, out_dir, log):
    d = Demo(case, dc)
    p = d.plan
    log(f"\n===== case {case} =====")
    log(f"잡는 높이: TCP 드론 좌표계 {np.round(np.array(p['tcp_in_drone']) * 1000, 1).tolist()} mm, 닿는 각도 예측 "
        f"{math.degrees(p['q']):.2f}°, 폭 {p['width'] * 1000:.1f} mm, 그리퍼–드론 아랫면 여유 {p['clearance'] * 1000:.2f} mm, "
        "")
    abort = None
    try:
        sequence(d)
    except Abort as e:
        abort = str(e)
    res = d.judge.finish(d.s.flight.t)
    res.update(case=case, abort=abort, push_max_mm=round(d.push_max_mm, 2), flight=d.s.backend,
               kill_delay=KILL_DELAY, hold=hold_metrics(d.rows), approach=hold_metrics(d.rows, "S2_approach"),
               f_other_max_N={k: round(v, 2) for k, v in d.f_other_max.items()},
               link_force_max_N={ph: {k: round(v, 2) for k, v in m.items()} for ph, m in d.link_force_max.items()},
               plan={k: (round(v, 5) if isinstance(v, float) else v) for k, v in p.items()})
    exp_state, exp_reason = EXPECT[case]
    reasons = exp_reason if isinstance(exp_reason, tuple) else (exp_reason,)
    ok = res["state"] == exp_state and (exp_reason is None or any(str(res["reason"]).startswith(r) for r in reasons))
    res["expected"] = {"state": exp_state, "reason": exp_reason}
    res["as_expected"] = ok
    for t, st, info in res["events"]:
        log(f"  t {t:7.2f} s  {st:10s} {info}")
    log(f"  결과 {res['state']} ({res['reason']}), 중단 {abort}, 잡는 동안 드론이 밀린 거리 최대 {res['push_max_mm']} mm, "
        f"모터 정지 뒤 내려앉음 {res['settle_mm']} mm, "
        f"잡은 뒤 미끄러짐 최대 {res['max_slip_mm']} mm·회전 {res['max_rot_deg']}°, "
        f"손가락 외 로봇–드론 접촉(> 1 N) {res['f_other_max_N'] or '없음'}")
    for ph, m in res["link_force_max_N"].items():
        log(f"    {ph:12s} 드론과 닿은 로봇 링크 (최대 N): {m}")
    if res["hold"] is not None:
        log(f"  잡은 채 모터 도는 {res['hold']['duration_s']} s (S3_hold) 떨림: {res['hold']}")
    if res["approach"] is not None:
        a = res["approach"]
        log(f"  접근 중 (S2_approach) TCP 기준 드론 위치 범위 {a['rel_pos_ptp_mm']} mm, 하향 거리 {a.get('ground_min_max_m')}")
    log(f"  기대 {exp_state}/{exp_reason} → {'맞음' if ok else '틀림'}")
    with open(os.path.join(out_dir, f"result_{case}.json"), "w", encoding="utf-8") as f:
        json.dump(res, f, ensure_ascii=False, indent=1, default=float)
    with open(os.path.join(out_dir, f"timeline_{case}.csv"), "w", encoding="utf-8") as f:
        f.write(",".join(ROW_KEYS) + "\n")
        for r in d.rows:
            f.write(",".join(f"{v:.5f}" if isinstance(v, float) else str(v) for v in r) + "\n")
    return d, ok


def _drone_config_with_params(path, kvs):
    """--px4-param 을 드론 설정 px4.params 에 덮어쓴 임시 yaml (리포트 폴더) 경로."""
    import drone as dr_

    src = path or dr_.DEFAULT_DRONE_CONFIG
    with open(src, encoding="utf-8") as fh:
        c = yaml.safe_load(fh)
    for kv in kvs:
        k, v = kv.split("=", 1)
        c["px4"]["params"] = {**(c["px4"].get("params") or {}), k.strip(): float(v) if "." in v else int(v)}
    c["usd"] = c["usd"] if os.path.isabs(c["usd"]) else os.path.join(ts.SIM_ROOT, c["usd"])
    sc = c["px4"]["sitl_config"]
    c["px4"]["sitl_config"] = sc if os.path.isabs(sc) else os.path.join(ts.SIM_ROOT, sc)
    os.makedirs(args.report_dir, exist_ok=True)
    out = os.path.join(args.report_dir, f"drone_config_px4param_{datetime.datetime.now():%Y%m%d_%H%M%S}.yaml")
    with open(out, "w", encoding="utf-8") as fh:
        yaml.safe_dump(c, fh, allow_unicode=True)
    return out


def main():
    global KILL_DELAY
    dc = load_demo_config(args.demo_config)
    KILL_DELAY = float(dc["kill_delay"] if args.kill_delay is None else args.kill_delay)
    out_dir = os.path.join(args.report_dir, f"grasp_demo_{datetime.datetime.now():%Y%m%d_%H%M%S}")
    os.makedirs(out_dir, exist_ok=True)
    lines = []

    def log(m=""):
        print(m, flush=True)
        lines.append(m)

    log(f"grasp_demo  {datetime.datetime.now().isoformat(timespec='seconds')}  flight {args.flight or '(설정)'}, "
        f"position_source {args.position_source or '(설정)'}, kill_delay {KILL_DELAY} s, px4_param {args.px4_param}, "
        f"mode {args.mode}, prop_spin {args.prop_spin}"
        + (f", tcp_offset {args.tcp_offset} m (명령줄)" if args.tcp_offset is not None else ""))
    cases = CASES if args.case == "all" else (args.case,)
    oks, d = [], None
    try:
        for case in cases:
            if d is not None:
                d.s.flight.stop()
                d.s.drive.stop()
            d, ok = run_case(case, dc, out_dir, log)
            oks.append((case, ok))
        log("\n" + "=" * 50)
        for case, ok in oks:
            log(f"  {case:8s} {'PASS' if ok else 'FAIL'}")
        log(f"  {sum(ok for _, ok in oks)}/{len(oks)} PASS  (판정이 기대와 같으면 PASS)")
        if args.hold and not args.headless and d is not None:
            print("[hold] 창을 닫으면 종료", flush=True)
            while simulation_app.is_running():
                ds.step(d.s)
    except Abort:
        pass
    finally:
        with open(os.path.join(out_dir, "report.txt"), "w", encoding="utf-8") as f:
            f.write("\n".join(lines) + "\n")
        print(f"report: {out_dir}", flush=True)
    return len(oks) == len(cases) and all(ok for _, ok in oks)


if __name__ == "__main__":
    ok = False
    try:
        ok = main()
    except Exception as e:  # noqa: BLE001
        import traceback
        print(f"[ERROR] {type(e).__name__}: {e}", flush=True)
        traceback.print_exc()
    finally:
        simulation_app.close(exit_code=0 if ok else 1)
    sys.exit(0 if ok else 1)
