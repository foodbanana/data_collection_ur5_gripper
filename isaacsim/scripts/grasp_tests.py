#!/usr/bin/env python3
# =============================================================
# grasp_tests.py  (docs/PLAN.md 1-6)
#
# 손가락 collider·재질 점검과 파지 시험. 테스트 씬은 1-5 와 같다 (fixed base, z = 0.762 m, drive_gains.yaml).
#   K. 점검 (설정은 바꾸지 않음)
#      K1 손가락 파지면: 원래 메시와 convex hull 의 파지면(안쪽 끝 0.5 mm 이내 면) 넓이·범위·기울기 비교,
#         손가락 기구학(박스 배치 계산용)을 sim 링크 위치와 비교
#      K2 접촉 거리·재질: 손가락 contact/rest offset 의 USD 값과 PhysX 값, PhysX 가 쓰는 마찰값이 설정과 같은지
#      K3 파지 접촉: G2 박스(G3 과 같은 무게)를 잡은 직후 PhysX 접촉점의 손가락 좌표계 위치(파지면 범위 안인지)와 손가락별 수직력
#   G1 작은 박스: 40 mm 정육면체 0.2 kg 를 잡고 20 cm 들어 3 초 유지 → 미끄러짐 < 1 mm, 떨림 < 0.1 mm
#   G2 박스: 60(잡는 방향) × 160(긴 방향) × 100 mm, 0.3·0.5·0.8 kg (0.8 은 참고), 무게중심을 긴 방향으로 0·3·5 cm.
#      들어 올린 뒤 wrist_1, wrist_2 를 각각 ±30° 회전 → 미끄러짐 < 1 mm, 잡는 축 회전 < 2°
#   G3 당김: G2 박스(실물 당김 시험 박스 무게, G3_MASS)를 잡고 들어 올린 뒤, 공구 축을 따라 손가락 밖으로(공구가 아래를 향하므로 아래로) 당기는 힘을
#      천천히 늘림 → 미끄러짐 1 mm 순간의 힘. maxForce 여러 값으로 곡선 (--max-forces)
#   G4 정지: G2 박스 0.5 kg 을 들고 5 초 → 관절 떨림 < 0.01°, 박스 떨림 < 0.1 mm, 스텝당 계산 시간
#
# 배치: 박스는 움직이는 받침(kinematic, 폭 20 mm 라 손가락 사이로 들어감) 위에 놓는다. 박스 위치는 손가락 기구학(URDF)과
#   충돌 메시로 계산한다: 손가락이 박스 폭에서 닿는 각도, 그때 파지면 높이, 손가락·그리퍼 몸체가 박스 윗면에 닿지 않는 깊이.
# 팔 자세: 공구가 아래를 향하는 자세를 평면 2링크 IK 로 (shoulder_lift·elbow·wrist_1 축이 평행 → 들어 올릴 때 수직 이동).
#   시작 시 IK 확인: 5 cm 올린 자세에서 그리퍼가 수직으로 5 cm 움직이는지 sim 으로 잰다 (틀리면 에러)
# 미끄러짐: 잡기 직후 파지점(두 파지면 가운데)에 있던 박스 위의 점이 그리퍼 좌표계에서 움직인 거리.
# 회전: 잡기 직후 대비 박스의 그리퍼 기준 자세 변화 (잡는 축 = 그리퍼 y 성분).
#
# 실행:
#   ~/isaacsim/python.sh ~/data_collection_ur5_gripper/isaacsim/scripts/grasp_tests.py --headless
#   일부만: --tests K,G1   /   GUI: --headless 빼고 --realtime
#   손목 카메라 창 같이 보기 (GUI): --wrist-view  (tilt 비교는 --wrist-tilt 10)
#   마찰 조합 비교: --frictions 0.4/0.3 0.6/0.5 ... (정지/운동, 손가락·박스 재질 함께. 조합마다 씬을 새로 만들고 끝에 요약표)
#
# 단위: 리포트의 거리 mm, 각도 degree, 힘 N
# =============================================================

import argparse
import datetime
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import test_scene as ts  # noqa: E402

ALL_TESTS = ("K", "G1", "G2", "G3", "G4")
parser = argparse.ArgumentParser(description="손가락 collider·재질 점검과 파지 시험 (PLAN 1-6)")
parser.add_argument("--headless", action="store_true")
parser.add_argument("--usd", default=ts.DEFAULT_USD)
parser.add_argument("--physics-variant", default="physx")
parser.add_argument("--config", default=ts.DEFAULT_CONFIG, help="설정 파일 ('file.yaml@키=값' 덮어쓰기 가능)")
parser.add_argument("--tests", default=",".join(ALL_TESTS), help=f"쉼표로 구분: {','.join(ALL_TESTS)}")
parser.add_argument("--max-forces", type=float, nargs="+", default=[1.0, 2.0, 2.28, 3.0, 4.0, 6.0],
                    help="G3 에 쓸 rh_r1_joint maxForce 값들 [Nm]")
parser.add_argument("--realtime", action="store_true", help="sim 시간을 실제 시간에 맞춤 (GUI 로 눈으로 볼 때)")
parser.add_argument("--frictions", nargs="+", default=None,
                    help="마찰 조합 비교: '정지/운동' 여러 개 (예: 0.4/0.3 0.6/0.5). 손가락·박스 재질을 함께 바꿔 조합마다 씬을 새로 만들고 요약표를 낸다")
parser.add_argument("--wrist-view", action="store_true",
                    help="GUI: 손목 카메라(config/wrist_camera.yaml)를 만들고 'Wrist camera' 뷰포트 창을 추가로 띄운다 (메인 뷰포트는 그대로)")
parser.add_argument("--wrist-tilt", type=float, default=0.0, help="--wrist-view 카메라 tilt [degree] (씬 레이어 비교용, 0 = 실물 마운트)")
parser.add_argument("--mimic-armature", type=float, default=None,
                    help="비교 시험용: mimic 3 관절에도 이 armature [kg·m²] 를 준다 (설정 파일은 구동 관절 rh_r1_joint 에만 줌)")
args, _ = parser.parse_known_args()
TESTS = [t.strip() for t in args.tests.split(",") if t.strip()]
for t in TESTS:
    if t not in ALL_TESTS:
        raise SystemExit(f"--tests 에 모르는 시험: {t} ({ALL_TESTS})")

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": args.headless})

import time  # noqa: E402

import isaacsim.core.experimental.utils.app as app_utils  # noqa: E402
import numpy as np  # noqa: E402
from isaacsim.core.experimental.prims import RigidPrim  # noqa: E402
from isaacsim.core.simulation_manager import SimulationEvent, SimulationManager  # noqa: E402
from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics  # noqa: E402
from scipy.spatial.transform import Rotation  # noqa: E402

import collision_geom as cg  # noqa: E402
from gripper_geom import (CLEAR, PAD_TOL, finger_T, grasp_geometry, grip_lever, inner_gap, pad_depths,  # noqa: E402,F401
                          pad_patch, verts)
from robot_drive import RobotDrive  # noqa: E402

# ── 시험 조건 ──
GRIP_CLOSED = 1.1351
G = 9.81
# 박스 크기 (x = 긴 방향, y = 잡는 방향(손가락 사이), z = 공구 축 방향 높이), 그리퍼 좌표계 기준 [m]
BOX_G1 = (0.04, 0.04, 0.04)
BOX_G2 = (0.16, 0.06, 0.10)
G1_MASS = 0.2
# 실물은 400 mA 로 드론(0.8 kg)을 잡기만 했고 들어 올리지는 않음 → 무거운 박스를 반드시 들어야 하는 기준은 두지 않는다 (PLAN 1-6)
G2_MASSES = (0.3, 0.5, 0.8)       # 0.8 kg 은 참고 (실물 들어 올림 미확인)
G4_MASS = 0.5
G2_OFFSETS = (0.0, 0.03, 0.05)     # 무게중심을 긴 방향(x)으로
POST = (0.14, 0.02, 0.30)          # 받침 크기. 잡는 방향 폭 20 mm → 손가락(박스 폭 40 mm 이상에서 닿음) 사이로 들어감
LIFT, LIFT_T = 0.20, 2.0           # 들어 올리는 높이 m, 시간 s
TILT = math.radians(30.0)
TILT_JOINTS = ("wrist_1_joint", "wrist_2_joint")
# G3_MASS: 실물 당김 시험 박스의 실제 무게로 맞춘다 (재기 전 임시 0.3 kg). 미끄럼 힘에 박스 무게가 더해지므로 실물과 같게
G3_MASS, G3_RATE, G3_FMAX = 0.3, 5.0, 80.0   # kg, N/s, N (여기까지 안 미끄러지면 "> FMAX")
SLIP_JUDGE = 1e-3                  # m
FINGERS = ("rh_p12_rn_r2", "rh_p12_rn_l2")
SENSORS = FINGERS + ("rh_p12_rn_r1", "rh_p12_rn_l1", "rh_p12_rn_base")  # 접촉 보고를 켜는 링크 (r2·l2 가 파지면)
# 평면 IK 기준점 (UR5 DH, wrist_1 관절 중심을 어깨 기준 평면 좌표로). 홈 자세는 (-0.39225, 0.425)
A2, A3 = -0.425, -0.39225
X_REF, Z_REF = -0.45, 0.05
PAN, W2, W3 = 0.0, -math.pi / 2, 0.0

CRIT = {"slip_mm": 1.0, "wobble_mm": 0.1, "rot_deg": 2.0, "jitter_deg": 0.01}

BOX_ROOT = "/World/GraspObjects"
PARK = {"box_g1": (2.0, 0.0), "box_g2": (2.0, 0.5), "post": (-2.0, 0.0)}


# ─────────────────────────── 팔 IK ───────────────────────────
def planar_ik(x, z):
    """wrist_1 관절 중심 (x, z) [어깨 기준 평면] → (q2, q3, q4), 공구가 아래를 향하게 (q2 + q3 + q4 = -π/2). 팔꿈치 위쪽 해."""
    c3 = (x * x + z * z - A2 * A2 - A3 * A3) / (2 * A2 * A3)
    if abs(c3) > 1:
        raise ValueError(f"평면 IK 도달 불가: ({x:.3f}, {z:.3f})")
    q3 = math.acos(c3)
    q2 = math.atan2(z, x) - math.atan2(A3 * math.sin(q3), A2 + A3 * math.cos(q3))
    q2 = (q2 + math.pi) % (2 * math.pi) - math.pi
    return q2, q3, -math.pi / 2 - q2 - q3


def arm_at(dz=0.0):
    q2, q3, q4 = planar_ik(X_REF, Z_REF + dz)
    return np.array([PAN, q2, q3, q4, W2, W3])


def smooth(s):
    s = min(max(s, 0.0), 1.0)
    return s * s * (3 - 2 * s)


def pose_R(quat):
    return cg.quat_to_R(quat)


def R_quat(R):
    x, y, z, w = Rotation.from_matrix(R).as_quat()
    return np.array([w, x, y, z])


# ─────────────────────────── 씬 ───────────────────────────
def add_box(stage, name, dims, color):
    """강체 Xform(질량 속성은 여기) + 크기 조절한 Cube 충돌 형상. 몸체 좌표계는 크기 조절 없음 (무게중심 좌표가 m 단위)."""
    path = f"{BOX_ROOT}/{name}"
    body = UsdGeom.Xform.Define(stage, path)
    body.AddTranslateOp().Set(Gf.Vec3d(*PARK[name], 0.5))
    body.AddOrientOp().Set(Gf.Quatf(1, 0, 0, 0))
    prim = body.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(prim)
    PhysxSchema.PhysxRigidBodyAPI.Apply(prim).CreateSleepThresholdAttr().Set(0.0)
    geom = UsdGeom.Cube.Define(stage, f"{path}/geom")
    geom.CreateSizeAttr().Set(1.0)
    geom.AddScaleOp().Set(Gf.Vec3f(*dims))
    geom.CreateDisplayColorAttr().Set([Gf.Vec3f(*color)])
    UsdPhysics.CollisionAPI.Apply(geom.GetPrim())
    return prim, geom.GetPrim()


def build_objects(stage):
    UsdGeom.Xform.Define(stage, BOX_ROOT)
    out = {}
    for name, dims, color in (("box_g1", BOX_G1, (0.9, 0.5, 0.1)), ("box_g2", BOX_G2, (0.2, 0.5, 0.9))):
        body, geom = add_box(stage, name, dims, color)
        UsdPhysics.MassAPI.Apply(body).CreateMassAttr().Set(0.5)   # 시험마다 tensor API 로 다시 정함
        ts.bind_object_material(stage, geom)
        out[name] = str(body.GetPath())
    post, _ = add_box(stage, "post", POST, (0.5, 0.5, 0.5))
    UsdPhysics.RigidBodyAPI(post).CreateKinematicEnabledAttr().Set(True)  # 받침은 재질 없음 (PhysX 기본값)
    out["post"] = str(post.GetPath())
    return out


class Ctx:
    pass


class Runner:
    """physics 스텝마다 명령(pre)과 기록(post). 콜백 예외는 저장 후 check() 에서 올림."""

    def __init__(self, c):
        self.c = c
        self.t = 0.0
        self.on = False
        self.rows = []
        self.cmd = None
        self.watch = None
        self.stop_flag = False
        self.err = None
        self.cb = [SimulationManager.register_callback(self._pre, event=SimulationEvent.PHYSICS_PRE_STEP, order=-1),
                   SimulationManager.register_callback(self._post, event=SimulationEvent.PHYSICS_POST_STEP)]

    def sample(self):
        c = self.c
        bp, bq = (x.numpy()[0] for x in c.base.get_world_poses())
        op, oq = (x.numpy()[0] for x in c.box.get_world_poses())
        return (self.t, c.robot.get_dof_positions().numpy()[0].copy(), bp.copy(), bq.copy(), op.copy(), oq.copy())

    def _pre(self, dt, ctx):
        try:
            if self.cmd is not None:
                self.cmd(self.t)
        except Exception as e:  # noqa: BLE001
            self.err = e

    def _post(self, dt, ctx):
        try:
            self.t += dt
            if self.on or self.watch is not None:
                s = self.sample()
                if self.on:
                    self.rows.append(s)
                if self.watch is not None and self.watch(s):
                    self.stop_flag = True
        except Exception as e:  # noqa: BLE001
            self.err = e

    def start(self):
        self.rows, self.on = [], True

    def stop(self):
        self.on = False
        return self.rows

    def close(self):
        for cb in self.cb:
            SimulationManager.deregister_callback(cb)

    def check(self):
        if self.err is not None:
            raise RuntimeError(f"콜백 오류: {self.err!r}")


def run_for(c, sec, until_flag=False):
    r = c.run
    t_end = r.t + sec
    wall0, sim0 = time.monotonic(), r.t
    r.stop_flag = False
    while r.t < t_end - 1e-9:
        n0, w0 = r.t, time.perf_counter()
        simulation_app.update()
        c.wall += time.perf_counter() - w0
        c.steps += int(round((r.t - n0) / ts.PHYSICS_DT))
        r.check()
        c.drive.check()
        if until_flag and r.stop_flag:
            return True
        if args.realtime:
            ahead = (r.t - sim0) - (time.monotonic() - wall0)
            if ahead > 0:
                time.sleep(ahead)
    return False


def open_scene(cfg):
    c = Ctx()
    c.cfg = cfg
    c.stage, c.robot, c.info = ts.build_test_scene(args.usd, args.physics_variant, base="fixed", config=cfg,
                                                   light=not args.headless)
    c.paths = build_objects(c.stage)
    c.cam_path = ts.add_wrist_camera(c.stage, ts.load_camera_config(), math.radians(args.wrist_tilt)) if args.wrist_view else None
    c.shapes, c.link_paths = cg.collect(c.stage)
    for n in SENSORS:  # 접촉 보고 (재생 전에 켜야 PhysX 가 접촉 데이터를 만든다). 물리에는 영향 없음
        PhysxSchema.PhysxContactReportAPI.Apply(c.stage.GetPrimAtPath(ts.find_link_path(c.stage, n))) \
            .CreateThresholdAttr().Set(0.0)
    app_utils.play()
    simulation_app.update()
    problems, c.verify_lines = ts.verify_articulation(c.stage, c.robot, c.info, cfg)
    if problems:
        raise RuntimeError(f"articulation 확인 실패: {problems}")
    c.drive = RobotDrive(c.robot, cfg)
    c.drive.apply()
    if args.mimic_armature is not None:
        arm = c.robot.get_dof_armatures().numpy()[0].copy()
        arm[c.drive.mimic_i] = args.mimic_armature
        c.robot.set_dof_armatures(arm)
        got = c.robot.get_dof_armatures().numpy()[0][c.drive.mimic_i]
        k, d = (x.numpy()[0][c.drive.mimic_i] for x in c.robot.get_dof_gains())
        if np.abs(got - args.mimic_armature).max() > 1e-6 or np.abs(k).max() > 0 or np.abs(d).max() > 0:
            raise RuntimeError(f"mimic armature 적용 확인 실패: armature {got}, K {k}, D {d}")
    c.drive.start()
    c.base = RigidPrim(ts.find_link_path(c.stage, ts.PAYLOAD_LINK))
    c.boxes = {n: RigidPrim(c.paths[n]) for n in ("box_g1", "box_g2")}
    c.post = RigidPrim(c.paths["post"])
    # 필터 개수가 prim 개수와 같으면 API 가 1:1 로 짝지으므로 개수가 다르게 둔다 (5 링크 × 필터 3 개 모두)
    c.filters = [c.paths["box_g1"], c.paths["box_g2"], c.paths["post"]]
    assert len(c.filters) != len(SENSORS)
    c.fingers = RigidPrim([ts.find_link_path(c.stage, n) for n in SENSORS], contact_filter_paths=c.filters,
                          max_contact_count=1024)
    c.box = c.boxes["box_g2"]
    c.gi = c.drive.idx[ts.GRIPPER_DRIVE]
    c.ai = np.array([c.drive.idx[n] for n in ts.ARM_JOINTS])
    c.fi = [c.drive.idx[n] for n in ts.GRIPPER_JOINTS]
    c.wall, c.steps = 0.0, 0
    c.run = Runner(c)
    calibrate(c)
    if not args.headless:
        from isaacsim.core.utils.viewports import set_camera_view

        p = c.P0
        set_camera_view(eye=list(p + np.array([0.7, 0.7, 0.25])), target=list(p + np.array([0, 0, -0.1])),
                        camera_prim_path="/OmniverseKit_Persp")
        if c.cam_path:
            show_wrist_window(c.cam_path)
    return c


_WRIST_WINDOW = []


def show_wrist_window(cam_path):
    """손목 카메라 뷰포트 창 (한 번 만들고, 씬을 새로 만들 때마다 카메라만 다시 연결)."""
    from omni.kit.viewport.utility import create_viewport_window
    from pxr import Sdf

    w, h = (int(x) for x in ts.load_camera_config()["resolution"])
    if not _WRIST_WINDOW:
        _WRIST_WINDOW.append(create_viewport_window("Wrist camera", width=w, height=h, camera_path=Sdf.Path(cam_path)))
    api = _WRIST_WINDOW[0].viewport_api
    api.camera_path = Sdf.Path(cam_path)
    api.resolution = (w, h)


def close_scene(c):
    c.drive.stop()
    c.run.close()
    app_utils.stop()
    simulation_app.update()


def base_pose(c):
    p, q = (x.numpy()[0] for x in c.base.get_world_poses())
    return p.copy(), pose_R(q)


def finger_contacts(c):
    """SENSORS 링크와 시험 박스 사이 PhysX 접촉: 이름 → (접촉 수, 수직력 합 N, 링크 좌표계 접촉점 (n, 3))."""
    forces, points, _, _, counts, starts = c.fingers.get_contact_force_data(dt=ts.PHYSICS_DT)  # 충격량 / dt = 힘
    forces, points = forces.numpy().reshape(-1), points.numpy().reshape(-1, 3)
    counts, starts = counts.numpy().astype(int), starts.numpy().astype(int)
    if counts.shape != (len(SENSORS), len(c.filters)):
        raise RuntimeError(f"접촉 데이터 모양이 예상과 다름: {counts.shape}")
    pos, quat = (x.numpy() for x in c.fingers.get_world_poses())
    out = {}
    for i, n in enumerate(SENSORS):
        idx = np.concatenate([np.arange(starts[i, k], starts[i, k] + counts[i, k]) for k in (0, 1)])  # 박스 2 개만
        out[n] = (len(idx), float(forces[idx].sum()), (points[idx] - pos[i]) @ pose_R(quat[i]))
    return out


def calibrate(c):
    """기준 자세에서 그리퍼 base 자세를 재고, 평면 IK 로 5 cm 올렸을 때 수직으로 5 cm 움직이는지 확인."""
    c.drive.reset_pose(arm_at(0.0), 0.0)
    run_for(c, 0.5)
    c.P0, c.R0 = base_pose(c)
    down = float(c.R0[:, 2] @ np.array([0, 0, -1.0]))
    if down < math.cos(math.radians(1.0)):
        raise RuntimeError(f"기준 자세에서 공구가 아래를 향하지 않음 (공구 축·아래 사이 {math.degrees(math.acos(down)):.2f}°)")
    c.drive.reset_pose(arm_at(0.05), 0.0)
    run_for(c, 0.5)
    P1, R1 = base_pose(c)
    err = np.linalg.norm((P1 - c.P0) - np.array([0, 0, 0.05]))
    rot = math.degrees(Rotation.from_matrix(R1 @ c.R0.T).magnitude())
    c.calib = f"IK 확인: 5 cm 올림 → 이동 {np.round((P1 - c.P0) * 1000, 2)} mm (오차 {err * 1000:.2f} mm), 자세 변화 {rot:.3f}°"
    if err > 5e-4 or rot > 0.1:
        raise RuntimeError(c.calib + " → 평면 IK 가 sim 과 맞지 않음")


def park(c):
    for n, v in c.boxes.items():
        v.set_world_poses(positions=np.array([[*PARK[n], 0.3]]), orientations=np.array([[1.0, 0, 0, 0]]))
        v.set_velocities(np.zeros((1, 3)), np.zeros((1, 3)))


def set_box_mass(c, dims, mass, com_dx):
    lx, ly, h = dims
    inertia = mass / 12.0 * np.array([ly ** 2 + h ** 2, lx ** 2 + h ** 2, lx ** 2 + ly ** 2])  # 균일 박스 근사
    c.box.set_masses(np.array([[mass]]))
    c.box.set_inertias(np.diag(inertia).reshape(1, 9))
    c.box.set_coms(positions=np.array([[com_dx, 0.0, 0.0]]), orientations=np.array([[1.0, 0, 0, 0]]))
    m = float(c.box.get_masses().numpy().ravel()[0])
    com = c.box.get_coms()[0].numpy().ravel()
    if abs(m - mass) > 1e-4 or abs(com[0] - com_dx) > 1e-5 or np.abs(com[1:3]).max() > 1e-5:
        raise RuntimeError(f"박스 질량·무게중심 적용 실패: mass {m} (설정 {mass}), com {com} (설정 x {com_dx})")


def place_and_grasp(c, name, dims, geo, mass, com_dx):
    """기준 자세·그리퍼 열림으로 순간이동 → 받침·박스 배치 → 닫기. 잡은 직후 기준(ref)을 돌려준다."""
    c.drive.reset_pose(arm_at(0.0), 0.0)
    park(c)
    c.box = c.boxes[name]
    run_for(c, 0.3)
    P, R = base_pose(c)
    lx, ly, h = dims
    box_c = P + R @ np.array([0, 0, geo["z_top"] + h / 2])
    post_c = P + R @ np.array([0, 0, geo["z_top"] + h + POST[2] / 2])
    q = R_quat(R).reshape(1, 4)
    c.post.set_world_poses(positions=post_c.reshape(1, 3), orientations=q)
    c.box.set_world_poses(positions=box_c.reshape(1, 3), orientations=q)
    c.box.set_velocities(np.zeros((1, 3)), np.zeros((1, 3)))
    set_box_mass(c, dims, mass, com_dx)
    run_for(c, 0.3)
    c.drive.set_gripper_goal(GRIP_CLOSED)
    run_for(c, GRIP_CLOSED / c.drive.gripper.velocity + 0.5)
    return make_ref(c.run.sample(), geo)


def make_ref(s, geo):
    """잡은 직후: 그리퍼 좌표계의 파지점 g 와, 그 순간 g 에 있던 박스 위 점(박스 좌표계)."""
    _, _, bp, bq, op, oq = s
    Rb, Ro = pose_R(bq), pose_R(oq)
    g = np.array([0.0, 0.0, geo["grip_z"]])
    g_world = bp + Rb @ g
    return dict(g=g, g_box=Ro.T @ (g_world - op), Rrel0=Rb.T @ Ro, box_z0=op[2])


def slip_rot(s, ref):
    """(미끄러짐 벡터 [그리퍼 좌표계, m], 회전 벡터 [그리퍼 좌표계, rad]) — 잡은 직후 대비."""
    _, _, bp, bq, op, oq = s
    Rb, Ro = pose_R(bq), pose_R(oq)
    g_now = Rb.T @ (op + Ro @ ref["g_box"] - bp)
    rv = Rotation.from_matrix((Rb.T @ Ro) @ ref["Rrel0"].T).as_rotvec()
    return g_now - ref["g"], rv


def summarize(rows, ref):
    sl = np.array([np.linalg.norm(slip_rot(s, ref)[0]) for s in rows])
    rv = np.array([slip_rot(s, ref)[1] for s in rows])
    return dict(slip=float(sl.max()) * 1000, rot_y=math.degrees(float(np.abs(rv[:, 1]).max())),
                rot=math.degrees(float(np.linalg.norm(rv, axis=1).max())))


def lift(c, sec=LIFT_T):
    t0 = c.run.t

    def cmd(t):
        c.drive.set_arm_targets(arm_at(LIFT * smooth((t - t0) / sec)))

    c.run.cmd = cmd
    run_for(c, sec)
    c.run.cmd = None
    c.drive.set_arm_targets(arm_at(LIFT))


def move_joint(c, j, a_from, a_to, sec):
    base = arm_at(LIFT)
    t0 = c.run.t

    def cmd(t):
        q = base.copy()
        q[j] += a_from + (a_to - a_from) * smooth((t - t0) / sec)
        c.drive.set_arm_targets(q)

    c.run.cmd = cmd
    run_for(c, sec)
    c.run.cmd = None
    q = base.copy()
    q[j] += a_to
    c.drive.set_arm_targets(q)


# ─────────────────────────── 시험 ───────────────────────────
def test_k(c, L):
    """점검 K1·K2. 문제 목록을 돌려준다 (설정 확인 실패는 에러)."""
    shapes = c.shapes
    L("")
    L(f"[K1] 손가락 파지면: 원래 메시 vs convex hull (파지면 = 안쪽 끝에서 {PAD_TOL * 1000:.1f} mm 이내·법선 10° 이내 면, 링크 좌표계)")
    L(f"  {'link':14s} {'':6s} {'끝 위치 mm':>10s} {'넓이 mm²':>9s} {'x 범위 mm':>16s} {'z 범위 mm':>16s} {'기울기 °':>8s}")
    for link, d in (("rh_p12_rn_r2", np.array([0, -1.0, 0])), ("rh_p12_rn_l2", np.array([0, 1.0, 0]))):
        n_shapes = len(shapes[link])
        for k, s in enumerate(shapes[link]):
            for label, mesh in (("raw", s.raw), ("hull", s.hull)):
                p = pad_patch(mesh, d)
                L(f"  {link + (f'[{k}]' if n_shapes > 1 else ''):14s} {label:6s} {p['support'] * 1000:10.2f} {p['area'] * 1e6:9.1f} "
                  f"{p['x'][0] * 1000:7.1f}~{p['x'][1] * 1000:7.1f} {p['z'][0] * 1000:7.1f}~{p['z'][1] * 1000:7.1f} {p['tilt_deg']:8.2f}")
            if s.is_mesh:
                L(f"  {'':14s} 원래 메시 닫힘(watertight) = {s.raw.is_watertight}")
                X, Z, dr, dh = pad_depths(s, d)
                gap = dr - dh
                L(f"  {'':14s} 파지면 쪽 x {X.min() * 1000:.1f}~{X.max() * 1000:.1f}, z {Z.min() * 1000:.1f}~{Z.max() * 1000:.1f} mm 에서"
                  f" 안쪽 끝 평면 기준 깊이: 원래 메시 최대 {np.nanmax(dr) * 1000:.2f} mm, convex hull 최대 {np.nanmax(dh) * 1000:.2f} mm,"
                  f" 둘 차이 최대 {np.nanmax(gap) * 1000:.2f} mm")
                for zlo in np.arange(Z.min(), Z.max(), 0.005):
                    m = (Z >= zlo) & (Z < zlo + 0.005)
                    L(f"  {'':14s}   z {zlo * 1000:5.1f}~{(zlo + 0.005) * 1000:5.1f}: 원래 {np.nanmean(dr[m]) * 1000:5.2f} mm, "
                      f"hull {np.nanmean(dh[m]) * 1000:5.2f} mm (평균)")
    L("  (한 방향의 가장 바깥 점은 convex hull 과 원래 메시가 같으므로 '끝 위치'는 항상 같다."
      " 평평한 박스 면은 가장 바깥 면에만 닿으므로, hull 이 메운 오목한 부분이 끝 평면보다 뒤에 있으면 박스 접촉은 같다)")

    # 손가락 기구학 확인: 그리퍼를 0.3·0.6 rad 로 순간이동시키고 sim 링크 위치와 URDF 식 비교
    L("")
    L("  손가락 기구학(URDF 식) vs sim 링크 위치 (그리퍼 base 좌표계, mm):")
    views = {n: RigidPrim(ts.find_link_path(c.stage, n)) for n in ("rh_p12_rn_r2", "rh_p12_rn_l2")}
    worst = 0.0
    for q in (0.3, 0.6):
        c.drive.reset_pose(arm_at(0.0), q)
        run_for(c, 0.3)  # 순간이동 직후 과도 상태가 지나가게 (per_finger 는 왼손가락이 몇 스텝 흔들림)
        P, R = base_pose(c)
        T = finger_T(q)
        qs = c.robot.get_dof_positions().numpy()[0][c.fi]
        L(f"    q={q:.1f} 0.3 초 뒤 관절: " + ", ".join(f"{n} {v:.4f}" for n, v in zip(ts.GRIPPER_JOINTS, qs)))
        for n, v in views.items():
            p = v.get_world_poses()[0].numpy()[0]
            loc = R.T @ (p - P)
            e = float(np.linalg.norm(loc - T[n][1]))
            worst = max(worst, e)
            L(f"    q={q:.1f} {n}: sim {np.round(loc * 1000, 2)}  식 {np.round(T[n][1] * 1000, 2)}  차이 {e * 1000:.3f}")
    if worst > 5e-4:
        raise RuntimeError(f"손가락 기구학이 sim 과 {worst * 1000:.2f} mm 다름 → 박스 배치 계산을 믿을 수 없음")
    c.drive.reset_pose(arm_at(0.0), 0.0)
    run_for(c, 0.3)

    for name, dims in (("G1", BOX_G1), ("G2", BOX_G2)):
        g = grasp_geometry(shapes, dims)
        L(f"  {name} 박스 폭 {dims[1] * 1000:.0f} mm: 닿는 각도 {math.degrees(g['qc']):.2f}° ({g['qc']:.4f} rad), 손가락 사이 열림 "
          f"{g['g_open'] * 1000:.1f} / 닫힘 {g['g_closed'] * 1000:.1f} mm, 파지면 z {g['pad_z'][0] * 1000:.1f}~{g['pad_z'][1] * 1000:.1f} mm, "
          f"박스 윗면 z {g['z_top'] * 1000:.1f} mm (파지면 {g['engaged'] * 100:.0f}% 덮음)")

    L("")
    L(f"[K3] 파지 접촉: G2 박스 {G3_MASS} kg 을 잡은 직후 PhysX 접촉점 (손가락 링크 좌표계 z, mm) 과 손가락별 수직력")
    geo = grasp_geometry(shapes, BOX_G2)
    place_and_grasp(c, "box_g2", BOX_G2, geo, G3_MASS, 0.0)
    run_for(c, 0.2)
    fc = finger_contacts(c)
    for n in SENSORS[len(FINGERS):]:
        L(f"  {n}: 박스와 접촉 {fc[n][0]} 개, 수직력 {fc[n][1]:.2f} N (0 이어야 함: 파지면 말고는 박스에 닿지 않게 배치)")
    for n in FINGERS:
        cnt, f, loc = fc[n]
        d = np.array([0, -1.0, 0]) if n == "rh_p12_rn_r2" else np.array([0, 1.0, 0])
        pz = pad_patch(cg.trimesh.util.concatenate([s.raw for s in shapes[n]]), d)["z"]
        out = int(((loc[:, 2] < pz[0] - 5e-4) | (loc[:, 2] > pz[1] + 5e-4)).sum()) if cnt else 0
        zr = f"{loc[:, 2].min() * 1000:.1f}~{loc[:, 2].max() * 1000:.1f}" if cnt else "-"
        L(f"  {n}: 접촉 {cnt} 개, 수직력 {f:.2f} N, 접촉점 z {zr} (원래 메시 파지면 z {pz[0] * 1000:.1f}~{pz[1] * 1000:.1f}), "
          f"파지면 밖 {out} 개")
    q = np.degrees(c.robot.get_dof_positions().numpy()[0][c.fi])
    L("  그리퍼 관절 각도 (잡은 상태, mimic 이 단단하면 네 값이 같음): "
      + ", ".join(f"{n} {v:.3f}°" for n, v in zip(ts.GRIPPER_JOINTS, q)) + f"  (기하 예상 {math.degrees(geo['qc']):.3f}°)")
    r = c.robot
    fi = c.fi
    diag = {"목표 rad": r.get_dof_position_targets().numpy()[0][fi], "위치 rad": r.get_dof_positions().numpy()[0][fi],
            "K": r.get_dof_gains()[0].numpy()[0][fi], "D": r.get_dof_gains()[1].numpy()[0][fi],
            "maxForce": r.get_dof_max_efforts().numpy()[0][fi], "drive type": np.array(r.get_dof_drive_types()[0])[fi]}
    for k, v in zip(("정지 마찰", "운동 마찰", "점성 마찰"), r.get_dof_friction_properties()):
        diag[k] = v.numpy()[0][fi]
    for k, v in zip(("speed-effort", "max act vel", "vel 저항"), r.get_dof_drive_model_properties()):
        diag[k] = v.numpy()[0][fi]
    for k, v in diag.items():
        L(f"  {k:12s}: " + ", ".join(str(np.round(x, 4)) if not isinstance(x, str) else x for x in v))
    tau = c.robot.get_dof_projected_joint_forces().numpy()[0][c.fi]
    lever = grip_lever(float(r.get_dof_positions().numpy()[0][c.gi]))
    fsum = fc[FINGERS[0]][1] + fc[FINGERS[1]][1]
    L("  PhysX 관절 방향 힘 (projected joint forces, Nm): " + ", ".join(f"{n} {v:.3f}" for n, v in zip(ts.GRIPPER_JOINTS, tau)))
    ideal = float(c.cfg["gripper"][ts.GRIPPER_DRIVE]["max_force"])
    L(f"  가상일 확인: 파지면이 rh_r1 1 rad 당 {lever * 1000:.1f} mm 안쪽으로 움직임 → 수직력 합 {fsum:.2f} N × {lever * 1000:.1f} mm"
      f" = {fsum * lever:.3f} Nm (이상적이면 maxForce {ideal:.2f} Nm, 전달률 {fsum * lever / ideal * 100:.0f}%)")
    park(c)

    L("")
    L("[K2] 접촉 거리·재질 (손가락 r2·l2, 시험 박스, 받침)")
    for link in ("rh_p12_rn_r2", "rh_p12_rn_l2"):
        for path in ts._link_collision_meshes(c.stage, c.stage.GetPrimAtPath(ts.find_link_path(c.stage, link))):
            api = PhysxSchema.PhysxCollisionAPI(c.stage.GetPrimAtPath(path))
            has = c.stage.GetPrimAtPath(path).HasAPI(PhysxSchema.PhysxCollisionAPI)
            co = api.GetContactOffsetAttr().Get() if has else None
            ro = api.GetRestOffsetAttr().Get() if has else None
            L(f"  USD {path.split('/')[-1]:10s} ({link}): PhysxCollisionAPI={has}, contactOffset={co}, restOffset={ro} "
              f"(없음/-inf = PhysX 자동)")
    problems = []
    cfg_c = c.cfg["contact"]
    names = ["rh_p12_rn_r2", "rh_p12_rn_l2", "box_g2", "post"]
    view = RigidPrim([ts.find_link_path(c.stage, n) for n in names[:2]] + [c.paths["box_g2"], c.paths["post"]])
    pv = view._physics_rigid_body_view
    co = pv.get_contact_offsets().numpy().reshape(len(names), -1)
    ro = pv.get_rest_offsets().numpy().reshape(len(names), -1)
    mp = pv.get_material_properties().numpy().reshape(len(names), -1, 3)
    shape_counts = [len(c.shapes[n]) for n in names[:2]] + [1, 1]
    for k, n in enumerate(names):
        ns = shape_counts[k]
        L(f"  PhysX {n:14s}: contact offset {np.round(co[k, :ns] * 1000, 3)} mm, rest offset {np.round(ro[k, :ns] * 1000, 3)} mm, "
          f"마찰 정지/운동/반발 {[tuple(np.round(m, 3)) for m in mp[k, :ns]]}")
    for k, n, want in ((0, "rh_p12_rn_r2", cfg_c["finger_material"]), (1, "rh_p12_rn_l2", cfg_c["finger_material"]),
                       (2, "box_g2", cfg_c["object_material"])):
        exp = np.array([want["static_friction"], want["dynamic_friction"], want["restitution"]])
        got = mp[k, :shape_counts[k]]
        if np.abs(got - exp).max() > 1e-4:
            problems.append(f"{n} 의 PhysX 재질 {got.tolist()} != 설정 {exp.tolist()}")
    L(f"  재질 합치는 방식: 손가락 {cfg_c['finger_material']['friction_combine']}, 박스 {cfg_c['object_material']['friction_combine']}"
      f" → 손가락–박스 마찰 정지 {min(cfg_c['finger_material']['static_friction'], cfg_c['object_material']['static_friction'])}")
    if problems:
        raise RuntimeError("재질 확인 실패: " + "; ".join(problems))


def test_g1(c):
    geo = grasp_geometry(c.shapes, BOX_G1)
    ref = place_and_grasp(c, "box_g1", BOX_G1, geo, G1_MASS, 0.0)
    q_grip = float(c.robot.get_dof_positions().numpy()[0][c.gi])
    c.run.start()
    lift(c)
    run_for(c, 3.0)
    rows = c.run.stop()
    s = summarize(rows, ref)
    last = [r for r in rows if r[0] >= rows[-1][0] - 2.0]
    g = np.array([slip_rot(r, ref)[0] for r in last])
    s["wobble"] = float((g.max(axis=0) - g.min(axis=0)).max()) * 1000
    s["rise"] = (rows[-1][4][2] - ref["box_z0"]) * 1000
    s["q_grip"], s["q_pred"] = math.degrees(q_grip), math.degrees(geo["qc"])
    return s


def test_g2(c):
    geo = grasp_geometry(c.shapes, BOX_G2)
    res = []
    for mass in G2_MASSES:
        for dx in G2_OFFSETS:
            ref = place_and_grasp(c, "box_g2", BOX_G2, geo, mass, dx)
            q_grip = float(c.robot.get_dof_positions().numpy()[0][c.gi])
            c.run.start()
            lift(c)
            run_for(c, 0.5)
            lift_rows = list(c.run.rows)
            per = {}
            axis = {}
            for jn in TILT_JOINTS:
                j = ts.ARM_JOINTS.index(jn)
                n0 = len(c.run.rows)
                _, R_a = base_pose(c)
                move_joint(c, j, 0.0, TILT, 1.0)
                _, R_b = base_pose(c)
                run_for(c, 0.5)
                move_joint(c, j, TILT, -TILT, 2.0)
                run_for(c, 0.5)
                move_joint(c, j, -TILT, 0.0, 1.0)
                run_for(c, 0.5)
                per[jn] = summarize(c.run.rows[n0:], ref)
                rv = Rotation.from_matrix(R_a.T @ R_b).as_rotvec()  # 그리퍼 좌표계에서 본 기울임 축
                axis[jn] = math.degrees(math.acos(min(1.0, abs(rv[1]) / max(np.linalg.norm(rv), 1e-12))))
            rows = c.run.stop()
            s = summarize(rows, ref)
            s.update(mass=mass, dx=dx, q_grip=math.degrees(q_grip), lift=summarize(lift_rows, ref), per=per, axis=axis,
                     rise=(lift_rows[-1][4][2] - ref["box_z0"]) * 1000)
            res.append(s)
    return res, geo


def test_g3(c):
    geo = grasp_geometry(c.shapes, BOX_G2)
    cfg_mf = float(c.cfg["gripper"][ts.GRIPPER_DRIVE]["max_force"])
    res = []
    try:
        di = [c.gi]
        for mf in args.max_forces:
            c.robot.set_dof_max_efforts(np.full(len(di), mf, dtype=np.float32), dof_indices=di)
            got = c.robot.get_dof_max_efforts().numpy()[0][di]
            if np.abs(got - mf).max() > 1e-3 * max(1.0, mf):
                raise RuntimeError(f"그리퍼 maxForce 적용 실패: {got} != {mf}")
            ref = place_and_grasp(c, "box_g2", BOX_G2, geo, G3_MASS, 0.0)
            q_grip = float(c.robot.get_dof_positions().numpy()[0][c.gi])
            lift(c)
            run_for(c, 0.5)
            fc = finger_contacts(c)
            now = c.run.sample()
            lift_slip = float(np.linalg.norm(slip_rot(now, ref)[0]))   # 잡은 뒤 들어 올리는 동안 미끄러진 양
            ref = make_ref(now, geo)                                     # 당김 미끄러짐은 당기기 직전 기준
            q_now = float(now[1][c.gi])
            st = {"F": 0.0, "t0": c.run.t, "first": None}
            com_local = np.zeros(3)

            def push(t):
                F = G3_RATE * (t - st["t0"])
                st["F"] = F
                bp, bq = (x.numpy()[0] for x in c.base.get_world_poses())
                op, oq = (x.numpy()[0] for x in c.box.get_world_poses())
                d = pose_R(bq)[:, 2]  # 공구 축, 손가락 끝 방향 = 손가락 밖으로
                p = op + pose_R(oq) @ com_local
                c.box.apply_forces_and_torques_at_pos(forces=(F * d).reshape(1, 3), positions=p.reshape(1, 3),
                                                      local_frame=False)

            def watch(s):
                sl = float(np.linalg.norm(slip_rot(s, ref)[0]))
                if st["first"] is None and sl > 1e-4:
                    st["first"] = st["F"]
                return sl > SLIP_JUDGE

            c.run.cmd, c.run.watch = push, watch
            slipped = run_for(c, G3_FMAX / G3_RATE, until_flag=True)
            c.run.cmd, c.run.watch = None, None
            _, R = base_pose(c)
            w_axis = G3_MASS * G * float(R[:, 2] @ np.array([0, 0, -1.0]))  # 박스 무게 중 당김 방향 성분
            N = [fc[n][1] for n in FINGERS]
            res.append(dict(mf=mf, slipped=slipped, F=st["F"], F_first=st["first"], total=st["F"] + w_axis, w_axis=w_axis,
                            q_grip=math.degrees(q_grip), N=N, lift_slip=lift_slip * 1000,
                            trans=sum(N) * grip_lever(q_now) / mf))
    finally:
        c.robot.set_dof_max_efforts(np.full(len(di), cfg_mf, dtype=np.float32), dof_indices=di)
    return res


def test_g4(c):
    geo = grasp_geometry(c.shapes, BOX_G2)
    ref = place_and_grasp(c, "box_g2", BOX_G2, geo, G4_MASS, 0.0)
    lift(c)
    run_for(c, 1.0)
    w0, s0 = c.wall, c.steps
    c.run.start()
    run_for(c, 5.0)
    rows = c.run.stop()
    q = np.array([r[1] for r in rows])
    last = np.array([r[0] for r in rows]) >= rows[-1][0] - 4.0
    arm = np.degrees(q[last][:, c.ai].max(axis=0) - q[last][:, c.ai].min(axis=0))
    grip = np.degrees(q[last][:, c.fi].max(axis=0) - q[last][:, c.fi].min(axis=0))
    op = np.array([r[4] for r in rows])[last]
    g = np.array([slip_rot(r, ref)[0] for r, keep in zip(rows, last) if keep])
    fc = finger_contacts(c)
    N = [fc[n][1] for n in FINGERS]
    mf = float(c.cfg["gripper"][ts.GRIPPER_DRIVE]["max_force"])
    slip = float(np.linalg.norm(slip_rot(rows[-1], ref)[0])) * 1000   # 잡은 직후 대비 (들어 올림 포함)
    return dict(N=N, trans=sum(N) * grip_lever(float(q[-1, c.gi])) / mf, slip=slip,
                rise=(rows[-1][4][2] - ref["box_z0"]) * 1000,
                arm=float(arm.max()), arm_joint=ts.ARM_JOINTS[int(arm.argmax())], grip=float(grip.max()),
                box=float((op.max(axis=0) - op.min(axis=0)).max()) * 1000,
                box_rel=float((g.max(axis=0) - g.min(axis=0)).max()) * 1000,
                nan=not np.isfinite(q).all(), ms=1000 * (c.wall - w0) / max(1, c.steps - s0))


# ─────────────────────────── main ───────────────────────────
def pf(ok):
    return "P" if ok else "F"


def friction_override(pair):
    """'정지/운동' (예: '0.6/0.5') → 손가락·박스 재질 둘 다 바꾸는 설정 덮어쓰기 문자열 (합치는 방식 min 이라 둘 다 바꿔야 함)."""
    s, d = (float(x) for x in pair.split("/"))
    if not 0 < d <= s:
        raise ValueError(f"--frictions 는 '정지/운동', 0 < 운동 ≤ 정지: {pair}")
    return "".join(f"@contact.{m}.static_friction={s}@contact.{m}.dynamic_friction={d}"
                   for m in ("finger_material", "object_material"))


def run_config(path, L):
    """설정 하나로 씬을 만들고 고른 시험을 돌린다. 요약 dict 를 돌려준다."""
    S = {}
    cfg = ts.load_config(path)
    gm = cfg["gripper"][ts.GRIPPER_DRIVE]
    fmat, omat = cfg["contact"]["finger_material"], cfg["contact"]["object_material"]
    S["mu"] = (fmat["static_friction"], fmat["dynamic_friction"])
    L("")
    L(f"===== {os.path.basename(path)} =====")
    L(f"  그리퍼 maxForce {gm['max_force']} Nm, armature {gm['armature']}, 손가락 재질 {fmat}, 박스 재질 {omat}")
    c = open_scene(cfg)
    try:
        for ln in c.verify_lines:
            L("  " + ln)
        L("  " + c.calib)
        L(f"  기준 자세 {np.round(arm_at(0.0), 4).tolist()} rad, 그리퍼 base {np.round(c.P0, 4).tolist()} m")
        if "K" in TESTS:
            test_k(c, L)

        if "G1" in TESTS:
            s = test_g1(c)
            ok = s["slip"] < CRIT["slip_mm"] and s["wobble"] < CRIT["wobble_mm"] and s["rise"] > LIFT * 1000 * 0.95
            S["G1"] = dict(s, ok=ok)
            L("")
            L(f"[G1] 40 mm 정육면체 {G1_MASS} kg, 20 cm 들어 3 초: 미끄러짐 최대 mm / 떨림(마지막 2 초) mm / 회전 ° / 올라간 높이 mm"
              f"  [기준 < {CRIT['slip_mm']} mm, < {CRIT['wobble_mm']} mm]")
            L(f"  {s['slip']:.3f} / {s['wobble']:.4f} / {s['rot']:.3f} / {s['rise']:.1f}  "
              f"(잡은 각도 {s['q_grip']:.2f}°, 기하 예상 {s['q_pred']:.2f}°) {pf(ok)}")

        if "G2" in TESTS:
            res, geo = test_g2(c)
            S["G2"] = res
            L("")
            L(f"[G2] 60×160×100 mm 박스, 무게중심 긴 방향 오프셋, 들어 올린 뒤 wrist_1·wrist_2 각각 ±30°"
              f"  [미끄러짐 < {CRIT['slip_mm']} mm, 잡는 축 회전 < {CRIT['rot_deg']}°]")
            L(f"  잡은 각도 기하 예상 {math.degrees(geo['qc']):.2f}°. 칸: 미끄러짐 mm / 잡는 축 회전 ° / 전체 회전 °")
            L(f"  {'kg':>4s} {'cm':>3s} {'잡은 각도':>8s} {'들어 올림':>22s} {'wrist_1 ±30°':>22s} {'wrist_2 ±30°':>22s} {'전체':>22s}")
            for s in res:
                ok = s["slip"] < CRIT["slip_mm"] and s["rot_y"] < CRIT["rot_deg"] and s["rise"] > LIFT * 1000 * 0.9
                cell = lambda d: f"{d['slip']:6.3f}/{d['rot_y']:6.3f}/{d['rot']:6.3f}"  # noqa: E731
                must = " (참고)" if s["mass"] == 0.8 else ""
                L(f"  {s['mass']:4.1f} {s['dx'] * 100:3.0f} {s['q_grip']:7.2f}° {cell(s['lift']):>22s} "
                  f"{cell(s['per']['wrist_1_joint']):>22s} {cell(s['per']['wrist_2_joint']):>22s} {cell(s):>22s} {pf(ok)}{must}")
            a = res[0]["axis"]
            L(f"  기울임 축과 잡는 축(그리퍼 y) 사이 각도: wrist_1 {a['wrist_1_joint']:.1f}°, wrist_2 {a['wrist_2_joint']:.1f}° "
              f"(0° = 잡는 축 둘레로 기울임)")

        if "G3" in TESTS:
            res = test_g3(c)
            S["G3"] = res
            L("")
            L(f"[G3] 당김: G2 박스 {G3_MASS} kg, 공구 축을 따라 손가락 밖으로 {G3_RATE} N/s 램프 → 미끄러짐 {SLIP_JUDGE * 1000:.0f} mm 순간")
            L("  미끄럼 힘(당김 방향 합계) = 가한 힘 + 박스 무게의 당김 방향 성분 (공구가 아래를 향하면 = 가한 힘 + m·g)."
              " 실물: 러기지 스케일 값 × 9.81 + 박스 무게")
            L("  파지 수직력 = 당기기 직전 손가락별 PhysX 접촉 수직력 합 (r2 / l2). 전달률 = 수직력 합 × 파지면 이동비 / maxForce"
              " (가상일, 100% = solver 수렴). 미끄럼 힘 / 수직력 합 ≈ 마찰계수")
            L("  들어 올림 미끄러짐 = 잡은 직후 → 당기기 직전. 당김 미끄러짐은 당기기 직전 기준")
            L(f"  {'maxForce Nm':>11s} {'잡은 각도':>9s} {'파지 수직력 N':>15s} {'전달률':>6s} {'들어올림 mm':>10s} {'0.1 mm 시작 N':>13s}"
              f" {'가한 힘 N':>9s} {'무게 성분 N':>11s} {'미끄럼 힘 N':>11s} {'힘/N합':>7s}")
            for r in res:
                if sum(r["N"]) <= 0.0:  # 당기기 전에 이미 손가락에서 빠짐 → 당김 값은 의미 없음
                    L(f"  {r['mf']:11.2f} {r['q_grip']:8.2f}°  들어 올리다 놓침 (들어올림 미끄러짐 {r['lift_slip']:.1f} mm)")
                    continue
                F_first = f"{r['F_first']:.2f}" if r["F_first"] is not None else "-"
                tot = f"{r['total']:.2f}" if r["slipped"] else f"> {r['total']:.1f}"
                mu = r["total"] / max(1e-9, sum(r["N"]))
                L(f"  {r['mf']:11.2f} {r['q_grip']:8.2f}° {r['N'][0]:7.2f} / {r['N'][1]:5.2f} {r['trans'] * 100:5.0f}% "
                  f"{r['lift_slip']:10.3f} {F_first:>13s} {r['F']:9.2f} {r['w_axis']:11.2f} {tot:>11s} {mu:7.3f}")

        if "G4" in TESTS:
            s = test_g4(c)
            held = s["rise"] > LIFT * 1000 * 0.9 and s["slip"] < CRIT["slip_mm"]
            ok = (not s["nan"]) and held and s["arm"] < CRIT["jitter_deg"] and s["box"] < CRIT["wobble_mm"]
            S["G4"] = dict(s, held=held, ok=ok)
            L("")
            L(f"[G4] G2 박스 {G4_MASS} kg 을 들고 5 초 (마지막 4 초): 팔 떨림 ° / 손가락 떨림 ° / 박스 떨림 mm (world) / 박스 떨림 mm (그리퍼 기준)"
              f" / 스텝당 계산 ms  [< {CRIT['jitter_deg']}°, < {CRIT['wobble_mm']} mm]")
            L(f"  {s['arm']:.5f} ({s['arm_joint']}) / {s['grip']:.5f} / {s['box']:.4f} / {s['box_rel']:.4f} / {s['ms']:.2f} ms {pf(ok)}")
            L(f"  들고 있음 {held} (박스 올라간 높이 {s['rise']:.1f} mm, 잡은 직후 대비 미끄러짐 {s['slip']:.3f} mm [< {CRIT['slip_mm']} mm]),"
              f" 끝에서 파지 수직력 {s['N'][0]:.2f} / {s['N'][1]:.2f} N, 전달률 {s['trans'] * 100:.0f}%")

        S["ms"] = 1000 * c.wall / max(1, c.steps)
        L("")
        L(f"[계산 시간] 전체 {S['ms']:.2f} ms/step ({c.steps} 스텝)")
    finally:
        close_scene(c)
    return S


def summary_table(results, L):
    """마찰 조합 비교 요약 (--frictions 로 여러 설정을 돌렸을 때)."""
    L("")
    L("===== 요약: 마찰 조합 비교 =====")
    L("  G3 칸 = 유효 마찰(미끄럼 힘 / 수직력 합) · 들어올림 미끄러짐 mm · 미끄럼 힘 N  ('놓침' = 당기기 전에 빠짐)")
    L(f"  G2 칸 = 가장 나쁜 경우 미끄러짐 mm / 잡는 축 회전 °,  G4 = {G4_MASS} kg 들고 있음 · 잡은 직후 대비 미끄러짐 mm")
    for _, S in results:
        L(f"  --- 정지/운동 {S['mu'][0]}/{S['mu'][1]}")
        if "G1" in S:
            g = S["G1"]
            L(f"    G1 미끄러짐 {g['slip']:.3f} mm, 떨림 {g['wobble']:.4f} mm {pf(g['ok'])}")
        if "G3" in S:
            cells = []
            for r in S["G3"]:
                if sum(r["N"]) <= 0.0:
                    cells.append(f"{r['mf']:.2f}Nm 놓침")
                else:
                    cells.append(f"{r['mf']:.2f}Nm {r['total'] / sum(r['N']):.3f}·{r['lift_slip']:.2f}·"
                                 f"{r['total']:.1f}{'' if r['slipped'] else '+'}")
            L("    G3 " + " | ".join(cells))
        if "G2" in S:
            for mass in G2_MASSES:
                rs = [s for s in S["G2"] if s["mass"] == mass]
                w = max(rs, key=lambda s: s["slip"])
                n_ok = sum(s["slip"] < CRIT["slip_mm"] and s["rot_y"] < CRIT["rot_deg"] and s["rise"] > LIFT * 1000 * 0.9
                           for s in rs)
                L(f"    G2 {mass} kg: 통과 {n_ok}/{len(rs)}, 가장 나쁜 경우 (무게중심 {w['dx'] * 100:.0f} cm) "
                  f"{w['slip']:.3f} mm / {w['rot_y']:.3f}°")
        if "G4" in S:
            g = S["G4"]
            L(f"    G4 들고 있음 {g['held']}, 미끄러짐 {g['slip']:.3f} mm, 떨림 {g['box']:.4f} mm {pf(g['ok'])}")


def main():
    lines = []

    def L(m=""):
        print(m, flush=True)
        lines.append(m)

    L(f"grasp_tests  {datetime.datetime.now().isoformat(timespec='seconds')}  physics dt = {ts.PHYSICS_DT:.5f} s  "
      f"시험 {','.join(TESTS)}")
    paths = [args.config + friction_override(p) for p in args.frictions] if args.frictions else [args.config]
    results = [(p.split("@", 1)[-1] if "@" in p else os.path.basename(p), run_config(p, L)) for p in paths]
    if len(results) > 1:
        summary_table(results, L)

    os.makedirs(ts.REPORT_DIR, exist_ok=True)
    path = os.path.join(ts.REPORT_DIR, f"grasp_tests_{datetime.datetime.now():%Y%m%d_%H%M%S}.txt")
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    print(f"report: {path}", flush=True)


if __name__ == "__main__":
    code = 0
    try:
        main()
    except Exception:
        import traceback
        traceback.print_exc()
        code = 1
    finally:
        simulation_app.close(exit_code=code)
    sys.exit(code)
