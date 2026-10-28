# =============================================================
# moving_base_scene.py — 움직이는 베이스 씬: Husky A200 위 UR5 (PLAN 8단계). 고정 베이스 씬 (drone_scene.py) 과 별도
#
#   바닥 + 조명 + Husky (kinematic 직선 왕복, moving_base.KinematicBase) + UR5 (USD 원본 구조 = floating, PLAN 8단계 방식 A:
#   root_joint 의 body0 을 Husky 차체로 바꿔 윗판 위 마운트 위치에 고정) + 손목 카메라 + 드론 (world 고정 호버, 기하 제어기)
#   + third view 카메라 (world 고정, 고정 씬과 같은 base 기준 위치). 테이블 없음
#   설정: config/scene_husky.yaml (+ base_husky.yaml). scene_drone.yaml 은 읽기만 (시작 자세·조명)
#
# 실행 (레포 최상위):
#   ~/isaacsim/python.sh isaacsim/scripts/moving_base_scene.py                    # GUI (Husky 왕복, 창을 닫으면 끝)
#   ~/isaacsim/python.sh isaacsim/scripts/moving_base_scene.py --headless --duration 30
#   ~/isaacsim/python.sh isaacsim/scripts/moving_base_scene.py --headless --check     # 판정 C1~C6 → PASS / FAIL (리포트·이미지 isaacsim/reports/)
#     C1 Husky 실제 위치 = 궤적, C2 robot_mount = 마운트 위치, C3 팔 자세 유지, C4 팔 ↔ Husky 접촉 없음, C5 드론 호버, C6 카메라 영상
#   옵션: --scene-config <yaml>, --duration S (sim s, 0 = 창을 닫을 때까지), --prop-spin on|off, --periods N (--check 왕복 횟수)
#
# 다른 스크립트는 SimulationApp 을 만든 뒤 import moving_base_scene → build_scene(...).
# =============================================================

import argparse
import math
import os
import sys
import traceback

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import numpy as np  # noqa: E402
import yaml  # noqa: E402

import test_scene as ts  # noqa: E402

DEFAULT_SCENE_CONFIG = os.path.join(ts.CONFIG_DIR, "scene_husky.yaml")
SCENE_KEYS = ("base_config", "fixed_scene_config", "mount", "init_pose", "drone_base", "drone_flight",
              "third_view_camera_config", "collision_filter")


def load_scene_config(path=None):
    path = path or DEFAULT_SCENE_CONFIG
    with open(path, encoding="utf-8") as f:
        sc = yaml.safe_load(f)
    for k in SCENE_KEYS:
        if k not in sc:
            raise KeyError(f"{path}: '{k}' 항목이 없습니다")
    for k in ("xyz", "yaw_deg"):
        if k not in sc["mount"]:
            raise KeyError(f"{path}: 'mount.{k}' 항목이 없습니다")
    if sc["drone_flight"] != "geometric":
        raise ValueError(f"{path}: drone_flight 는 지금 geometric 만 (PX4 이륙은 이 씬에 아직 없음): {sc['drone_flight']}")
    sc["path"] = path
    return sc


def _rz(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _yaw_quat(yaw):
    return [math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)]


def find_root_joint(stage, mount_path):
    """robot_mount 를 body1 로 갖는 고정 관절 (USD 원본의 root_joint) 하나."""
    from pxr import Usd, UsdPhysics

    found = []
    for p in Usd.PrimRange(stage.GetPrimAtPath(ts.ROBOT_PATH)):
        if p.IsA(UsdPhysics.FixedJoint):
            b1 = UsdPhysics.Joint(p).GetBody1Rel().GetTargets()
            if [str(t) for t in b1] == [mount_path]:
                found.append(p)
    if len(found) != 1:
        raise RuntimeError(f"root_joint (body1 = {mount_path}) 가 하나가 아님: {[str(p.GetPath()) for p in found]}")
    return UsdPhysics.FixedJoint(found[0])


def attach_robot(stage, base, mount_xyz, mount_yaw):
    """방식 A: root_joint 의 body0 을 Husky 차체로, 차체 좌표계의 마운트 위치에 robot_mount 를 고정 (메모리 stage 만)."""
    from pxr import Gf

    j = find_root_joint(stage, ts.MOUNT_PATH)
    j.GetBody0Rel().SetTargets([base.body_path])
    j.CreateLocalPos0Attr().Set(Gf.Vec3f(*[float(x) for x in mount_xyz]))
    q = _yaw_quat(mount_yaw)
    j.CreateLocalRot0Attr().Set(Gf.Quatf(q[0], Gf.Vec3f(*q[1:])))
    j.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    j.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, Gf.Vec3f(0.0, 0.0, 0.0)))
    # articulation 밖 구속 (maximal joint) 으로 둔다. 없으면 USD Physics 해석기가 이 관절을 따라 Husky 차체까지
    # UR5 articulation 의 링크로 흡수한다 (OpenUSD usdPhysics parseUtils.cpp _TraverseHierarchy: 상대 body 가 강체이고
    # 관절이 켜져 있으면 링크로 따라 들어감, excludeFromArticulation 이면 따라가지 않음).
    # 흡수된 차체는 kinematic 궤적을 따르지 않고 바닥으로 내려앉아 제자리에 남았다 (2026-10-28 실험: robot_mount 높이
    # 0.384 → 0.2515 = 차체 높이 0.132 만큼 내려감, y 고정). CLAUDE.md 의 시험 하중 FixedJoint 흡수와 같은 현상
    j.CreateExcludeFromArticulationAttr().Set(True)
    return str(j.GetPath())


def filter_base_collisions(stage, base, robot_links):
    """UR5 base 쪽 링크 ↔ Husky 강체 충돌 끔 (UsdPhysics.FilteredPairsAPI, 메모리 stage 만).
    Husky 쪽 prim 에 적는다 (충돌 제외는 양방향): 로봇 안의 제외 쌍은 ts.verify_articulation 이 설정과 같은지 보므로 건드리지 않음."""
    from pxr import Usd, UsdPhysics

    links = []
    for name in robot_links:
        path = ts.find_link_path(stage, name)
        if not stage.GetPrimAtPath(path).HasAPI(UsdPhysics.RigidBodyAPI):
            raise RuntimeError(f"collision_filter: {path} 는 강체가 아님")
        links.append(path)
    husky_bodies = [p for p in Usd.PrimRange(stage.GetPrimAtPath(base.path)) if p.HasAPI(UsdPhysics.RigidBodyAPI)]
    for p in husky_bodies:
        UsdPhysics.FilteredPairsAPI.Apply(p).CreateFilteredPairsRel().SetTargets(links)
    return links, len(husky_bodies)


class Scene:
    pass


def build_scene(scene_config=None, prop_spin=False, seed=0, pre_play=None):
    """씬을 만들고 재생까지 한다. Husky 는 start_base() 를 부를 때 움직이기 시작한다. Scene 을 돌려준다.
    pre_play(s): 재생 직전에 부를 함수 (접촉 보고·카메라 센서 등)."""
    import isaacsim.core.experimental.utils.app as app_utils
    from isaacsim.core.experimental.objects import DomeLight
    from isaacsim.core.experimental.prims import XformPrim

    import drone as dr
    import drone_flight as fl
    import drone_scene as ds
    import moving_base as mb
    from robot_drive import RobotDrive

    s = Scene()
    s.scene_cfg = load_scene_config(scene_config)
    s.base_cfg = mb.load_base_config(ts._abs(s.scene_cfg["base_config"]))
    fixed = ds.load_scene_config(ts._abs(s.scene_cfg["fixed_scene_config"]))      # 읽기만
    s.robot_cfg = ts.use_robot()
    s.drive_cfg = ts.load_config()
    s.cam_cfg = ts.load_camera_config()
    s.drone_cfg = dr.load_drone_config(dr.DEFAULT_DRONE_CONFIG)
    tv = ds.load_third_view_config(ts._abs(s.scene_cfg["third_view_camera_config"]))

    # 배치: 로봇 base 는 Husky 궤적 가운데에서 world (0, 0), 정면 +x. Husky 는 거기서 마운트 위치를 거꾸로 따라가 놓는다
    mount_xyz = np.asarray(s.scene_cfg["mount"]["xyz"], dtype=float)
    mount_yaw = math.radians(float(s.scene_cfg["mount"]["yaw_deg"]))
    husky_yaw = -mount_yaw                                  # 로봇 yaw 0 = Husky yaw + 마운트 yaw
    tr = s.base_cfg["trajectory"]
    d = np.asarray(tr["direction"], dtype=float)
    dir_world = _rz(husky_yaw) @ (d / np.linalg.norm(d))
    mid_robot = np.zeros(3)
    start_robot = mid_robot - 0.5 * float(tr["distance"]) * dir_world   # 궤적이 가운데 기준 ± distance/2 가 되게
    husky_xy = start_robot[:2] - (_rz(husky_yaw) @ mount_xyz)[:2]
    s.base_z = float(s.base_cfg["base_link_height"]) + float(mount_xyz[2])        # 로봇 base 높이 (바닥 위)
    start_robot[2] = s.base_z
    s.mid_robot = np.array([0.0, 0.0, s.base_z])

    # 로봇 (새 stage, PhysX, ground). base="floating" = USD 원본 구조 (articulation root = robot_mount, root_joint 는 바깥 구속)
    s.stage, s.robot, s.robot_info = ts.build_test_scene(ts.DEFAULT_USD, ts.PHYSICS_VARIANT, base_z=s.base_z, base="floating",
                                                         config=s.drive_cfg, ground=True, light=False)
    XformPrim(ts.ROBOT_PATH).set_world_poses(positions=[start_robot.tolist()], orientations=[_yaw_quat(0.0)])
    DomeLight("/World/DomeLight").set_intensities(float(fixed["light"]["dome_intensity"]))

    s.base = mb.KinematicBase(s.base_cfg, start_xy=husky_xy, start_yaw_deg=math.degrees(husky_yaw))
    s.base_info = s.base.add(s.stage)
    s.root_joint = attach_robot(s.stage, s.base, mount_xyz, mount_yaw)
    s.filtered, n_husky = filter_base_collisions(s.stage, s.base, s.scene_cfg["collision_filter"]["robot_links"])
    print(f"[moving_base_scene] Husky {s.base_info}, UR5 root_joint {s.root_joint} → {s.base.body_path} "
          f"(마운트 {mount_xyz.tolist()}, yaw {math.degrees(mount_yaw):+.1f}°), 충돌 끔: {len(s.filtered)} 링크 × Husky {n_husky}",
          flush=True)

    s.wrist_cam = ts.add_wrist_camera(s.stage, s.cam_cfg, tilt=0.0)
    # third view: 설정 world 값은 테이블 위 base (z = table.top_z) 기준 → 이 씬의 base 높이로 옮기고 받침대는 바닥부터
    dz = s.base_z - float(fixed["table"]["top_z"])
    tv = dict(tv, position=[tv["position"][0], tv["position"][1], tv["position"][2] + dz],
              look_at=[tv["look_at"][0], tv["look_at"][1], tv["look_at"][2] + dz])
    s.third_cam, s.mount_h = ds.add_third_view_camera(s.stage, tv, 0.0)

    s.drone_pos = s.mid_robot + np.asarray(s.scene_cfg["drone_base"], dtype=float)
    s.drone_info = dr.add_drone(s.stage, s.drone_cfg, position=s.drone_pos, prop_spin=prop_spin)
    s.pre_play = pre_play(s) if pre_play is not None else None

    app_utils.play()
    ds._update()
    problems, _ = ts.verify_articulation(s.stage, s.robot, s.robot_info, s.drive_cfg)
    if problems:
        raise RuntimeError("로봇 articulation 설정 확인 실패: " + "; ".join(problems))
    s.drive = RobotDrive(s.robot, s.drive_cfg)
    s.drive.apply()
    s.drive.start()
    named = fixed.get("init_poses") or {}
    if s.scene_cfg["init_pose"] not in named:
        raise ValueError(f"init_pose '{s.scene_cfg['init_pose']}' 이 {fixed_path(s)} init_poses 에 없음 ({sorted(named)})")
    s.init_pose = np.asarray(named[s.scene_cfg["init_pose"]], dtype=float)
    s.drive.reset_pose(s.init_pose, gripper=0.0)
    s.flight = fl.DroneFlight(s.drone_cfg, s.drone_info, mode="static", seed=seed, target=s.drone_pos, backend="geometric")
    s.flight.start()
    s.flight.arm()
    return s


def fixed_path(s):
    return s.scene_cfg["fixed_scene_config"]


def mount_gap(s):
    """(robot_mount world 위치, Husky 궤적 + 마운트 위치에서 떨어진 거리 [mm]).
    robot_mount 는 floating articulation 의 root 라 articulation view 로 읽는다
    (재생 중에 로봇 링크로 RigidPrim 을 새로 만들면 PhysX 가 로봇을 다시 읽어 tensor view 가 무효가 됨, 2026-10-28)."""
    p_mount = s.robot._physics_articulation_view.get_root_transforms().numpy()[0, :3].copy()
    p_husky = s.base.target(s.base.t)[0]
    want = p_husky + _rz(s.base.yaw) @ np.asarray(s.scene_cfg["mount"]["xyz"], dtype=float)
    return p_mount, float(np.linalg.norm(p_mount - want) * 1000)


def start_base(s):
    s.base.start()


def check_all(s):
    s.base.check()
    s.drive.check()
    s.flight.check()


def stop_all(s):
    s.base.stop()
    s.flight.stop()
    s.drive.stop()


# ── --check: 판정 (PLAN 8단계) ──
# drone_mm: drone_scene --check 의 geometric 기준과 같음. 드론은 재생 ~ 모터 켬 사이 자유 낙하로 처음 약 57 mm 처졌다가 2 s 안에
#   돌아오므로 (2026-10-28) Husky 를 움직이기 전에 DRONE_SETTLE_SEC 동안 자리를 잡게 하고, 판정은 Husky 가 움직이는 동안만
CRIT = {"husky_mm": 1.0, "mount_mm": 1.0, "mount_deg": 0.1, "arm_deg": 0.5, "contact_n": 1.0, "drone_mm": 10.0,
        "img_min_mean": 10.0, "img_min_diff": 2.0}
DRONE_SETTLE_SEC = 5.0
# 보호 정지 기준 (ros2_iface.yaml protective_stop, 읽기만) 은 리포트에 여유로 같이 적는다
SNAP_FRACS = (0.03, 0.47)        # 한 주기 중 이미지 저장 시각 (시작 근처 = 한쪽 끝, 0.47 = 반대쪽 끝에 멈춘 구간)


def _check_setup(s):
    """재생 전: 로봇 링크 접촉 보고 켜기, 카메라 센서 2 대."""
    from isaacsim.sensors.experimental.rtx import CameraSensor

    import protective_stop as ps

    ps.enable_contact_report(s.stage)
    sensors = {}
    for name, path, res in (("wrist", s.wrist_cam, s.cam_cfg["resolution"]), ("third_view", s.third_cam, (640, 480))):
        w, h = (int(x) for x in res)
        sensors[name] = CameraSensor(path, resolution=(h, w), annotators=["rgb"])
    return sensors


def _quat_to_R(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def _grab(simulation_app, sensor):
    for _ in range(200):
        if sensor.has_data():
            break
        simulation_app.update()
    rgb, _ = sensor.get_data("rgb")
    if rgb is None:
        raise RuntimeError("카메라 데이터를 받지 못함")
    return rgb.numpy()[..., :3].astype(np.uint8)


def run_check(s, simulation_app, periods, report_dir):
    import datetime

    from isaacsim.core.simulation_manager import SimulationManager
    from PIL import Image

    import protective_stop as ps

    out = report_dir or os.path.join(ts.REPORT_DIR, f"moving_base_husky_{datetime.datetime.now():%Y%m%d_%H%M%S}")
    os.makedirs(out, exist_ok=True)
    lines, results = [], []

    def log(msg=""):
        print(msg, flush=True)
        lines.append(msg)

    def result(no, title, ok, why):
        results.append((no, title, ok))
        log(f"[{no}] {title}: {why}\n  => {'PASS' if ok else 'FAIL'}")

    # 읽기용 view: Husky 차체는 강체 view, 로봇 base 는 articulation root, 팔 링크 ↔ Husky 접촉은 접촉 view (모두 sim view 에서 직접.
    #   재생 중에 로봇 링크로 RigidPrim 을 만들면 PhysX 가 로봇을 다시 읽어 view 가 무효가 됨)
    sim_view = SimulationManager._physics_sim_view__warp
    husky_view = sim_view.create_rigid_body_view([s.base.body_path])
    from pxr import Usd, UsdPhysics
    husky_bodies = [str(p.GetPath()) for p in Usd.PrimRange(s.stage.GetPrimAtPath(s.base.path)) if p.HasAPI(UsdPhysics.RigidBodyAPI)]
    arm_links = [p for p in ps.robot_links(s.stage) if p not in s.filtered]      # 충돌을 끈 base 링크는 뺌
    contact_view = sim_view.create_rigid_contact_view(arm_links, filter_patterns=[husky_bodies for _ in arm_links],
                                                      max_contact_data_count=256)
    if husky_view is None or contact_view is None or not contact_view.check():
        raise RuntimeError("검사용 view 를 만들지 못함")
    ros2 = yaml.safe_load(open(os.path.join(ts.CONFIG_DIR, "ros2_iface.yaml"), encoding="utf-8"))["protective_stop"]
    mount_xyz = np.asarray(s.scene_cfg["mount"]["xyz"], dtype=float)
    mount_yaw = math.radians(float(s.scene_cfg["mount"]["yaw_deg"]))
    rv = s.robot._physics_articulation_view
    arm_i = np.asarray(s.drive.arm_i)

    sec = periods * s.base.traj.period
    log(f"[moving_base_scene --check] {datetime.datetime.now():%Y-%m-%d %H:%M:%S}, 설정 {s.scene_cfg['path']}, {sec:.1f} s (sim) = {periods:g} 주기")
    log(f"  마운트 {mount_xyz.tolist()} yaw {math.degrees(mount_yaw):+.1f}°, 궤적 {s.base.traj.distance} m 왕복 최고 {s.base.traj.v_peak} m/s")
    rows, snaps = [], {}
    q_prev, t_prev = None, None
    snap_t = sorted(f * s.base.traj.period for f in SNAP_FRACS)
    # 드론 자리 잡기 (Husky 정지)
    t_end, sag = SimulationManager.get_simulation_time() + DRONE_SETTLE_SEC, 0.0
    while SimulationManager.get_simulation_time() < t_end:
        simulation_app.update()
        check_all(s)
        sag = max(sag, float(np.linalg.norm(s.flight._body_pose()[0] - s.drone_pos)))
    settle_end = float(np.linalg.norm(s.flight._body_pose()[0] - s.drone_pos))
    log(f"  드론 자리 잡기 {DRONE_SETTLE_SEC:g} s (Husky 정지): 처짐 최대 {sag * 1000:.1f} mm → 끝 {settle_end * 1000:.2f} mm (판정 안 함)")
    start_base(s)
    while s.base.t < sec - 1e-9:
        simulation_app.update()
        check_all(s)
        t = s.base.t
        hp = husky_view.get_transforms().numpy()[0]                     # xyz + xyzw
        p_h, q_h = hp[:3].astype(float), hp[[6, 3, 4, 5]].astype(float)
        rt = rv.get_root_transforms().numpy()[0]
        p_m, q_m = rt[:3].astype(float), rt[[6, 3, 4, 5]].astype(float)
        p_cmd = s.base.target(t)[0]
        Rh = _quat_to_R(q_h)
        want_p = p_h + Rh @ mount_xyz
        R_rel = (Rh @ _rz(mount_yaw)).T @ _quat_to_R(q_m)                # 0 이어야 하는 마운트 회전 오차
        ang = math.degrees(math.acos(np.clip((np.trace(R_rel) - 1) / 2, -1, 1)))
        q = rv.get_dof_positions().numpy()[0][arm_i].astype(float)
        qt = rv.get_dof_position_targets().numpy()[0][arm_i].astype(float)
        qd = np.zeros(6) if q_prev is None else (q - q_prev) / max(t - t_prev, 1e-9)
        q_prev, t_prev = q, t
        f = np.linalg.norm(contact_view.get_contact_force_matrix(ts.PHYSICS_DT).numpy(), axis=2).sum(axis=1)
        p_d = s.flight._body_pose()[0]
        rows.append(dict(t=t, husky_err=np.linalg.norm(p_h - p_cmd), mount_err=np.linalg.norm(p_m - want_p), mount_ang=ang,
                         arm_err=np.abs(q - s.init_pose).max(), track=np.abs(qt - q).max(), speed=np.abs(qd).max(),
                         contact=float(f.max()), contact_link=arm_links[int(f.argmax())], drone_err=np.linalg.norm(p_d - s.drone_pos),
                         nan=not np.all(np.isfinite(np.concatenate([p_h, p_m, q]))), y=p_m[1]))
        if snap_t and t >= snap_t[0]:
            k = f"t{snap_t.pop(0):04.1f}"
            snaps[k] = {name: _grab(simulation_app, sen) for name, sen in s.pre_play.items()}
            for name, img in snaps[k].items():
                Image.fromarray(img).save(os.path.join(out, f"{name}_{k}.png"))
    stop_all(s)

    a = {k: np.array([r[k] for r in rows]) for k in ("t", "husky_err", "mount_err", "mount_ang", "arm_err", "track", "speed",
                                                     "contact", "drone_err", "y")}
    log(f"  샘플 {len(rows)} 개, 로봇 base y {a['y'].min():+.4f} ~ {a['y'].max():+.4f} m")
    result("C1", "Husky 실제 위치 = 궤적 (PhysX)", a["husky_err"].max() * 1000 < CRIT["husky_mm"],
           f"오차 최대 {a['husky_err'].max() * 1000:.3f} mm (기준 < {CRIT['husky_mm']})")
    result("C2", "UR5 robot_mount = Husky 마운트 위치", a["mount_err"].max() * 1000 < CRIT["mount_mm"] and a["mount_ang"].max() < CRIT["mount_deg"],
           f"위치 최대 {a['mount_err'].max() * 1000:.3f} mm, 회전 최대 {a['mount_ang'].max():.4f}° (기준 < {CRIT['mount_mm']} mm, {CRIT['mount_deg']}°)")
    nan = any(r["nan"] for r in rows)
    result("C3", "팔 시작 자세 유지", (not nan) and math.degrees(a["arm_err"].max()) < CRIT["arm_deg"]
           and math.degrees(a["track"].max()) < float(ros2["tracking_error_deg"]) and math.degrees(a["speed"].max()) < float(ros2["joint_speed_dps"]),
           f"자세 오차 최대 {math.degrees(a['arm_err'].max()):.3f}° (기준 < {CRIT['arm_deg']}), 목표−측정 최대 {math.degrees(a['track'].max()):.3f}° "
           f"(보호 정지 {ros2['tracking_error_deg']}), 관절 속도 최대 {math.degrees(a['speed'].max()):.2f} °/s (보호 정지 {ros2['joint_speed_dps']}), NaN {nan}")
    i = int(a["contact"].argmax())
    result("C4", "팔 ↔ Husky 접촉 없음", a["contact"].max() < CRIT["contact_n"],
           f"접촉력 최대 {a['contact'].max():.3f} N ({rows[i]['contact_link'].split('/')[-1]}, t {rows[i]['t']:.2f}) (기준 < {CRIT['contact_n']} N, "
           f"충돌을 끈 base 링크 {len(s.filtered)} 개 제외)")
    result("C5", "드론 호버 유지 (world 고정, Husky 가 움직이는 동안)", a["drone_err"].max() * 1000 < CRIT["drone_mm"],
           f"목표에서 최대 {a['drone_err'].max() * 1000:.2f} mm (기준 < {CRIT['drone_mm']}, drone_scene --check geometric 과 같음)")
    keys = sorted(snaps)
    cam_ok, why = len(keys) == 2, []
    for name in ("wrist", "third_view"):
        if len(keys) != 2:
            break
        m0, m1 = (float(snaps[k][name].mean()) for k in keys)
        diff = float(np.abs(snaps[keys[0]][name].astype(int) - snaps[keys[1]][name].astype(int)).mean())
        ok = min(m0, m1) > CRIT["img_min_mean"] and diff > CRIT["img_min_diff"]
        cam_ok &= ok
        why.append(f"{name} 밝기 {m0:.0f}/{m1:.0f}, {keys[0]}↔{keys[1]} 차이 {diff:.1f}")
    result("C6", "카메라 영상 (검지 않음, 베이스가 움직이면 바뀜)", cam_ok,
           "; ".join(why) + f" (기준 밝기 > {CRIT['img_min_mean']}, 차이 > {CRIT['img_min_diff']})" if why else "이미지 저장 시각에 닿지 않음")
    n_pass = sum(ok for *_, ok in results)
    log("")
    log("=== 요약 ===")
    for no, title, ok in results:
        log(f"  {no} {title}: {'PASS' if ok else 'FAIL'}")
    log(f"  {n_pass}/{len(results)} PASS")
    log(f"  리포트: {out}")
    with open(os.path.join(out, "report.txt"), "w", encoding="utf-8") as fh:
        fh.write("\n".join(lines) + "\n")
    np.savez(os.path.join(out, "log.npz"), **a)
    return n_pass == len(results)


def main():
    p = argparse.ArgumentParser(description="움직이는 베이스 씬: Husky 위 UR5 (PLAN 8단계)")
    p.add_argument("--headless", action="store_true")
    p.add_argument("--scene-config", default=None)
    p.add_argument("--duration", type=float, default=0.0, help="[sim s] 0 = 창을 닫을 때까지 (GUI)")
    p.add_argument("--prop-spin", choices=("on", "off"), default=None, help="기본 GUI on, headless off")
    p.add_argument("--check", action="store_true", help="한 주기 돌며 판정 (C1~C6) → PASS / FAIL, 리포트 isaacsim/reports/")
    p.add_argument("--periods", type=float, default=1.0, help="--check 왕복 횟수")
    p.add_argument("--report-dir", default=None)
    args = p.parse_args()
    if args.headless and args.duration <= 0 and not args.check:
        p.error("--headless 에서는 --duration 또는 --check 가 필요")

    from isaacsim import SimulationApp

    simulation_app = SimulationApp({"headless": args.headless})

    prop = (args.prop_spin or ("off" if args.headless else "on")) == "on"
    ok = False
    try:
        if args.check:
            s = build_scene(args.scene_config, prop_spin=prop, pre_play=_check_setup)
            ok = run_check(s, simulation_app, args.periods, args.report_dir)
            return
        s = build_scene(args.scene_config, prop_spin=prop)
        start_base(s)
        last = -1.0
        while simulation_app.is_running() and (args.duration <= 0 or s.base.t < args.duration):
            simulation_app.update()
            check_all(s)
            if s.base.t - last >= 1.0:
                last = s.base.t
                p_mount, gap_mm = mount_gap(s)
                q = s.robot.get_dof_positions().numpy()[0][s.drive.arm_i]
                print(f"[moving_base_scene] t {s.base.t:5.1f} s  Husky {np.round(s.base.target(s.base.t)[0][:2], 4).tolist()}  "
                      f"robot_mount {np.round(p_mount, 4).tolist()} (마운트 위치와 {gap_mm:.3f} mm)  "
                      f"팔 오차 최대 {math.degrees(np.abs(q - s.init_pose).max()):.3f}°", flush=True)
        stop_all(s)
        ok = True
    except Exception:
        traceback.print_exc()        # close() 가 프로세스를 끝내 traceback 이 묻히지 않게
        raise
    finally:
        simulation_app.close(exit_code=0 if ok else 1)


if __name__ == "__main__":
    main()
