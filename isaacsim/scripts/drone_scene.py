#!/usr/bin/env python3
# =============================================================
# drone_scene.py  (docs/PLAN.md 2-3)
#
# 드론 파지 씬: 스크립트 한 번으로 만든다 (ROS 없음). build_scene() 은 2-4 파지 데모·3단계 ROS 브리지도 그대로 쓴다
#   (다른 스크립트는 SimulationApp 을 만든 뒤 import drone_scene → build_scene(...). 인자·app 생성은 이 파일의 main 에서만).
#   바닥 + 테이블(박스 collider) + 로봇(로봇 설정 yaml, fixed base, 확정 drive, 홈 자세) + Dome Light(headless 에서도)
#   + 손목 카메라(1-7) + third view 카메라(base 근처에서 올려다봄. 모양 = 로봇 USD 의 D435i 메시, 받침대 포함)
#   + 비행하는 드론(drone_flight, Pegasus 방식)
#   로봇·드론 USD 원본은 수정하지 않는다 (모두 메모리 stage 의 씬 레이어)
#
# 실행:
#   GUI:  ~/isaacsim/python.sh ~/data_collection_ur5_gripper/isaacsim/scripts/drone_scene.py [--mode hover] [--prop-spin on]
#         (손목·third view 카메라 창을 같이 띄움, 실제 시간 속도. 창을 닫으면 종료)
#   점검: ~/isaacsim/python.sh ~/data_collection_ur5_gripper/isaacsim/scripts/drone_scene.py --headless --check
#         (CHECK_SEC 동안 돌린 뒤 로봇 자세 유지·드론 비행·카메라 영상 확인, 영상 PNG 와 리포트를 isaacsim/reports/ 에)
#   인자: --robot-config --scene-config --drone-config --drone-pos x y z --mode static|hover --seed --prop-spin on|off --base fixed
#         --init-pose q1..q6 (팔 6 관절 시작 각도 [rad], 기본 drive 설정 home_pose. 그리퍼는 항상 완전 열림으로 시작)
#   예: 그리퍼가 +z 를 보고 드론(기본 위치) 바로 아래 약 21 cm (손목 카메라로 드론이 보이는지 확인)
#         --init-pose -0.1888 -0.7854 0.9599 1.3963 -1.5708 0.1888
#   미구현: --mode trajectory, --base kinematic (8단계) → 에러로 중단
# =============================================================

import argparse
import datetime
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import test_scene as ts  # noqa: E402

DEFAULT_SCENE_CONFIG = os.path.join(ts.CONFIG_DIR, "scene_drone.yaml")
BASE_CHOICES = ("fixed", "kinematic")

import numpy as np  # noqa: E402
import yaml  # noqa: E402

import drone as dr  # noqa: E402
import drone_flight as fl  # noqa: E402

TABLE_PATH = "/World/Table"
THIRD_VIEW_ROOT = "/World/ThirdView"
THIRD_VIEW_NAME = "third_view_color_camera"
CHECK_SEC = 10.0
# drone_err_mm: geometric 은 정답 상태로 mm 단위 유지, px4 는 PX4 위치 유지 흔들림 (모션캡처 호버 RMS 19 mm, 최대 약 33 mm, 2026-10-02)
CRIT = {"arm_dev_deg": 0.1, "drone_err_mm": {"geometric": 10.0, "px4": 50.0}, "drone_tilt_deg": 2.0, "img_mean": (10.0, 245.0),
        "img_std": 5.0}


def _update():
    """SimulationApp.update() 와 같음 (이 모듈은 app 객체를 갖지 않음)."""
    import omni.kit.app

    omni.kit.app.get_app().update()


def _running():
    import omni.kit.app

    return omni.kit.app.get_app().is_running()


def parse_waypoint(text):
    """'x,y,z' → [x, y, z] (argparse type)."""
    try:
        v = [float(t) for t in text.split(",")]
    except ValueError:
        v = []
    if len(v) != 3:
        raise argparse.ArgumentTypeError(f"waypoint 는 'x,y,z' (m): {text}")
    return v


def make_parser():
    parser = argparse.ArgumentParser(description="드론 파지 씬 (PLAN 2-3)")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--robot-config", default=ts.DEFAULT_ROBOT_CONFIG, help="로봇 설정 yaml (기본 config/robot_ur5_rh_p12.yaml)")
    parser.add_argument("--scene-config", default=DEFAULT_SCENE_CONFIG, help="씬 배치 yaml (기본 config/scene_drone.yaml)")
    parser.add_argument("--drone-config", default=None, help="드론 설정 yaml (기본 config/drone_iris.yaml)")
    parser.add_argument("--drone-pos", type=float, nargs=3, default=None, metavar=("X", "Y", "Z"),
                        help="드론 위치 [m] world (기본 scene 설정 drone_pos)")
    parser.add_argument("--mode", default="static", choices=("static", "hover", "trajectory"), help="드론 비행 모드")
    parser.add_argument("--seed", type=int, default=0, help="hover 사인파 seed (px4: 센서 잡음 seed)")
    parser.add_argument("--flight", default=None, choices=fl.BACKENDS, help="드론 제어기 (기본: 드론 설정 flight.backend)")
    parser.add_argument("--position-source", default=None, choices=("mocap", "flow", "gps"), help="px4 위치 정보 (기본: px4_sitl.yaml)")
    parser.add_argument("--drone-waypoints", nargs="+", default=None, metavar="X,Y,Z", type=parse_waypoint,
                        help="px4: 이륙 뒤 차례로 들를 위치 (world m, 'x,y,z'). 첫 점 높이까지 수직 상승 → 각 점 → --drone-pos 에서 호버")
    parser.add_argument("--prop-spin", choices=("on", "off"), default="off", help="프로펠러를 보여 주기용으로 돌림")
    parser.add_argument("--base", default="fixed", choices=BASE_CHOICES, help="로봇 베이스 (kinematic 은 8단계, 미구현)")
    parser.add_argument("--init-pose", type=float, nargs=6, default=None, metavar="Q",
                        help="팔 6 관절 시작 각도 [rad] (shoulder_pan … wrist_3). 기본 drive 설정 home_pose. 그리퍼는 항상 열림")
    parser.add_argument("--check", action="store_true", help="점검: 로봇 자세 유지·드론 비행·카메라 영상 확인 후 종료")
    parser.add_argument("--realtime", action="store_true", help="--check 도 실제 시간 속도로 (GUI 는 항상 실제 시간)")
    parser.add_argument("--report-dir", default=ts.REPORT_DIR)
    return parser


def load_scene_config(path):
    with open(path, encoding="utf-8") as f:
        sc = yaml.safe_load(f)
    for k in ("table", "drone_pos", "light", "third_view_camera_config"):
        if k not in sc:
            raise KeyError(f"{path}: '{k}' 항목이 없습니다")
    for k in ("size", "top_z", "center_xy", "color"):
        if k not in sc["table"]:
            raise KeyError(f"{path}: 'table.{k}' 항목이 없습니다")
    return sc


def load_third_view_config(path):
    with open(path, encoding="utf-8") as f:
        c = yaml.safe_load(f)
    for k in ("resolution", "intrinsics", "distortion", "horizontal_aperture", "clipping_range", "position", "look_at",
              "model", "mount"):
        if k not in c:
            raise KeyError(f"{path}: '{k}' 항목이 없습니다")
    for k in ("usd", "prim", "optical_frame", "bottom_center"):
        if k not in c["model"]:
            raise KeyError(f"{path}: 'model.{k}' 항목이 없습니다")
    if any(abs(float(x)) > 0 for x in c["distortion"]):
        raise ValueError(f"{path}: 왜곡 계수가 0 이 아님 — sim 카메라는 왜곡 없는 핀홀")
    return c


def optical_look_at(pos, target):
    """optical frame(+z 전방, +y 아래, +x 오른쪽) world 4x4 (열벡터 규약): pos 에서 target 을 보고 화면 위쪽이 world +z 쪽."""
    p, t = np.asarray(pos, dtype=float), np.asarray(target, dtype=float)
    f = (t - p) / np.linalg.norm(t - p)
    right = np.cross(f, [0.0, 0.0, 1.0])
    if np.linalg.norm(right) < 1e-6:
        raise ValueError("카메라가 수직으로 위·아래를 봄 (화면 위쪽을 정할 수 없음)")
    right /= np.linalg.norm(right)
    up = np.cross(right, f)
    T = np.eye(4)
    T[:3, 0], T[:3, 1], T[:3, 2], T[:3, 3] = right, -up, f, p
    return T


def add_third_view_camera(stage, c, table_top):
    """D435i 모양(로봇 USD 의 손목 카메라 링크 하위 트리 reference) + 받침대 + 렌더링 카메라(color optical frame 에).
    렌즈(optical frame)가 c['position'] 에서 c['look_at'] 을 보도록 카메라 링크를 놓는다. 렌더링 Camera prim 경로를 돌려준다."""
    from pxr import Gf, Usd, UsdGeom, UsdPhysics

    import collision_geom as cg

    m = c["model"]
    link_path = f"{THIRD_VIEW_ROOT}/d435i"
    UsdGeom.Xform.Define(stage, THIRD_VIEW_ROOT)
    prim = stage.DefinePrim(link_path, "Xform")
    prim.GetReferences().AddReference(ts._abs(m["usd"]), m["prim"])
    opt = next((p for p in Usd.PrimRange(prim) if p.GetName() == m["optical_frame"]), None)
    if opt is None:
        raise RuntimeError(f"third view 카메라 모델에 {m['optical_frame']} 이 없음 ({m['usd']} {m['prim']})")
    # 링크 → optical 상대 변환 (열벡터 규약 4x4)
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    rel = cache.GetLocalToWorldTransform(opt) * cache.GetLocalToWorldTransform(prim).GetInverse()
    T_lo = cg.mat_np(rel).T
    T_wl = optical_look_at(c["position"], c["look_at"]) @ np.linalg.inv(T_lo)
    ts._set_matrix(prim, Gf.Matrix4d(T_wl.T.tolist()))
    cam_path = f"{opt.GetPath()}/{THIRD_VIEW_NAME}"
    cam = ts.define_pinhole_camera(stage, cam_path, c)
    ts._set_matrix(cam.GetPrim(), ts._rx(np.pi))   # optical(+z 전방, +y 아래) → USD camera(−z 전방, +y 위)

    # 받침대: 상판 → 카메라 아랫면 가운데
    mt = c["mount"]
    top = T_wl @ np.array([*m["bottom_center"], 1.0])
    h = float(top[2]) - table_top
    if h <= 0:
        raise ValueError(f"third view 카메라 아랫면이 테이블 상판보다 낮음 ({top[2]:.3f} m)")
    for name, r, z0, z1 in (("plate", mt["plate_radius"], table_top, table_top + mt["plate_height"]),
                            ("post", mt["post_radius"], table_top, float(top[2]))):
        g = UsdGeom.Cylinder.Define(stage, f"{THIRD_VIEW_ROOT}/mount_{name}")
        g.CreateAxisAttr().Set("Z")
        g.CreateRadiusAttr().Set(float(r))
        g.CreateHeightAttr().Set(z1 - z0)
        g.CreateExtentAttr().Set([Gf.Vec3f(-r, -r, -(z1 - z0) / 2), Gf.Vec3f(r, r, (z1 - z0) / 2)])
        UsdGeom.Xformable(g).AddTranslateOp().Set(Gf.Vec3d(float(top[0]), float(top[1]), (z0 + z1) / 2))
        g.CreateDisplayColorAttr().Set([Gf.Vec3f(*mt["color"])])
        UsdPhysics.CollisionAPI.Apply(g.GetPrim())
    return cam_path, h


def add_table(stage, tc):
    from pxr import Gf, UsdGeom, UsdPhysics

    sx, sy = (float(x) for x in tc["size"])
    h = float(tc["top_z"])
    g = UsdGeom.Cube.Define(stage, TABLE_PATH)
    g.CreateSizeAttr().Set(1.0)
    x = UsdGeom.Xformable(g)
    x.AddTranslateOp().Set(Gf.Vec3d(float(tc["center_xy"][0]), float(tc["center_xy"][1]), h / 2))
    x.AddScaleOp().Set(Gf.Vec3f(sx, sy, h))
    g.CreateDisplayColorAttr().Set([Gf.Vec3f(*tc["color"])])
    UsdPhysics.CollisionAPI.Apply(g.GetPrim())   # 정적 collider (강체 아님)


class Scene:
    pass


def build_scene(robot_config=None, scene_config=None, drone_config=None, drone_pos=None, mode="static", seed=0,
                prop_spin=False, base="fixed", init_pose=None, extra_setup=None, flight=None, position_source=None,
                report_dir=None, realtime=False, waypoints=None):
    """씬을 만들고 재생까지 한다. 로봇은 init_pose (기본 drive 설정 home_pose, 그리퍼 열림), 드론은 drone_pos 에서 비행 시작.
    flight: 드론 제어기 (None = 드론 설정 flight.backend). px4 면 테이블 위 이륙 지점에서 PX4 로 이륙 → (waypoints) → drone_pos 호버까지 마치고 돌려준다.
    extra_setup(s): 재생 전에 호출할 함수 (예: 접촉 보고 켜기). Scene 을 돌려준다."""
    import isaacsim.core.experimental.utils.app as app_utils
    from isaacsim.core.experimental.objects import DomeLight

    from robot_drive import RobotDrive

    if base not in BASE_CHOICES:
        raise ValueError(f"base 는 {BASE_CHOICES} 중 하나: {base}")
    if base == "kinematic":
        raise NotImplementedError("--base kinematic 은 아직 구현하지 않음 (PLAN 8단계)")
    if mode == "trajectory":
        raise NotImplementedError("trajectory 모드는 아직 구현하지 않음 (PLAN 2단계: 나중에)")
    s = Scene()
    s.robot_cfg = ts.use_robot(robot_config)
    s.scene_cfg = load_scene_config(scene_config or DEFAULT_SCENE_CONFIG)
    s.drone_cfg = dr.load_drone_config(drone_config or dr.DEFAULT_DRONE_CONFIG)
    s.drive_cfg = ts.load_config()
    s.cam_cfg = ts.load_camera_config()
    s.tv_cfg = load_third_view_config(ts._abs(s.scene_cfg["third_view_camera_config"]))
    s.drone_pos = np.asarray(drone_pos if drone_pos is not None else s.scene_cfg["drone_pos"], dtype=float)
    tc = s.scene_cfg["table"]

    # 로봇 (새 stage, PhysX, ground, fixed base override, articulation·충돌·재질 설정). 위치 = (0, 0, 상판 높이)
    s.stage, s.robot, s.robot_info = ts.build_test_scene(ts.DEFAULT_USD, ts.PHYSICS_VARIANT, base_z=float(tc["top_z"]),
                                                         base="fixed", config=s.drive_cfg, ground=True, light=False)
    DomeLight("/World/DomeLight").set_intensities(float(s.scene_cfg["light"]["dome_intensity"]))
    add_table(s.stage, tc)
    s.wrist_cam = ts.add_wrist_camera(s.stage, s.cam_cfg, tilt=0.0)
    s.third_cam, s.mount_h = add_third_view_camera(s.stage, s.tv_cfg, float(tc["top_z"]))
    s.extra_setup = extra_setup(s) if extra_setup is not None else None   # 재생 전 (접촉 보고 등)
    s.backend = flight or s.drone_cfg["flight"]["backend"]
    if waypoints and s.backend != "px4":
        raise ValueError("--drone-waypoints 는 px4 드론에서만 (geometric 은 drone_pos 에서 바로 시작)")
    s.waypoints = [np.asarray(w, dtype=float) for w in (waypoints or [])]
    s.px4_cfg = s.scene_cfg["px4"] if s.backend == "px4" else None
    if s.px4_cfg is not None:
        if "px4" not in s.scene_cfg:
            raise KeyError("scene 설정에 px4 항목이 없습니다")
        spawn = np.array([*s.px4_cfg["takeoff_xy"], float(tc["top_z"]) + float(s.px4_cfg["spawn_above_table"])])
    else:
        spawn = s.drone_pos
    s.drone_info = dr.add_drone(s.stage, s.drone_cfg, position=spawn, prop_spin=prop_spin)

    app_utils.play()
    _update()
    problems, _ = ts.verify_articulation(s.stage, s.robot, s.robot_info, s.drive_cfg)
    if problems:
        raise RuntimeError("로봇 articulation 설정 확인 실패: " + "; ".join(problems))
    s.drive = RobotDrive(s.robot, s.drive_cfg)
    s.drive.apply()
    s.drive.start()
    s.home = np.asarray(s.drive_cfg["home_pose"], dtype=float)
    s.init_pose = s.home.copy() if init_pose is None else np.asarray(init_pose, dtype=float)
    if s.init_pose.shape != (len(ts.ARM_JOINTS),):
        raise ValueError(f"init_pose 는 팔 {len(ts.ARM_JOINTS)} 관절 값: {init_pose}")
    lo, hi = (x.numpy()[0][s.drive.arm_i] for x in s.robot.get_dof_limits())
    bad = [f"{n} {q:+.4f} (한계 {a:+.4f} ~ {b:+.4f})" for n, q, a, b in zip(ts.ARM_JOINTS, s.init_pose, lo, hi)
           if not a <= q <= b]
    if bad:
        raise ValueError("init_pose 가 관절 한계 밖: " + ", ".join(bad))
    s.drive.reset_pose(s.init_pose, gripper=0.0)   # 그리퍼는 항상 완전 열림으로 시작
    s.flight = fl.DroneFlight(s.drone_cfg, s.drone_info, mode=mode, seed=seed, target=s.drone_pos, backend=s.backend,
                              report_dir=report_dir, position_source=position_source)
    s.flight.start()
    if s.backend == "px4":
        px4_takeoff(s, realtime)
    else:
        s.flight.arm()
    return s


def px4_takeoff(s, realtime=False):
    """PX4 드론: 연결 → 이륙 지점 위로 수직 상승 (높이 = 첫 waypoint 또는 드론 위치) → waypoint 차례로 → 드론 위치 → settle_time 호버.
    도착 = PX4 추정 위치가 arrive_tol 안 (실물처럼 실제 위치는 표류만큼 다를 수 있음). 실패하면 에러."""
    c, f = s.px4_cfg, s.flight
    t0 = f.t
    while not (f.px4.status == "lockstep" and f.cmd.connected):
        step(s)
        if f.t - t0 > c["connect_timeout"]:
            raise RuntimeError(f"PX4 연결 안 됨 ({f.px4.status}, 명령 링크 {f.cmd.connected}), 로그 {f.px4.run_dir}")
    run(s, 5.0, realtime)                         # EKF 초기화 여유 (바닥에 놓인 채)
    p0 = f._body_pose()[0]
    first_z = s.waypoints[0][2] if s.waypoints else s.drone_pos[2]
    legs = [np.array([p0[0], p0[1], first_z]), *[w.copy() for w in s.waypoints], s.drone_pos.copy()]
    f.arm(legs[0])
    for i, goal in enumerate(legs):
        f.cmd.goto(goal)
        t1 = f.t

        def arrived():
            e = f.cmd.estimate_enu
            return f.px4.armed and f.cmd.mode == "OFFBOARD" and e is not None and np.linalg.norm(e - goal) < c["arrive_tol"]

        wall1 = time.monotonic()
        while not arrived():
            step(s)
            if realtime:
                ahead = (f.t - t1) - (time.monotonic() - wall1)
                if ahead > 0:
                    time.sleep(ahead)
            if f.t - t1 > (c["connect_timeout"] if i == 0 else 0) + c["leg_timeout"]:
                msgs = "; ".join(m for _, _, m in f.cmd.messages[-5:])
                raise RuntimeError(f"PX4 이동 {i} 실패 (모드 {f.cmd.mode}, armed {f.px4.armed}, 추정 {f.cmd.estimate_enu}) {msgs}")
    f.ref.p0 = s.drone_pos.copy()
    f.t_arm = f.t
    run(s, c["settle_time"], realtime)
    s.px4_ready_t = f.t
    f.cmd.cli_ready = True                        # 다른 터미널 drone_cmd.py 이동 명령 받기 시작


def step(s):
    _update()
    s.drive.check()
    s.flight.check()


def run(s, sec, realtime=False, on_step=None):
    t_end = s.flight.t + sec
    wall0, sim0 = time.monotonic(), s.flight.t
    while s.flight.t < t_end - 1e-9:
        step(s)
        if on_step is not None:
            on_step(s)
        if realtime:
            ahead = (s.flight.t - sim0) - (time.monotonic() - wall0)
            if ahead > 0:
                time.sleep(ahead)


def camera_sensors(s):
    from isaacsim.sensors.experimental.rtx import CameraSensor

    out = {}
    for name, path, cfg in (("wrist", s.wrist_cam, s.cam_cfg), ("third_view", s.third_cam, s.tv_cfg)):
        w, h = (int(x) for x in cfg["resolution"])
        out[name] = CameraSensor(path, resolution=(h, w), annotators=["rgb"])
    return out


def grab(sensor):
    for _ in range(200):
        if sensor.has_data():
            break
        _update()
    rgb, _ = sensor.get_data("rgb")
    if rgb is None:
        raise RuntimeError("카메라 데이터를 받지 못함")
    return rgb.numpy()[..., :3].astype(np.uint8)


def check(s, prop_spin, realtime, report_dir):
    from PIL import Image

    import collision_geom as cg

    lines = []

    def log(m=""):
        print(m, flush=True)
        lines.append(m)

    log(f"drone_scene --check  {datetime.datetime.now().isoformat(timespec='seconds')}")
    log(f"로봇 설정 {s.robot_cfg['path']}, 드론 설정 {s.drone_cfg['path']}, 드론 위치 {s.drone_pos.tolist()}, "
        f"모드 {s.flight.ref.mode}, prop_spin {prop_spin}")
    log(f"팔 시작 자세 [rad] {np.round(s.init_pose, 4).tolist()}" + (" (home_pose)" if np.allclose(s.init_pose, s.home) else ""))
    log(f"third view 카메라: 렌즈 {s.tv_cfg['position']} → {s.tv_cfg['look_at']}, 받침대 높이 {s.mount_h * 1000:.0f} mm, "
        f"prim {s.third_cam}")
    sensors = camera_sensors(s)
    rows = []

    def rec(sc):
        q = sc.robot.get_dof_positions().numpy()[0][sc.drive.arm_i]
        p, qd = sc.flight._body_pose()
        tilt = math.degrees(math.acos(np.clip(cg.quat_to_R(qd)[2, 2], -1, 1)))
        rows.append((sc.flight.t, q, p, tilt))

    run(s, CHECK_SEC, realtime=realtime, on_step=rec)
    results = []
    half = [r for r in rows if r[0] >= CHECK_SEC / 2]
    dev = max(float(np.degrees(np.abs(r[1] - s.init_pose)).max()) for r in half)
    log(f"[1] 로봇 시작 자세 유지 (뒤 {CHECK_SEC / 2:.0f} s): 팔 관절 최대 편차 {dev:.4f}° "
        f"(크면 팔이 테이블·드론·받침대에 닿았거나 자세를 버티지 못함)")
    results.append(("로봇 자세 유지", dev <= CRIT["arm_dev_deg"]))
    # 목표 = px4 면 지금 명령 목표 (다른 터미널 drone_cmd.py goto 로 바뀌었을 수 있음), geometric 이면 기준 궤적
    goal = s.flight.cmd.target if s.backend == "px4" and s.flight.cmd.target is not None else s.flight.ref(s.flight.t)[0]
    err = np.linalg.norm(rows[-1][2] - goal) * 1000
    tilt = max(r[3] for r in half)
    log(f"[2] 드론 비행 ({s.backend}, {s.flight.ref.mode}): {CHECK_SEC:.0f} s 뒤 목표와 거리 {err:.2f} mm (기준 {CRIT['drone_err_mm'][s.backend]:.0f}), 뒤 절반 기울기 최대 {tilt:.2f}°, "
        f"로터 ω {np.round(s.flight.omega, 1).tolist()} rad/s")
    results.append(("드론 비행", err <= CRIT["drone_err_mm"][s.backend] and tilt <= CRIT["drone_tilt_deg"]))
    out_dir = os.path.join(report_dir, f"drone_scene_{datetime.datetime.now():%Y%m%d_%H%M%S}")
    os.makedirs(out_dir, exist_ok=True)
    imgs, ok_img = {}, True
    for name, sensor in sensors.items():
        img = grab(sensor)
        imgs[name] = img
        Image.fromarray(img).save(os.path.join(out_dir, f"{name}.png"))
        mean, std = float(img.mean()), float(img.std())
        good = CRIT["img_mean"][0] <= mean <= CRIT["img_mean"][1] and std >= CRIT["img_std"]
        ok_img &= good
        log(f"[3] 카메라 {name}: {img.shape[1]}x{img.shape[0]}, 밝기 평균 {mean:.1f}, 표준편차 {std:.1f} → {'정상' if good else '이상'}")
    Image.fromarray(np.concatenate([imgs["wrist"], imgs["third_view"]], axis=1)).save(os.path.join(out_dir, "both.png"))
    results.append(("카메라 영상", ok_img))
    log(f"  영상: {out_dir}/ (wrist.png, third_view.png, both.png)")
    log("")
    for name, ok in results:
        log(f"  {name:12s} {'PASS' if ok else 'FAIL'}")
    n = sum(ok for _, ok in results)
    log(f"  {n}/{len(results)} PASS")
    with open(os.path.join(out_dir, "report.txt"), "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    return n == len(results)


def open_camera_windows(s, old=None):
    """GUI: 메인 뷰포트 시점을 맞추고 'Wrist camera'·'Third view camera' 뷰포트 창을 띄운다 (old 창들은 닫음). 창 목록을 돌려준다."""
    from isaacsim.core.rendering_manager import ViewportManager
    from omni.kit.viewport.utility import create_viewport_window
    from pxr import Sdf

    for win in old or []:
        win.destroy()
    ViewportManager.set_camera_view("/OmniverseKit_Persp", eye=[1.9, -1.6, 1.7], target=[0.4, 0.0, 1.1])
    wins = []
    for title, path, cfg in (("Wrist camera", s.wrist_cam, s.cam_cfg), ("Third view camera", s.third_cam, s.tv_cfg)):
        w, h = (int(x) for x in cfg["resolution"])
        win = create_viewport_window(title, width=w, height=h, camera_path=Sdf.Path(path))
        win.viewport_api.resolution = (w, h)
        wins.append(win)
    return wins


def view(s):
    open_camera_windows(s)
    print(f"[view] 드론 {s.flight.ref.mode} 비행, 로봇 시작 자세 {np.round(s.init_pose, 4).tolist()}. 'Wrist camera'·'Third view camera' 창. 창을 닫으면 종료", flush=True)
    wall0, sim0 = time.monotonic(), s.flight.t
    while _running():
        step(s)
        ahead = (s.flight.t - sim0) - (time.monotonic() - wall0)
        if ahead > 0:
            time.sleep(ahead)


def main():
    args, _ = make_parser().parse_known_args()
    from isaacsim import SimulationApp

    simulation_app = SimulationApp({"headless": args.headless})
    ok = False
    try:
        scene = build_scene(args.robot_config, args.scene_config, args.drone_config, args.drone_pos, args.mode, args.seed,
                            args.prop_spin == "on", args.base, args.init_pose, flight=args.flight,
                            position_source=args.position_source, report_dir=args.report_dir,
                            realtime=not args.headless or args.realtime, waypoints=args.drone_waypoints)
        if args.check:
            ok = check(scene, args.prop_spin, args.realtime, args.report_dir)
        elif args.headless:
            raise SystemExit("--headless 는 --check 와 같이 쓴다 (GUI 없이 계속 돌리는 모드는 3단계 ROS 브리지에서)")
        else:
            view(scene)
            ok = True
    except Exception as e:  # noqa: BLE001
        import traceback
        print(f"[ERROR] {type(e).__name__}: {e}", flush=True)
        traceback.print_exc()
    finally:
        simulation_app.close(exit_code=0 if ok else 1)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
