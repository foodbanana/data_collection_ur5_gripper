#!/usr/bin/env python3
# =============================================================
# lerobot_v21_common.py   (docs/PLAN.md 4-4)
#
# 1단계 중간 파일 (bag_lerobot_intermediate/<bag>/, schema_version 2) → LeRobot v2.1 공용 부분.
# lerobot_stage2_build_dataset_v21.py (에피소드 1 개) 와 lerobot_merge_episodes_v21.py (여러 개 병합) 가 같이 쓴다.
#   데이터셋 스키마 (CLAUDE.md, 0단계 확정):
#     observation.state (7,)  = 팔 6관절 (rad) + 그리퍼 present / 1150 (0~1)
#     action (7,)             = 팔 6관절 목표 (rad) + 그리퍼 goal / 1150 ({0.0, 1.0})
#     observation.base_imu (6,) = 각속도 xyz (rad/s) + 선가속도 xyz (m/s²)
#     observation.images.wrist / observation.images.third_view (480, 640, 3)
# 입력이 스키마와 다르면 ValueError (조용한 fallback 금지). lerobot 은 import 하지 않는다 (검사만 시스템 python 으로도 돌릴 수 있게)
# =============================================================

import os
import json

import numpy as np
from PIL import Image

SCHEMA_VERSION = 2
IMAGE_KEYS = ('wrist', 'third_view')
BASE_IMU_NAMES = ['gyro_x', 'gyro_y', 'gyro_z', 'accel_x', 'accel_y', 'accel_z']
GRIPPER_ACTION_VALUES = {0.0, 1.0}      # 그리퍼 action 은 이진 (0 = 열림, 1 = 닫힘)
ARM_LIMIT = 2 * np.pi + 1e-3            # 팔 관절각 한계 [rad] (UR5 ±2π)
GRASP_STATE_MAX = 0.95                  # 닫기 명령 중 그리퍼 state 최댓값이 이보다 작으면 "물체에 막혀 멈춤 = 파지 신호"
                                        #   (sim 빈손 닫기는 손가락끼리 닿아 0.985 에서 멈춤, 드론을 잡으면 약 0.25)


def read_png(path):
    """png 를 (H, W, 3) uint8 RGB numpy 로 읽기.
    1단계에서 cv2(BGR)로 저장했으므로 PIL 로 열면 RGB 로 바르게 읽힌다
    (cv2.imwrite 는 BGR 배열을 파일에는 표준 RGB 로 저장하기 때문)."""
    img = Image.open(path).convert('RGB')
    return np.asarray(img, dtype=np.uint8)   # (H, W, 3)


def load_intermediate(in_dir):
    """1단계 중간 파일 로드. → dict(states, actions, base_imu, stamps, meta)"""
    meta_path = os.path.join(in_dir, 'meta.json')
    if not os.path.isfile(meta_path):
        raise ValueError(f"meta.json 이 없음: {in_dir}")
    with open(meta_path) as f:
        meta = json.load(f)
    if meta.get('schema_version') != SCHEMA_VERSION:
        raise ValueError(f"중간 파일 형식이 schema_version {meta.get('schema_version')} (기대 {SCHEMA_VERSION}). "
                         f"최신 lerobot_stage1_extract_bag.py 로 1단계를 다시 실행하세요")
    out = {'meta': meta}
    for key, fname in (('states', 'states.npy'), ('actions', 'actions.npy'), ('base_imu', 'base_imu.npy'), ('stamps', 'timestamps.npy')):
        p = os.path.join(in_dir, fname)
        if not os.path.isfile(p):
            raise ValueError(f"{fname} 가 없음: {in_dir}")
        out[key] = np.load(p)
    return out


GRIPPER_IDLE_EPS = 0.005                # 그리퍼 state 가 이만큼 (0~1 기준) 안 변하면 멈춘 것으로 본다


def idle_frames(actions, states, eps_rad):
    """앞·뒤 정지 구간 길이 [프레임]. 정지 = action (팔 명령·그리퍼 명령) 과 state (팔·그리퍼 측정값) 가 모두
    첫 프레임 / 마지막 프레임 값에서 그대로 (팔 eps_rad, 그리퍼 state GRIPPER_IDLE_EPS 안) 인 구간.
    state 도 보는 이유: 닫기 명령 뒤 그리퍼가 닫히는 약 2 s 는 action 이 안 변해도 정지가 아니다 (자르면 파지 장면이 잘린다).
    → (앞, 뒤). 앞 = 처음으로 값이 바뀌는 프레임 번호, 뒤 = 마지막으로 값이 바뀐 뒤 남은 프레임 수"""
    a = np.asarray(actions, dtype=np.float64)
    s = np.asarray(states, dtype=np.float64)
    def run(k, a_, s_):
        moved = (np.abs(a_[:, :6] - a_[k, :6]).max(axis=1) > eps_rad) | (a_[:, 6] != a_[k, 6]) \
            | (np.abs(s_[:, :6] - s_[k, :6]).max(axis=1) > eps_rad) | (np.abs(s_[:, 6] - s_[k, 6]) > GRIPPER_IDLE_EPS)
        return int(np.argmax(moved)) if moved.any() else len(a_)
    return run(0, a, s), run(0, a[::-1], s[::-1])


def check_episode(name, in_dir, ref=None):
    """한 에피소드를 읽어 스키마와 맞는지 검사. 문제가 있으면 ValueError.
    ref: 앞 에피소드와 맞아야 하는 값 (fps, 이미지 크기, 팔 action 종류, base_imu 방식). None 이면 검사 안 함.
    → info dict (병합·검수가 쓰는 요약 + 데이터)"""
    d = load_intermediate(in_dir)
    states, actions, base_imu, stamps, meta = d['states'], d['actions'], d['base_imu'], d['stamps'], d['meta']
    N = states.shape[0]
    if N < 2:
        raise ValueError(f"프레임이 {N} 개")
    if states.shape != (N, 7) or actions.shape != (N, 7) or base_imu.shape != (N, 6) or stamps.shape != (N,):
        raise ValueError(f"배열 크기가 다름: states {states.shape}, actions {actions.shape}, base_imu {base_imu.shape}, timestamps {stamps.shape}")
    for key, arr in (('states', states), ('actions', actions), ('base_imu', base_imu), ('timestamps', stamps)):
        if not np.isfinite(arr).all():
            raise ValueError(f"{key} 에 유한하지 않은 값 {int((~np.isfinite(arr)).sum())} 개")
    fps = float(meta['fps'])
    dts = np.diff(stamps)
    if np.abs(dts - 1.0 / fps).max() > 1e-6:
        raise ValueError(f"timestamps 간격이 1/{fps:g} s 가 아님 ({dts.min():.6f} ~ {dts.max():.6f})")

    for cam in IMAGE_KEYS:
        cam_dir = os.path.join(in_dir, cam)
        if not os.path.isdir(cam_dir):
            raise ValueError(f"{cam}/ 폴더 없음")
        n_png = len([p for p in os.listdir(cam_dir) if p.endswith('.png')])
        if n_png != N:
            raise ValueError(f"{cam}/ png 개수({n_png}) != 프레임 수({N})")
    hwc = tuple(read_png(os.path.join(in_dir, IMAGE_KEYS[0], '000000.png')).shape)

    if np.abs(states[:, :6]).max() > ARM_LIMIT or np.abs(actions[:, :6]).max() > ARM_LIMIT:
        raise ValueError(f"팔 관절각이 ±2π 밖 (state 최대 {np.abs(states[:, :6]).max():.3f}, action 최대 {np.abs(actions[:, :6]).max():.3f} rad)")
    gs, ga = states[:, 6], actions[:, 6]
    if gs.min() < 0.0 or gs.max() > 1.0:
        raise ValueError(f"그리퍼 state 가 0~1 밖: {gs.min():.4f} ~ {gs.max():.4f}")
    bad = sorted(set(np.unique(ga).astype(float).tolist()) - GRIPPER_ACTION_VALUES)
    if bad:
        raise ValueError(f"그리퍼 action 이 이진 {{0, 1}} 이 아님. 이상값: {bad[:5]}")

    cur = dict(fps=fps, hwc=hwc, arm_action=meta['arm_action'], base_imu=meta['base_imu'])
    if ref is not None:
        for k, v in cur.items():
            if ref[k] != v:
                raise ValueError(f"{k} 가 앞 에피소드와 다름: 이 에피소드 {v} vs 기준 {ref[k]}"
                                 + (" (freedrive 와 텔레옵은 팔 action 의미가 달라 섞지 않는다)" if k == 'arm_action' else ""))

    close = ga == 1.0
    ps = meta.get('protective_stop')
    return dict(name=name, dir=in_dir, N=N, **cur, mode=meta['mode'],
                state_names=meta['state_names'], action_names=meta['action_names'],
                grip_state_range=(float(gs.min()), float(gs.max())),
                close_frames=int(close.sum()), grip_state_max_while_closing=float(gs[close].max()) if close.any() else None,
                grasp_signal=bool(close.any() and gs[close].max() < GRASP_STATE_MAX),
                protective_stop=bool(ps and ps.get('occurred')),
                joint_command=meta.get('joint_command'), drone_kill=meta.get('drone_kill'),
                data=d)


def make_features(info):
    H, W, C = info['hwc']
    img = lambda: {'dtype': 'video', 'shape': (H, W, C), 'names': ['height', 'width', 'channels']}
    return {
        'observation.state': {'dtype': 'float32', 'shape': (7,), 'names': info['state_names']},
        'action': {'dtype': 'float32', 'shape': (7,), 'names': info['action_names']},
        'observation.base_imu': {'dtype': 'float32', 'shape': (6,), 'names': BASE_IMU_NAMES},
        **{f'observation.images.{k}': img() for k in IMAGE_KEYS},
    }


def add_episode(dataset, info, task, start=0, end=None, log=print):
    """info (check_episode 결과) 의 프레임 [start, end) 를 데이터셋에 넣고 에피소드를 저장한다. → 넣은 프레임 수"""
    d = info['data']
    end = info['N'] if end is None else end
    for i in range(start, end):
        frame = {
            'observation.state': d['states'][i].astype(np.float32),
            'action': d['actions'][i].astype(np.float32),
            'observation.base_imu': d['base_imu'][i].astype(np.float32),
        }
        for k in IMAGE_KEYS:
            frame[f'observation.images.{k}'] = read_png(os.path.join(info['dir'], k, f'{i:06d}.png'))
        dataset.add_frame(frame, task=task)
        if (i - start + 1) % 200 == 0 or i == end - 1:
            log(f"    {i - start + 1}/{end - start}")
    log("    에피소드 저장(비디오 인코딩)...")
    dataset.save_episode()
    return end - start
