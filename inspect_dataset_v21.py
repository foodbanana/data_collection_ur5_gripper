#!/usr/bin/env python3
# =============================================================
# inspect_dataset_v21.py   (docs/PLAN.md 4-4)
#
# 병합한 LeRobot v2.1 데이터셋 폴더를 검수한다 (스키마 = CLAUDE.md 0단계). 항목마다 PASS / FAIL, 하나라도 FAIL 이면 종료 코드 1.
#   1 형식        : meta/info.json codebase_version v2.1, fps 25, feature 키·크기 (state 7, action 7, base_imu 6, 카메라 wrist·third_view)
#   2 에피소드 수 : info.json 총계 = parquet 파일 수 = 카메라별 mp4 수, 프레임 수 합
#   3 값 범위     : 유한값, 팔 관절각 ±2π, 그리퍼 state 0~1, 그리퍼 action 값 종류 {0.0, 1.0}  (= 이상치 0)
#   4 파지 신호   : 모든 에피소드에서 닫기 명령 (action 1) 중 그리퍼 state 가 중간에서 멈춤 (최댓값 < 0.95)
#   5 (--load)    : LeRobotDataset 으로 열어 첫·마지막 프레임을 읽음 (비디오 디코딩 포함. lerobot 필요)
# 에피소드별 표: 프레임 수, 그리퍼, 파지 신호, 앞·뒤 정지 구간 (action·state 가 안 변하는 구간, 자를지 판단용),
#   /joint_command 간격 통계 (meta/merge_manifest.json 이 있으면: 중앙값·최댓값·40 ms 넘은 횟수 → 텔레옵 명령이 끊긴 에피소드 찾기)
#
# 사용 (pandas·pyarrow 가 있는 환경, 예 conda lerobot_v2):
#   python3 inspect_dataset_v21.py <데이터셋 폴더> [--load] [--idle-eps-deg 0.1]
# =============================================================

import os
import sys
import glob
import json
import argparse

import numpy as np
import pandas as pd

import lerobot_v21_common as common

EXPECT = {'observation.state': [7], 'action': [7], 'observation.base_imu': [6],
          'observation.images.wrist': [480, 640, 3], 'observation.images.third_view': [480, 640, 3]}
VIDEO_KEYS = [f'observation.images.{k}' for k in common.IMAGE_KEYS]
FPS = 25


def main():
    ap = argparse.ArgumentParser(description='LeRobot v2.1 데이터셋 검수')
    ap.add_argument('root', help='데이터셋 폴더 (meta/info.json 이 있는 곳)')
    ap.add_argument('--load', action='store_true', help='LeRobotDataset 으로 열어 프레임을 읽어 본다 (lerobot 필요)')
    ap.add_argument('--idle-eps-deg', type=float, default=0.1, help='[deg] 팔 명령이 이만큼 안 변하면 정지')
    args = ap.parse_args()
    root = os.path.abspath(os.path.expanduser(args.root))
    results = []

    def item(name, ok, detail):
        results.append(ok)
        print(f"[{'PASS' if ok else 'FAIL'}] {name}: {detail}")

    info_path = os.path.join(root, 'meta', 'info.json')
    if not os.path.isfile(info_path):
        print(f"[오류] meta/info.json 이 없음: {root}")
        sys.exit(1)
    with open(info_path) as f:
        info = json.load(f)
    print(f"=== 검수: {root} ===")

    # 1 형식
    feats = info.get('features', {})
    wrong = [f"{k} {feats.get(k, {}).get('shape')} (기대 {v})" for k, v in EXPECT.items() if list(feats.get(k, {}).get('shape', [])) != v]
    not_video = [k for k in VIDEO_KEYS if feats.get(k, {}).get('dtype') != 'video']
    ok = info.get('codebase_version') == 'v2.1' and info.get('fps') == FPS and not wrong and not not_video
    item('1 형식', ok, f"codebase_version {info.get('codebase_version')}, fps {info.get('fps')}, "
         + (f"feature 크기 다름 {wrong}" if wrong else f"feature {list(EXPECT)} 있음") + (f", 비디오가 아님 {not_video}" if not_video else ""))

    # 2 에피소드 수
    parquets = sorted(glob.glob(os.path.join(root, 'data', 'chunk-*', 'episode_*.parquet')))
    n_mp4 = {k: len(glob.glob(os.path.join(root, 'videos', 'chunk-*', k, 'episode_*.mp4'))) for k in VIDEO_KEYS}
    dfs = [pd.read_parquet(p) for p in parquets]
    frames = sum(len(d) for d in dfs)
    ok = len(parquets) == info.get('total_episodes') and all(n == len(parquets) for n in n_mp4.values()) and frames == info.get('total_frames') and len(parquets) > 0
    item('2 에피소드 수', ok, f"info.json {info.get('total_episodes')} 에피소드·{info.get('total_frames')} 프레임, parquet {len(parquets)} 개·{frames} 프레임, mp4 {list(n_mp4.values())}")

    mm_path = os.path.join(root, 'meta', 'merge_manifest.json')
    mm = json.load(open(mm_path)) if os.path.isfile(mm_path) else None
    src = {e['episode_index']: e for e in mm['episodes']} if mm else {}

    # 3·4 에피소드별
    eps = np.radians(args.idle_eps_deg)
    bad_values, no_grasp, rows = [], [], []
    for p, df in zip(parquets, dfs):
        ep = int(df['episode_index'].iloc[0])
        s = np.stack(df['observation.state'].to_numpy()).astype(np.float64)
        a = np.stack(df['action'].to_numpy()).astype(np.float64)
        b = np.stack(df['observation.base_imu'].to_numpy()).astype(np.float64)
        probs = []
        if not (np.isfinite(s).all() and np.isfinite(a).all() and np.isfinite(b).all()):
            probs.append('유한하지 않은 값')
        if np.abs(s[:, :6]).max() > common.ARM_LIMIT or np.abs(a[:, :6]).max() > common.ARM_LIMIT:
            probs.append('팔 관절각 ±2π 밖')
        if s[:, 6].min() < 0.0 or s[:, 6].max() > 1.0:
            probs.append(f"그리퍼 state {s[:, 6].min():.3f}~{s[:, 6].max():.3f}")
        vals = sorted(set(np.unique(a[:, 6]).tolist()))
        if not set(vals).issubset(common.GRIPPER_ACTION_VALUES):
            probs.append(f"그리퍼 action 값 {vals[:5]}")
        if probs:
            bad_values.append(f"에피소드 {ep}: {', '.join(probs)}")
        close = a[:, 6] == 1.0
        gmax = float(s[close, 6].max()) if close.any() else None
        grasp = gmax is not None and gmax < common.GRASP_STATE_MAX
        if not grasp:
            no_grasp.append(ep)
        lead, trail = common.idle_frames(a, s, eps)
        jc = (src.get(ep) or {}).get('joint_command') or {}
        over = next((v for k, v in jc.items() if k.startswith('intervals_over_')), None)
        rows.append((ep, (src.get(ep) or {}).get('source', os.path.basename(p)), len(df), vals, f"{s[:, 6].min():.3f}~{s[:, 6].max():.3f}",
                     '없음' if gmax is None else f"{gmax:.3f}", '있음' if grasp else '없음', f"{lead / FPS:.2f} / {trail / FPS:.2f}",
                     '-' if not jc else f"{jc['interval_median_ms']:.1f} / {jc['interval_max_ms']:.1f} / {over}"))
    item('3 값 범위 (이상치 0)', not bad_values, '유한값, 팔 ±2π, 그리퍼 state 0~1, 그리퍼 action {0.0, 1.0}' if not bad_values else '; '.join(bad_values))
    item('4 파지 신호', not no_grasp and bool(rows), f"모든 에피소드에서 닫기 명령 중 그리퍼 state 최댓값 < {common.GRASP_STATE_MAX}"
         if not no_grasp else f"파지 신호 없는 에피소드 {no_grasp} (닫기 명령이 없거나 끝까지 닫힘 = 빈손)")

    # 5 LeRobot 로드
    if args.load:
        try:
            from lerobot.datasets.lerobot_dataset import LeRobotDataset
            ds = LeRobotDataset(mm['repo_id'] if mm else 'local/dataset', root=root)
            first, last = ds[0], ds[len(ds) - 1]
            shapes = {k: tuple(first[k].shape) for k in EXPECT}
            ok = len(ds) == frames and ds.num_episodes == len(parquets) and all(np.isfinite(last[k].numpy()).all() for k in EXPECT)
            item('5 LeRobotDataset 로드', ok, f"길이 {len(ds)}, 에피소드 {ds.num_episodes}, 첫 프레임 {shapes}")
        except Exception as e:  # noqa: BLE001
            item('5 LeRobotDataset 로드', False, f"{type(e).__name__}: {e}")

    print("\n── 에피소드별 ──")
    hdr = ('ep', '원본', '프레임', '그리퍼 action', '그리퍼 state', '닫는 중 state 최대', '파지 신호', '정지 앞/뒤 [s]', '/joint_command 간격 중앙/최대 [ms]/40ms 초과')
    print(' | '.join(hdr))
    for r in rows:
        print(' | '.join(str(x) for x in r))
    if mm:
        print(f"\n병합 기록: 팔 action {mm.get('arm_action')}, base_imu {mm.get('base_imu')}, 정지 구간 자르기 {mm.get('trim_idle') or '안 함'}, "
              f"제외 {[e['source'] + ' (' + e['reason'] + ')' for e in mm.get('excluded', [])] or '없음'}")
    n_ok = sum(results)
    print(f"\n  {n_ok}/{len(results)} PASS")
    sys.exit(0 if n_ok == len(results) else 1)


if __name__ == '__main__':
    main()
