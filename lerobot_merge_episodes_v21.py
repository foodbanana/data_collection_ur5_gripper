#!/usr/bin/env python3
# =============================================================
# lerobot_merge_episodes_v21.py  [conda(lerobot_v2) 환경에서 실행]
#
# 여러 개의 1단계 중간 파일(bag_lerobot_intermediate/<bag>/)을 매니페스트로 받아
# LeRobotDataset 하나로 병합한다. (lerobot 0.3.3 -> CODEBASE_VERSION "v2.1")
#
# 방식: 병합 대상들의 parquet/mp4를 이어붙이는 대신,
#       중간 파일에서 데이터셋 하나를 create() 하고
#       에피소드마다 add_frame -> save_episode 를 반복한다.
#       episode_index/index/task_index/stats/info.json 총계는 lerobot 이 계산.
#
# 뼈대는 lerobot_stage2_build_dataset_v21.py 와 동일.
# 바뀐 곳:
#   1) 입력이 단일 폴더 -> 매니페스트(폴더 목록)
#   2) create() 는 한 번, save_episode() 는 에피소드마다 (이중 루프)
#   3) create 전에 전 에피소드 사전검사 (실패 시 즉시 중단, 조용한 fallback 없음)
#   4) meta/merge_manifest.json 에 어떤 폴더가 어떤 episode_index 로 갔는지 기록
#
# 실행:
#   conda activate lerobot_v2
#   python lerobot_merge_episodes_v21.py \
#       --manifest merge_ur5_gripper_drone.txt \
#       [--intermediate-dir bag_lerobot_intermediate] \
#       [--repo-id foodbanana/ur5_gripper_drone] \
#       [--task "pick up the drone"] \
#       [--fps 25] [--root <출력폴더>]
# =============================================================

import os
import sys
import json
import argparse
from datetime import datetime, timezone

import numpy as np
from PIL import Image

from lerobot.datasets.lerobot_dataset import LeRobotDataset, CODEBASE_VERSION


# ---- 검수 기준 (inspect_parquet.py 와 동일 개념) ----
GRIPPER_MAX = 1150            # 그리퍼 raw 상한 (0=열림, 1150=닫힘)
OVERFLOW_THRESHOLD = 4.0e9    # 42억(4294967295) 근처 이상치 감지선
GRIPPER_ACTION_VALUES = {0.0, 1150.0}   # 그리퍼 action 은 이진


def read_png(path):
    """png 를 (H, W, 3) uint8 RGB 로 읽기 (stage2 와 동일)."""
    img = Image.open(path).convert('RGB')
    return np.asarray(img, dtype=np.uint8)


def load_intermediate(in_dir):
    """1단계 중간 파일 로드 (stage2 와 동일)."""
    states = np.load(os.path.join(in_dir, 'states.npy'))         # (N, 7)
    actions_path = os.path.join(in_dir, 'actions.npy')
    if not os.path.isfile(actions_path):
        raise FileNotFoundError(
            f"actions.npy 가 없음: {in_dir}\n"
            f"       최신 lerobot_stage1_extract_bag.py 로 1단계를 다시 실행하세요."
        )
    actions = np.load(actions_path)                              # (N, 7)
    stamps = np.load(os.path.join(in_dir, 'timestamps.npy'))     # (N,)
    with open(os.path.join(in_dir, 'meta.json')) as f:
        meta = json.load(f)
    return states, actions, stamps, meta


def parse_manifest(path, intermediate_root):
    """매니페스트에서 폴더 이름 목록 읽기. '#' 주석/빈 줄 무시.
    각 폴더의 절대경로를 (이름, 경로) 튜플 리스트로 반환. 날짜시간 이름 정렬."""
    names = []
    with open(path) as f:
        for raw in f:
            line = raw.strip()
            if not line or line.startswith('#'):
                continue
            names.append(line)
    if not names:
        print(f"[오류] 매니페스트에 유효한 폴더가 하나도 없음: {path}")
        sys.exit(1)

    # 중복 제거(순서 보존) 후 이름순 정렬 -> episode_index 결정적
    seen = set()
    uniq = [n for n in names if not (n in seen or seen.add(n))]
    uniq.sort()

    entries = []
    for name in uniq:
        d = os.path.join(intermediate_root, name)
        if not os.path.isdir(d):
            print(f"[오류] 중간 파일 폴더가 없음: {d}")
            sys.exit(1)
        entries.append((name, d))
    return entries


def precheck_episode(name, in_dir, ref):
    """한 에피소드 사전검사. 문제가 있으면 (False, 사유) 반환.
    ref: 첫 에피소드 기준값 dict (없으면 이 에피소드로 채움)."""
    states, actions, stamps, meta = load_intermediate(in_dir)

    N = states.shape[0]
    # 프레임 수 일치 (states / actions / timestamps)
    if actions.shape[0] != N:
        return False, f"states({N}) vs actions({actions.shape[0]}) 프레임 수 불일치"
    if stamps.shape[0] != N:
        return False, f"states({N}) vs timestamps({stamps.shape[0]}) 프레임 수 불일치"

    # png 개수 일치
    for cam in ('head', 'third_view'):
        cam_dir = os.path.join(in_dir, cam)
        if not os.path.isdir(cam_dir):
            return False, f"{cam}/ 폴더 없음"
        n_png = len([p for p in os.listdir(cam_dir) if p.endswith('.png')])
        if n_png != N:
            return False, f"{cam}/ png 개수({n_png}) != 프레임 수({N})"

    state_dim = states.shape[1]
    action_dim = actions.shape[1]
    fps = int(round(meta.get('fps', 25)))
    first_head = read_png(os.path.join(in_dir, 'head', '000000.png'))
    H, W, C = first_head.shape

    # 첫 에피소드 기준 채우기 / 이후 에피소드 일치 검사
    cur = dict(state_dim=state_dim, action_dim=action_dim, fps=fps, hwc=(H, W, C))
    if ref['filled']:
        for k in ('state_dim', 'action_dim', 'fps', 'hwc'):
            if cur[k] != ref[k]:
                return False, f"{k} 불일치: 이 에피소드 {cur[k]} vs 기준 {ref[k]}"
    else:
        ref.update(cur)
        ref['filled'] = True

    # 그리퍼(마지막 열) 검사
    grip_state = states[:, -1]
    grip_action = actions[:, -1]

    # 42억 이상치 / 음수 (state)
    if np.any(grip_state >= OVERFLOW_THRESHOLD):
        cnt = int(np.sum(grip_state >= OVERFLOW_THRESHOLD))
        return False, f"그리퍼 state 42억 이상치 {cnt}개 (clamp 수정 전 bag일 수 있음)"
    if np.any(grip_state < 0):
        cnt = int(np.sum(grip_state < 0))
        return False, f"그리퍼 state 음수 {cnt}개"
    if np.any(grip_state > GRIPPER_MAX + 1):
        mx = float(np.max(grip_state))
        return False, f"그리퍼 state 범위 초과 (max={mx}, 상한 {GRIPPER_MAX})"

    # 그리퍼 action 이진 {0, 1150}
    uniq_a = set(np.unique(grip_action).astype(float).tolist())
    if not uniq_a.issubset(GRIPPER_ACTION_VALUES):
        bad = uniq_a - GRIPPER_ACTION_VALUES
        return False, f"그리퍼 action 이 이진{{0,1150}}이 아님. 이상값: {sorted(bad)[:5]}"

    return True, dict(N=N, state_dim=state_dim, action_dim=action_dim,
                      fps=fps, hwc=(H, W, C),
                      state_names=meta.get('state_names'),
                      action_names=meta.get('action_names'))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--manifest', default='merge_ur5_gripper_drone.txt',
                    help='병합 대상 폴더 목록 파일')
    ap.add_argument('--intermediate-dir', default='bag_lerobot_intermediate',
                    help='중간 파일들이 모여있는 폴더')
    ap.add_argument('--repo-id', default='foodbanana/ur5_gripper_drone',
                    help="병합 데이터셋 repo id ('user/name')")
    ap.add_argument('--task', default='pick up the drone',
                    help='언어 명령 (모든 에피소드 공통)')
    ap.add_argument('--fps', type=int, default=None,
                    help='fps (기본: 각 meta.json 값, 사전검사에서 일치 확인)')
    ap.add_argument('--root', default=None,
                    help='출력 폴더 (기본: <intermediate상위>/lerobot_dataset_v21/<repo_id>)')
    args = ap.parse_args()

    # ── v2.1 형식 확인 ──
    print(f"lerobot CODEBASE_VERSION = {CODEBASE_VERSION}")
    if CODEBASE_VERSION != 'v2.1':
        print(f"[오류] 이 환경 lerobot 은 v2.1 이 아니라 '{CODEBASE_VERSION}'. "
              f"lerobot 0.3.3(lerobot_v2)에서 실행하세요.")
        sys.exit(1)

    # 경로 정리
    manifest = os.path.expanduser(args.manifest)
    if not os.path.isfile(manifest):
        print(f"[오류] 매니페스트 파일 없음: {manifest}")
        sys.exit(1)
    intermediate_root = os.path.expanduser(args.intermediate_dir)
    if not os.path.isdir(intermediate_root):
        print(f"[오류] 중간 파일 폴더 없음: {intermediate_root}")
        sys.exit(1)

    entries = parse_manifest(manifest, intermediate_root)
    print(f"\n=== 병합 대상 {len(entries)}개 (이름순) ===")
    for name, _ in entries:
        print(f"  - {name}")

    # ── Phase 1: 전 에피소드 사전검사 (실패 시 즉시 중단) ──
    print(f"\n=== Phase 1: 사전 검사 ===")
    ref = dict(filled=False)
    infos = []
    for name, d in entries:
        ok, res = precheck_episode(name, d, ref)
        if not ok:
            print(f"[중단] {name}: {res}")
            print("       조용한 fallback 없음. 문제를 고치거나 매니페스트에서 제외 후 다시 실행.")
            sys.exit(1)
        print(f"  [OK] {name}: 프레임 {res['N']}, state {res['state_dim']}, "
              f"이미지 {res['hwc'][0]}x{res['hwc'][1]}")
        res['name'] = name
        res['dir'] = d
        infos.append(res)

    total_frames = sum(r['N'] for r in infos)
    fps = args.fps if args.fps is not None else infos[0]['fps']
    H, W, C = infos[0]['hwc']
    state_dim = infos[0]['state_dim']
    action_dim = infos[0]['action_dim']
    print(f"사전검사 통과. 에피소드 {len(infos)}개, 총 프레임 {total_frames}, fps {fps}")

    # ── 출력 위치 ──
    if args.root:
        root = os.path.expanduser(args.root)
    else:
        collection = os.path.dirname(intermediate_root)   # .../data_collection_ur5_gripper
        root = os.path.join(collection, 'lerobot_dataset_v21', args.repo_id)
    print(f"출력: {root}")
    if os.path.exists(root) and os.listdir(root):
        print(f"\n[중단] 출력 폴더가 이미 있고 비어있지 않음:\n  {root}")
        print("기존 걸 지우거나 --repo-id / --root 로 다른 위치를 지정하세요.")
        sys.exit(1)

    # ── feature 정의 (stage2 와 동일) ──
    state_names = infos[0]['state_names'] or \
        ['shoulder_pan', 'shoulder_lift', 'elbow', 'wrist_1', 'wrist_2', 'wrist_3', 'gripper']
    action_names = infos[0]['action_names'] or state_names
    features = {
        'observation.state': {'dtype': 'float32', 'shape': (state_dim,), 'names': state_names},
        'action': {'dtype': 'float32', 'shape': (action_dim,), 'names': action_names},
        'observation.images.head': {'dtype': 'video', 'shape': (H, W, C),
                                    'names': ['height', 'width', 'channels']},
        'observation.images.third_view': {'dtype': 'video', 'shape': (H, W, C),
                                          'names': ['height', 'width', 'channels']},
    }

    # ── Phase 2: 빌드 (create 1회, save_episode 는 에피소드마다) ──
    print(f"\n=== Phase 2: 병합 빌드 ===")
    dataset = LeRobotDataset.create(
        repo_id=args.repo_id,
        fps=fps,
        features=features,
        root=root,
        robot_type='ur5',
        use_videos=True,
    )

    manifest_record = []
    for ep_idx, r in enumerate(infos):
        name, d, N = r['name'], r['dir'], r['N']
        print(f"[{ep_idx}] {name}  (프레임 {N})")
        states, actions, stamps, meta = load_intermediate(d)
        head_dir = os.path.join(d, 'head')
        third_dir = os.path.join(d, 'third_view')
        for i in range(N):
            dataset.add_frame(
                {
                    'observation.state': states[i].astype(np.float32),
                    'action': actions[i].astype(np.float32),
                    'observation.images.head': read_png(os.path.join(head_dir, f'{i:06d}.png')),
                    'observation.images.third_view': read_png(os.path.join(third_dir, f'{i:06d}.png')),
                },
                task=args.task,
            )
            if (i + 1) % 200 == 0 or i == N - 1:
                print(f"    {i+1}/{N}")
        print(f"    에피소드 저장(비디오 인코딩)...")
        dataset.save_episode()
        manifest_record.append({'episode_index': ep_idx, 'source': name, 'frames': N})

    # ── Phase 3: 추적 기록 ──
    rec = {
        'created_utc': datetime.now(timezone.utc).isoformat(),
        'repo_id': args.repo_id,
        'task': args.task,
        'fps': fps,
        'total_episodes': len(infos),
        'total_frames': total_frames,
        'episodes': manifest_record,
    }
    meta_dir = os.path.join(root, 'meta')
    os.makedirs(meta_dir, exist_ok=True)
    with open(os.path.join(meta_dir, 'merge_manifest.json'), 'w') as f:
        json.dump(rec, f, indent=2, ensure_ascii=False)

    print(f"\n-- 완료 --")
    print(f"데이터셋 위치: {root}")
    print(f"  에피소드 {len(infos)}개, 총 프레임 {total_frames}, fps {fps}")
    print(f"  추적 기록: {os.path.join(meta_dir, 'merge_manifest.json')}")
    print(f"\n검수:")
    print(f"  python3 -c \"import json; d=json.load(open('{os.path.join(root,'meta','info.json')}')); "
          f"print('episodes:',d['total_episodes'],'frames:',d['total_frames'],'ver:',d['codebase_version'])\"")


if __name__ == '__main__':
    main()