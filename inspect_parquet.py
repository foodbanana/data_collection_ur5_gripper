#!/usr/bin/env python3
# =============================================================
# inspect_parquet.py
#   LeRobot v2.1 / v3.0 에피소드 parquet 내용 확인 도구
#
#   parquet 경로를 인자로 받아, 내용을 사람이 보기 좋게 출력한다.
#   - 열 목록, 행(프레임) 수, dtype
#   - observation.state / action 배열을 관절별 열로 펼쳐서 미리보기
#   - 그리퍼 검수: action 그리퍼 값 종류({0,1150} 기대),
#     state 그리퍼 범위(연속), 42억 이상치 유무
#   - (옵션) 펼친 표를 .xlsx 로 저장
#
#   ※ 이미지(head/third_view)는 parquet 에 없고 별도 mp4 로 저장되므로
#     여기서는 숫자 데이터(state/action/메타)만 다룬다.
#
# 사용:
#   python3 inspect_parquet.py <parquet경로> [--rows N] [--xlsx [출력경로]]
# 예:
#   python3 inspect_parquet.py \
#     lerobot_dataset_v21/foodbanana/ur5_gripper_drone_XXX/data/chunk-000/episode_000000.parquet
#   python3 inspect_parquet.py <parquet> --rows 20 --xlsx
#
#   conda 환경(pandas, pyarrow 있는 곳)에서 실행:
#     conda activate lerobot_v2
# =============================================================

import os
import sys
import argparse

import numpy as np
import pandas as pd


# state/action 의 마지막 차원이 그리퍼(6 관절 + 그리퍼 1 = 7)
GRIPPER_IDX = -1
JOINT_LABELS = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6', 'grip']


def is_array_col(series):
    """셀 값이 배열/리스트인 열인지 판별."""
    for v in series:
        if v is None:
            continue
        return isinstance(v, (list, tuple, np.ndarray))
    return False


def stack_col(series):
    """배열 열을 (N, dim) numpy 로 쌓기."""
    return np.stack([np.asarray(v, dtype=np.float64) for v in series])


def labels_for(dim):
    """차원 수에 맞는 열 이름. 7이면 j1..j6,grip, 아니면 c0..c{d-1}."""
    if dim == len(JOINT_LABELS):
        return list(JOINT_LABELS)
    return [f'c{i}' for i in range(dim)]


def main():
    ap = argparse.ArgumentParser(
        description='LeRobot 에피소드 parquet 내용 확인')
    ap.add_argument('parquet', help='episode_XXXXXX.parquet 경로')
    ap.add_argument('--rows', type=int, default=10,
                    help='미리보기 행 수 (기본 10)')
    ap.add_argument('--xlsx', nargs='?', const='__AUTO__', default=None,
                    help='펼친 표를 xlsx 로 저장. 경로 생략 시 parquet 옆에 저장')
    args = ap.parse_args()

    path = os.path.expanduser(args.parquet)
    if not os.path.isfile(path):
        print(f'파일이 아님: {path}')
        sys.exit(1)

    df = pd.read_parquet(path)

    # ── 기본 정보 ──
    print('=' * 60)
    print(f'parquet: {path}')
    print(f'행(프레임) 수: {len(df)}')
    print(f'열 수: {len(df.columns)}')
    print('=' * 60)

    print('\n── 열 목록 (dtype) ──')
    for c in df.columns:
        kind = '배열' if is_array_col(df[c]) else str(df[c].dtype)
        print(f'  {c:35s} [{kind}]')

    # ── 배열 열(state/action)을 펼치기 ──
    array_cols = [c for c in df.columns if is_array_col(df[c])]
    flat = pd.DataFrame()

    for c in array_cols:
        arr = stack_col(df[c])                 # (N, dim)
        dim = arr.shape[1]
        labs = labels_for(dim)
        short = c.replace('observation.', '').replace('.', '_')
        for i, lab in enumerate(labs):
            flat[f'{short}__{lab}'] = arr[:, i]

    # 스칼라 열(timestamp, frame_index 등)도 함께
    scalar_cols = [c for c in df.columns if c not in array_cols]
    for c in scalar_cols:
        # 이미지/바이너리 같이 큰 건 건너뜀
        if df[c].dtype == object:
            sample = df[c].iloc[0] if len(df) else None
            if isinstance(sample, (bytes, dict)):
                continue
        flat[c] = df[c].values

    # ── 미리보기 ──
    n = min(args.rows, len(flat))
    print(f'\n── 앞 {n}행 미리보기 (배열은 관절별 열로 펼침) ──')
    with pd.option_context('display.max_columns', None,
                           'display.width', 200,
                           'display.float_format', lambda x: f'{x:.4f}'):
        print(flat.head(n).to_string())

    # ── 그리퍼 검수 ──
    print('\n── 그리퍼 검수 ──')
    for c in array_cols:
        arr = stack_col(df[c])
        g = arr[:, GRIPPER_IDX]
        short = c.replace('observation.', '').replace('.', '_')
        uniq = np.unique(g)
        over = int((g > 1e9).sum())
        neg = int((g < 0).sum())
        # 값 종류가 많으면 앞 8개만
        uniq_show = uniq if len(uniq) <= 8 else \
            list(np.round(uniq[:8], 2)) + ['...']
        print(f'  [{short}] 그리퍼 열: min {g.min():.1f}, max {g.max():.1f}, '
              f'값종류 {uniq_show}, 42억이상 {over}개, 음수 {neg}개')

    # action 그리퍼가 이진({0,1150})인지, state 그리퍼가 연속인지 요약
    if 'action' in df.columns and 'observation.state' in df.columns:
        ga = stack_col(df['action'])[:, GRIPPER_IDX]
        gs = stack_col(df['observation.state'])[:, GRIPPER_IDX]
        binary_ok = set(np.unique(ga)).issubset({0.0, 1150.0})
        print(f'\n  action 그리퍼 이진(0/1150) 여부: '
              f'{"OK" if binary_ok else "아님 -> " + str(np.unique(ga))}')
        print(f'  state 그리퍼 연속 범위: {gs.min():.0f} ~ {gs.max():.0f} '
              f'(파지 시 중간값이면 정상)')
        # 파지 신호 예시: action=1150 인데 state<1150 인 프레임
        # 파지 신호: 닫기 명령(action=1150) 구간에서 state 가 얼마나 닫혔는지.
        #   닫기 명령 직후엔 state 가 아직 0 (그리퍼가 막 닫히기 시작)이므로,
        #   "명령 구간의 state 최대 지점" = 물체에 막혀 멈춘 실제 파지 깊이를 본다.
        close = (ga == 1150)
        if close.any():
            gs_close = gs[close]
            gmax = gs_close.max()
            i = int(np.where(close)[0][np.argmax(gs_close)])   # 최대 지점의 프레임 번호
            pct = np.round(np.percentile(gs_close, [0, 25, 50, 75, 100])).astype(int)
            print(f'  닫기 명령(action=1150) 프레임 수: {int(close.sum())}')
            print(f'  닫기 명령 중 state 분포 [min/25/50/75/max]: {pct.tolist()}')
            if gmax < 1140:
                print(f'  → 파지 신호 확인: 프레임 {i}에서 action=1150(꽉 닫아라)인데 '
                      f'state={gmax:.0f}에서 멈춤 (물체에 막힘)')
            else:
                print(f'  → state 가 {gmax:.0f}까지 닫힘 (거의 완전 닫힘 = 허공/얇은 물체)')
        else:
            print('  닫기 명령(action=1150) 없음 (이 에피소드엔 그리퍼 닫기 동작이 없음)')

    # ── xlsx 저장 ──
    if args.xlsx is not None:
        if args.xlsx == '__AUTO__':
            out = os.path.splitext(path)[0] + '_view.xlsx'
        else:
            out = os.path.expanduser(args.xlsx)
        try:
            flat.to_excel(out, index=False)
            # 보기 좋게 기본 폰트만 Arial 로
            try:
                from openpyxl import load_workbook
                from openpyxl.styles import Font
                wb = load_workbook(out)
                ws = wb.active
                for row in ws.iter_rows():
                    for cell in row:
                        cell.font = Font(name='Arial', size=10)
                wb.save(out)
            except Exception:
                pass
            print(f'\n[xlsx 저장] {out}  ({flat.shape[0]}행 x {flat.shape[1]}열)')
        except Exception as e:
            print(f'\n[xlsx 저장 실패] {e}')

    print('\n-- 완료 --')


if __name__ == '__main__':
    main()