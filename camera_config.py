#!/usr/bin/env python3
# =============================================================
# camera_config.py  (docs/PLAN.md 4-0)
#
# config/cameras.yaml (카메라 역할 ↔ 장치) 공용 로더. 녹화(record_toggle.py)와 변환(stage1)이 같은 표를 쓴다.
# 항목이 없거나 형식이 다르면 에러 (조용한 fallback 금지).
# =============================================================

import os

import yaml

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_PATH = os.path.join(SCRIPT_DIR, 'config', 'cameras.yaml')
ROLES = ('wrist', 'third_view')          # 데이터셋 스키마의 카메라 키 (observation.images.<역할>)


def load_cameras(path=DEFAULT_PATH):
    """→ {'path', 'resolution': (w, h), 'roles': {역할: {'topic', 'info_topic', 'real': {...}, 'sim': {...}}}}"""
    if not os.path.isfile(path):
        raise FileNotFoundError(f"카메라 설정 파일이 없습니다: {path}")
    with open(path) as f:
        c = yaml.safe_load(f)
    for k in ('topic', 'info_topic', 'resolution', 'roles'):
        if k not in c:
            raise KeyError(f"{path}: '{k}' 항목이 없습니다")
    if set(c['roles']) != set(ROLES):
        raise ValueError(f"{path}: roles 는 {list(ROLES)} 이어야 합니다 (지금 {list(c['roles'])})")
    roles = {}
    for role in ROLES:
        r = c['roles'][role]
        for k in ('real', 'sim'):
            if k not in r:
                raise KeyError(f"{path}: 'roles.{role}.{k}' 항목이 없습니다")
        for k in ('model', 'serial'):
            if not r['real'].get(k):
                raise KeyError(f"{path}: 'roles.{role}.real.{k}' 항목이 없습니다")
        if not isinstance(r['real']['serial'], str):
            raise TypeError(f"{path}: 'roles.{role}.real.serial' 은 따옴표로 감싼 문자열이어야 합니다 (숫자로 읽히면 앞자리 0 이 사라짐)")
        roles[role] = {'topic': c['topic'].format(role=role), 'info_topic': c['info_topic'].format(role=role),
                       'real': dict(r['real']), 'sim': dict(r['sim'])}
    w, h = (int(x) for x in c['resolution'])
    return {'path': os.path.abspath(path), 'resolution': (w, h), 'roles': roles}


if __name__ == '__main__':
    cams = load_cameras()
    print(f"{cams['path']}  {cams['resolution'][0]}x{cams['resolution'][1]}")
    for role, r in cams['roles'].items():
        print(f"  {role:10s} {r['topic']:34s} real {r['real']['model']} {r['real']['serial']}  sim {r['sim']['config']}")
