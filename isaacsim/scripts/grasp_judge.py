# =============================================================
# grasp_judge.py — 드론 파지 자동 판정 (docs/PLAN.md 2-4, 3단계 에피소드 메타데이터·7단계 sim 평가에서도 씀)
#
# 실물 시나리오: 떠 있는 드론을 잡는다 → 드론 모터 정지 → 사람이 떼어 감. 그래서 성공 = held (내려놓기는 하지 않음, 2026-10-01)
# 상태: approach → grasped → held (성공)  / 실패하면 failed + 사유. 매 관측(update)마다 판정
#   grasped: 닫기 명령 중 그리퍼가 멈춤(stop_time 동안 위치 변화 < stop_vel·stop_time; 관절 속도 읽기는 쓰지 않음, 1-5),
#            빈손 각도(empty_close)보다 덜 닫힘, 양쪽 손가락 모두 드론 접촉력 > contact_force
#   held:    모터 정지 뒤 hold_time 이 지났을 때 TCP 기준 드론 내려앉음 ≤ settle_max, 마지막 still_window 동안 변화 ≤ still_mm,
#            회전 ≤ hold_rot. 아래에서 잡으면 무게가 실려 조금(약 5~6 mm) 내려앉은 뒤 멈추는 것이 정상
#   실패 사유: no_grasp_empty(빈손으로 끝까지 닫힘), no_grasp_contact(멈췄지만 양쪽 접촉 없음), slip(잡은 뒤 미끄러짐),
#            dropped(잡은 뒤 TCP 기준 drop 이상 멀어짐), opened_early(held 전에 그리퍼 열림), timeout:<상태>
# 관측 dict: t, grip_q, grip_closing, armed, f_right, f_left, drone_p, drone_R, tcp_p, tcp_R
# =============================================================

import math
from collections import deque

import numpy as np

JUDGE_KEYS = ("stop_vel", "stop_time", "empty_close_deg", "contact_force", "hold_time", "settle_max_mm", "still_window",
              "still_mm", "hold_rot_deg", "drop_mm")


def rot_angle(R):
    return math.degrees(math.acos(np.clip((np.trace(R) - 1) / 2, -1, 1)))


class GraspJudge:
    def __init__(self, cfg, q_pred=None):
        for k in JUDGE_KEYS:
            if k not in cfg:
                raise KeyError(f"judge.{k} 항목이 없습니다")
        self.c = cfg
        self.q_pred = q_pred
        self.state, self.reason = "approach", None
        self.events = []                    # (t, 상태, 정보)
        self.hist = deque()                 # (t, grip_q) 닫는 중
        self.rel0 = None                    # 잡은 순간 TCP 기준 드론 (R, p)
        self.t_release = None
        self.max_slip_mm, self.max_rot_deg = 0.0, 0.0
        self.settle_mm = None               # 모터 정지 뒤 내려앉음 (held 순간)
        self.recent = deque()               # (t, TCP 기준 드론 위치) 모터 정지 뒤
        self.grasp_info = {}

    # ── 보조 ──
    def _rel(self, o):
        Rt = o["tcp_R"]
        return Rt.T @ o["drone_R"], Rt.T @ (o["drone_p"] - o["tcp_p"])

    def _drift(self, o):
        R, p = self._rel(o)
        return float(np.linalg.norm(p - self.rel0[1]) * 1000), rot_angle(self.rel0[0].T @ R)

    def _go(self, t, state, **info):
        self.state = state
        self.events.append((t, state, info))

    def fail(self, t, reason, **info):
        if self.done:
            return
        self.reason = reason
        self._go(t, "failed", reason=reason, **info)

    @property
    def done(self):
        return self.state in ("failed", "held")

    # ── 판정 ──
    def update(self, o):
        if self.done:
            return
        c, t = self.c, o["t"]
        if self.state == "approach":
            if not o["grip_closing"]:
                self.hist.clear()
                return
            self.hist.append((t, o["grip_q"]))
            while self.hist and self.hist[0][0] < t - c["stop_time"]:
                self.hist.popleft()
            if t - self.hist[0][0] < 0.9 * c["stop_time"] or len(self.hist) < 3:
                return
            if abs(self.hist[-1][1] - self.hist[0][1]) >= c["stop_vel"] * c["stop_time"]:
                return
            q = o["grip_q"]
            info = dict(q_deg=math.degrees(q), f_right=o["f_right"], f_left=o["f_left"])
            if self.q_pred is not None:
                info["q_pred_deg"] = math.degrees(self.q_pred)
            if math.degrees(q) >= c["empty_close_deg"]:
                self.fail(t, "no_grasp_empty", **info)
            elif min(o["f_right"], o["f_left"]) < c["contact_force"]:
                self.fail(t, "no_grasp_contact", **info)
            else:
                self.rel0 = self._rel(o)
                self.grasp_info = info
                self._go(t, "grasped", **info)
            return

        # grasped: 모터 정지를 기다려 버티는지 본다
        if not o["grip_closing"]:
            self.fail(t, "opened_early")
            return
        slip, rot = self._drift(o)
        self.max_slip_mm, self.max_rot_deg = max(self.max_slip_mm, slip), max(self.max_rot_deg, rot)
        if slip > c["drop_mm"]:
            self.fail(t, "dropped", slip_mm=slip, rot_deg=rot)
            return
        if o["armed"]:
            return
        if self.t_release is None:
            self.t_release = t
        self.recent.append((t, self._rel(o)[1].copy()))
        while self.recent and self.recent[0][0] < t - c["still_window"]:
            self.recent.popleft()
        if slip > c["settle_max_mm"] or rot > c["hold_rot_deg"]:
            self.fail(t, "slip", phase="settle", slip_mm=slip, rot_deg=rot)
            return
        if t - self.t_release >= c["hold_time"]:
            moved = float(np.linalg.norm(self.recent[-1][1] - self.recent[0][1]) * 1000)
            if moved > c["still_mm"]:
                self.fail(t, "slip", phase="still", moved_mm=moved, slip_mm=slip)
                return
            self.settle_mm = slip
            self._go(t, "held", settle_mm=slip, still_mm=moved, rot_deg=rot)

    def finish(self, t):
        if not self.done:
            self.fail(t, f"timeout:{self.state}")
        return self.result()

    def result(self):
        return {"success": self.state == "held", "state": self.state, "reason": self.reason,
                "events": [(round(t, 3), s, {k: (round(v, 4) if isinstance(v, float) else v) for k, v in i.items()})
                           for t, s, i in self.events],
                "max_slip_mm": round(self.max_slip_mm, 3), "max_rot_deg": round(self.max_rot_deg, 3),
                "settle_mm": None if self.settle_mm is None else round(self.settle_mm, 3),
                "grasp": {k: (round(v, 4) if isinstance(v, float) else v) for k, v in self.grasp_info.items()}}
