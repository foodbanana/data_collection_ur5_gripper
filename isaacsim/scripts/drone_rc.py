# =============================================================
# drone_rc.py — 실물 조종기로 sim 의 PX4 드론 조종 (docs/PLAN.md 5단계 5-A, docs/drone_teleop.md)
#
#   DroneRc: 루프마다 update() → 조종기 입력 (rc_input.RcInput) 을 PX4Commander 에 넣고 넘기기·되돌리기를 정한다
#     - 넘기기: 드론이 Offboard 로 호버 중 (이륙·리셋이 끝남, cmd.cli_ready) + 조종기 신호가 있음 + 네 축이 가운데 근처 → handover()
#       (PX4 수동 모드 POSCTL, 스틱 값은 명령 링크 UDP 14540 의 MANUAL_CONTROL). 가운데가 아니면 넘기지 않고 알린다
#     - 되돌리기: 넘긴 뒤 조종기 신호가 끊기거나 입력이 잘못되면 takeback() → 그 자리에서 Offboard 호버, 터미널에 알림.
#       신호가 돌아오고 스틱이 가운데면 다시 넘긴다
#     - 리셋 (/sim/reset) 은 PX4 를 새로 띄우므로 넘김 상태가 풀린다 → 재이륙이 끝나면 위 조건으로 다시 넘어간다
#     - 모터 정지 (/sim/drone_kill) 뒤에는 아무것도 하지 않는다 (다음 리셋까지)
#   PX4 드론 전용 (기하 제어기 드론에는 MANUAL_CONTROL 이 없음). 조종기 입력은 녹화하지 않는다
#   조종기를 쓸 때의 PX4 파라미터 (스틱 끝 속도 등) 는 rc_input.yaml px4_params → 드론 설정 px4.params 에 합쳐 PX4 시작 때 넣는다
# =============================================================

import rc_input

NOTICE_SEC = 5.0       # 같은 알림을 다시 내는 간격 (sim s)


class DroneRc:
    def __init__(self, flight, cfg, port=None):
        if flight.backend != "px4":
            raise ValueError(f"조종기 텔레옵은 PX4 드론에서만 (지금 {flight.backend})")
        self.f, self.cmd, self.cfg = flight, flight.cmd, cfg
        self.rc = rc_input.RcInput(cfg, port)
        self.sticks, self.why = None, "받은 줄 없음"
        self.n_handover, self.n_takeback = 0, 0
        self._notice = (None, -1e9)

    def _say(self, msg, repeat=True):
        last, t = self._notice
        if msg != last or (repeat and self.f.t - t >= NOTICE_SEC):
            print(f"[drone_rc] t {self.f.t:7.1f} s  {msg}", flush=True)
            self._notice = (msg, self.f.t)

    def update(self):
        c = self.cmd
        self.sticks, self.why = self.rc.poll()
        if c.want_manual:
            if self.sticks is None:
                c.takeback()
                self.n_takeback += 1
                self._say(f"조종기 입력 없음 ({self.why}) → 드론을 그 자리 호버로 되돌림 (Offboard)")
            else:
                c.set_sticks(*self.sticks)
            return
        if not (c.cli_ready and c.connected and c.armed):
            return                                              # 이륙·리셋 중이거나 모터 정지 뒤
        if self.sticks is None:
            self._say(f"조종기 입력 없음 ({self.why}) → 넘기지 않음 (드론은 자동 호버)")
            return
        if c.sticks is None:
            c.set_sticks(0.0, 0.0, 0.0, 0.0)                    # PX4 가 조종 입력이 있다고 보게 가운데 값부터 보냄
        if c.mode != "OFFBOARD":
            return                                              # 되돌리는 중 (Offboard 로 바뀌기를 기다림)
        if not self.rc.map.centered(self.sticks):
            names = [n for n, x in zip(rc_input.AXES, self.sticks) if abs(x) > self.cfg["handover_center"]]
            self._say(f"스틱이 가운데가 아님 ({', '.join(names)}) → 가운데에 두면 조종기에 넘김")
            return
        c.set_sticks(*self.sticks)
        c.handover()
        self.n_handover += 1
        self._say("조종기에 넘김 (PX4 수동 모드 요청)", repeat=False)

    def summary(self):
        return (f"조종기: 넘김 {self.n_handover} 번, 되돌림 {self.n_takeback} 번, 지금 {'조종기' if self.cmd.want_manual else '자동'} "
                f"(모드 {self.cmd.mode}), 스틱 {None if self.sticks is None else [round(x, 2) for x in self.sticks]}")

    def close(self):
        self.rc.close()
