#!/usr/bin/env python3
# rosbag2 녹화 토글 (r 키로 시작/종료). sim·실물 공용 (docs/PLAN.md 4-1)
#   r (대기 중) : <YYYYMMDD_HHMMSS>_<작업명> 으로 새 bag 녹화 시작
#                 --sim 이면 먼저 /sim/reset (에피소드 리셋, 약 24 s) 을 부르고 성공 응답이 온 뒤 녹화 시작
#   r (녹화 중) : 현재 bag을 SIGINT로 안전 종료 → 대기 상태 복귀
#   k (--sim)   : 드론 모터 정지 /sim/drone_kill (잡은 뒤. 시각을 episode.json 에 기록). 다시 켜는 것은 다음 r 의 리셋
#   d (대기 중) : 방금 저장한 에피소드를 bags/_discarded/ 로 이동 (y 로 확인, 삭제는 안 함). 실패한 에피소드는 이렇게 버린다
#   q           : 종료 (녹화 중이면 안전 종료 후 종료)
#   r~r 구간 1개 = bag 1개 = 에피소드 1개
#
# 사용: python3 record_toggle.py [작업명] [--sim]     (보통 6_record_bag.sh 를 통해 실행)
# 저장: <이 파일이 있는 폴더>/bags/<이름>/  (+ episode.json: 녹화 정보, 카메라 역할 → 장치, sim 은 리셋 응답)
#
# 녹화 시작 전에 상태 토픽 (팔·그리퍼·카메라) 에 발행자가 있는지 확인하고, 없으면 녹화하지 않는다 (조용히 빈 bag 을 만들지 않음).
# 카메라는 RELIABLE 로 받는다: best effort 면 640x480 이미지가 UDP 조각 손실로 통째로 버려진다 (docs/sim_performance.md 5장)

import os
import sys
import tty
import json
import time
import shutil
import signal
import select
import termios
import argparse
import tempfile
import subprocess
from datetime import datetime

from camera_config import load_cameras

# ── 녹화할 토픽 ──
#   상태 토픽 (STATE): 항상 발행되고 있어야 한다. 녹화 시작 전에 발행자를 확인
#   명령 토픽 (COMMAND): 텔레옵 장치가 발행. 녹화 중에 나타나도 ros2 bag record 가 찾아서 받는다
#     /joint_command = 팔 action, /gripper/target = 그리퍼 action (30Hz 상시 발행), /gripper/command 는 참고용
#   카메라 토픽은 config/cameras.yaml (역할 → /cam/<역할>/color/image_raw, camera_info)
STATE_TOPICS = ['/joint_states', '/gripper/joint_states', '/gripper/target']
COMMAND_TOPICS = ['/joint_command', '/gripper/command']
SIM_STATE_TOPICS = ['/protective_stop', '/clock']     # sim 전용 (보호 정지 흉내, sim time)

SIM_RESET_SERVICE = '/sim/reset'
SIM_KILL_SERVICE = '/sim/drone_kill'
SIM_RESET_TIMEOUT = 120.0   # [s, 실제 시간] 리셋은 약 24 s (sim). 이 시간 안에 응답이 없으면 에러

# 녹화기 수신 큐 길이 [s 분량]. 녹화기 (ros2 bag record) 는 뜬 직후 0.4~0.7 s 동안 메시지를 꺼내 가지 못할 때가 있고
#   (다른 노드가 같이 뜰 때 30 번 중 10 번), 그동안 온 메시지는 수신 큐에 쌓인다. 큐가 짧으면 (기본 10 개 = 카메라 0.33 s) 넘쳐서 버려진다
#   → 상태 토픽마다 이만큼의 메시지를 담을 수 있게 QoS override 로 큐를 늘린다 (docs/PLAN.md 4단계 남은 문제 A1)
QUEUE_SEC = 3.0
TOPIC_HZ = {'/joint_states': 125.0, '/clock': 120.0}     # 나머지 상태 토픽은 30 Hz

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
BAG_DIR = os.path.join(SCRIPT_DIR, 'bags')
DISCARD_DIR = os.path.join(BAG_DIR, '_discarded')   # 버린 에피소드 보관 (실제 삭제는 수동으로)

STOP_WARN_SEC = 10.0   # 종료 대기 중 이 간격마다 안내 출력 (강제 종료는 하지 않음)


IDLE_MSG = "[IDLE] 대기 중. 'r' 녹화 시작 / 'd' 방금 에피소드 버리기 / 'q' 종료"


def say(*args, **kwargs):
    # 터미널 창이 닫힌 뒤(SIGHUP)에도 출력 오류 때문에 bag 안전 종료가 막히지 않도록
    try:
        print(*args, **kwargs)
    except OSError:
        pass


class RosLink:
    """녹화 전 토픽 확인과 sim 리셋 호출 (rclpy). 녹화 자체는 ros2 bag record 프로세스가 한다."""

    def __init__(self, sim, cameras):
        import rclpy
        from rclpy.signals import SignalHandlerOptions
        from std_srvs.srv import Trigger

        self.rclpy, self.sim, self.cameras = rclpy, sim, cameras
        # 신호 처리는 이 스크립트가 한다 (Ctrl+C → bag 안전 종료)
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self.node = rclpy.create_node('record_toggle')
        self.reset_cli = self.node.create_client(Trigger, SIM_RESET_SERVICE) if sim else None
        self.kill_cli = self.node.create_client(Trigger, SIM_KILL_SERVICE) if sim else None
        self.Trigger = Trigger
        self.image_topics = [r['topic'] for r in cameras['roles'].values()]
        self.info_topics = [r['info_topic'] for r in cameras['roles'].values()]

    @property
    def state_topics(self):
        return STATE_TOPICS + self.image_topics + self.info_topics + (SIM_STATE_TOPICS if self.sim else [])

    @property
    def topics(self):
        return self.state_topics + COMMAND_TOPICS

    def spin(self, sec):
        self.rclpy.spin_once(self.node, timeout_sec=sec)

    def check_topics(self, wait=2.0):
        """상태 토픽마다 발행자가 있는지, 카메라 발행자가 RELIABLE 인지 확인. → 문제 목록 (비면 정상)"""
        from rclpy.qos import ReliabilityPolicy

        t_end = time.monotonic() + wait          # 노드를 만든 직후에는 발견(discovery)이 덜 끝났을 수 있다
        while True:
            problems = []
            for t in self.state_topics:
                pubs = self.node.get_publishers_info_by_topic(t)
                if not pubs:
                    problems.append(f"{t}: 발행자 없음")
                elif t in self.image_topics and any(p.qos_profile.reliability != ReliabilityPolicy.RELIABLE for p in pubs):
                    problems.append(f"{t}: 발행자가 RELIABLE 이 아님 (best effort 로는 이미지가 통째로 버려짐. 카메라 드라이버 QoS 를 RELIABLE 로)")
            if not problems or time.monotonic() > t_end:
                return problems
            self.spin(0.1)

    def _call(self, cli, name, timeout, on_tick=None):
        """Trigger 서비스를 부르고 응답을 기다린다. → (성공 여부, 응답 JSON dict 또는 오류 문자열)"""
        if not cli.wait_for_service(timeout_sec=2.0):
            return False, f"{name} 서비스가 없음 (sim_ros2.py 가 떠 있는지 확인)"
        fut = cli.call_async(self.Trigger.Request())
        t0 = time.monotonic()
        while not fut.done():
            self.spin(0.1)
            el = time.monotonic() - t0
            if on_tick is not None:
                on_tick(el)
            if el > timeout:
                return False, f"{name} 응답이 {timeout:g} s 안에 없음"
        res = fut.result()
        try:
            info = json.loads(res.message)
        except json.JSONDecodeError:
            return False, f"{name} 응답이 JSON 이 아님: {res.message!r}"
        if not res.success:
            return False, f"{name} 실패: {info.get('error', res.message)}"
        return True, info

    def sim_reset(self, on_tick=None):
        """에피소드 리셋 (약 24 s). 응답 = 시드, 새 드론 위치 등"""
        return self._call(self.reset_cli, SIM_RESET_SERVICE, SIM_RESET_TIMEOUT, on_tick)

    def drone_kill(self):
        """드론 모터 정지. 응답 = sim 시각"""
        return self._call(self.kill_cli, SIM_KILL_SERVICE, 5.0)

    def close(self):
        self.node.destroy_node()
        self.rclpy.shutdown()


def write_qos_overrides(link):
    """ros2 bag record QoS override 파일 (상태 토픽). → 경로
    - 큐 길이: QUEUE_SEC 분량 (녹화기가 뜬 직후 못 꺼내 가는 동안 넘치지 않게)
    - 카메라 이미지는 RELIABLE (check_topics 가 발행자도 RELIABLE 인지 확인). 다른 토픽은 발행자와 같은 reliability
      (발행자가 best effort 인데 RELIABLE 로 받으면 연결이 안 되어 조용히 빈 토픽이 된다)"""
    from rclpy.qos import ReliabilityPolicy

    f = tempfile.NamedTemporaryFile('w', prefix='record_toggle_qos_', suffix='.yaml', delete=False)
    for t in link.state_topics:
        pubs = link.node.get_publishers_info_by_topic(t)
        reliable = t in link.image_topics or (bool(pubs) and all(p.qos_profile.reliability == ReliabilityPolicy.RELIABLE for p in pubs))
        depth = int(round(QUEUE_SEC * TOPIC_HZ.get(t, 30.0)))
        f.write(f"{t}:\n  history: keep_last\n  depth: {depth}\n  reliability: {'reliable' if reliable else 'best_effort'}\n  durability: volatile\n")
    f.close()
    return f.name


class BagRecorder:
    def __init__(self, task_name, link):
        self.task_name = task_name
        self.link = link
        self.qos_path = None         # QoS override 파일 (녹화 시작마다 발행자 QoS 를 보고 다시 씀)
        self.proc = None
        self.out_dir = None
        self.t_start = None
        self.info = None             # 녹화 중인 에피소드의 episode.json 내용
        self.episode_count = 0
        self.last_saved = None       # 이번 세션에서 방금 저장한 에피소드 경로 ('d' 대상)
        self.last_elapsed = None

    @property
    def recording(self):
        return self.proc is not None

    def start(self, reset_info=None):
        """녹화 시작. 상태 토픽에 문제가 있으면 시작하지 않고 False. reset_info: sim 리셋 응답 (episode.json 에 기록)"""
        problems = self.link.check_topics()
        if problems:
            say("\n[ERROR] 녹화를 시작하지 않음 — 토픽 문제:")
            for p in problems:
                say(f"          {p}")
            say(IDLE_MSG)
            return False
        self._remove_qos_file()
        self.qos_path = write_qos_overrides(self.link)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        ep_name = f"{stamp}_{self.task_name}" if self.task_name else stamp
        self.out_dir = os.path.join(BAG_DIR, ep_name)
        cams = self.link.cameras
        mode = 'sim' if self.link.sim else 'real'
        self.info = {
            'episode': ep_name, 'mode': mode, 'task_name': self.task_name, 'recorded_at': datetime.now().isoformat(timespec='seconds'),
            'topics': self.link.topics, 'camera_reliability': 'reliable', 'queue_sec': QUEUE_SEC,
            'cameras': {role: {'topic': r['topic'], **r[mode]} for role, r in cams['roles'].items()},
            'sim_reset': reset_info,
            'drone_kill': [],            # 녹화 중 k 키로 드론 모터를 끈 시각 (sim time)
        }

        cmd = ['ros2', 'bag', 'record',
               '-o', self.out_dir,
               '--disable-keyboard-controls',
               '--qos-profile-overrides-path', self.qos_path,
               '--topics', *self.link.topics]
        # 별도 세션으로 띄움: 터미널의 Ctrl+C가 자식에게 직접 가지 않게 해서
        # SIGINT가 정확히 한 번만(우리가 보낼 때만) 전달되도록 함.
        self.proc = subprocess.Popen(cmd, stdin=subprocess.DEVNULL,
                                     start_new_session=True)
        self.t_start = time.monotonic()
        say(f"\a\n[REC ●] 녹화 시작: {self.out_dir}")
        say("        'r' 로 녹화 종료")
        return True

    def stop(self):
        """SIGINT로 안전 종료. 버퍼 flush + metadata.yaml 마무리까지 기다림 (SIGKILL 금지)."""
        if self.proc is None:
            return
        if self.proc.poll() is None:
            self.proc.send_signal(signal.SIGINT)
        say("\n[REC ■] 녹화 종료 중... (bag 마무리 대기)")
        while True:
            try:
                self.proc.wait(timeout=STOP_WARN_SEC)
                break
            except subprocess.TimeoutExpired:
                say("        아직 마무리 중... (mcap 손상 방지를 위해 강제 종료하지 않음)")
            except KeyboardInterrupt:
                # 마무리 중 Ctrl+C는 무시하고 계속 기다림
                pass
        self._finish()

    def check_alive(self):
        """녹화 중 ros2 bag record가 스스로 죽었는지 확인."""
        if self.proc is not None and self.proc.poll() is not None:
            say(f"\n[ERROR] ros2 bag record가 예기치 않게 종료됨 (exit code {self.proc.returncode})")
            self._finish()
            return False
        return True

    def _finish(self):
        elapsed = time.monotonic() - self.t_start
        if os.path.isfile(os.path.join(self.out_dir, 'metadata.yaml')):
            self.info['wall_duration_s'] = round(elapsed, 2)
            with open(os.path.join(self.out_dir, 'episode.json'), 'w') as f:
                json.dump(self.info, f, ensure_ascii=False, indent=2)
            self.episode_count += 1
            self.last_saved = self.out_dir
            self.last_elapsed = elapsed
            say(f"[SAVED] {self.out_dir}  ({elapsed:.1f}s)  "
                f"— 이번 세션 {self.episode_count}번째 에피소드")
        else:
            say(f"[WARN] metadata.yaml 없음 → bag이 정상 마무리되지 않았을 수 있음: {self.out_dir}")
        self.proc = None
        self.out_dir = None
        self.t_start = None
        self.info = None
        say(IDLE_MSG)

    def discard_last(self):
        """방금 저장한 에피소드를 bags/_discarded/ 로 이동 (삭제하지 않음)."""
        os.makedirs(DISCARD_DIR, exist_ok=True)
        dst = os.path.join(DISCARD_DIR, os.path.basename(self.last_saved))
        try:
            os.rename(self.last_saved, dst)
        except OSError as e:
            say(f"[ERROR] 이동 실패: {e}")
            return
        self.episode_count -= 1
        self.last_saved = None
        self.last_elapsed = None
        say(f"[DISCARDED] → {dst}")
        say(f"            (되살리려면 이 폴더를 {BAG_DIR}/ 로 다시 옮기면 됨) "
            f"— 이번 세션 에피소드 {self.episode_count}개")

    def kill_drone(self):
        """--sim: 드론 모터 정지. 녹화 중이면 시각을 episode.json 에 남긴다."""
        ok, info = self.link.drone_kill()
        if not ok:
            say(f"\n[ERROR] {info}")
            return
        say(f"\n[KILL] 드론 모터 정지 (sim t {info.get('t')} s). 다시 켜려면 녹화를 끝내고 'r' (리셋)")
        if self.recording:
            self.info['drone_kill'].append(info)

    def _remove_qos_file(self):
        if self.qos_path is not None:
            try:
                os.unlink(self.qos_path)
            except OSError:
                pass
            self.qos_path = None

    def close(self):
        self._remove_qos_file()


def reset_then_record(recorder, link):
    """--sim: /sim/reset → 성공하면 녹화 시작. 리셋 동안 sim 은 팔·그리퍼 명령을 무시한다."""
    say(f"\n[RESET] {SIM_RESET_SERVICE} 호출 (약 24 s: 팔 홈, 그리퍼 열림, 드론 재이륙). 끝나면 녹화가 바로 시작됩니다")
    ok, info = link.sim_reset(on_tick=lambda el: say(f"\r[RESET] {el:5.1f} s ", end='', flush=True))
    if not ok:
        say(f"\n[ERROR] {info}")
        say("        녹화를 시작하지 않음. sim 터미널 로그를 확인하고 'r' 로 다시 시도")
        say(IDLE_MSG)
        return False
    say(f"\n[RESET] 완료: 시드 {info.get('seed')}, 드론 {info.get('drone_pos')}, sim {info.get('duration_s')} s")
    return recorder.start(reset_info=info)


def print_help(task_name, link):
    say()
    say("=== rosbag2 녹화 토글 ===")
    say(f"  모드     : {'sim (r → /sim/reset → 녹화)' if link.sim else '실물'}")
    say(f"  작업명   : {task_name if task_name else '(없음)'}")
    say(f"  저장 위치: {BAG_DIR}/<YYYYMMDD_HHMMSS>{'_' + task_name if task_name else ''}/")
    say(f"  토픽     : {' '.join(link.topics)}")
    say("  'r' : 녹화 시작 / 녹화 종료 (토글)")
    if link.sim:
        say("  'k' : 드론 모터 정지 (잡은 뒤)")
    say("  'd' : 방금 저장한 에피소드 버리기 → bags/_discarded/ 로 이동 ('y' 로 확인)")
    say("  'q' : 종료 (녹화 중이면 안전 종료 후 종료)")
    say("=========================")
    say("(이 터미널에 포커스를 두고 키를 누르세요)")
    say(IDLE_MSG)


def read_key_nonblocking():
    # sys.stdin.read(1)은 파이썬 내부 버퍼에 여러 글자를 미리 읽어둬서 tcflush로 못 버림
    # → os.read로 직접 읽고, 한 번에 들어온 입력 중 첫 글자만 사용 (나머지 = 키 반복, 버림)
    fd = sys.stdin.fileno()
    dr, _, _ = select.select([fd], [], [], 0)
    if dr:
        data = os.read(fd, 64).decode('utf-8', errors='ignore')
        if data:
            return data[0]
    return None


def _raise_interrupt(signum, frame):
    raise KeyboardInterrupt


def main():
    ap = argparse.ArgumentParser(description='rosbag2 녹화 토글 (r 시작/종료, d 버리기, q 종료)')
    ap.add_argument('task_name', nargs='?', default='', help='bag 이름 뒤에 붙는 작업명')
    ap.add_argument('--sim', action='store_true', help='Isaac Sim: r 을 누르면 /sim/reset 뒤 녹화 시작, sim 전용 토픽도 녹화')
    args = ap.parse_args()
    task_name = args.task_name

    if shutil.which('ros2') is None:
        say("[ERROR] ros2 명령을 찾을 수 없음. ROS 환경을 source 하거나 6_record_bag.sh 로 실행하세요.")
        sys.exit(1)
    if not sys.stdin.isatty():
        say("[ERROR] 키 입력을 받으려면 터미널에서 직접 실행해야 합니다.")
        sys.exit(1)

    os.makedirs(BAG_DIR, exist_ok=True)
    link = RosLink(args.sim, load_cameras())
    recorder = BagRecorder(task_name, link)
    print_help(task_name, link)

    # 터미널 창 닫힘(SIGHUP) / kill(SIGTERM) 시에도 bag을 안전 종료하도록
    signal.signal(signal.SIGTERM, _raise_interrupt)
    signal.signal(signal.SIGHUP, _raise_interrupt)

    fd = sys.stdin.fileno()
    old_attr = termios.tcgetattr(fd)
    last_status = 0.0
    confirm_discard = False   # 'd' 를 누른 뒤 'y' 확인을 기다리는 중

    try:
        tty.setcbreak(fd)
        while True:
            ch = read_key_nonblocking()
            if confirm_discard and ch is not None:
                # 'd' 다음 키: y 면 이동, 그 외 아무 키나 취소 (그 키는 다른 동작으로 이어지지 않음)
                confirm_discard = False
                if ch in ('y', 'Y', 'ㅛ'):
                    recorder.discard_last()
                else:
                    say("[CANCEL] 버리기 취소 — 에피소드 유지")
                say(IDLE_MSG)
                termios.tcflush(fd, termios.TCIFLUSH)
            elif ch in ('q', 'Q', 'ㅂ'):
                break
            elif ch in ('d', 'D', 'ㅇ'):
                if recorder.recording:
                    say("\n[INFO] 녹화 중에는 버릴 수 없음. 먼저 'r' 로 녹화를 종료하세요.")
                elif recorder.last_saved is None:
                    say("\n[INFO] 버릴 에피소드 없음 (이번 세션에서 방금 저장한 에피소드만 대상)")
                else:
                    confirm_discard = True
                    say(f"\n[DISCARD?] {os.path.basename(recorder.last_saved)} "
                        f"({recorder.last_elapsed:.1f}s) 를 _discarded/ 로 옮기려면 'y' (다른 키 = 취소)")
                termios.tcflush(fd, termios.TCIFLUSH)
            elif ch in ('r', 'R', 'ㄱ'):   # 'ㄱ'/'ㅂ'/'ㅇ'/'ㅛ' = 한글 입력 상태에서의 r/q/d/y
                if recorder.recording:
                    recorder.stop()
                elif args.sim:
                    reset_then_record(recorder, link)
                else:
                    recorder.start()
                # 키를 꾹 눌러 생긴 반복 입력 / 종료·리셋 대기 중 눌린 키는 버림
                termios.tcflush(fd, termios.TCIFLUSH)
                last_status = time.monotonic()   # 경과시간 표시는 recorder 시작 로그가 지나간 1초 뒤부터
            elif ch in ('k', 'K', 'ㅏ') and args.sim:   # 실물 드론은 조종자가 끈다 (실물 모드에는 k 없음)
                recorder.kill_drone()
                termios.tcflush(fd, termios.TCIFLUSH)
            elif ch is not None:
                say(f"\n지원하지 않는 키: {repr(ch)}")

            if recorder.recording and recorder.check_alive():
                now = time.monotonic()
                if now - last_status >= 1.0:
                    last_status = now
                    sec = int(now - recorder.t_start)
                    say(f"\r[REC ●] {sec // 60:02d}:{sec % 60:02d} ", end='', flush=True)
            time.sleep(1.0 / 100.0)
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        signal.signal(signal.SIGHUP, signal.SIG_IGN)
        if recorder.recording:
            recorder.stop()
        recorder.close()
        link.close()
        try:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_attr)
        except termios.error:
            pass
        say("\n[EXIT] 종료")


if __name__ == '__main__':
    main()
