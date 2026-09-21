#!/usr/bin/env python3
# rosbag2 녹화 토글 (r 키로 시작/종료)
#   r (대기 중) : <YYYYMMDD_HHMMSS>_<작업명> 으로 새 bag 녹화 시작
#   r (녹화 중) : 현재 bag을 SIGINT로 안전 종료 → 대기 상태 복귀
#   d (대기 중) : 방금 저장한 에피소드를 bags/_discarded/ 로 이동 (y 로 확인, 삭제는 안 함)
#   q           : 종료 (녹화 중이면 안전 종료 후 종료)
#   r~r 구간 1개 = bag 1개 = 에피소드 1개
#
# 사용: python3 record_toggle.py [작업명]     (보통 6_record_bag.sh 를 통해 실행)
# 저장: <이 파일이 있는 폴더>/bags/<이름>/

import os
import sys
import tty
import time
import shutil
import signal
import select
import termios
import subprocess
from datetime import datetime

# ── 녹화할 토픽 ──
#   로봇팔 상태 / 그리퍼 상태(present) / 그리퍼 목표(goal) / 그리퍼 명령 / 카메라 color 2대
#   ※ 그리퍼 action 소스는 /gripper/target (30Hz 상시 발행), /gripper/command는 참고용
#   ※ 로봇팔 대신 fake_joint_states.py 를 켜두면 /joint_states 도 채워짐.
TOPICS = [
    '/joint_states',
    '/gripper/joint_states',
    '/gripper/target',
    '/gripper/command',
    '/d435i/d435i/color/image_raw',
    '/d456/d456/color/image_raw',
]

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


class BagRecorder:
    def __init__(self, task_name):
        self.task_name = task_name
        self.proc = None
        self.out_dir = None
        self.t_start = None
        self.episode_count = 0
        self.last_saved = None       # 이번 세션에서 방금 저장한 에피소드 경로 ('d' 대상)
        self.last_elapsed = None

    @property
    def recording(self):
        return self.proc is not None

    def start(self):
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        ep_name = f"{stamp}_{self.task_name}" if self.task_name else stamp
        self.out_dir = os.path.join(BAG_DIR, ep_name)

        cmd = ['ros2', 'bag', 'record',
               '-o', self.out_dir,
               '--disable-keyboard-controls',
               '--topics', *TOPICS]
        # 별도 세션으로 띄움: 터미널의 Ctrl+C가 자식에게 직접 가지 않게 해서
        # SIGINT가 정확히 한 번만(우리가 보낼 때만) 전달되도록 함.
        self.proc = subprocess.Popen(cmd, stdin=subprocess.DEVNULL,
                                     start_new_session=True)
        self.t_start = time.monotonic()
        say(f"\n[REC ●] 녹화 시작: {self.out_dir}")
        say("        'r' 로 녹화 종료")

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


def print_help(task_name):
    say()
    say("=== rosbag2 녹화 토글 ===")
    say(f"  작업명   : {task_name if task_name else '(없음)'}")
    say(f"  저장 위치: {BAG_DIR}/<YYYYMMDD_HHMMSS>{'_' + task_name if task_name else ''}/")
    say("  'r' : 녹화 시작 / 녹화 종료 (토글)")
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
    task_name = sys.argv[1] if len(sys.argv) > 1 else ''

    if shutil.which('ros2') is None:
        say("[ERROR] ros2 명령을 찾을 수 없음. ROS 환경을 source 하거나 6_record_bag.sh 로 실행하세요.")
        sys.exit(1)
    if not sys.stdin.isatty():
        say("[ERROR] 키 입력을 받으려면 터미널에서 직접 실행해야 합니다.")
        sys.exit(1)

    os.makedirs(BAG_DIR, exist_ok=True)
    recorder = BagRecorder(task_name)
    print_help(task_name)

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
                else:
                    recorder.start()
                # 키를 꾹 눌러 생긴 반복 입력 / 종료 대기 중 눌린 키는 버림
                termios.tcflush(fd, termios.TCIFLUSH)
                last_status = time.monotonic()   # 경과시간 표시는 recorder 시작 로그가 지나간 1초 뒤부터
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
        try:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_attr)
        except termios.error:
            pass
        say("\n[EXIT] 종료")


if __name__ == '__main__':
    main()
