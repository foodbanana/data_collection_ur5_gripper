#!/bin/bash
# =============================================================
# 0_setup_network.sh
# 노트북 <-> UR5 이더넷 연결 설정 (부팅 후 1회만 실행)
# sudo 권한 필요 (IP 부여/인터페이스 활성화)
# =============================================================

# ── 사용자 환경에 맞게 수정 ──
IFACE="enx00e04c680cc8"      # 유선 이더넷 인터페이스 이름 (ip link 로 확인)
LAPTOP_IP="192.168.56.5"     # 노트북 IP
ROBOT_IP="192.168.56.10"     # UR5 로봇 IP

echo "[0] 네트워크 인터페이스 목록:"
ip link

echo ""
echo "[0] '${IFACE}' 에 IP ${LAPTOP_IP}/24 부여 + 활성화..."
sudo ip addr add ${LAPTOP_IP}/24 dev ${IFACE}
sudo ip link set ${IFACE} up

echo ""
echo "[0] 로봇(${ROBOT_IP}) 연결 확인 (ping 4회)..."
ping -c 4 ${ROBOT_IP}

echo ""
echo "[0] 완료. ping 응답이 왔으면 성공."
echo "    (응답이 없으면: 케이블/로봇전원/인터페이스 이름 확인)"
echo ""
echo "    다음: 폴리스코프에서 External Control 프로그램 준비."
echo "    캘리브레이션 파일(~/my_robot_calibration.yaml)이 없다면 아래 실행:"
echo ""
echo "    ros2 launch ur_calibration calibration_correction.launch.py \\"
echo "      robot_ip:=${ROBOT_IP} \\"
echo "      target_filename:=\"\${HOME}/my_robot_calibration.yaml\""
