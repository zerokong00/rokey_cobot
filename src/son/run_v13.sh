#!/usr/bin/env bash
# v1_3 통합 시연 실행기 — 모드 3가지뿐이다.
#
#   ./run_v13.sh floor1     아래층만 (T 분기 정찰)
#   ./run_v13.sh floor2     윗층만   (결함 2곳 용접 수리)
#   ./run_v13.sh both       두 층 동시   ← 기본값
#
# 뒤에 붙이는 인자는 그대로 넘어간다:
#   ./run_v13.sh floor2 --headless        창 없이
#   ./run_v13.sh both --nocam             카메라 끄고(가볍게)
#   ./run_v13.sh floor2 --steps 50000     스텝 수 지정
#   ./run_v13.sh floor2 --glass2          floor2 배관 유리(v1_2 --glass 와 같음)
#   ./run_v13.sh floor2 --detect          검출 창(센터링·관경 링·결함) 띄우기
#
# 🚨 배관 유리·검증 레시피·ROS 도메인은 전부 내장이다 — 따로 줄 것이 없다.
set -e

COURSE="${1:-both}"
shift || true

ISAAC=~/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release
HUMBLE="$ISAAC/exts/isaacsim.ros2.bridge/humble"

# Isaac(3.11) 내장 rclpy — ROS 발행에 필요. 런처 환경변수로 줘야 로드된다.
export LD_LIBRARY_PATH="$HUMBLE/lib:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="$HUMBLE/rclpy:${PYTHONPATH:-}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-143}"      # 팀 규격서 값
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export PYTHONUNBUFFERED=1
export DISPLAY="${DISPLAY:-:1}"

cd "$(dirname "$0")"

# 🎯 `--detect` 면 검출 뷰어를 **같이** 띄운다.
#    🚨 뷰어는 반드시 **시스템 python3** — Isaac 내장 cv2 는 headless
#       빌드라 imshow 가 없다(tkinter·PyQt5 도 없음, 실측).
case " $* " in
  *" --detect "*)
    rm -f /dev/shm/cobot3_detect.jpg
    python3 tools/detect_view.py &
    VIEWER=$!
    trap 'kill $VIEWER 2>/dev/null' EXIT
    ;;
esac

"$ISAAC/python.sh" real_map_demo_v1_3.py \
     --course "$COURSE" --hold --steps 220000 "$@"
