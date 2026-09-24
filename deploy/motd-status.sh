#!/bin/bash
# 로그인 시 지금 뭘 쓸 수 있는지 실측해서 보여줌. ~/.bashrc에서 source.

echo "── tomato-picker 현재 상태 ($(hostname), $(date '+%m-%d %H:%M')) ──"

# 배타적 리소스 (팔 포트/8090/주행): 동시에 하나만 뜬다
for svc in click-server tomato-voice controller-drive; do
    if systemctl is-active --quiet "$svc" 2>/dev/null; then
        echo "  ● $svc  실행중"
    fi
done

# 카메라 발행기 (D405/Astra/line-cam도 동시 제약)
for svc in depth-cam astra-cam line-cam; do
    state=$(systemctl is-active "$svc" 2>/dev/null)
    [ "$state" = "active" ] && echo "  ● $svc  실행중"
done

# 시리얼 장치
[ -e /dev/ttyUSB0 ] && echo "  ✓ /dev/ttyUSB0 (모터보드)"
[ -e /dev/ttyACM0 ] && echo "  ✓ /dev/ttyACM0 (팔로워)"

# X11 forwarding
if [ -n "$DISPLAY" ]; then
    echo "  ✓ X11 DISPLAY=$DISPLAY"
    echo "    xclock          - 연결 테스트"
    echo "    xterm &         - 터미널 창 추가"
    echo "    nautilus &      - 파일 탐색기"
    echo "    xeyes           - 데모용"
fi

ip=$(hostname -I | awk '{print $1}')
listening=$(ss -ltnH 2>/dev/null | awk '{print $4}' | sed -E 's/.*://' | sort -un)
for p in $listening; do
    case "$p" in
        8090) echo "  → 줄기 잡기 조작대: http://$ip:8090" ;;
        5000|8080|8000) echo "  → 대시보드(추정): http://$ip:$p" ;;
    esac
done
echo "──────────────────────────────────────────"
