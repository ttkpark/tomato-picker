#!/usr/bin/env bash
# 하루 종일 역보정 — PC에서 돈다: 젯슨 수집(kincal_collect) → 가져오기 → 풀이(kincal_solve) → 그 모형으로 다음 라운드.
#   bash ros2/tools/kincal_day.sh <첫 라운드 번호> <끝낼 시각 HH:MM> [이미 있는 세션...]
#   예) bash ros2/tools/kincal_day.sh 3 20:00 s1 s2
# 환경변수: WALLPLANE=벽 평면 json(깊이로 찾은 것) · NEAR_DEG=가 본 자세 둘레로만(도, 0=전 범위) · ROUND_N=라운드당 장수.
# 멈춤: 젯슨에서 touch ~/kincal/STOP (수집기가 대기 자세로 돌아가 끝낸다) — 이 스크립트도 그걸 보고 끝난다.
# 남겨 둔 자세 오차가 6px를 넘으면(모형이 나빠졌다) 다음 라운드를 안 띄우고 끝낸다.
set -u
cd "$(dirname "$0")/../.."
R=${1:?라운드 번호}; UNTIL=${2:?끝낼 시각}; shift 2
SES=("$@")
J="server@100.85.187.87"; K="-i /c/Users/parkg/.ssh/id_ed25519 -o ConnectTimeout=15 -o ServerAliveInterval=30"
D=data/kincal
export PYTHONIOENCODING=utf-8

pull() { mkdir -p "$D/$1"; scp $K -q "$J:kincal/$1/samples.jsonl" "$J:kincal/$1/session.json" "$D/$1/"; }

while :; do
  for s in "${SES[@]}"; do pull "$s" || { echo "가져오기 실패 $s"; exit 1; }; done
  python ros2/tools/kincal_solve.py "${SES[@]/#/$D/}" --out "$D/m_r$R.json" ${WALLPLANE:+--wallplane "$WALLPLANE"} > "$D/solve_r$R.log" 2>&1 || { echo "풀이 실패 r$R"; tail -5 "$D/solve_r$R.log"; exit 1; }
  sed -n '/단계\]/,/가장 나쁜/p' "$D/solve_r$R.log"
  HOLD=$(python -c "import json;print(json.load(open('$D/m_r$R.json'))['holdout_px'])")
  python -c "import sys;sys.exit(0 if float('$HOLD')<6.0 else 1)" || { echo "남겨 둔 자세 ${HOLD}px > 6 — 모형을 못 믿는다. 멈춘다."; exit 2; }
  [ "$(date +%H:%M)" \< "$UNTIL" ] || { echo "끝낼 시각 $UNTIL — 마지막 모형 $D/m_r$R.json"; exit 0; }
  ssh $K $J 'test -e ~/kincal/STOP' && { echo "STOP 파일 — 끝"; exit 0; }
  NEAR=${NEAR_DEG:-0}; N=${ROUND_N:-600}                  # 모형이 깊이·남겨 둔 자세로 검증된 뒤로는 전 범위(NEAR_DEG=25 처럼 좁힐 수 있다)
  scp $K -q "$D/m_r$R.json" "$J:kincal/m_r$R.json" || exit 1
  ARGS="--out ~/kincal/s$R --model ~/kincal/m_r$R.json --n $N --seed $R"
  [ $NEAR -gt 0 ] && ARGS="$ARGS --radius $NEAR --near $(printf '~/kincal/%s ' "${SES[@]}")"
  echo "=== 라운드 $R 시작 $(date +%H:%M) · 반경 $NEAR · $N장 · 지금까지 ${#SES[@]}세션 (남김 ${HOLD}px)"
  ssh $K $J "cd ~/tomato-picker; nohup setsid ~/lerobot/.venv/bin/python -u ros2/tools/kincal_collect.py $ARGS > ~/kincal/s$R.log 2>&1 < /dev/null &"
  sleep 60
  # ssh가 잠깐 끊겨도 "끝났다"로 읽지 않는다 — DONE이라고 또렷이 답할 때만 넘어간다
  while [ "$(ssh $K $J 'pgrep -f "[k]incal_collect.py" >/dev/null && echo RUN || echo DONE')" != DONE ]; do sleep 60; done
  ssh $K $J "tail -3 ~/kincal/s$R.log"
  ssh $K $J "grep -q '추종 오차 3연속\|예외로 중단\|안전하면서 보드가 보이는 자세가 없다\|길이 안전하지 않다' ~/kincal/s$R.log" && { echo "수집기가 스스로 멈췄다 — 사람이 봐야 한다"; exit 3; }
  SES+=("s$R"); R=$((R + 1))
done
