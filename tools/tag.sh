#!/usr/bin/env bash
# 태그를 규칙대로 단다 — docs/ros2-이행계획.md §2.
#
#     tools/tag.sh v2.0.0-ros.2 "손-눈 보정 실측 통과"
#     tools/tag.sh --list
#
# 왜 스크립트인가 — 태그 규칙은 "두 계통이 태그 문자열에서 드러나야 한다"인데,
# 그건 사람이 지키기로 한 약속이라 **반드시 한 번은 어긴다.** 어긴 태그는
# 이미 밀어 놓은 뒤에야 눈에 띄고, 그때는 되돌리면 남의 클론이 깨진다.
# 그래서 다는 순간에 막는다.
#
# ⚠ 이 스크립트는 **밀지 않는다.** 태그를 미는 것은 되돌릴 수 없는 행위라
#    사람이 한 번 더 보고 눌러야 한다. 명령줄은 마지막에 찍어 준다.
set -euo pipefail

cd "$(dirname "$0")/.."

usage() {
  cat <<'USAGE'
사용법:
  tools/tag.sh <태그> "<한 줄 설명>"
  tools/tag.sh --list

태그 형식 (docs/ros2-이행계획.md §2)
  기존 스택   v1.MINOR.PATCH        예) v1.0.0    — master 계열에서만
  ROS 계통    v2.0.0-ros.N          예) v2.0.0-ros.2 — ros 브랜치에서만
  대체 완료   v2.0.0

  SemVer 우선순위가 곧 계통의 선후다:  1.0.0 < 2.0.0-ros.1 < 2.0.0
USAGE
}

if [ $# -eq 0 ] || [ "${1:-}" = "-h" ] || [ "${1:-}" = "--help" ]; then
  usage; exit 0
fi

if [ "${1:-}" = "--list" ]; then
  echo "기존 스택 (v1.*)"
  git tag -l 'v1.*' --sort=-v:refname | sed 's/^/  /' || true
  echo "ROS 계통 (v2.0.0-ros.*)"
  git tag -l 'v2.0.0-ros.*' --sort=-v:refname | sed 's/^/  /' || true
  echo "대체 완료 (v2.0.0)"
  git tag -l 'v2.0.0' | sed 's/^/  /' || true
  exit 0
fi

TAG="$1"
MESSAGE="${2:-}"
BRANCH="$(git rev-parse --abbrev-ref HEAD)"

if [ -z "$MESSAGE" ]; then
  echo "✗ 설명이 없다. 태그는 '이 커밋이 어느 이정표인가'를 말해야 한다." >&2
  echo "   예) tools/tag.sh $TAG \"손-눈 보정 실측 통과\"" >&2
  exit 1
fi

# --- 형식 ---
LINEAGE=""
case "$TAG" in
  v1.[0-9]*.[0-9]*)        LINEAGE="legacy" ;;
  v2.0.0-ros.[0-9]*)       LINEAGE="ros" ;;
  v2.0.0)                  LINEAGE="merged" ;;
  *)
    echo "✗ 형식에 안 맞는 태그: $TAG" >&2
    echo >&2
    usage >&2
    exit 1
    ;;
esac

# --- 계통 ↔ 브랜치 ---
# 계통을 섞어 달면 태그 목록이 더 이상 계통을 말해 주지 못한다. 그게 이 규칙의 전부다.
case "$LINEAGE" in
  legacy)
    if [[ "$BRANCH" == *ros* ]]; then
      echo "✗ ROS 브랜치($BRANCH)에 기존 계통 태그($TAG)를 달려고 한다." >&2
      echo "   ROS 계통의 이정표라면 v2.0.0-ros.N 을 써라." >&2
      exit 1
    fi ;;
  ros)
    if [[ "$BRANCH" != *ros* ]]; then
      echo "✗ ROS 태그($TAG)를 ROS 브랜치가 아닌 곳($BRANCH)에 달려고 한다." >&2
      echo "   기존 계통의 이정표라면 v1.MINOR.PATCH 를 써라." >&2
      exit 1
    fi ;;
esac

if git rev-parse -q --verify "refs/tags/$TAG" >/dev/null; then
  echo "✗ $TAG 는 이미 있다: $(git log -1 --format='%h %s' "$TAG")" >&2
  echo "   태그는 옮기지 않는다(남의 클론이 조용히 다른 것을 가리킨다)." >&2
  echo "   다음 번호를 써라." >&2
  exit 1
fi

if [ -n "$(git status --porcelain)" ]; then
  echo "⚠ 작업 트리가 깨끗하지 않다 — 태그는 **커밋**을 가리키므로, 지금 고친 것은"
  echo "   이 태그에 안 들어간다:"
  git status --short | sed 's/^/     /'
  printf "   그래도 달까? [y/N] "
  read -r answer
  [ "$answer" = "y" ] || { echo "취소."; exit 1; }
fi

git tag -a "$TAG" -m "$MESSAGE"
echo "✓ $TAG  ($LINEAGE)  ← $(git log -1 --format='%h %s')"
echo
echo "밀려면:  git push origin $TAG"
echo "  (미는 것은 되돌릴 수 없다 — 한 번 더 보고 눌러라)"
