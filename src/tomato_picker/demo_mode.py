"""오프라인 데모 모드 — 망 없이 도는 결정적 자동 시퀀스.

부스 와이파이가 불안정하거나 API가 죽어도 전체 수확 데모가 한 번에
돌아가게 하는 필수 폴백. Claude 없이 스킬 함수만 순서대로 호출한다.
"""

from __future__ import annotations

from .config import TREES
from .skills import TomatoPicker


def run_demo(robot: TomatoPicker) -> None:
    """나무를 순회하며 빨강(익은) 열매만 모두 수확하고 복귀."""
    print("[데모 모드] 익은 토마토 전체 수확 시퀀스 시작\n")

    for tree_id in sorted(TREES):
        print(f"== 나무 {tree_id} ==")
        print(robot.drive_to_tree(tree_id))
        scan = robot.scan_tree()
        ripe_ids = [f["id"] for f in scan["fruits"] if f["ripe"]]
        print(f"  스캔: 열매 {len(scan['fruits'])}개, 익은 것 {len(ripe_ids)}개")

        for fid in ripe_ids:
            print("  " + robot.pick_fruit(fid))
            print("  " + robot.place_in_basket())
        print()

    print(robot.home())
    print(f"\n[데모 모드] 완료 — 총 {robot.basket_count}개 수확")
