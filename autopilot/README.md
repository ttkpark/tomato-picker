# 자동운전 — 15일 동안 일을 안 끊는 장치

`claude -p`를 **역할 넷**으로 번갈아 띄운다. 각 사이클은 새 세션이고, 연속성은
문맥이 아니라 **파일**(작업판 · 일지 · git)이 잇는다. 그래서 문맥이 넘쳐도, PC가
꺼졌다 켜져도 다음 사이클이 이어 간다.

| 역할 | 무엇을 | 산출물 |
|---|---|---|
| **planner** | 지금부터 할 수 있는 일을 계속 찾아 작업판을 채운다 | 작업판 갱신 · 졸업 채점 · `[사람]` 할 일 |
| **builder** | 계속 만든다. 한 사이클에 한 덩이 | 코드 · 자체검증 통과 |
| **auditor** | 명세와 실물을 대조하고 빠진 디테일을 채운다 | 문서 · 새 검사 · 구체적 작업 |
| **tester** | 계속 시험하고 숫자를 남긴다 | 자체검증 종수 · 실기 기록 jsonl |

기본 한 바퀴: `planner → builder → tester → builder → auditor → builder → tester`
(`config.json`의 `rotation`).

## 켜기

```powershell
python autopilot\runner.py run          # 지금 바로 (이 창을 닫으면 멈춘다)
powershell -ExecutionPolicy Bypass -File autopilot\install-task.ps1   # 재부팅·크래시를 넘기려면
```

## 끄기 · 멈추기

| 하고 싶은 것 | 하는 법 |
|---|---|
| 잠깐 멈춤(내가 직접 만질 동안) | `autopilot\PAUSE` 파일을 만든다. 지우면 재개 |
| 완전히 끝 | `autopilot\STOP` 파일을 만든다 |
| 스케줄러에서 떼기 | `install-task.ps1 -Remove` |

## 보기

```powershell
python autopilot\runner.py status       # 며칠째 · 사이클 · 비용 · 젯슨 IP · 열린 일
python autopilot\board.py list --open --wide
type autopilot\journal\2026-09-17.md    # 오늘 무슨 일이 있었나
type autopilot\logs\runner.log
```

아침에 **일지 하나만** 읽으면 된다. 사람이 풀어 줘야 하는 것은 플래너가
`[사람]`을 붙여 p1로 올려 둔다.

## 손으로 한 사이클만

```powershell
python autopilot\runner.py once --role builder
python autopilot\runner.py probe        # 젯슨 IP만 다시 찾기
```

## 설계 결정 (왜 이렇게)

- **왜 순차인가** — 넷이 같은 작업 트리와 같은 git 인덱스를 쓴다. 동시에 돌리면 서로의
  편집을 덮고 `index.lock`에서 싸운다. 대신 한 바퀴를 짧게(=자주) 돈다.
- **왜 매번 새 세션인가** — 15일을 한 문맥으로 끌 수 없다. 이어 가는 힘을 문맥이 아니라
  파일에 두면 길이가 문제가 안 된다.
- **왜 작업판이 CLI인가** — 에이전트가 JSON을 손으로 고치다 깨뜨리면 루프가 멈춘다.
  `board.py`만 쓰게 하고, 파일 락 + 원자적 교체로 지킨다.
- **왜 매 사이클 커밋인가** — 히스토리가 곧 되돌리기다. 자동운전이 뭔가를 망가뜨려도
  어느 사이클인지 바로 보인다.
- **왜 젯슨 IP를 찾아다니나** — DHCP라 바뀐다. 바뀌었다고 15일이 멈추면 안 된다.
  후보 IP → 서브넷 22번 포트 훑기 순서로 찾고 `state.json`에 기억한다.

## 파일

```
OBJECTIVE.md     최상위 목표와 합격 기준 (플래너만 고친다)
config.json      주기·모델·브랜치·젯슨·예산
roles/*.md       역할 헌장 (+ _공통.md 는 넷 모두에게 붙는다)
board.py         작업판 CLI
runner.py        감독자 루프
state/           board.json · state.json · heartbeat.json
journal/         날짜별 일지 (사람이 읽는 것)
logs/            사이클별 프롬프트·원문 출력 (git에 안 들어감)
watchdog.ps1     죽으면 되살림
install-task.ps1 작업 스케줄러 등록
```

## 안전 장치

- master 브랜치에서는 켜지지 않는다.
- 역할 시스템 프롬프트에서 금지: master 푸시 · force push · 태그 삭제 · `reset --hard`
  · 저장소 밖 삭제 · `runner.py`/`board.py` 수정.
- 실기(팔·주행)는 `config.json`의 `hardware_allowed`로 끈다. 끄면 젯슨을 읽기만 한다.
- 사이클 상한 45분. 넘으면 프로세스 트리째 죽인다(장치 포트를 놓게).
- 연속 실패는 4·8·16분… 로 물러나고, 6회면 30분 쉰다(사용량 제한은 기다리면 풀린다).
