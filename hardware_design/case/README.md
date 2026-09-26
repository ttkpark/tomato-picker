# 케이스 (트레이 + 뚜껑 + 180° 회전판)

원본 = [`tomato_case.scad`](tomato_case.scad) (OpenSCAD 2021.01에서 컴파일 확인).
치수 출처 = [`docs/design/3D-parts-mapping.md`](../../docs/design/3D-parts-mapping.md).

![조립](../../docs/design/case-render-assembly.png)

## 구조
| 부품 | 들어가는 것 |
|---|---|
| `tray` | 젯슨·아두이노·배터리·전원 컨버터. 바닥이 차체 상판의 **M3 50×30 패턴 두 벌**에 그대로 박힌다. 좌우 환기, 뒷벽 포트, 전면 "ForNerds" 양각 |
| `lid` | 회전 베어링 하부 링 볼트, 밑에 STS3215 서보 걸이, 180° 배선 슬롯, 스토퍼 홈 |
| `platform` | SO-101 베이스 구멍, Orbbec 1/4" 나사 자리, 서보 혼, 베어링 상부 링, 스토퍼 핀 |

## 설계 판단 (바꾸기 전에 읽을 것)
- **팔 무게는 베어링(레이지수잔, 구매품)이 받는다.** 팔을 뻗으면 서보 축에 굽힘 모멘트가
  걸려 축·기어가 먼저 망가진다. 서보는 돌리기만 한다.
- **배선은 중심을 못 지난다** — 서보 축이 중심에 있다. 뚜껑의 180° 호형 슬롯을 따라
  회전판의 구멍이 함께 돈다. 슬롯은 서보 몸체 반대편(−X).
- **180°는 스토퍼 핀+홈으로 기계적으로 막는다** — 소프트 리밋만 믿으면 오지령 한 번에 배선이 끊긴다.

## 뽑기
```
openscad -D 'part="tray"'     -o tray.stl     tomato_case.scad
openscad -D 'part="lid"'      -o lid.stl      tomato_case.scad
openscad -D 'part="platform"' -o platform.stl tomato_case.scad
```
- `lid`는 뒤집어서(서보 걸이가 위로) 출력.
- 콘솔에 `⚠`가 뜨면 베드 초과·부품 겹침이다.

## 출력 전 실측 (scad의 `[실측]` 줄)
차체 상판 외곽 · 50×30 패턴 두 벌 위치 · 보드/배터리/컨버터 위치 · 젯슨 구멍 ·
SO-101 베이스 구멍 · 베어링 규격 · 서보 축 위치·혼 규격. 지금 값은 형상 확인용 가짜다.
