#!/usr/bin/env python3
"""06_LEARNING_COURSE.md + 02_PAPERS_CATALOG.md → 손으로 채우는 워크시트(worksheet.pdf, A4).

    python docs/study/build_worksheet.py

노트 앱(굿노트·OneNote·삼성노트 등)에 PDF로 불러와 펜으로 쓰는 용도다 — 쓸 자리는 줄 친 빈칸.
md를 고치면 다시 돌린다. PDF는 Edge 헤드리스 인쇄로 뽑는다(Windows 기본 탑재).
"""
import html
import pathlib
import re
import shutil
import subprocess
import sys
import tempfile

HERE = pathlib.Path(__file__).resolve().parent
OUT = HERE / "worksheet.pdf"
EDGE = [r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe",
        r"C:\Program Files\Microsoft\Edge\Application\msedge.exe"]


STATUS = {"✅": "검증됨", "✏️": "서지 정정됨", "🔁": "교체된 논문"}  # 02 카탈로그의 검증 표시


def md(s):
    """한 줄짜리 md 인라인만: `code` **굵게** *기울임* [글](링크) <url>."""
    s = re.sub(r"\[([^\]]+)\]\([^)]+\)", r"\1", s)
    s = re.sub(r"<(https?://[^>]+)>", r"\1", s)
    s = html.escape(s, quote=False)
    s = re.sub(r"`([^`]+)`", r"<code>\1</code>", s)
    s = re.sub(r"\*\*(.+?)\*\*", r"<b>\1</b>", s)
    s = re.sub(r"(?<![\w*])\*(?!\s)(.+?)(?<!\s)\*(?![\w*])", r"<i>\1</i>", s)
    return s


def parse_modules(text):
    """### Mn. 제목 (기간) 아래의 `- **키**: 글` + 하위 항목을 {키: (글, [항목])}로."""
    mods = []
    for m in re.finditer(r"^### (M\d)\. (.+?)(?: \(([^)]*)\))?\n(.*?)(?=^### |^---|\Z)", text, re.M | re.S):
        keys, cur = {}, None
        for line in m.group(4).splitlines():
            top = re.match(r"^- \*\*(.+?)\*\*(.*)$", line)
            sub = re.match(r"^\s+(?:-|\d+\.) (.*)$", line)
            if top:
                cur = re.sub(r"\(.*?\)", "", top.group(1)).strip()
                keys[cur] = [top.group(2).lstrip(":： ").strip(), []]
            elif sub and cur:
                keys[cur][1].append(sub.group(1).strip())
            elif line.startswith(" ") and line.strip() and cur:
                items = keys[cur][1]
                if items:
                    items[-1] += " " + line.strip()
                else:
                    keys[cur][0] += " " + line.strip()
        mods.append(dict(id=m.group(1), title=m.group(2), span=m.group(3) or "", keys=keys))
    return mods


def cite(item):
    """'저자, "제목", 학회 — 설명'에서 설명을 뗀다."""
    return item.split(" — ")[0].strip()


def lines(n=None):
    return '<div class="lines fill"></div>' if n is None else f'<div class="lines" style="height:{n * 8}mm"></div>'


def box(label, body):
    return f'<div class="box"><div class="lab">{label}</div>{body}</div>'


def page(body, cls=""):
    return f'<section class="page {cls}">{body}</section>'


def paper_note(no, title, meta=""):
    rows = "".join(f'<div class="row"><span>{k}</span>{lines(2)}</div>'
                   for k in ("문제", "입력 → 출력", "핵심 아이디어", "숨은 가정", "우리 로봇에선?"))
    return (f'<div class="paper"><div class="ptitle"><b>{no}</b> {title}</div>'
            f'<div class="pmeta">{meta}<span class="chk">☐ 1회(10분) ☐ 2회(1시간) ☐ 3회(재현) &nbsp; 날짜 ______</span></div>'
            f'{rows}</div>')


def paper_pages(notes):
    return [page("".join(notes[i:i + 2]), "papers") for i in range(0, len(notes), 2)]


def build():
    course = (HERE / "06_LEARNING_COURSE.md").read_text(encoding="utf-8")
    catalog = (HERE / "02_PAPERS_CATALOG.md").read_text(encoding="utf-8")
    mods = parse_modules(course)
    big = re.findall(r"^\| (\d) \| (.+?) \| (.+?) \|$", course.split("## 5.")[1].split("\n## ")[0], re.M)
    cat = re.findall(r"^### \[Paper (\d+)\] (.+?) (✅|✏️|🔁)\n- \*\*(?:저자/연도|출처)\*\*: (.+)$", catalog, re.M)
    assert mods and big and len(cat) == 24, (len(mods), len(big), len(cat))

    pages = [page(f"""
<h1>토마토 로봇 스터디 · 워크시트</h1>
<p class="sub">손목 카메라가 줄기를 보고 자르기까지 — 06 학습 과정(M1~M7)을 손으로 채우는 노트</p>
{box("쓰는 법", "<ol><li><b>보지 않고 먼저 쓴다.</b> 식·그림은 책을 덮고 써 본 뒤 맞춰 본다. 틀린 자리가 공부할 자리다.</li>"
     "<li><b>내 말로 쓴다.</b> 책 문장을 옮기지 말고 \"그래서 우리 로봇에선?\"으로 끝낸다.</li>"
     "<li><b>졸업 질문에 이 페이지만 보고 답할 수 있으면</b> 다음 모듈로 간다.</li>"
     "<li>논문은 칸 다섯 개(문제 / 입력→출력 / 핵심 아이디어 / 숨은 가정 / 우리 로봇에선?)만 채우면 완독이다.</li></ol>")}
{box("속도", "<p>주 6~8시간, 모듈 하나에 2주. 논문은 <b>모듈당 2~3편을 3회 읽기</b>로 하고, "
     "부록의 24편은 1회 읽기(10분)로 지도를 넓힌다. 읽은 편수보다 <b>졸업 질문에 답한 개수</b>가 진도다.</p>")}
{box("차례", "<ol class='toc'><li>워밍업 — 핀홀 카메라를 손으로 유도하기</li>"
     + "".join(f"<li>{m['id']}. {md(m['title'])}</li>" for m in mods)
     + "<li>큰 그림 논문 7편</li><li>부록 — 논문 카탈로그 24편 1회 읽기</li><li>주간 기록 · 워밍업 정답</li></ol>")}
<div class="name">이름 ____________________ &nbsp;&nbsp; 시작한 날 ____________</div>
""")]

    # 워밍업: 01 문서 §1.1~1.2를 손으로
    pages.append(page(f"""
<h2>워밍업 · 핀홀 카메라를 손으로 유도하기</h2>
<p class="hint">책(01_SPATIAL_PERCEPTION §1)을 덮고 쓴다. 막히면 그 단계만 보고 다시 덮는다.</p>
{box("① 닮은꼴 삼각형", "<p>구멍 C, 화면까지 거리 f, 물체 높이 Y, 거리 Z인 옆모습을 그리고 y를 구하라.</p>" + lines(4) + "<p>y = ____________ &nbsp;&nbsp; x = ____________</p>")}
{box("② mm → 픽셀, 원점 옮기기", "<p>왜 f가 fx, fy 둘로 나뉘나? cx, cy는 왜 더하나?</p>" + lines(3) + "<p>u = ____________ &nbsp;&nbsp; v = ____________</p>")}
{box("③ 행렬로 쓰기", "<p>K·[X Y Z]ᵀ 를 직접 곱해 펼쳐라. 결과를 Z로 나누면? <b>왜 1/Z는 행렬 밖에 있나?</b></p>" + lines(5))}
{box("④ 역투영", "<p>②의 식을 X, Y에 대해 풀어라. 깊이 d가 없으면 왜 못 푸나?</p>" + lines(3) + "<p>X = ____________ &nbsp;&nbsp; Y = ____________ &nbsp;&nbsp; Z = ____</p>")}
"""))
    pages.append(page(f"""
<h2>워밍업 · 숫자로 확인하고 코드에서 찾기</h2>
{box("⑤ 숫자로 확인 (fx = fy = 400, cx = 320, cy = 240, 단위 m)",
     "<table><tr><th>3D 점 (X, Y, Z)</th><th>u</th><th>v</th><th>한 줄 해석</th></tr>"
     "<tr><td>(0.05, 0, 0.25)</td><td></td><td></td><td></td></tr>"
     "<tr><td>(0.05, 0, 0.50)</td><td></td><td></td><td></td></tr>"
     "<tr><td>(−0.03, 0.02, 0.30)</td><td></td><td></td><td></td></tr>"
     "<tr><th>(u, v, d) → X, Y</th><th colspan=2>X, Y</th><th></th></tr>"
     "<tr><td>(400, 240, 0.25)</td><td colspan=2></td><td></td></tr></table>")}
{box("⑥ 코드에서 찾기", "<p><code>src/tomato_picker/hardware/handeye.py</code> 109행의 <code>ppx</code>, <code>fx</code>는 위 식의 무엇인가?</p>" + lines(2))}
<div class="box grow"><div class="lab">⑦ 헷갈렸던 것 · 다시 볼 것</div>{lines()}</div>
"""))

    for m in mods:
        k = m["keys"]
        core = k.get("핵심 식") or k.get("핵심") or ["", []]
        books = k.get("책", ["", []])[0]
        pages.append(page(f"""
<div class="mhead"><span>{m['id']}</span>{md(m['title'])}<em>{md(m['span'])}</em></div>
{box("이 모듈의 질문", f"<p class='q'>{md(m['title'])}</p><p class='hint'>{md(core[0])}</p>")}
{box("핵심 식 — 보지 않고 써 보기", lines(3))}
{box("책", f"<p>{md(books)}</p><p class='chk'>☐ 읽음 &nbsp; 읽은 쪽: ______________</p>") if books else ""}
<div class="box grow"><div class="lab">배운 것 — 내 말로, 그림 포함</div>{lines()}</div>
"""))
        tries = k.get("해보기", ["", []])[1]
        extra = ""
        if m["id"] == "M1":
            extra = box("기록표 — 깊이 잡음 vs 거리 (100프레임)",
                        "<table><tr><th>거리</th><th>평균 (mm)</th><th>표준편차 (mm)</th><th>무효 화소 %</th><th>메모</th></tr>"
                        + "".join(f"<tr><td>{d} cm</td><td></td><td></td><td></td><td></td></tr>" for d in (7, 15, 30, 50))
                        + "</table><p class='hint'>표준편차가 거리²에 비례하나? (Keselman 2017)</p>")
        trybody = "".join(f"<div class='try'><p>☐ {md(t)}</p>{lines(3)}</div>" for t in tries)
        grad = k.get("졸업 질문", ["", []])[0]
        pages.append(page(f"""
<div class="mhead small"><span>{m['id']}</span>해보기 · 졸업 질문</div>
{box("해보기", trybody) if tries else ""}
{extra}
<div class="box grow grad"><div class="lab">졸업 질문 — 이 칸만 보고 답할 수 있으면 다음 모듈로</div><p class="q">{md(grad)}</p>{lines()}</div>
"""))
        ps = k.get("논문", ["", []])[1]
        pages += paper_pages([paper_note(f"{m['id']}-{i}", md(cite(p))) for i, p in enumerate(ps, 1)])

    pages += paper_pages([paper_note(f"큰그림 {n}", md(t), f"<span>볼 것: {md(w)}</span>") for n, t, w in big])
    pages += paper_pages([paper_note(f"P{n}", md(t), f"<span>{md(a)} · {STATUS[s]}</span>") for n, t, s, a in cat])

    rows = "".join(f"<tr><td>{w}</td><td></td><td></td><td></td><td></td></tr>" for w in range(1, 15))
    pages.append(page(f"""<h2>주간 기록</h2>
<table class="week"><tr><th>주</th><th>모듈</th><th>한 것</th><th>막힌 것</th><th>다음 주</th></tr>{rows}</table>"""))
    pages.append(page("""<h2>워밍업 정답</h2>
<div class="box"><ol>
<li>y = f·Y/Z, x = f·X/Z — 멀수록(Z↑) 작게 찍힌다.</li>
<li>u = fx·X/Z + cx, v = fy·Y/Z + cy — 화소가 정사각형이 아닐 수 있어 fx≠fy, 화소는 왼쪽 위에서 세므로 화면 중심(cx, cy)을 더한다.</li>
<li>K·[X Y Z]ᵀ = [fx·X + cx·Z, fy·Y + cy·Z, Z]ᵀ → Z로 나누면 [u, v, 1]. 행렬은 곱하기·더하기만 하므로 '나누기'는 밖으로 뺀다(동차좌표).</li>
<li>X = (u − cx)·d/fx, Y = (v − cy)·d/fy, Z = d — 한 화소 뒤에는 한 줄의 점들이 겹쳐 있어 d 없이는 하나로 못 정한다.</li>
<li>(0.05,0,0.25) → u=400, v=240 · (0.05,0,0.50) → u=360, v=240 (멀어지면 중심 쪽으로) · (−0.03,0.02,0.30) → u=280, v≈266.7 · (400,240,0.25) → X=0.05, Y=0.</li>
<li>ppx = cx(주점), fx = 초점거리(화소). 이름만 RealSense식이다.</li>
</ol></div>"""))
    return pages


CSS = """
@page { size: A4; margin: 12mm 14mm; }
* { box-sizing: border-box; -webkit-print-color-adjust: exact; print-color-adjust: exact; }
body { margin: 0; font: 10.5pt/1.55 "Malgun Gothic", "Noto Sans KR", sans-serif; color: #1f2328; background: #fff; }
.page { height: 272mm; display: flex; flex-direction: column; gap: 3mm; page-break-after: always; overflow: hidden; }
.page:last-child { page-break-after: auto; }
h1 { font-size: 22pt; margin: 18mm 0 2mm; } h2 { font-size: 15pt; margin: 0; border-bottom: 2px solid #d1242f; padding-bottom: 1mm; }
.sub { color: #59636e; margin: 0 0 6mm; }
.box { border: 1px solid #d1d9e0; border-radius: 2mm; padding: 2.5mm 3.5mm; }
.box p { margin: 0 0 1.5mm; } .box ol { margin: 0; padding-left: 5mm; }
.grow { flex: 1; display: flex; flex-direction: column; min-height: 0; }
.lab { font-weight: 700; color: #d1242f; font-size: 9.5pt; margin-bottom: 1mm; }
.lines { background: url("data:image/svg+xml,%3Csvg xmlns='http://www.w3.org/2000/svg' width='10' height='80'%3E%3Cline x1='0' y1='79.5' x2='10' y2='79.5' stroke='%23b8bfc8' stroke-width='1'/%3E%3C/svg%3E") 0 0 / 100% 8mm repeat-y; }
.fill { flex: 1; min-height: 16mm; }
.hint { color: #59636e; font-size: 9pt; } .q { font-weight: 700; } .chk { color: #59636e; font-size: 9pt; }
code { font-family: Consolas, monospace; font-size: 9pt; background: #f6f8fa; padding: 0 1mm; border-radius: 1mm; }
.mhead { font-size: 15pt; font-weight: 700; display: flex; align-items: baseline; gap: 3mm; border-bottom: 2px solid #d1242f; padding-bottom: 1mm; }
.mhead span { background: #d1242f; color: #fff; padding: 0 2.5mm; border-radius: 1.5mm; }
.mhead em { margin-left: auto; font-style: normal; font-size: 10pt; color: #59636e; }
.mhead.small { font-size: 12pt; }
.try p { margin: 1mm 0 0; font-size: 9.5pt; }
table { border-collapse: collapse; width: 100%; } th, td { border: 1px solid #d1d9e0; padding: 1.5mm 2mm; text-align: left; height: 10mm; font-size: 9.5pt; }
th { background: #f6f8fa; height: auto; }
.week td { height: 16.5mm; } .week td:first-child { width: 9mm; text-align: center; } .week td:nth-child(2) { width: 18mm; }
.papers { gap: 5mm; } .paper { flex: 1; border: 1px solid #d1d9e0; border-radius: 2mm; padding: 2.5mm 3.5mm; display: flex; flex-direction: column; }
.ptitle { font-size: 10.5pt; } .ptitle b { color: #d1242f; margin-right: 1.5mm; }
.pmeta { display: flex; justify-content: space-between; gap: 3mm; color: #59636e; font-size: 8.5pt; margin: 0.5mm 0 1.5mm; border-bottom: 1px dashed #d1d9e0; padding-bottom: 1mm; }
.pmeta .chk { white-space: nowrap; margin-left: auto; }
.row { display: flex; gap: 2mm; flex: 1; } .row span { width: 24mm; flex-shrink: 0; font-weight: 700; font-size: 9pt; padding-top: 2mm; } .row .lines { flex: 1; height: auto !important; }
.toc { font-size: 10pt; } .name { margin-top: auto; text-align: right; color: #59636e; }
"""


def main():
    edge = next((p for p in EDGE if pathlib.Path(p).exists()), None) or shutil.which("msedge")
    if not edge:
        sys.exit("Edge(msedge)를 찾지 못했다 — worksheet.html을 브라우저에서 PDF로 인쇄하라.")
    pages = build()
    doc = f'<!doctype html><html lang="ko"><head><meta charset="utf-8"><title>토마토 로봇 워크시트</title><style>{CSS}</style></head><body>{"".join(pages)}</body></html>'
    with tempfile.TemporaryDirectory() as tmp:
        src = pathlib.Path(tmp) / "worksheet.html"
        src.write_text(doc, encoding="utf-8")
        subprocess.run([edge, "--headless", "--disable-gpu", "--no-pdf-header-footer", "--print-to-pdf-no-header",
                        f"--user-data-dir={tmp}\\edge", f"--print-to-pdf={OUT}", src.as_uri()],
                       check=True, capture_output=True, timeout=120)
    if not OUT.exists() or OUT.stat().st_size < 10_000:
        sys.exit("PDF가 만들어지지 않았다.")
    print(f"{OUT.name}: {len(pages)}쪽")


if __name__ == "__main__":
    main()
