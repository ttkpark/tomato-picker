#!/usr/bin/env python3
"""역보정 수집 현황판 — 젯슨에서 띄우고 Tailscale로 본다(표준 라이브러리뿐, 읽기만 한다).

    nohup python3 ros2/tools/kincal_status.py > ~/kincal/status.log 2>&1 &
    → http://100.85.187.87:8091/      (10초마다 스스로 새로 고침)

~/kincal 의 수집 기록(s*.log · s*/samples.jsonl)과 마지막 모형(m_r*.json), 손목 카메라의 지금 화면을 보여 준다.
팔 포트도 카메라도 열지 않는다 — 수집기가 써 둔 파일만 읽는다.
"""
import glob
import html
import json
import os
import subprocess
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

ROOT = os.path.expanduser("~/kincal")
LIVE = "/dev/shm/d405_color.jpg"
PORT = 8091


def _num(path):
    return int("".join(c for c in os.path.basename(path).split(".")[0] if c.isdigit()) or 0)


def _tail(path, n):
    try:
        with open(path, "rb") as fh:
            fh.seek(0, 2)
            fh.seek(max(0, fh.tell() - 4000 * n))               # 표본 한 줄이 1.5KB쯤 된다
            return fh.read().decode("utf-8", "replace").splitlines()[-n:]
    except OSError:
        return []


def _last_json(path):
    for line in reversed(_tail(path, 3)):
        try:
            return json.loads(line)
        except ValueError:
            continue
    return None


def page() -> str:
    running = subprocess.call(["pgrep", "-f", "kincal_collect.py --out"], stdout=subprocess.DEVNULL) == 0
    logs = sorted(glob.glob(os.path.join(ROOT, "s*.log")), key=_num)
    logs = [p for p in logs if os.path.basename(p)[1:-4].isdigit()]
    rows, total = [], 0
    for p in logs:
        name = os.path.basename(p)[:-4]
        try:
            n = sum(1 for _ in open(os.path.join(ROOT, name, "samples.jsonl"), "rb"))
        except OSError:
            n = 0
        text = open(p, encoding="utf-8", errors="replace").read()
        end = next((line for line in reversed(text.splitlines()) if line.startswith("끝")), "")
        total += n
        rows.append("<tr><td>%s</td><td>%d</td><td>%d</td><td>%s</td><td>%s</td></tr>" % (
            name, n, text.count("⚠"), time.strftime("%H:%M", time.localtime(os.path.getmtime(p))), html.escape(end[:60])))
    cur = logs[-1] if logs else ""
    name = os.path.basename(cur)[:-4] if cur else ""
    last = _last_json(os.path.join(ROOT, name, "samples.jsonl")) if name else None
    models = sorted(glob.glob(os.path.join(ROOT, "m_r*.json")), key=_num)
    mtxt = "아직 없음"
    if models:
        try:
            m = json.load(open(models[-1], encoding="utf-8"))
            w = m.get("wall") or {}
            mtxt = ("<b>%s</b> (%s) · 단계 %s · 자세 %d (밑동 넘어감 %s 뺌) · 재투영 <b>%.2f px ≈ %.2f mm</b> · "
                    "남겨 둔 자세 <b>%.2f px</b>%s") % (
                os.path.basename(models[-1]), time.strftime("%H:%M", time.localtime(os.path.getmtime(models[-1]))),
                m.get("stage"), m.get("frames", 0), m.get("rocking", "-"), m.get("rms_px", 0), m.get("rms_mm", 0),
                m.get("holdout_px", 0),
                " · 벽 마커(독립 검증) %d장 중앙 %.1f px ≈ %.1f mm" % (w["frames"], w["px_median"], w["mm"]) if w else "")
        except (OSError, ValueError, KeyError, TypeError) as e:
            mtxt = "읽기 실패: %s" % e
    tele = ""
    if last:
        tele = "마지막 표본 #%d (%s 전) · 서보 최고 %d°C · 전압 %.1fV · 보드 모서리 %d개 · 추종 오차 %.1f°" % (
            last["i"], "%d초" % (time.time() - last["t"]), max(last["temp"]), sorted(last["volt"])[len(last["volt"]) // 2] / 10.0,
            len(last["ch"]), last["err_deg"])
    limg = "/sample.jpg?%d" % (last["i"] if last else 0)
    return """<!doctype html><html lang="ko"><head><meta charset="utf-8"><meta http-equiv="refresh" content="10">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>역보정 현황</title>
<style>body{font-family:system-ui,sans-serif;margin:16px;background:#14181d;color:#dfe6ee;max-width:1000px}
h1{font-size:20px}h2{font-size:15px;margin:18px 0 6px;color:#9fb3c8}table{border-collapse:collapse;width:100%%}
td,th{border-bottom:1px solid #2b333d;padding:4px 8px;text-align:left;font-size:14px}pre{background:#0c0f13;padding:10px;
overflow-x:auto;font-size:12px;line-height:1.35}img{max-width:100%%;border-radius:6px}.g{display:grid;grid-template-columns:
repeat(auto-fit,minmax(300px,1fr));gap:12px}.on{color:#5fd38d}.off{color:#f2a65a}small{color:#8a97a6}</style></head><body>
<h1>역보정 수집 현황 <span class="%s">● %s</span> <small>%s · 누적 %d장</small></h1>
<p>%s</p><h2>지금 모형</h2><p>%s</p>
<div class="g"><div><h2>손목 카메라 지금</h2><img src="/live.jpg?%d"></div><div><h2>마지막으로 찍은 표본</h2><img src="%s"></div></div>
<h2>라운드</h2><table><tr><th>세션</th><th>장수</th><th>경고</th><th>마지막 기록</th><th>끝난 까닭</th></tr>%s</table>
<h2>%s 기록(끝 15줄)</h2><pre>%s</pre>
<p><small>멈추기: 젯슨에서 <code>touch ~/kincal/STOP</code> — 팔이 시작 자세로 돌아가 토크를 켠 채 끝낸다. 이 화면은 파일만 읽는다.</small></p>
</body></html>""" % ("on" if running else "off", "수집 중" if running else "멈춤", time.strftime("%m-%d %H:%M:%S"), total,
                     html.escape(tele), mtxt, int(time.time()), limg, "".join(rows), html.escape(name),
                     html.escape("\n".join(_tail(cur, 15))) if cur else "")


class H(BaseHTTPRequestHandler):
    def do_GET(self):                                           # noqa: N802
        path = self.path.split("?")[0]
        try:
            if path == "/":
                body, ctype = page().encode("utf-8"), "text/html; charset=utf-8"
            elif path == "/live.jpg":
                body, ctype = open(LIVE, "rb").read(), "image/jpeg"
            elif path == "/sample.jpg":                         # 가장 최근 세션의 마지막 사진 — 경로는 서버가 정한다(요청에서 안 받는다)
                d = max(glob.glob(os.path.join(ROOT, "s*", "img")), key=os.path.getmtime)
                body, ctype = open(max(glob.glob(os.path.join(d, "*.jpg"))), "rb").read(), "image/jpeg"
            else:
                self.send_error(404)
                return
        except (OSError, ValueError) as e:
            self.send_error(503, str(e))
            return
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *a):                                  # 조용히
        pass


if __name__ == "__main__":
    ThreadingHTTPServer(("0.0.0.0", PORT), H).serve_forever()
