# 자동배선 (KiCad 9 python + Freerouting). gen_pcb.py로 배치한 **직후의 빈 판**에서:
#   python route.py prep     → 넷클래스(.kicad_pro) + mecanum_stm32.dsn
#   java -jar freerouting.jar -de mecanum_stm32.dsn -do mecanum_stm32.ses -mp 60
#   python route.py finish   → .ses의 선·비아를 판에 넣고 GND 동박(양면) 채워 저장
#
# 왜 DSN을 직접 쓰나: 이 빌드(9.0.4)의 pcbnew.ExportSpecctraDSN은 헤드리스 파이썬에서 **멈춘다**(보이지 않는
# assert 창). 그래서 패드 모양(절대좌표 다각형)·넷·넷클래스·GND 평면만 Specctra 문법으로 쓴다.
# ⚠ 이 빌드에선 b.Remove()가 SWIG 타입을 깨뜨린다 → 지우지 말고 gen_pcb.py부터 다시 돌린다.
# 절연: GND 동박은 두 칸 — USB 칸(GNDU, ADuM1201 중심선 왼쪽·y<ISO_Y)과 나머지(GND), 사이 간격 ISO_GAP.
import json, os, re, sys
import pcbnew

HERE = os.path.dirname(os.path.abspath(__file__))
PCB, PRO = os.path.join(HERE, "mecanum_stm32.kicad_pcb"), os.path.join(HERE, "mecanum_stm32.kicad_pro")
DSN, SES = PCB[:-10] + ".dsn", PCB[:-10] + ".ses"
W, H, ISO_Y, ISO_GAP = 90.0, 64.0, 27.0, 1.5

# (이름, 선폭, 간격, 비아지름, 비아드릴 mm, 넷 패턴)
# 선폭은 **핀에 들어갈 수 있는 굵기**로 배선한다 — DRV8874는 0.65mm, MCU는 0.5mm 피치라 0.8mm 선은 핀 사이 간격을
# 못 지켜 Freerouting이 통째로 포기한다(첫 시도: 미배선 145). 굵기는 배선 뒤 widen()이 간격이 되는 구간만 키운다.
DEFAULT = (0.18, 0.15, 0.6, 0.3)   # 0.18/0.15mm = 일반 업체 기본 공정(JLC 최소 0.1/0.1) 안. 0.2면 신호 2개가 못 지나간다
CLASSES = [
    ("Power", 0.4, 0.2, 0.8, 0.4, ["VBAT_IN", "VLOG", "SW5"]),
    # VM은 24핀이 판 전체에 퍼져 0.4mm·0.8비아로는 길을 못 찾는다(3회 연속 VM만 18연결 실패) → 가늘게 잇고 widen이 키운다
    ("Motor", 0.3, 0.15, 0.6, 0.3, ["VM", "M?+", "M?-"]),   # 모터 출력도(4차: MC− 1개만 실패)
    ("Supply", 0.25, 0.15, 0.6, 0.3, ["5V", "3V3", "VBUS", "GND", "GNDU"]),
]
WIDEN = {"Power": [0.8, 0.6], "Motor": [0.8, 0.6, 0.4], "Supply": [0.5]}   # 배선 후 시도할 굵기(큰 것부터)
SCREW_RING = 6.5   # mm, 고정구멍 둘레 동박 금지 지름(M3 나사머리 ≈5.5) — H2는 USB(GNDU) 칸 안이라 특히 중요
UM = 1000.0  # DSN 단위 = µm (resolution um 1)


def set_netclasses():
    d = json.load(open(PRO, encoding="utf8"))
    ns = d["net_settings"]
    base = next(c for c in ns["classes"] if c["name"] == "Default")
    base.update(track_width=DEFAULT[0], clearance=DEFAULT[1], via_diameter=DEFAULT[2], via_drill=DEFAULT[3])
    ns["classes"] = [base] + [dict(base, name=n, track_width=tw, clearance=cl, via_diameter=vd, via_drill=vdr, priority=i)
                              for i, (n, tw, cl, vd, vdr, _) in enumerate(CLASSES)]
    ns["netclass_patterns"] = [{"netclass": n, "pattern": p} for n, *_, pats in CLASSES for p in pats]
    # 방열 스포크 최소 1 — GND 패드는 선으로도 이어져 있어 스포크가 하나뿐이어도 연결이 끊기지 않는다
    d["board"]["design_settings"]["rules"]["min_resolved_spokes"] = 1
    json.dump(d, open(PRO, "w", encoding="utf8"), indent=2)


def class_of(net):
    for name, tw, cl, vd, vdr, pats in CLASSES:
        if any(re.fullmatch(p.replace("?", ".").replace("+", r"\+"), net) for p in pats):
            return name, tw, cl, vd, vdr
    return ("Default",) + DEFAULT


def iso_x(b):
    return pcbnew.ToMM(next(f for f in b.GetFootprints() if f.GetReference() == "U5").GetPosition().x)


def pour_polys(b):
    ix, e, g = iso_x(b), 0.4, ISO_GAP / 2
    return [("GNDU", [(e, e), (ix - g, e), (ix - g, ISO_Y - g), (e, ISO_Y - g)], 1),
            ("GND", [(ix + g, e), (W - e, e), (W - e, H - e), (e, H - e), (e, ISO_Y + g), (ix + g, ISO_Y + g)], 0)]


# ---------------- DSN 쓰기 ----------------
def q(s): return '"' + s.replace('"', "") + '"'
def xy(v): return f"{v.x / UM:.1f} {-v.y / UM:.1f}"          # nm → µm, y 위로
def mm(x, y): return f"{x * 1000:.1f} {-y * 1000:.1f}"


def export_dsn(b):
    L = ['(pcb "mecanum_stm32"', ' (parser (string_quote ") (space_in_quoted_tokens on) (host_cad "KiCad") (host_version "9"))',
         ' (resolution um 1) (unit um)', ' (structure', '  (layer F.Cu (type signal) (property (index 0)))',
         '  (layer B.Cu (type signal) (property (index 1)))',
         f'  (boundary (rect pcb 0 {-H * 1000:.0f} {W * 1000:.0f} 0))']
    # GND 평면은 DSN에 안 넣는다 — 넣으면 Freerouting이 그 층 전체를 장애물로 봐서 2층 판이 1층이 된다
    # (2차 시도: 미배선 68에서 정체). GND도 선으로 잇고, 동박은 배선 뒤 add_pours()가 양면에 채운다.
    vias = {}
    for name, tw, cl, vd, vdr in [("Default",) + DEFAULT] + [c[:5] for c in CLASSES]:
        vias[name] = f"Via[0-1]_{vd * 1000:.0f}:{vdr * 1000:.0f}_um"
    L.append("  (via " + " ".join(q(v) for v in dict.fromkeys(vias.values())) + ")")
    L.append(f"  (rule (width {DEFAULT[0] * 1000:.0f}) (clearance {DEFAULT[1] * 1000:.0f}) (clearance 150 (type smd_smd)))")
    for x0, y0, x1, y1 in [(0, 0, W, 0.5), (0, H - 0.5, W, H), (0, 0, 0.5, H), (W - 0.5, 0, W, H)]:   # 판 가장자리 0.5mm 띠
        L.append(f'  (keepout "" (rect signal {x0 * 1000:.0f} {-y1 * 1000:.0f} {x1 * 1000:.0f} {-y0 * 1000:.0f}))')
    holes = []
    comps, imgs, stacks, nets = [], [], [], {}
    for fp in b.GetFootprints():
        ref = fp.GetReference()
        pins, seen = [], {}
        for k, pad in enumerate(fp.Pads()):
            pos = pad.GetPosition()
            if pad.GetAttribute() == pcbnew.PAD_ATTRIB_NPTH:   # 고정구멍·커넥터 위치핀
                d = max(pad.GetDrillSize().x, pad.GetSize(pcbnew.F_Cu).x) + pcbnew.FromMM(0.6)   # 구멍 간격 0.3mm 여유
                if fp.GetReference().startswith("H"):
                    d = pcbnew.FromMM(SCREW_RING)   # 고정나사 머리·스탠드오프가 동박에 닿지 않게
                holes.append(f'  (keepout "" (circle signal {d / UM:.0f} {xy(pos)}))')
                continue
            if not pad.GetNumber():                            # 페이스트 창(번호 없음) — 동박 아님
                continue
            num = pad.GetNumber(); seen[num] = seen.get(num, 0) + 1
            pid = num if seen[num] == 1 else f"{num}@{seen[num]}"
            ps = f"ps_{ref}_{k}"
            shapes = []
            for lay, lname in ((pcbnew.F_Cu, "F.Cu"), (pcbnew.B_Cu, "B.Cu")):
                if not pad.IsOnLayer(lay): continue
                poly = pad.GetEffectivePolygon(lay)
                o = poly.Outline(0)
                pts = " ".join(f"{(o.CPoint(i).x - pos.x) / UM:.1f} {-(o.CPoint(i).y - pos.y) / UM:.1f}" for i in range(o.PointCount()))
                shapes.append(f"(shape (polygon {lname} 0 {pts}))")
            if not shapes: continue
            stacks.append(f"  (padstack {q(ps)} " + " ".join(shapes) + " (attach off))")
            pins.append(f"   (pin {q(ps)} {q(pid)} {xy(pos)})")
            if pad.GetNetname():
                nets.setdefault(pad.GetNetname(), []).append(f"{ref}-{pid}")
        imgs.append(f"  (image {q('img_' + ref)}\n" + "\n".join(pins) + ")")
        comps.append(f'  (component {q("img_" + ref)} (place {q(ref)} 0 0 front 0))')
    L += holes + [" )", " (placement"] + comps + [" )", " (library"] + imgs + stacks
    for name, tw, cl, vd, vdr in [("Default",) + DEFAULT] + [c[:5] for c in CLASSES]:
        v = vias[name]
        L.append(f"  (padstack {q(v)} (shape (circle F.Cu {vd * 1000:.0f})) (shape (circle B.Cu {vd * 1000:.0f})) (attach off))")
    L += [" )", " (network"]
    for n, pins in nets.items():
        L.append(f"  (net {q(n)} (pins " + " ".join(pins) + "))")   # 핀 참조는 따옴표 없이(REF-핀)
    groups = {}
    for n in nets: groups.setdefault(class_of(n), []).append(n)
    for (name, tw, cl, vd, vdr), ns in groups.items():
        L.append(f"  (class {q(name)} " + " ".join(q(n) for n in ns) +
                 f" (circuit (use_via {q(vias[name])})) (rule (width {tw * 1000:.0f}) (clearance {cl * 1000:.0f})))")
    L += [" )", " (wiring)", ")"]
    open(DSN, "w", encoding="utf8").write("\n".join(L))
    return len(nets), len(stacks)


# ---------------- SES 읽기 ----------------
def sexp(text):
    tok = re.findall(r'\(|\)|"[^"]*"|[^\s()]+', text)
    st = [[]]
    for t in tok:
        if t == "(": st.append([])
        elif t == ")": x = st.pop(); st[-1].append(x)
        else: st[-1].append(t.strip('"'))
    return st[0][0]


def find(node, key):
    for c in node:
        if isinstance(c, list) and c and c[0] == key: yield c


def import_ses(b):
    s = sexp(open(SES, encoding="utf8", errors="ignore").read())
    routes = next(find(s, "routes"))
    res = next(find(routes, "resolution"), ["resolution", "um", "1"])
    k = 1000.0 / float(res[2]) if res[1] == "um" else 1e6 / float(res[2])   # SES 단위 → nm
    layers = {"F.Cu": pcbnew.F_Cu, "B.Cu": pcbnew.B_Cu}
    vsz = {}
    for ps in find(next(find(routes, "library_out"), []), "padstack"):
        circ = next(find(next(find(ps, "shape")), "circle"))
        m = re.search(r"_(\d+):(\d+)_um", ps[1])
        vsz[ps[1]] = (float(circ[2]) * k, float(m.group(2)) * 1000 if m else float(circ[2]) * k / 2)
    nt = nv = 0
    for net in find(next(find(routes, "network_out")), "net"):
        ni = b.FindNet(net[1])
        for w in find(net, "wire"):
            p = next(find(w, "path")); lay, wd = layers[p[1]], float(p[2]) * k
            c = [float(v) * k for v in p[3:]]
            pts = [pcbnew.VECTOR2I(int(c[i]), int(-c[i + 1])) for i in range(0, len(c), 2)]
            for a, z in zip(pts, pts[1:]):
                t = pcbnew.PCB_TRACK(b); t.SetStart(a); t.SetEnd(z); t.SetWidth(int(wd)); t.SetLayer(lay); t.SetNet(ni)
                b.Add(t); nt += 1
        for v in find(net, "via"):
            d, dr = vsz.get(v[1], (600e3, 300e3))
            x = pcbnew.PCB_VIA(b); x.SetPosition(pcbnew.VECTOR2I(int(float(v[2]) * k), int(-float(v[3]) * k)))
            x.SetWidth(int(d)); x.SetDrill(int(dr)); x.SetNet(ni); b.Add(x); nv += 1
    return nt, nv


def widen(b):
    """배선 뒤 전원선 굵히기 — 다른 넷 동박·NPTH·판 가장자리와 간격이 되는 선분만 WIDEN 순서로 키운다.
    핀 근처 선분은 실패해서 가늘게 남는다(=자연스러운 넥다운). ponytail: O(선분×물체) bbox 거르기 — 이 판 크기면 수 초."""
    edge = pcbnew.FromMM(0.5)
    objs = []   # (bbox, layerset, net, shape-getter)
    tracks = list(b.GetTracks())
    objs = [(p, p.GetNetCode()) for p in b.GetPads()] + [(t, t.GetNetCode()) for t in tracks]
    # ⚠ bbox는 매번 새로 잰다 — 미리 재 두면 나중에 굵어진 이웃을 못 봐서 간격 위반이 났다(1차 결과 5건)
    done = {}
    for t in tracks:
        if t.Type() != pcbnew.PCB_TRACE_T: continue
        cls = class_of(t.GetNetname())
        tries = WIDEN.get(cls[0])
        if not tries: continue
        lay, orig = t.GetLayer(), t.GetWidth()
        for w in tries:
            t.SetWidth(pcbnew.FromMM(w))
            bb = t.GetBoundingBox(); clr = pcbnew.FromMM(max(cls[2], 0.2))   # 굵힌 선은 0.2mm 이상 띄운다
            ok = bb.GetLeft() > edge and bb.GetTop() > edge and bb.GetRight() < pcbnew.FromMM(W) - edge and bb.GetBottom() < pcbnew.FromMM(H) - edge
            if ok:
                sh = t.GetEffectiveShape(lay)
                bb.Inflate(clr)
                for o, net in objs:
                    if o is t or not bb.Intersects(o.GetBoundingBox()): continue
                    if o.Type() == pcbnew.PCB_PAD_T and o.GetAttribute() == pcbnew.PAD_ATTRIB_NPTH:
                        if o.GetEffectiveHoleShape().Collide(sh, clr): ok = False; break
                        continue
                    if net == t.GetNetCode() or not o.IsOnLayer(lay): continue
                    if o.GetEffectiveShape(lay).Collide(sh, clr): ok = False; break
            if ok:
                done[w] = done.get(w, 0) + 1; break
            t.SetWidth(orig)
    return done


def add_pours(b):
    for net, pts, prio in pour_polys(b):
        for lay in (pcbnew.F_Cu, pcbnew.B_Cu):
            z = pcbnew.ZONE(b)
            z.SetLayer(lay); z.SetNet(b.FindNet(net)); z.SetAssignedPriority(prio)
            z.SetLocalClearance(pcbnew.FromMM(0.3)); z.SetMinThickness(pcbnew.FromMM(0.25))
            z.SetPadConnection(pcbnew.ZONE_CONNECTION_THERMAL)
            z.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_ALWAYS)   # 아무 데도 안 붙은 동박 조각 제거
            z.SetThermalReliefGap(pcbnew.FromMM(0.3)); z.SetThermalReliefSpokeWidth(pcbnew.FromMM(0.5))
            c = pcbnew.SHAPE_LINE_CHAIN()
            for x, y in pts: c.Append(pcbnew.FromMM(x), pcbnew.FromMM(y))
            c.SetClosed(True)   # 안 닫으면 assert 창이 떠서 헤드리스에선 멈춘다
            z.Outline().AddOutline(c)
            b.Add(z)
    for fp in b.GetFootprints():          # 고정구멍 둘레: 동박·선·비아 금지 구역(양면)
        if not fp.GetReference().startswith("H"): continue
        z = pcbnew.ZONE(b); z.SetIsRuleArea(True)
        z.SetDoNotAllowCopperPour(True); z.SetDoNotAllowTracks(True); z.SetDoNotAllowVias(True)
        z.SetDoNotAllowPads(False); z.SetDoNotAllowFootprints(False)
        ls = pcbnew.LSET(); ls.AddLayer(pcbnew.F_Cu); ls.AddLayer(pcbnew.B_Cu); z.SetLayerSet(ls)
        c = pcbnew.SHAPE_LINE_CHAIN(); p = fp.GetPosition(); r = pcbnew.FromMM(SCREW_RING / 2)
        import math
        for k in range(24):
            c.Append(int(p.x + r * math.cos(k * math.pi / 12)), int(p.y + r * math.sin(k * math.pi / 12)))
        c.SetClosed(True); z.Outline().AddOutline(c); b.Add(z)
    pcbnew.ZONE_FILLER(b).Fill(b.Zones())


if __name__ == "__main__":
    step = sys.argv[1]
    set_netclasses()   # ⚠ 판을 열기 전에 — gen_pcb.py가 저장하며 .kicad_pro 넷클래스를 기본값으로 덮어쓴다(500건 위반의 원인)
    b = pcbnew.LoadBoard(PCB)
    assert not len(b.Zones()) and not len(b.GetTracks()), "gen_pcb.py를 먼저 다시 돌릴 것(배치 직후의 빈 판)"
    if step == "prep":
        print("DSN nets/padstacks", export_dsn(b))
    elif step == "finish":
        print("SES tracks/vias", import_ses(b))
        b.BuildConnectivity()
        print("widened", widen(b))
        out = os.environ.get("ROUTE_NOPOUR")   # 보고서용: 동박을 안 채운 판(배선만 보이게)을 따로 저장
        if out:
            pcbnew.SaveBoard(out, b); sys.exit(0)
        add_pours(b)
        b.BuildConnectivity()
        pcbnew.SaveBoard(PCB, b)
        print("unrouted", b.GetConnectivity().GetUnconnectedCount(True))
