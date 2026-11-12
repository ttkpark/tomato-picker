# 도(度)로 박아 둔 자세를 고친 눈금으로 환산 — **같은 실제 자세**(같은 서보 값)가 되게.
#   옛: deg = ref + (norm − zero)·per_old      새: deg' = ref + (norm − zero)·k   →   deg' = ref + (deg − ref)·k/per_old
import json
import re
import sys

ses = json.load(open("data/kincal/s6/session.json", encoding="utf-8"))
cal, cart = ses["cal"], ses["cart"]
J = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
K = 4095.0 / 4096.0
REF = [float(cart["ref_deg"].get(j, 0.0)) for j in J]
PER = [abs(float((cart.get("deg_per_norm") or {}).get(j) or (cal[j]["range_max"] - cal[j]["range_min"]) * 360 / 4096 / 200)) for j in J]
print("옛 per:", [round(p, 4) for p in PER], "→ 새", round(K, 5))


def conv(vals):
    return [REF[i] + (v - REF[i]) * K / PER[i] for i, v in enumerate(vals)]


def fmt(vals):
    return ",".join(("%.1f" % v).rstrip("0").rstrip(".") if abs(v - round(v)) < 0.05 else "%.1f" % v for v in vals)


def pose(s):
    return fmt(conv([float(x) for x in s.split(",")]))


EDITS = [
    ("ros2/tools/space_map.py", 'PARK = "0.6,22,78,81,6"', 'PARK = "%s"' % pose("0.6,22,78,81,6")),
    ("ros2/tools/space_map.py", 'GRASP_START = "1.6,93.9,115.7,-34.7,6.4"', 'GRASP_START = "%s"' % pose("1.6,93.9,115.7,-34.7,6.4")),
    ("ros2/tools/grasp_repeat.py", 'START = "-7.1,75.5,132.2,-28.8,7.1"', 'START = "%s"' % pose("-7.1,75.5,132.2,-28.8,7.1")),
    ("ros2/tools/grasp_repeat.py", "def look_around(pans=(8.0, 20.0, -20.0, -35.0, 35.0)):",
     "def look_around(pans=(%s)):" % ", ".join("%.1f" % (p * K / PER[0]) for p in (8.0, 20.0, -20.0, -35.0, 35.0))),
    ("ros2/tools/handeye_collect.py", 'default="2.8,-9.1,100.8,83.2,-3.6"', 'default="%s"' % pose("2.8,-9.1,100.8,83.2,-3.6")),
    ("ros2/tools/handeye_collect.py", "PAN_GRID = (-18.0, -9.0, 0.0, 9.0, 18.0)",
     "PAN_GRID = (-15.0, -7.5, 0.0, 7.5, 15.0)      # 11/12 눈금 고침: 옛 ±18은 실제 ±14.8°였다"),
    ("ros2/tools/harvest_loop.py", 'PARK = "60,65,0,-92,6"', 'PARK = "%s"' % pose("60,65,0,-92,6")),
    ("ros2/tools/harvest_loop.py", 'STAGE_CLOSE = "60,55,15,-92,6"', 'STAGE_CLOSE = "%s"' % pose("60,55,15,-92,6")),
    ("ros2/tools/swab_trials.py", 'START = "-25,10,120,64,0"', 'START = "%s"' % pose("-25,10,120,64,0")),
    ("ros2/tools/swab_trials.py", "DU_DPAN = -7.7", "DU_DPAN = %.1f" % (-7.7 * PER[0] / K)),
    ("tools/harvest_trials.py", '"far": "86.9,85.8,-26.3,-95,0"', '"far": "%s"' % pose("86.9,85.8,-26.3,-95,0")),
    ("tools/harvest_trials.py", '"bent": "86.9,75.8,-11.2,-95,0"', '"bent": "%s"' % pose("86.9,75.8,-11.2,-95,0")),
    ("tools/harvest_trials.py", '"bent2": "86.9,65.7,3.8,-95,0"', '"bent2": "%s"' % pose("86.9,65.7,3.8,-95,0")),
    ("ros2/tools/click_server.py", 'placeholder="60,65,0,-100,6"', 'placeholder="%s"' % pose("60,65,0,-100,6")),
]
start_old = (1.6, 93.9, 115.7, -34.7, 6.4)
start_new = "(" + ", ".join("%.1f" % v for v in conv(start_old)) + ")"
for f in ("ros2/tools/arm_twin.py", "ros2/src/tomato_bridge/tomato_bridge/twin_io.py"):
    EDITS.append((f, "(1.6, 93.9, 115.7, -34.7, 6.4)", start_new))
    EDITS.append((f, '"deg_per_norm": {"wrist_roll": 0.999}', '"deg_per_norm": {j: 0.99976 for j in kin.JOINTS}'))
dry = "--apply" not in sys.argv
for f, a, b in EDITS:
    s = open(f, encoding="utf-8").read()
    n = s.count(a)
    print("%-52s %d곳  %s  →  %s" % (f, n, a[:44], b[:60]))
    assert n == 1, (f, a, n)
    if not dry:
        open(f, "w", encoding="utf-8", newline="\n").write(s.replace(a, b))
# 검산: 환산한 자세가 옛 자세와 같은 서보 값인가
zero = [float(cart["zero"].get(j, 0.0)) for j in J]
for name, old in (("PARK", "0.6,22,78,81,6"), ("GRASP_START", "1.6,93.9,115.7,-34.7,6.4"), ("harvest PARK", "60,65,0,-92,6")):
    o = [float(x) for x in old.split(",")]
    n_old = [zero[i] + (o[i] - REF[i]) / PER[i] for i in range(5)]
    nw = [float(x) for x in pose(old).split(",")]
    n_new = [zero[i] + (nw[i] - REF[i]) / K for i in range(5)]
    print("%-13s 서보 값 차이(도) %s" % (name, [round(a - b, 2) for a, b in zip(n_old, n_new)]))
print("적용함" if not dry else "(시험 — --apply 로 적용)")
