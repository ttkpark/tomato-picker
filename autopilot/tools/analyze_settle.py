"""settle 발산 사례 분석 스크립트 (T52)."""
import json

with open('docs/\uc2dc\ud5d8\uae30\ub85d/arm-load-boundary-2026-09-18.jsonl', encoding='utf-8') as f:
    lines = [json.loads(l) for l in f if l.strip()]

print('=== restored=True cases (diverged) ===')
for i, r in enumerate(lines):
    s = r.get('settle', {})
    if s.get('restored'):
        stop = s['stop']
        start = s['start_err_deg']
        final = s['err_deg']
        print(f'Line {i+1}: stop={stop} start={start} final={final}')
        for rnd in s['rounds']:
            g = rnd.get('gain', 'N/A')
            print(f'  n={rnd["n"]} err={rnd["err_deg"]} gain={g} worst={rnd["worst"]}')
        print()

print()
print('=== All gain values (to spot pattern) ===')
for i, r in enumerate(lines):
    s = r.get('settle', {})
    restored = s.get('restored', False)
    rounds = s.get('rounds', [])
    gains = [rnd.get('gain') for rnd in rounds if rnd.get('gain') is not None]
    neg_gains = [g for g in gains if g is not None and g < 0]
    if gains:
        print(f'Line {i+1} stop={s.get("stop")} restored={restored} gains={gains}')
        if any(abs(g) > 1.0 for g in neg_gains):
            print(f'  *** LARGE NEGATIVE GAIN: {neg_gains}')
