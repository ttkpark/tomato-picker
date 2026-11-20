#!/usr/bin/env python3
"""docs/study/0*.md 를 한 HTML(study.html)로 묶는다. md를 고치면 다시 돌린다.

    python docs/study/build_html.py

md는 그대로 싣고 브라우저에서 그린다(marked·KaTeX·mermaid, CDN) — 인터넷이 없으면 원문 텍스트로 보인다.
"""
import json
import pathlib

HERE = pathlib.Path(__file__).resolve().parent
OUT = HERE / "study.html"

TEMPLATE = r"""<!doctype html>
<html lang="ko">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>토마토 로봇 스터디</title>
<link rel="stylesheet" href="https://cdn.jsdelivr.net/npm/katex@0.16.11/dist/katex.min.css">
<script src="https://cdn.jsdelivr.net/npm/marked@12.0.2/marked.min.js"></script>
<script src="https://cdn.jsdelivr.net/npm/katex@0.16.11/dist/katex.min.js"></script>
<script src="https://cdn.jsdelivr.net/npm/mermaid@10.9.1/dist/mermaid.min.js"></script>
<style>
:root { --bg:#fff; --fg:#1f2328; --muted:#59636e; --line:#d1d9e0; --side:#f6f8fa; --code:#f6f8fa; --accent:#d1242f; }
@media (prefers-color-scheme: dark) { :root:not([data-theme="light"]) {
  --bg:#0d1117; --fg:#e6edf3; --muted:#9198a1; --line:#3d444d; --side:#151b23; --code:#151b23; --accent:#ff7b72; } }
:root[data-theme="dark"] { --bg:#0d1117; --fg:#e6edf3; --muted:#9198a1; --line:#3d444d; --side:#151b23; --code:#151b23; --accent:#ff7b72; }
* { box-sizing:border-box; }
body { margin:0; background:var(--bg); color:var(--fg); font:16px/1.7 -apple-system,"Segoe UI","Noto Sans KR",sans-serif; display:flex; }
nav { width:270px; flex-shrink:0; position:sticky; top:0; height:100vh; overflow-y:auto; background:var(--side); border-right:1px solid var(--line); padding:16px 10px; }
nav h1 { font-size:15px; margin:0 8px 12px; }
nav a { display:block; padding:7px 10px; border-radius:6px; color:var(--muted); text-decoration:none; font-size:14px; line-height:1.4; }
nav a:hover { background:var(--line); }
nav a.on { color:var(--accent); font-weight:600; }
nav a b { display:inline-block; width:26px; }
nav .sub a { padding:4px 10px 4px 36px; font-size:13px; }
main { flex:1; min-width:0; max-width:980px; padding:32px 40px 80px; }
main a { color:var(--accent); }
table { border-collapse:collapse; display:block; overflow-x:auto; margin:16px 0; }
th, td { border:1px solid var(--line); padding:6px 10px; vertical-align:top; }
th { background:var(--side); }
pre { background:var(--code); padding:14px; border-radius:6px; overflow-x:auto; font-size:13px; line-height:1.5; }
code { font-family:Consolas,"D2Coding",monospace; font-size:0.9em; }
:not(pre) > code { background:var(--code); padding:1px 5px; border-radius:4px; }
blockquote { margin:16px 0; padding:4px 16px; border-left:4px solid var(--line); color:var(--muted); }
h1, h2 { border-bottom:1px solid var(--line); padding-bottom:6px; }
.katex-display { overflow-x:auto; overflow-y:hidden; }
.mermaid { background:#fff; border-radius:6px; padding:8px; overflow-x:auto; }
.pager { display:flex; justify-content:space-between; margin-top:48px; padding-top:16px; border-top:1px solid var(--line); }
@media (max-width:800px) {
  body { display:block; }
  nav { position:static; width:auto; height:auto; border-right:0; border-bottom:1px solid var(--line); }
  main { padding:16px; }
}
</style>
</head>
<body>
<nav><h1>🍅 토마토 로봇 스터디</h1><div id="toc"></div></nav>
<main id="doc"></main>
<script>
const DOCS = __DOCS__;
const $doc = document.getElementById('doc'), $toc = document.getElementById('toc');
const ready = typeof marked !== 'undefined';
if (typeof mermaid !== 'undefined') mermaid.initialize({ startOnLoad:false, theme:'default' });

const slug = s => s.trim().toLowerCase().replace(/[^\p{L}\p{N}\s-]/gu, '').replace(/\s/g, '-');

// 수식을 marked보다 먼저 빼 둔다(밑줄·역슬래시가 강조/이스케이프로 먹힌다). 코드 안의 $는 건드리지 않는다.
function render(md) {
  if (!ready) return '<pre>' + md.replace(/[&<]/g, c => c === '&' ? '&amp;' : '&lt;') + '</pre>';
  const math = [];
  md = md.replace(/(```[\s\S]*?```|`[^`\n]+`)|\$\$([\s\S]+?)\$\$|\$([^$\n]+?)\$/g, (m, code, disp, inl) => {
    if (code) return code;
    math.push([disp ?? inl, disp !== undefined]);
    return `@@MATH${math.length - 1}@@`;
  });
  let html = marked.parse(md);
  return html.replace(/@@MATH(\d+)@@/g, (_, i) => {
    const [tex, display] = math[i];
    try { return katex.renderToString(tex, { displayMode:display, throwOnError:false }); }
    catch { return tex; }
  });
}

function show(id) {
  const d = DOCS.find(x => x.id === id) || DOCS[0];
  $doc.innerHTML = render(d.md);
  $doc.querySelectorAll('h1,h2,h3,h4').forEach(h => h.id = slug(h.textContent));
  $doc.querySelectorAll('pre code.language-mermaid').forEach(c => {
    const div = document.createElement('div');
    div.className = 'mermaid'; div.textContent = c.textContent;
    c.parentElement.replaceWith(div);
  });
  if (typeof mermaid !== 'undefined') mermaid.run({ querySelector:'#doc .mermaid' }).catch(() => {});
  // 다른 스터디 문서 링크는 이 페이지 안으로, 저장소 상대 링크는 그대로 둔다.
  $doc.querySelectorAll('a[href]').forEach(a => {
    const href = a.getAttribute('href');
    const other = DOCS.find(x => href.split('#')[0] === x.file);
    if (other) a.setAttribute('href', '#' + other.id);
    else if (href.startsWith('#')) a.onclick = e => {
      e.preventDefault();
      document.getElementById(decodeURIComponent(href.slice(1)))?.scrollIntoView();
    };
  });
  const i = DOCS.indexOf(d), prev = DOCS[i - 1], next = DOCS[i + 1];
  $doc.insertAdjacentHTML('beforeend', '<div class="pager">' +
    (prev ? `<a href="#${prev.id}">← ${prev.title}</a>` : '<span></span>') +
    (next ? `<a href="#${next.id}">${next.title} →</a>` : '<span></span>') + '</div>');
  // 목차: 문서 목록 + 지금 문서의 절(h2) 목록
  const esc = t => t.replace(/[&<]/g, c => c === '&' ? '&amp;' : '&lt;');
  const subs = [...$doc.querySelectorAll('h2')].map(h => `<a href="#${d.id}" data-anchor="${h.id}">${esc(h.textContent)}</a>`).join('');
  $toc.innerHTML = DOCS.map(x => `<a href="#${x.id}" data-id="${x.id}" class="${x.id === d.id ? 'on' : ''}"><b>${x.id}</b>${x.title}</a>` +
    (x.id === d.id ? `<div class="sub">${subs}</div>` : '')).join('');
  $toc.querySelectorAll('[data-anchor]').forEach(a => a.onclick = e => {
    e.preventDefault(); document.getElementById(a.dataset.anchor)?.scrollIntoView();
  });
  document.title = d.title + ' · 토마토 로봇 스터디';
  window.scrollTo(0, 0);
}

const route = () => show(location.hash.slice(1) || DOCS[0].id);
window.addEventListener('hashchange', route);
route();
</script>
</body>
</html>
"""


def main():
    docs = []
    for p in sorted(HERE.glob("[0-9][0-9]_*.md")):
        md = p.read_text(encoding="utf-8")
        title = next((l[2:].strip() for l in md.splitlines() if l.startswith("# ")), p.stem)
        docs.append({"id": p.name[:2], "file": p.name, "title": title, "md": md})
    assert docs, f"{HERE}에 NN_*.md가 없다"
    # md 안의 "</script>"가 스크립트 블록을 끊지 않게.
    payload = json.dumps(docs, ensure_ascii=False).replace("</", "<\\/")
    OUT.write_text(TEMPLATE.replace("__DOCS__", payload), encoding="utf-8", newline="\n")
    print(f"{OUT.name}: {len(docs)}편 ({', '.join(d['id'] for d in docs)})")


if __name__ == "__main__":
    main()
