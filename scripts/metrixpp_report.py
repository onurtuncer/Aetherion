#!/usr/bin/env python3
"""Generate a self-contained HTML report and Markdown step-summary from metrixpp.db."""

import argparse
import csv
import html as _html
import io
import json
import os
import statistics
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

CC_COL = "std.code.complexity:cyclomatic"
SLOC_COL = "std.code.lines:code"

BUCKETS = [
    (1,  3,  "#2ea043", "1–3"),
    (4,  6,  "#3fb950", "4–6"),
    (7,  10, "#d29922", "7–10"),
    (11, 15, "#e36209", "11–15"),
    (16, 20, "#f0883e", "16–20"),
    (21, 30, "#f85149", "21–30"),
    (31, 9999, "#da3633", "31+"),
]

# ── CSS / JS kept as plain strings so {braces} don't need escaping ────────────

_CSS = """
:root {
  --bg: #0d1117; --bg2: #161b22; --bg3: #21262d;
  --border: #30363d; --text: #e6edf3; --muted: #8b949e;
  --accent: #58a6ff;
}
* { box-sizing: border-box; margin: 0; padding: 0; }
body { background: var(--bg); color: var(--text);
       font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
       font-size: 14px; line-height: 1.5; }
a { color: var(--accent); text-decoration: none; }

header { background: var(--bg2); border-bottom: 1px solid var(--border);
         padding: 18px 32px; display: flex; align-items: center;
         justify-content: space-between; flex-wrap: wrap; gap: 8px; }
header h1 { font-size: 18px; font-weight: 600; }
.meta { color: var(--muted); font-size: 12px; }
.meta code { background: var(--bg3); border-radius: 4px; padding: 1px 5px; }

.container { max-width: 1200px; margin: 0 auto; padding: 24px 32px; }

.cards { display: grid; grid-template-columns: repeat(auto-fit, minmax(180px, 1fr));
         gap: 16px; margin-bottom: 32px; }
.card { background: var(--bg2); border: 1px solid var(--border);
        border-radius: 8px; padding: 16px 20px; }
.card-value { font-size: 30px; font-weight: 700; }
.card-label { color: var(--muted); font-size: 12px; margin-top: 4px; }
.card.warn  .card-value { color: #d29922; }
.card.danger .card-value { color: #f85149; }

section { margin-bottom: 32px; }
h2 { font-size: 15px; font-weight: 600; margin-bottom: 14px; }

.chart-wrap { background: var(--bg2); border: 1px solid var(--border);
              border-radius: 8px; padding: 20px 24px; display: inline-block; }

.controls { display: flex; gap: 10px; margin-bottom: 10px; align-items: center; flex-wrap: wrap; }
input[type=text] { background: var(--bg2); border: 1px solid var(--border);
                   border-radius: 6px; color: var(--text); padding: 6px 12px;
                   font-size: 13px; width: 260px; outline: none; }
input[type=text]:focus { border-color: var(--accent); }
select { background: var(--bg2); border: 1px solid var(--border);
         border-radius: 6px; color: var(--text); padding: 6px 10px;
         font-size: 13px; cursor: pointer; outline: none; }
.count-label { color: var(--muted); font-size: 12px; }

table { width: 100%; border-collapse: collapse; background: var(--bg2);
        border: 1px solid var(--border); border-radius: 8px; overflow: hidden; }
thead { background: var(--bg3); }
th { padding: 10px 14px; text-align: left; font-size: 11px; font-weight: 600;
     color: var(--muted); text-transform: uppercase; letter-spacing: .05em;
     cursor: pointer; user-select: none; white-space: nowrap; }
th:hover { color: var(--text); }
th.sorted { color: var(--accent); }
td { padding: 8px 14px; border-top: 1px solid var(--border); vertical-align: middle; }
tr:hover td { background: var(--bg3); }

.badge { display: inline-block; min-width: 34px; padding: 2px 8px;
         border-radius: 12px; font-weight: 700; font-size: 13px;
         text-align: center; color: #fff; }
.cat { color: var(--muted); font-size: 12px; margin-left: 4px; }
.num { text-align: right; font-variant-numeric: tabular-nums; color: var(--muted); }
.mono { font-family: ui-monospace, "Cascadia Code", "Fira Code", monospace; font-size: 12px; }
.file-path { color: var(--muted); word-break: break-all; }
"""

_JS = r"""
const DATA = __DATA__;

function ccColor(cc) {
  if (cc <=  3) return "#2ea043";
  if (cc <=  6) return "#3fb950";
  if (cc <= 10) return "#d29922";
  if (cc <= 15) return "#e36209";
  if (cc <= 20) return "#f0883e";
  if (cc <= 30) return "#f85149";
  return "#da3633";
}
function ccLabel(cc) {
  if (cc <=  5) return "Low";
  if (cc <= 10) return "Moderate";
  if (cc <= 20) return "High";
  return "Very High";
}
function esc(s) {
  return String(s)
    .replace(/&/g,"&amp;").replace(/</g,"&lt;")
    .replace(/>/g,"&gt;").replace(/"/g,"&quot;");
}

let sortCol = "cc", sortDir = -1;

function renderTable(rows) {
  const tbody = document.getElementById("tbody");
  tbody.innerHTML = rows.map(r => `
    <tr>
      <td><span class="badge" style="background:${ccColor(r.cc)}">${r.cc}</span>
          <span class="cat">${ccLabel(r.cc)}</span></td>
      <td class="num">${r.sloc}</td>
      <td class="mono">${esc(r.region)}</td>
      <td class="mono file-path">${esc(r.file)}</td>
    </tr>`).join("");
  const n = rows.length;
  document.getElementById("count-label").textContent = n + (n === 1 ? " function" : " functions");
}

function sortRows(rows) {
  return [...rows].sort((a, b) => {
    const av = a[sortCol], bv = b[sortCol];
    return typeof av === "string"
      ? sortDir * av.localeCompare(bv)
      : sortDir * (av - bv);
  });
}

function applyFilter() {
  const text = document.getElementById("filter-input").value.toLowerCase();
  const cat  = document.getElementById("cat-filter").value.toLowerCase();
  const rows = DATA.filter(r => {
    const okText = !text || r.file.toLowerCase().includes(text) || r.region.toLowerCase().includes(text);
    const okCat  = !cat  || ccLabel(r.cc).toLowerCase() === cat;
    return okText && okCat;
  });
  renderTable(sortRows(rows));
}

function sortBy(col) {
  if (sortCol === col) sortDir = -sortDir;
  else { sortCol = col; sortDir = (col === "cc" || col === "sloc") ? -1 : 1; }
  ["cc","sloc","region","file"].forEach(c => {
    const arr = document.getElementById("arr-" + c);
    arr.textContent = (c === sortCol) ? (sortDir === -1 ? " ↓" : " ↑") : "";
    document.getElementById("th-" + c).className = (c === sortCol) ? "sorted" : "";
  });
  applyFilter();
}

applyFilter();
"""

# ── helpers ──────────────────────────────────────────────────────────────────

def cc_color(cc: int) -> str:
    if cc <= 3:  return "#2ea043"
    if cc <= 6:  return "#3fb950"
    if cc <= 10: return "#d29922"
    if cc <= 15: return "#e36209"
    if cc <= 20: return "#f0883e"
    if cc <= 30: return "#f85149"
    return "#da3633"


def cc_label(cc: int) -> str:
    if cc <= 5:  return "Low"
    if cc <= 10: return "Moderate"
    if cc <= 20: return "High"
    return "Very High"


def export_metrics(db_file: Path) -> list[dict]:
    result = subprocess.run(
        ["metrix++", "export", "--db-file", str(db_file)],
        capture_output=True, text=True, check=True,
    )
    reader = csv.DictReader(io.StringIO(result.stdout))
    # Normalise column names to lowercase so header capitalisation doesn't matter
    rows = []
    for raw in reader:
        row = {k.lower(): v for k, v in raw.items()}
        if row.get("type") != "function":
            continue
        cc_str   = row.get(CC_COL,   "").strip()
        sloc_str = row.get(SLOC_COL, "").strip()
        if not cc_str:
            continue
        try:
            row["_cc"]   = int(cc_str)
            row["_sloc"] = int(sloc_str) if sloc_str else 0
        except ValueError:
            continue
        rows.append(row)
    return sorted(rows, key=lambda r: -r["_cc"])


def _histogram_svg(rows: list[dict]) -> str:
    counts = [0] * len(BUCKETS)
    for r in rows:
        cc = r["_cc"]
        for i, (lo, hi, _, _) in enumerate(BUCKETS):
            if lo <= cc <= hi:
                counts[i] += 1
                break

    bar_w, gap = 58, 14
    pad_l, pad_t, pad_b = 10, 10, 32
    chart_h = 140
    total_w  = pad_l + len(BUCKETS) * (bar_w + gap) - gap + pad_l
    total_h  = pad_t + chart_h + pad_b
    max_c    = max(counts) or 1

    parts = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {total_w} {total_h}" '
             f'width="{total_w}" height="{total_h}" aria-label="Complexity distribution">']
    for i, (_, _, color, label) in enumerate(BUCKETS):
        x     = pad_l + i * (bar_w + gap)
        bh    = max(2, int(counts[i] / max_c * chart_h))
        y     = pad_t + chart_h - bh
        parts.append(f'<rect x="{x}" y="{y}" width="{bar_w}" height="{bh}" fill="{color}" rx="3"/>')
        if counts[i]:
            parts.append(f'<text x="{x + bar_w // 2}" y="{y - 5}" '
                         f'text-anchor="middle" font-size="12" fill="#e6edf3">{counts[i]}</text>')
        parts.append(f'<text x="{x + bar_w // 2}" y="{total_h - 6}" '
                     f'text-anchor="middle" font-size="11" fill="#8b949e">{label}</text>')
    parts.append("</svg>")
    return "\n".join(parts)


# ── report builders ──────────────────────────────────────────────────────────

def build_html(rows: list[dict], commit: str, timestamp: str, repo_url: str) -> str:
    ccs  = [r["_cc"] for r in rows]
    n    = len(ccs)
    max_cc   = max(ccs)            if ccs else 0
    mean_cc  = statistics.mean(ccs) if ccs else 0.0
    n_gt10   = sum(1 for c in ccs if c > 10)
    n_gt20   = sum(1 for c in ccs if c > 20)

    warn_gt10  = ' class="card warn"'   if n_gt10  else ' class="card"'
    warn_gt20  = ' class="card danger"' if n_gt20  else ' class="card"'
    repo_link  = (f' &nbsp;·&nbsp; <a href="{_html.escape(repo_url)}">'
                  f'{_html.escape(repo_url.rstrip("/").split("/")[-1])}</a>')  \
                 if repo_url else ""

    histogram = _histogram_svg(rows)

    table_rows = "\n".join(
        f'<tr>'
        f'<td><span class="badge" style="background:{cc_color(r["_cc"])}">{r["_cc"]}</span>'
        f'<span class="cat">{cc_label(r["_cc"])}</span></td>'
        f'<td class="num">{r["_sloc"]}</td>'
        f'<td class="mono">{_html.escape(r.get("region",""))}</td>'
        f'<td class="mono file-path">{_html.escape(r.get("file",""))}</td>'
        f'</tr>'
        for r in rows
    )

    js_data = json.dumps([
        {"cc": r["_cc"], "sloc": r["_sloc"],
         "region": r.get("region", ""), "file": r.get("file", "")}
        for r in rows
    ], separators=(",", ":"))
    final_js = _JS.replace("__DATA__", js_data)

    return f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Metrix++ Complexity Report</title>
<style>{_CSS}</style>
</head>
<body>
<header>
  <h1>Metrix++ Cyclomatic Complexity Report</h1>
  <div class="meta">Generated {_html.escape(timestamp)} &nbsp;·&nbsp;
    commit <code>{_html.escape(commit)}</code>{repo_link}</div>
</header>
<div class="container">

  <div class="cards">
    <div class="card">
      <div class="card-value">{n}</div>
      <div class="card-label">Functions analyzed</div>
    </div>
    <div class="card">
      <div class="card-value">{max_cc}</div>
      <div class="card-label">Max complexity</div>
    </div>
    <div class="card">
      <div class="card-value">{mean_cc:.1f}</div>
      <div class="card-label">Mean complexity</div>
    </div>
    <div{warn_gt10}>
      <div class="card-value">{n_gt10}</div>
      <div class="card-label">Functions CC &gt; 10</div>
    </div>
    <div{warn_gt20}>
      <div class="card-value">{n_gt20}</div>
      <div class="card-label">Functions CC &gt; 20</div>
    </div>
  </div>

  <section>
    <h2>Complexity Distribution</h2>
    <div class="chart-wrap">{histogram}</div>
  </section>

  <section>
    <h2>Functions</h2>
    <div class="controls">
      <input type="text" id="filter-input"
             placeholder="Filter by file or function&hellip;" oninput="applyFilter()">
      <select id="cat-filter" onchange="applyFilter()">
        <option value="">All categories</option>
        <option value="low">Low (1–5)</option>
        <option value="moderate">Moderate (6–10)</option>
        <option value="high">High (11–20)</option>
        <option value="very high">Very High (21+)</option>
      </select>
      <span class="count-label" id="count-label">{n} functions</span>
    </div>
    <table>
      <thead>
        <tr>
          <th id="th-cc"     onclick="sortBy('cc')"    >CC<span     id="arr-cc"> ↓</span></th>
          <th id="th-sloc"   onclick="sortBy('sloc')"  >SLOC<span   id="arr-sloc"></span></th>
          <th id="th-region" onclick="sortBy('region')" >Function<span id="arr-region"></span></th>
          <th id="th-file"   onclick="sortBy('file')"  >File<span   id="arr-file"></span></th>
        </tr>
      </thead>
      <tbody id="tbody">
{table_rows}
      </tbody>
    </table>
  </section>
</div>
<script>{final_js}</script>
</body>
</html>
"""


def build_markdown(rows: list[dict], commit: str, repo_url: str) -> str:
    ccs    = [r["_cc"] for r in rows]
    n      = len(ccs)
    max_cc = max(ccs)             if ccs else 0
    mean_cc = statistics.mean(ccs) if ccs else 0.0
    n_gt10  = sum(1 for c in ccs if c > 10)
    n_gt20  = sum(1 for c in ccs if c > 20)

    icon_gt10 = ":warning:"       if n_gt10 else ":white_check_mark:"
    icon_gt20 = ":no_entry:"      if n_gt20 else ":white_check_mark:"

    lines = [
        "## Metrix++ Cyclomatic Complexity",
        "",
        f"| Metric | Value |",
        f"|--------|-------|",
        f"| Functions analyzed | **{n}** |",
        f"| Max CC | **{max_cc}** |",
        f"| Mean CC | **{mean_cc:.1f}** |",
        f"| {icon_gt10} Functions CC > 10 | **{n_gt10}** |",
        f"| {icon_gt20} Functions CC > 20 | **{n_gt20}** |",
        "",
    ]

    top = [r for r in rows if r["_cc"] > 10][:20]
    if top:
        lines += [
            "### Functions exceeding CC 10",
            "",
            "| CC | Category | Function | File | SLOC |",
            "|----|----------|----------|------|------|",
        ]
        for r in top:
            lines.append(
                f"| **{r['_cc']}** | {cc_label(r['_cc'])} "
                f"| `{r.get('region','')}` | `{r.get('file','')}` | {r['_sloc']} |"
            )
        lines.append("")

    if repo_url:
        lines.append(f"[Full HTML report]({repo_url.rstrip('/')}/metrics/) &nbsp;·&nbsp; "
                     f"commit `{commit}`")
    else:
        lines.append(f"Commit: `{commit}`")

    return "\n".join(lines) + "\n"


# ── entry point ───────────────────────────────────────────────────────────────

def main() -> int:
    ap = argparse.ArgumentParser(
        description="Generate HTML + Markdown report from metrixpp.db")
    ap.add_argument("--db-file",  default="metrixpp.db")
    ap.add_argument("--output",   default="metrixpp-report.html",
                    help="Path for the HTML report")
    ap.add_argument("--markdown", default=None,
                    help="Write Markdown step-summary to this file")
    ap.add_argument("--commit",
                    default=(os.environ.get("GITHUB_SHA") or "unknown")[:7])
    ap.add_argument("--repo-url",
                    default=os.environ.get("GITHUB_SERVER_URL", "").rstrip("/")
                    + "/"
                    + os.environ.get("GITHUB_REPOSITORY", ""))
    args = ap.parse_args()

    db = Path(args.db_file)
    if not db.exists():
        print(f"error: {db} not found", file=sys.stderr)
        return 1

    print(f"Reading metrics from {db} …")
    rows = export_metrics(db)
    if not rows:
        print("warning: no function-level data found in DB", file=sys.stderr)

    ts = datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M UTC")

    html_path = Path(args.output)
    html_path.parent.mkdir(parents=True, exist_ok=True)
    html_path.write_text(
        build_html(rows, args.commit, ts, args.repo_url), encoding="utf-8")
    print(f"HTML report  → {html_path}")

    if args.markdown:
        md_path = Path(args.markdown)
        md_path.parent.mkdir(parents=True, exist_ok=True)
        md_path.write_text(
            build_markdown(rows, args.commit, args.repo_url), encoding="utf-8")
        print(f"Markdown     → {md_path}")

    if rows:
        ccs = [r["_cc"] for r in rows]
        print(f"Functions: {len(rows)}  Max CC: {max(ccs)}  "
              f"Mean CC: {statistics.mean(ccs):.1f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
