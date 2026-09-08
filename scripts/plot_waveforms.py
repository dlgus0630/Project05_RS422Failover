"""Render actual XSim VCD transitions as dependency-free SVGs (time in ns)."""
import argparse
import bisect
import html
import json
from pathlib import Path

p = argparse.ArgumentParser()
p.add_argument('vcd', type=Path)
p.add_argument('out', type=Path)
a = p.parse_args()
a.out.mkdir(parents=True, exist_ok=True)
signals, widths, values, scope = {}, {}, {}, []
time, scale, in_header = 0, None, True
tokens = a.vcd.read_text().split()
i = 0
while i < len(tokens):
    tok = tokens[i]
    i += 1
    if tok == '$timescale':
        number = tokens[i]
        if number.isdigit():
            unit = tokens[i + 1]
            scale = float(number) * {'s': 1e9, 'ms': 1e6, 'us': 1e3, 'ns': 1, 'ps': .001, 'fs': .000001}[unit]
        else:
            import re
            n, unit = re.fullmatch(r'(\d+)(\w+)', number).groups()
            scale = float(n) * {'s': 1e9, 'ms': 1e6, 'us': 1e3, 'ns': 1, 'ps': .001, 'fs': .000001}[unit]
    if tok == '$scope':
        scope.append(tokens[i + 1])
    elif tok == '$upscope':
        scope.pop()
    elif tok == '$var':
        width, code, name = int(tokens[i + 1]), tokens[i + 2], tokens[i + 3]
        signals['/'.join(scope + [name])] = code
        widths[code] = width
        values.setdefault(code, [])
    elif tok == '$enddefinitions':
        in_header = False
    elif not in_header:
        if tok.startswith('#'):
            if scale is None:
                raise ValueError('VCD timescale missing')
            time = int(tok[1:]) * scale
        elif tok[0:1] in ('b', 'B'):
            code = tokens[i]
            i += 1
            values[code].append((time, tok[1:]))
        elif tok[0:1] in '01xXzZ' and tok[1:] in values:
            values[tok[1:]].append((time, tok[0]))

def trace(name):
    matches = [c for n, c in signals.items() if n.endswith('/dut/' + name)]
    if len(matches) != 1:
        raise ValueError(f'Signal missing/ambiguous: {name}')
    return values[matches[0]], widths[matches[0]]

def at(name, t):
    data, _ = trace(name)
    idx = bisect.bisect_right([x[0] for x in data], t) - 1
    return data[idx][1] if idx >= 0 else 'x'

def rising(name):
    return [t for t, v in trace(name)[0] if v == '1']

def draw(filename, title, names, start, end):
    left, right, row = 230, 1230, 48
    height = 115 + row * len(names)
    def x(t):
        return left + (t - start) / (end - start) * (right - left)
    svg = [f'<svg xmlns="http://www.w3.org/2000/svg" width="1280" height="{height}" viewBox="0 0 1280 {height}">',
           '<rect width="100%" height="100%" fill="#101b2b"/>',
           '<style>text{font-family:monospace;fill:#dce6f1;font-size:13px}.title{font-family:sans-serif;font-size:20px;font-weight:bold}</style>',
           f'<text x="24" y="30" class="title">{html.escape(title)}</text>',
           '<text x="24" y="52">XSim VCD | accelerated testbench | time : ns | buses : hex</text>']
    for j in range(6):
        t = start + j * (end - start) / 5
        xx = x(t)
        svg.append(f'<path d="M{xx},76 V{height-36}" stroke="#293b50"/>')
        svg.append(f'<text x="{xx}" y="{height-14}" text-anchor="middle">{t:.0f}</text>')
    for idx, name in enumerate(names):
        y = 91 + idx * row
        svg.append(f'<text x="16" y="{y+5}">{name}</text>')
        data, width = trace(name)
        points = [(start, at(name, start))] + [(t, v) for t, v in data if start < t < end] + [(end, at(name, end))]
        if width == 1:
            path = ''
            for k, (t, v) in enumerate(points):
                yy = y - 12 if v == '1' else y + 12
                path += (f'M{x(t):.2f},{yy}' if k == 0 else f'H{x(t):.2f}V{yy}')
            svg.append(f'<path d="{path}" fill="none" stroke="#7bd9ca" stroke-width="2"/>')
        else:
            for (t, v), (next_t, _) in zip(points, points[1:]):
                x1, x2 = x(t), x(next_t)
                svg.append(f'<path d="M{x1},{y} L{x1+2},{y-12} H{x2-2} L{x2},{y} L{x2-2},{y+12} H{x1+2} Z" fill="#20344c" stroke="#8bb7e6"/>')
                if x2 - x1 > 32:
                    label = f'{int(v,2):0{(width+3)//4}X}' if all(c in '01' for c in v) else v
                    svg.append(f'<text x="{(x1+x2)/2}" y="{y+5}" text-anchor="middle">{label}</text>')
    svg.append('</svg>')
    (a.out / filename).write_text('\n'.join(svg))

decisions = []
for t in rising('decision_valid'):
    decisions.append({'time_ns': t, **{n: at(n, t) for n in ['decision_accept', 'decision_degraded', 'decision_mismatch_drop', 'decision_selected_b', 'decision_sequence']}})
normal = decisions[0]['time_ns']
mismatch = next(d['time_ns'] for d in decisions if d['decision_mismatch_drop'] == '1')
fallback = next(d['time_ns'] for d in decisions if d['decision_degraded'] == '1' and d['decision_selected_b'] == '1' and int(d['decision_sequence'], 2) == 0x13)
dup = rising('duplicate_drop')[0]
common = ['a_crc_ok', 'b_crc_ok', 'decision_valid', 'decision_accept', 'decision_sequence', 'duplicate_out_valid', 'raw_tx_valid', 'rs422_tx_out']
draw('sim_normal.svg', '01 / Matching frames : accept once and start UART output', common, normal-100, normal+400)
draw('sim_mismatch.svg', '02 / Payload mismatch : reject and raise diagnostic IRQ', ['a_crc_ok','b_crc_ok','decision_valid','decision_mismatch_drop','decision_accept','duplicate_out_valid','irq','rs422_tx_out'], mismatch-100, mismatch+400)
draw('sim_fallback.svg', '03 / A CRC error : wait for pair timeout, then select channel B', ['a_crc_error','b_crc_ok','pair_wait_active','decision_valid','decision_degraded','decision_selected_b','duplicate_out_valid','rs422_tx_out'], fallback-800, fallback+300)
draw('sim_duplicate.svg', '04 / Late duplicate : drop previously forwarded ID + sequence', ['decision_valid','decision_accept','decision_sequence','duplicate_drop','duplicate_out_valid','raw_tx_valid','rs422_tx_out'], dup-100, dup+300)
(a.out / 'waveform_events.json').write_text(json.dumps({'timescale_ns': scale, 'decisions': decisions, 'duplicate_drop_ns': rising('duplicate_drop'), 'crc_error_a_ns': rising('a_crc_error'), 'first_tx_start_ns': next(t for t,v in trace('rs422_tx_out')[0] if t > normal and v == '0')}, indent=2) + '\n')
