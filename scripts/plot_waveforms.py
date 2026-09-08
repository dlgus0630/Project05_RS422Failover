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

def draw(filename, title, names, start, end, notes):
    names = ['clk'] + names
    left, right, row = 290, 1360, 46
    bottom = 155 + row * len(names)
    height = bottom + 210
    def x(t):
        return left + (t - start) / (end - start) * (right - left)
    svg = [f'<svg xmlns="http://www.w3.org/2000/svg" width="1400" height="{height}" viewBox="0 0 1400 {height}">',
           '<rect width="100%" height="100%" fill="#0b1220"/>',
           '<style>text{font-family:"DejaVu Sans Mono",monospace;fill:#dce6f1;font-size:15px}.title,.note,.sub{font-family:"Noto Sans CJK KR",sans-serif}.title{font-size:25px;font-weight:bold}.note{fill:#ffe28a;font-size:16px}.sub{font-size:16px;fill:#b9c7d8}</style>',
           f'<text x="34" y="43" class="title">RS422 Failover : {html.escape(title)} — XSim 실측 파형</text>',
           f'<text x="34" y="75" class="sub">tb_redundant_link_core · {start:,.0f}~{end:,.0f} ns · 실제 core.vcd 신호 · UART 10 clocks/bit 가속 설정</text>',
           f'<rect x="20" y="100" width="1360" height="{height-145}" rx="14" fill="#121e30" stroke="#334155" stroke-width="2"/>',
           '<text x="62" y="137" class="sub">시간 [ns] · 버스 : hex</text>']
    step = 10 if end-start <= 500 else 50
    for t in range(int(start), int(end)+1, step):
        xx = x(t)
        major = (t-int(start)) % (step*5) == 0
        svg.append(f'<path d="M{xx},150 V{bottom}" stroke="{("#33455e" if major else "#1c2a3d")}"/>')
        if major:
            svg.append(f'<text x="{xx}" y="137" text-anchor="middle">{t:.0f}</text>')
    for idx, name in enumerate(names):
        y = 168 + idx * row
        color = '#24bdf3' if name == 'clk' else ('#ffad18' if any(k in name for k in ['error','drop','irq','degraded']) else '#00cc66')
        svg.append(f'<path d="M33,{y-17} V{y+17}" stroke="{color}" stroke-width="4" stroke-linecap="round"/>')
        svg.append(f'<text x="60" y="{y+5}">{name}</text>')
        data, width = trace(name)
        points = [(start, at(name, start))] + [(t, v) for t, v in data if start < t < end] + [(end, at(name, end))]
        if width == 1:
            path = ''
            for (t, v), (next_t, _) in zip(points, points[1:]):
                if v == '1':
                    svg.append(f'<rect x="{x(t)}" y="{y-12}" width="{x(next_t)-x(t)}" height="24" fill="{color}" opacity="0.13"/>')
            for k, (t, v) in enumerate(points):
                yy = y - 12 if v == '1' else y + 12
                path += (f'M{x(t):.2f},{yy}' if k == 0 else f'H{x(t):.2f}V{yy}')
            svg.append(f'<path d="{path}" fill="none" stroke="{color}" stroke-width="2"/>')
        else:
            for (t, v), (next_t, _) in zip(points, points[1:]):
                x1, x2 = x(t), x(next_t)
                bevel = min(6, (x2-x1)/2)
                svg.append(f'<path d="M{x1},{y} L{x1+bevel},{y-14} H{x2-bevel} L{x2},{y} L{x2-bevel},{y+14} H{x1+bevel} Z" fill="#211e49" stroke="#aa88ff" stroke-width="1.5"/>')
                if x2 - x1 > 32:
                    label = f'{int(v,2):0{(width+3)//4}X}' if all(c in '01' for c in v) else v
                    svg.append(f'<text x="{(x1+x2)/2}" y="{y+5}" text-anchor="middle">{label}</text>')
    for idx, (t, lines) in enumerate(notes):
        bx, by, bw = 60 + idx*440, bottom+35, 410
        xx = x(t)
        svg.append(f'<path d="M{xx},150 V{bottom+12} L{bx+bw/2},{by}" stroke="#ffad18" stroke-dasharray="6 5" fill="none"/>')
        svg.append(f'<circle cx="{xx}" cy="{bottom+12}" r="4" fill="#ffad18"/>')
        svg.append(f'<rect x="{bx}" y="{by}" width="{bw}" height="104" rx="9" fill="#162236" stroke="#ffad18" stroke-width="1.5"/>')
        for j, line in enumerate(lines):
            svg.append(f'<text x="{bx+14}" y="{by+25+j*24}" class="note">{html.escape(line)}</text>')
    svg.append(f'<text x="34" y="{height-18}" class="sub">청록 : 클럭 · 초록 : 유효/전달 · 주황 : 오류/차단/진단 · 보라 : 버스 · 점선 : 실제 이벤트 시각</text>')
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
tx = next(t for t,v in trace('rs422_tx_out')[0] if t > normal and v == '0')
forward = next(t for t in rising('duplicate_out_valid') if t >= normal)
crc = rising('a_crc_error')[0]
irq = next(t for t in rising('irq') if t >= mismatch)
fallback_tx = next(t for t,v in trace('rs422_tx_out')[0] if t > fallback and v == '0')
draw('sim_normal.svg', '정상 Pair 수신과 단일 출력', common, normal-100, normal+400, [
    (normal, ['① 동일 프레임 채택', f'accept=1 @{normal:,.0f} ns', 'SEQ=0x10 · 우선 채널 A 선택']),
    (forward, ['② 중복 검사 통과', f'out_valid=1 @{forward:,.0f} ns', 'ID·CMD 매핑 후 CRC 재계산']),
    (tx, ['③ UART 출력 시작', f'TX start bit @{tx:,.0f} ns', f'판정 이후 {tx-normal:.0f} ns / {(tx-normal)/10:.0f} clocks'])])
draw('sim_mismatch.svg', 'Payload 불일치 차단과 IRQ', ['a_crc_ok','b_crc_ok','decision_valid','decision_mismatch_drop','decision_accept','duplicate_out_valid','irq','rs422_tx_out'], mismatch-100, mismatch+400, [
    (mismatch, ['① 불일치 검출', f'mismatch_drop=1 @{mismatch:,.0f} ns', 'CRC 정상이어도 내용이 다르면 차단']),
    (mismatch+10, ['② 출력 경로 차단', 'decision_accept=0 유지', 'duplicate_out_valid=0 · TX idle']),
    (irq, ['③ 진단 인터럽트', f'IRQ=1 @{irq:,.0f} ns', 'AXI로 DATA_MISMATCH(0x0B) 확인'])])
draw('sim_fallback.svg', 'CRC 오류와 B 채널 Fallback', ['a_crc_error','b_crc_ok','pair_wait_active','decision_valid','decision_degraded','decision_selected_b','duplicate_out_valid','rs422_tx_out'], fallback-800, fallback+300, [
    (crc, ['① A 오류 / B 정상', f'A CRC error @{crc:,.0f} ns', '손상된 A 프레임은 전달하지 않음']),
    (fallback, ['② Pair 대기 후 B 채택', f'selected_b=1 @{fallback:,.0f} ns', 'degraded=1 · 정상 경로로 전달']),
    (fallback_tx, ['③ B 데이터 출력', 'payload=BE EF · CRC=57 9A', '출력 바이트 전체를 자동 비교'])])
draw('sim_duplicate.svg', '지연 중복 프레임 차단', ['decision_valid','decision_accept','decision_sequence','duplicate_drop','duplicate_out_valid','raw_tx_valid','rs422_tx_out'], dup-100, dup+300, [
    (dup-10, ['① 늦게 도착한 B 채택', f'accept=1 @{dup-10:,.0f} ns', 'SEQ=0x12 · 앞서 A로 전달된 명령']),
    (dup, ['② 이력에서 동일 ID·SEQ 검출', f'duplicate_drop=1 @{dup:,.0f} ns', '최근 4개 키와 비교하여 중복 제거']),
    (dup+10, ['③ 추가 전송 없음', 'duplicate_out_valid=0 유지', 'TX idle · 동일 명령 이중 실행 방지'])])
(a.out / 'waveform_events.json').write_text(json.dumps({'timescale_ns': scale, 'decisions': decisions, 'duplicate_drop_ns': rising('duplicate_drop'), 'crc_error_a_ns': rising('a_crc_error'), 'first_tx_start_ns': next(t for t,v in trace('rs422_tx_out')[0] if t > normal and v == '0')}, indent=2) + '\n')
