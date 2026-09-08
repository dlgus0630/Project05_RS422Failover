"""Run Verilog testbenches with Vivado 2024.2; keep generated files in output dir."""
import argparse
import pathlib
import re
import subprocess

p = argparse.ArgumentParser()
p.add_argument('root', type=pathlib.Path)
p.add_argument('out', type=pathlib.Path)
a = p.parse_args()
root, out = a.root.resolve(), a.out.resolve()
out.mkdir(parents=True, exist_ok=True)
src = root / 'Project04_RS422Failover.srcs'
rtl = sorted((src / 'sources_1/new').glob('*.v'))
tbs = sorted((src / 'sim_1/new').glob('tb_*.v'))

def run(args, name):
    r = subprocess.run(args, cwd=out, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    (out / name).write_text(r.stdout)
    if r.returncode:
        raise RuntimeError(f'{name}: exit {r.returncode}; inspect log')
    return r.stdout

run(['xvlog', *map(str, rtl + tbs)], 'compile.log')
results = []
for tb in tbs:
    top = tb.stem
    run(['xelab', top, '-debug', 'typical', '-s', top], top + '_elaborate.log')
    script = out / (top + '.tcl')
    vcd = ''
    if top == 'tb_redundant_link_core':
        vcd = 'open_vcd core.vcd\nlog_vcd [get_objects /tb_redundant_link_core/dut/*]\n'
    script.write_text(vcd + 'run all\n' + ('close_vcd\n' if vcd else '') + 'quit\n')
    log = run(['xsim', top, '-tclbatch', str(script)], top + '.log')
    ok = bool(re.search(r'PASS', log)) and not re.search(r'\[FAIL\]|FAILED|FATAL|ERROR:', log)
    results.append(f'{top}: {"PASS" if ok else "FAIL"}')
    print(results[-1], flush=True)
(out / 'simulation_summary.txt').write_text('\n'.join(results) + '\n')
if any(x.endswith('FAIL') for x in results):
    raise SystemExit(1)
