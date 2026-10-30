from pathlib import Path
import argparse
import json
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--iverilog', default='iverilog')
    ap.add_argument('--vvp', default='vvp')
    args = ap.parse_args()
    out = ROOT/'build/sim'
    out.mkdir(parents=True, exist_ok=True)
    (out/'gains.hex').write_text('500600\n')
    results = {}
    for name, module in [('frontend', 'raw_color_shading_4ppc.v'), ('core', 'vision_core_4ppc.v')]:
        exe = out/(name+'.vvp')
        subprocess.run([args.iverilog, '-g2012', '-s', 'tb', '-y', str(ROOT/'fpga/inputs'),
                        '-o', str(exe), str(ROOT/f'sim/tb_{name}.sv'), str(ROOT/'fpga/inputs'/module)], check=True)
        result = subprocess.run([args.vvp, str(exe)], cwd=out, capture_output=True, text=True, check=True)
        print(result.stdout, end='')
        results[name] = result.stdout.strip()
    (out/'result.json').write_text(json.dumps(results, indent=2), encoding='utf-8')


if __name__ == '__main__':
    main()
