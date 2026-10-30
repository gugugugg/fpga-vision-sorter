from pathlib import Path
import argparse
import html
import json
import os
import re
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def main():
    ap = argparse.ArgumentParser(description='构建 0x3922 SW2 切换工程，不执行烧录')
    ap.add_argument('--td-root', type=Path, default=Path('D:/anlu'))
    ap.add_argument('--output', type=Path, default=ROOT/'build')
    ap.add_argument('--prepare-only', action='store_true')
    args = ap.parse_args()
    source = ROOT/'fpga'
    project = (source/'contest_board_system.al').read_bytes()
    paths = [html.unescape(s) for s in re.findall(r'<File Path="([^"]+)"', project.decode('utf-8'))]
    missing = [s for s in paths if not (source/s).is_file()]
    if missing:
        raise SystemExit('请先导入 BSP/IP：\n'+'\n'.join(missing))
    out = args.output.resolve()
    shutil.copytree(source, out, dirs_exist_ok=True)
    if args.prepare_only:
        print(f'Prepared {len(paths)} project references in {out}')
        return
    env = os.environ.copy()
    env['TD_ROOT'] = args.td_root.resolve().as_posix()
    command = args.td_root/'bin/td_commands_prompt.exe'
    for stage in ('place', 'finish'):
        print(f'{stage}: {out/(stage+".log")}', flush=True)
        try:
            with (out/(stage+'.log')).open('wb') as log:
                subprocess.run([str(command), stage+'.tcl'], cwd=out, env=env,
                               stdout=log, stderr=subprocess.STDOUT, check=True)
        finally:
            (out/'contest_board_system.al').write_bytes(project)
        text = (out/(stage+'.log')).read_text(errors='replace')
        marker = 'PROFILE_PLACE_FINISHED' if stage == 'place' else 'PROFILE_FULL_FLOW_FINISHED'
        if marker not in text:
            raise SystemExit(f'{stage} 未完成，请查看日志')
    text = (out/'profile_routed.ts').read_text(errors='replace')
    metrics = {key: float(re.search(r'\b'+key+r':\s*([-+0-9.]+)ns', text)[1])
               for key in ('SWNS', 'STNS', 'HWNS', 'HTNS')}
    print(json.dumps(metrics, indent=2))
    print(out/'contest_board_system.bit')


if __name__ == '__main__':
    main()
