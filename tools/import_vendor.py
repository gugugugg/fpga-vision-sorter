from pathlib import Path
import argparse
import json
import shutil

ROOT = Path(__file__).resolve().parents[1]


def main():
    ap = argparse.ArgumentParser(description='导入官方 BSP/IP 文件')
    group = ap.add_mutually_exclusive_group(required=True)
    group.add_argument('--reference-project', type=Path)
    group.add_argument('--vendor-root', type=Path, nargs='+')
    args = ap.parse_args()
    entries = json.loads((ROOT/'dependencies/vendor_files.json').read_text(encoding='utf-8'))
    selected = []
    problems = []
    for item in entries:
        if args.reference_project:
            candidates = [args.reference_project/item['reference_path']]
            candidates = [p for p in candidates if p.is_file()]
        else:
            direct = [base/item['official_path'] for base in args.vendor_root if item['official_path']]
            candidates = [p for p in direct if p.is_file()]
            if not candidates:
                candidates = [p for base in args.vendor_root for p in base.rglob(Path(item['target']).name) if p.is_file()]
        if not candidates:
            problems.append('Missing: '+item['target'])
        elif any(p.read_bytes() != candidates[0].read_bytes() for p in candidates[1:]):
            problems.append('Multiple versions: '+item['target']+'\n  '+'\n  '.join(map(str, candidates)))
        else:
            selected.append((candidates[0], ROOT/item['target']))
    if problems:
        raise SystemExit('\n'.join(problems))
    for source, target in selected:
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
    print(f'Imported {len(selected)} BSP/IP files')


if __name__ == '__main__':
    main()
