from pathlib import Path
import argparse
import subprocess

ROOT=Path(__file__).resolve().parents[1]


def main():
    ap=argparse.ArgumentParser(description='运行按键模板采集与最近模板分类仿真')
    ap.add_argument('--iverilog',default='iverilog')
    ap.add_argument('--vvp',default='vvp')
    args=ap.parse_args()
    out=ROOT/'build/sim';out.mkdir(parents=True,exist_ok=True)
    rtl=ROOT/'fpga/inputs'
    jobs={
        'tb_manual_template_capture':['part_visual_grid.v','part_background_mask.v','part_mask_normalizer.v','part_binary_template.v','part_template_vote.v','part_frame_difference.v'],
        'tb_templates':['part_binary_template.v'],
        'tb_motion_noise_threshold':['part_frame_difference.v'],
    }
    for name,files in jobs.items():
        executable=out/(name+'.vvp')
        subprocess.run([args.iverilog,'-g2012','-s',name,'-o',str(executable),str(ROOT/'sim'/(name+'.sv'))]+[str(rtl/p) for p in files],check=True)
        subprocess.run([args.vvp,str(executable)],cwd=ROOT/'sim',check=True)


if __name__=='__main__':main()
