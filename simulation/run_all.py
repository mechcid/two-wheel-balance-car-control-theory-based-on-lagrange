"""项目根目录执行 .venv/bin/python simulation/run_all.py；不启动ROS/Gazebo。"""
from pathlib import Path
import argparse,importlib.util,json
from common import serial

def write_report(results,target):
    lines=['# 数学仿真运行结果','',
           '参数为临时示例；累计前向行程 s，质心距离 l。下表是实际运行值，不是 Gazebo/实车结果。','',
           '| 分支/场景 | 末端俯仰误差 rad | 末端位置误差 m | 电机峰值 N m | 限幅采样比例 |',
           '|---|---:|---:|---:|---:|']
    for branch,cases in results.items():
        for name,result in cases.items():
            e=result['final_error'];theta=e[2 if branch=='straight' else 3]
            position=abs(e[0]) if branch=='straight' else (e[0]**2+e[1]**2)**.5
            lines.append(f"| {branch}/{name} | {theta:.4g} | {position:.4g} | {max(result['peak_motor_torque_Nm']):.4g} | {result['saturation_fraction']:.4g} |")
    lines += ['', 'open_loop 的发散是预期现象，达到俯仰展示阈值后提前停止；其余为闭环。',
              '静止分支为六状态局部线性模型；转向分支每种场景分别运行小俯仰模型和精确模型。',
              '转向时评价的是相对运动参考的误差，世界位置和偏航角本身不趋于0；额外记录的累计行程 s 不参与控制。',
              '', '每个案例的 outputs 子目录包含 trajectory.csv、states.png、torques.png 和 result.json；转向还有 path.png，父目录有 comparison.png 和 comparison.json。',
              'A/B/Q/R/P/K 与当前物理参数保存在案例 result.json；七状态 K 作用于参考朝向下的旋转误差。']
    target.with_suffix('.md').write_text('\n'.join(lines)+'\n')

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--scenario',choices=['all','straight','turning'],default='all');args=parser.parse_args()
    root=Path(__file__).resolve().parent;results={}
    for key,folder in [('straight','straight_balance'),('turning','turning_balance')]:
        if args.scenario not in ('all',key):continue
        print('Running',key,flush=True)
        spec=importlib.util.spec_from_file_location('run_'+key,root/folder/'run.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
        results[key]=m.run_all();print('PASS',key,flush=True)
    target=root/('results.json' if args.scenario=='all' else args.scenario+'_results.json')
    target.write_text(json.dumps(results,indent=2,default=serial));write_report(results,target);print(target)
if __name__=='__main__':main()
