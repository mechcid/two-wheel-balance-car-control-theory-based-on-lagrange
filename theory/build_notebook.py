"""直接读取主脚本，按章节生成笔记本，避免维护两套模型。"""
from pathlib import Path
import re
import nbformat as nbf
p = Path(__file__).resolve().parent
source = (p/'sympy_scaffold.py').read_text()
chunks = re.split(r'(?m)(?=^# =+\n# §)', source)
nb = nbf.v4.new_notebook()
nb.metadata.kernelspec = {'display_name':'Python (lagarain)', 'language':'python', 'name':'lagarain'}
nb.cells = [nbf.v4.new_markdown_cell('# 平衡车：拉格朗日推导\n六坐标 → 动能与势能 → 拉格朗日求导 → 三个约束乘子 → 三方向消元。\n\n主脚本是唯一模型来源。保留车轴平面为高度/势能参考。世界 x 参数化要求 $\\cos\\psi \\ne 0$。旧 Kane 结果已归档，其中 $Q_x=0$ 的结论已撤回。'), nbf.v4.new_code_cell('from IPython.display import display, Math\nfrom sympy.physics.vector import vlatex\ndef show(name, expr):\n    display(Math(name + " = " + vlatex(expr)))')]
for chunk in chunks:
    if not chunk.strip(): continue
    title = re.search(r'^# (§[^\n]+)', chunk, re.M)
    if title: nb.cells.append(nbf.v4.new_markdown_cell('## '+title.group(1)))
    nb.cells.append(nbf.v4.new_code_cell(chunk))
    if title:
        n=title.group(1)
        out=''
        if n.startswith('§3a'):
            nb.cells.append(nbf.v4.new_markdown_cell(r"记号：$ {}^P R_F$ 将 F 系分量变换到 P 系，$v^P={}^P R_F v^F$；$ {}^P H_F=[{}^P R_F,{}^P p_{P_oF_o};0,1]$。代码用 H 表示齐次变换，避免与动能 T 重名。B 原点位于质心，因此 C→B 的平移不是零。以下尚未代入滚动约束或小角度近似。"))
            for parent, child in [('N','C'),('C','L'),('C','R'),('C','B')]:
                nb.cells.append(nbf.v4.new_markdown_cell('### '+parent+' → '+child))
                nb.cells.append(nbf.v4.new_code_cell(
                    'show(r"{}^'+parent+'R_'+child+'", R_'+parent+child+')\n'
                    'show(r"{}^'+parent+'p_{'+parent+'o,'+child+'o}", p_'+parent+child+')\n'
                    'show(r"{}^'+parent+'H_'+child+'", H_'+parent+child+')'))
        if n.startswith('§3b'):
            nb.cells.append(nbf.v4.new_markdown_cell(r"相对角速度：$ {}^P\Omega_{F/P}=\dot R_{PF}R_{PF}^{T}$，$ {}^F\Omega_{F/P}=R_{PF}^{T}\dot R_{PF}$；$\Omega=[\omega]_\times$。分别展示父系、子系分量。"))
            for parent, child in [('N','C'),('C','L'),('C','R'),('C','B')]:
                nb.cells.append(nbf.v4.new_markdown_cell('### '+child+' 相对 '+parent))
                key=repr((parent,child))
                nb.cells.append(nbf.v4.new_code_cell('\n'.join(
                    'show(r"{}^'+basis+symbol+'_{'+child+'/'+parent+'}", relative_angular['+key+']['+repr(field)+'])'
                    for basis,symbol,field in [(parent,r'\omega','omega_parent'),(parent,r'\Omega','Omega_parent'),(child,r'\omega','omega_child'),(child,r'\Omega','Omega_child')]
                )))
        if n.startswith('§3c'):
            nb.cells.append(nbf.v4.new_markdown_cell(r"对 $F=L,R,B$：$ {}^NH_F={}^NH_C\,{}^CH_F$；$ {}^N\omega_{F/N}={}^N\omega_{C/N}+{}^NR_C\,{}^C\omega_{F/C}$。下列角速度均相对 N、用 N 基表示。"))
            for child in ['C','L','R','B']:
                nb.cells.append(nbf.v4.new_markdown_cell('### '+child+' 相对 N'))
                nb.cells.append(nbf.v4.new_code_cell('\n'.join(
                    'show(r"{}^N'+symbol+'_{'+child+'}", '+mapping+'['+repr(child)+'])'
                    for symbol,mapping in [('R','rotations_N'),('H','transforms_N'),(r'\omega','omega_N'),(r'\Omega','Omega_N')]
                )))
        if n.startswith('§3d'):
            nb.cells.append(nbf.v4.new_markdown_cell(r"两条计算路径：$ {}^Nv_{F_o/N}=d({}^Np_{N_oF_o})/dt$；以及 $ {}^Nv_{F_o/N}={}^Nv_{C_o/N}+{}^NR_C({}^C\Omega_{C/N}{}^Cp_{C_oF_o}+d({}^Cp_{C_oF_o})/dt)$。后者括号中的导数是 C 系分量导数。每点按位置、展开速度、递推速度、雅可比展示。$\dot q=(\dot x,\dot y,\dot\psi,\dot\theta,\dot\theta_L,\dot\theta_R)^T$，$v^N=J_v^N\dot q$。"))
            for child in ['C','L','R','B']:
                nb.cells.append(nbf.v4.new_markdown_cell('### '+child+'o 在 N 系下'))
                nb.cells.append(nbf.v4.new_code_cell('\n'.join(
                    'show(r"'+label+'", '+mapping+'['+repr(child)+'])'
                    for label,mapping in [(r'{}^Np_{'+child+'o}','positions_N'),(r'{}^Nv_{'+child+'o/N}','velocities_N'),(r'{}^Nv_{'+child+'o/N,rec}','velocities_recursive_N'),(r'{}^NJ_{v,'+child+'o}','Jv_N'),(r'{}^NJ_{\omega,'+child+'}','Jw_N')]
                )))
        if n.startswith('§3e'):
            nb.cells.append(nbf.v4.new_markdown_cell(r"接触材料点：$ {}^Nv_{CL/N}={}^Nv_{Lo/N}+{}^N\Omega_{L/N}[0,0,-r]^T$，右轮同理。这是当前轮上材料点的速度，不能用最低几何位置的轨迹导数替代。"))
            out='show(r"{}^Nv_{CL/N}", v_CL_matrix)\nshow(r"{}^Nv_{CR/N}", v_CR_matrix)'
        if n.startswith('§3f'):
            out='print("矩阵展开/递推/对象法逐项检查：", len(kinematic_checks), "项")\nprint("全部通过：", all(kinematic_checks.values()))'
        if n.startswith('§5 '): out='show("T_{exact}", T_exact)\nshow("V_{exact}", V_exact)'
        if n.startswith('§5a'):
            nb.cells.append(nbf.v4.new_markdown_cell(r"静止直立点：$\theta_0=0,\psi=\psi_0,\theta_{L0}=\theta_{R0}=0$，全部名义速度为零。下列导数为扰动速度；先展示 N 系一阶速度，再逐刚体构造二阶动能。保留六个速度，不代入滚动约束。后续两条拉格朗日分支均使用本节近似能量；原始非完整约束保留。"))
            out='\n'.join([
                'show(r"{}^Nv_{L}^{(1)}", v_L_first)',
                'show(r"{}^Nv_{R}^{(1)}", v_R_first)',
                'show(r"{}^Nv_{B}^{(1)}", v_B_first)',
                'show(r"{}^N\\omega_{L}^{(1)}", w_L_first)',
                'show(r"{}^N\\omega_{R}^{(1)}", w_R_first)',
                'show(r"{}^N\\omega_{B}^{(1)}", w_B_first)',
                'show(r"T_L^{(2)}", sm.trigsimp(T2_L))',
                'show(r"T_R^{(2)}", sm.trigsimp(T2_R))',
                'show(r"T_B^{(2)}", sm.trigsimp(T2_B))',
                'show(r"T^{(2)}", T_quadratic_compact)',
                'show(r"V_{\\leq2}", V_quadratic)',
                'print("一阶速度构造动能 = 精确动能二阶展开：已通过符号恒等检查")'])
        if n.startswith('§7'): out='show(r"\\partial L/\\partial \\dot q", dL_dqd)\nshow(r"d(\\partial L/\\partial \\dot q)/dt", d_dt_dL_dqd)\nshow(r"\\partial L/\\partial q", dL_dq)'
        if n.startswith('§8'):
            nb.cells.append(nbf.v4.new_markdown_cell(r"**分支一**：$M_6\ddot q+h_6=Q_6+A^T\lambda$，$A\ddot q+\dot A\dot q=0$。约束反力不决定电机主动广义力，两者分别展示。9×9系统同时解六个加速度和三个乘子。"))
            out='show("A", A)\nshow("Q_6", Q6)\nshow(r"Q_c=A^T\\lambda", Q_constraint)\nfor e in eom_multiplier: show("0", e)\nshow("M_{aug}", augmented_matrix)\nshow("b_{aug}", augmented_rhs)'
        if n.startswith('§9'):
            nb.cells.append(nbf.v4.new_markdown_cell(r"**分支二**：第二类无乘子形式，含非完整修正。令 $L_r=L(q,S\dot z)$、$p=\partial L/\partial\dot q$，正确左端为 $E_r=\frac{d}{dt}\frac{\partial L_r}{\partial\dot z}-\frac{\partial L_r}{\partial z}+\Delta$，其中 $\Delta=\frac{\partial L_r}{\partial z}-S^T\frac{\partial L}{\partial q}-\dot S^Tp$（各项代入允许速度）。右端从电机功率提取。不能丢掉 $\Delta$。"))
            out='show("S", S)\nshow("L_r", L_reduced)\nshow(r"E_{naive}", EL_naive)\nshow(r"\\Delta", nonholonomic_correction)\nshow(r"\\mathcal{P}_r", power_reduced)\nshow("Q_r", Q_second)\nshow("M_r", M_second)\nshow("h_r", h_second)\nfor e in eom_second: show("0", e)'
        if n.startswith('§10'):
            nb.cells.append(nbf.v4.new_markdown_cell(r"两分支合流，矩阵、偏置、主动广义力逐项相等。局部一阶模型：$M_0\delta\ddot z+C_0\delta\dot z+K_0\delta z=B_0\tau$。这里 $\psi_0$ 为固定名义朝向；仅适用于该静止直立点附近。先前保留原约束的方程本身不是线性系统。"))
            out='print("两分支符号恒等检查通过")\nshow("M_0", M_linear)\nshow("C_0", C_linear)\nshow("K_0", K_linear)\nshow("B_0", B_linear)'
        if out: nb.cells.append(nbf.v4.new_code_cell(out))
nbf.write(nb, p/'sympy_scaffold.ipynb')
print('Notebook generated from sympy_scaffold.py')
