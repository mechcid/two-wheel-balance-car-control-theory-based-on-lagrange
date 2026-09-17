"""Read-only validation of numerical matrices against the final symbolic model."""
import sys,io,contextlib
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'theory'))
with contextlib.redirect_stdout(io.StringIO()):
    import sympy_scaffold as symbolic
from lqr_simulation import Parameters,build_state_space
from dataclasses import asdict
import numpy as np
for p in [Parameters(),Parameters(psi0=.4,m_R=.65,I_Ryy=.0032),Parameters(psi0=-.7)]:
    substitutions={getattr(symbolic,'psi_0' if k=='psi0' else k):v for k,v in asdict(p).items()}
    actual=build_state_space(p)[:4]
    for name,numerical in zip(['M_linear','C_linear','K_linear','B_linear'],actual):
        expected=np.array(getattr(symbolic,name).subs(substitutions),dtype=float)
        np.testing.assert_allclose(numerical,expected,rtol=1e-12,atol=1e-12)
print('PASS: all four numerical matrices match final symbolic code at 3 headings, including asymmetric wheels.')
