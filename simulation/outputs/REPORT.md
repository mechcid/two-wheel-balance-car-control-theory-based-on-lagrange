# LQR numerical validation

DEMO ONLY: these are NOT actual robot parameters.

Source: theory/sympy_scaffold.py final matrices. World x convention. psi0=0.
No saturation, delay, nonlinear plant, or measured actuator limits modeled.

## Parameters
```json
{
  "r": 0.1,
  "d": 0.4,
  "l": 0.3,
  "m_B": 2.0,
  "m_L": 0.5,
  "m_R": 0.5,
  "I_Lyy": 0.0025,
  "I_Ryy": 0.0025,
  "I_Lzz": 0.00125,
  "I_Rzz": 0.00125,
  "I_Byy": 0.08,
  "I_Bzz": 0.06,
  "g": 9.81,
  "psi0": 0.0
}
```

## M0
```text
[[3.5    0.     0.6   ]
 [0.     0.1225 0.    ]
 [0.6    0.     0.26  ]]
```

## C0
```text
[[0. 0. 0.]
 [0. 0. 0.]
 [0. 0. 0.]]
```

## K0
```text
[[ 0.     0.     0.   ]
 [ 0.     0.     0.   ]
 [ 0.     0.    -5.886]]
```

## B0
```text
[[10. 10.]
 [-2.  2.]
 [-1. -1.]]
```

## A
```text
[[ 0.          0.          0.          1.          0.          0.        ]
 [ 0.          0.          0.          0.          1.          0.        ]
 [ 0.          0.          0.          0.          0.          1.        ]
 [-0.         -0.         -6.42109091 -0.         -0.         -0.        ]
 [-0.         -0.         -0.         -0.         -0.         -0.        ]
 [-0.         -0.         37.45636364 -0.         -0.         -0.        ]]
```

## B
```text
[[  0.           0.        ]
 [  0.           0.        ]
 [  0.           0.        ]
 [  5.81818182   5.81818182]
 [-16.32653061  16.32653061]
 [-17.27272727 -17.27272727]]
```

## K_LQR
```text
[[-0.70710678 -0.5        -8.32450297 -1.37935257 -0.36142081 -2.04466231]
 [-0.70710678  0.5        -8.32450297 -1.37935257  0.36142081 -2.04466231]]
```

## Q
```text
[[ 10.   0.   0.   0.   0.   0.]
 [  0.   5.   0.   0.   0.   0.]
 [  0.   0. 400.   0.   0.   0.]
 [  0.   0.   0.   5.   0.   0.]
 [  0.   0.   0.   0.   2.   0.]
 [  0.   0.   0.   0.   0.  40.]]
```

## R
```text
[[10.  0.]
 [ 0. 10.]]
```

## Diagnostics
```json
{
  "mass_rank": 3,
  "mass_determinant": 0.06737500000000003,
  "mass_condition": 29.449319231563724,
  "mass_eigenvalues": [
    0.1225,
    0.15245839413344375,
    3.6075416058665564
  ],
  "controllability_rank": 6,
  "controllability_singular_values": [
    34783.35545436376,
    34783.35545436375,
    23.089201018336247,
    23.089201018336237,
    3.9825142472881883,
    3.9825142472868125
  ],
  "open_unstable": true,
  "open_poles": [
    [
      0.0,
      0.0
    ],
    [
      -0.0,
      0.0
    ],
    [
      6.12016042570484,
      0.0
    ],
    [
      -6.120160425704838,
      0.0
    ],
    [
      0.0,
      0.0
    ],
    [
      -0.0,
      0.0
    ]
  ],
  "closed_poles": [
    [
      -49.84821689407604,
      0.0
    ],
    [
      -3.1994134856119283,
      0.0
    ],
    [
      -0.7677552739081928,
      0.5996015534124732
    ],
    [
      -0.7677552739081928,
      -0.5996015534124732
    ],
    [
      -10.20101471332805,
      0.0
    ],
    [
      -1.6004810375298832,
      0.0
    ]
  ],
  "open_final_time": 0.42230822765479,
  "open_stopped_at_cutoff": true,
  "open_final_pitch": 0.19999999999999968,
  "closed_final_state": [
    7.741692138629921e-07,
    -2.1578415659694795e-21,
    2.2010633768923362e-07,
    -1.3714231621529274e-06,
    5.173229537768784e-21,
    -1.2164729930767885e-07
  ],
  "peak_abs_torque": [
    0.24973508896201824,
    0.24973508896201788
  ],
  "peak_abs_state": [
    0.035008779751497264,
    2.0176558821107617e-16,
    0.03,
    0.053461700319244194,
    4.450619998730466e-16,
    0.12267281489984075
  ],
  "care_relative_residual": 8.261162368253615e-15,
  "integration_expm_error": 9.592326932761353e-14,
  "mixed_final_state": [
    5.040881516005336e-07,
    1.3336380651995801e-12,
    2.0949107996026982e-07,
    -1.1540215096895971e-06,
    -2.134462431820821e-12,
    -1.3781155924865749e-07
  ],
  "lqr_backend": "scipy.linalg.solve_continuous_are"
}
```

Open-loop divergence is truncated at |theta|=0.2 rad as a reporting cutoff.
Closed-loop stability and convergence are verified only for this ideal local linear model.
Peak torques are reported; hardware feasibility requires real motor ratings and real parameters.
Mass condition number depends on coordinate scaling/units; it is not a hardware quality measure.