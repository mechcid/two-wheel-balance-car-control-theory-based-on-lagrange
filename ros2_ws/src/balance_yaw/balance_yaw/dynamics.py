"""Exact constrained yaw/pitch/forward dynamics for symmetric axisymmetric wheels.
State [l, psi, theta, v, omega, p]. l is accumulated forward travel, not world x.
Legacy physical.yaml["l"] is COM height; adapt it to params["com_height"].
"""
import numpy as np

def coefficients(p):
    if not np.isclose(p['m_L'],p['m_R']) or not np.isclose(p['I_Lyy'],p['I_Ryy']) or not np.isclose(p['I_Lzz'],p['I_Rzz']):
        raise ValueError('Yaw branch assumes symmetric wheels')
    r,d,lc=p['r'],p['d'],p['com_height']; m=p['m_B']
    return (m+p['m_L']+p['m_R']+(p['I_Lyy']+p['I_Ryy'])/r**2,
            m*lc,p['I_Byy']+m*lc*lc,
            p['I_Bzz']+p['I_Lzz']+p['I_Rzz']+d*d*(p['m_L']+p['m_R']+(p['I_Lyy']+p['I_Ryy'])/r**2)/4,
            p['I_Bxx']-p['I_Bzz']+m*lc*lc)

def matrices(theta,v,w,pitch_rate,params):
    a,c,e,J,D=coefficients(params); sn,cs=np.sin(theta),np.cos(theta)
    M=np.array([[a,0,c*cs],[0,J+D*sn*sn,0],[c*cs,0,e]])
    h=np.array([-c*sn*(pitch_rate**2+w**2), (2*D*sn*cs*pitch_rate+c*sn*v)*w,
                -D*sn*cs*w*w-c*params['g']*sn])
    r,d=params['r'],params['d']
    B=np.array([[1/r,1/r],[-d/(2*r),d/(2*r)],[-1,-1]])
    return M,h,B

def rhs(X,tau,params):
    M,h,B=matrices(X[2],X[3],X[4],X[5],params)
    return np.r_[X[3:],np.linalg.solve(M,B@tau-h)]

def linearize(params,v0,w0):
    a,c,e,J,D=coefficients(params)
    M,_,B0=matrices(0,v0,w0,0,params)
    # h_theta at straight upright relative equilibrium with constant forward/yaw speeds.
    h_theta=np.array([-c*w0*w0,c*v0*w0,-D*w0*w0-c*params['g']])
    A=np.zeros((6,6));A[:3,3:]=np.eye(3);A[3:,2]=-np.linalg.solve(M,h_theta)
    return A,np.vstack([np.zeros((3,2)),np.linalg.solve(M,B0)])

def profile(t):
    """Common smooth speed envelope and its integral: start2, ramp3, stop20..23."""
    if t<=2: return 0.,0.
    if t<5:
        z=(t-2)/3;return 3*z*z-2*z**3,3*(z**3-.5*z**4)
    if t<=20: return 1.,1.5+t-5
    if t<23:
        z=(t-20)/3;return 1-3*z*z+2*z**3,16.5+3*(z-z**3+.5*z**4)
    return 0.,18.

def reference(t,v0,w0,heading0):
    speed,distance=profile(t)
    return np.array([v0*distance,heading0+w0*distance,0.,v0*speed,w0*speed,0.])
