import numpy as np
import math
import matplotlib.pyplot as plt

def solve_spline_transition(P_A, theta_A_deg, kappa_A,
                            P_B, theta_B_deg, kappa_B,
                            L_orig, n_samples=50):
    """Solve Quintic Hermite spline for local transition."""
    th_A = math.radians(theta_A_deg)
    th_B = math.radians(theta_B_deg)
    
    T_A = np.array([math.sin(th_A), math.cos(th_A)])
    N_A = np.array([math.cos(th_A), -math.sin(th_A)])
    
    T_B = np.array([math.sin(th_B), math.cos(th_B)])
    N_B = np.array([math.cos(th_B), -math.sin(th_B)])
    
    # Scale parameter
    # For a gentle transition, sigma ~ L_orig or slightly adjusted by chord
    chord = np.linalg.norm(P_B - P_A)
    sigma = max(chord, L_orig * 0.95)
    
    V_A = sigma * T_A
    A_A = (sigma ** 2) * kappa_A * N_A
    
    V_B = sigma * T_B
    A_B = (sigma ** 2) * kappa_B * N_B
    
    u = np.linspace(0, 1, n_samples)
    u2 = u * u
    u3 = u2 * u
    u4 = u3 * u
    u5 = u4 * u
    
    h0 = 1.0 - 10.0*u3 + 15.0*u4 - 6.0*u5
    h1 = u - 6.0*u3 + 8.0*u4 - 3.0*u5
    h2 = 0.5*u2 - 1.5*u3 + 1.5*u4 - 0.5*u5
    h3 = 0.5*u3 - u4 + 0.5*u5
    h4 = -4.0*u3 + 7.0*u4 - 3.0*u5
    h5 = 10.0*u3 - 15.0*u4 + 6.0*u5
    
    pos = (np.outer(h0, P_A) + np.outer(h1, V_A) + np.outer(h2, A_A) +
           np.outer(h3, A_B) + np.outer(h4, V_B) + np.outer(h5, P_B))
           
    dh0 = -30.0*u2 + 60.0*u3 - 30.0*u4
    dh1 = 1.0 - 18.0*u2 + 32.0*u3 - 15.0*u4
    dh2 = u - 4.5*u2 + 6.0*u3 - 2.5*u4
    dh3 = 1.5*u2 - 4.0*u3 + 2.5*u4
    dh4 = -12.0*u2 + 28.0*u3 - 15.0*u4
    dh5 = 30.0*u2 - 60.0*u3 + 30.0*u4
    vel = (np.outer(dh0, P_A) + np.outer(dh1, V_A) + np.outer(dh2, A_A) +
           np.outer(dh3, A_B) + np.outer(dh4, V_B) + np.outer(dh5, P_B))
           
    d2h0 = -60.0*u + 180.0*u2 - 120.0*u3
    d2h1 = -36.0*u + 96.0*u2 - 60.0*u3
    d2h2 = 1.0 - 9.0*u + 18.0*u2 - 10.0*u3
    d2h3 = 3.0*u - 12.0*u2 + 10.0*u3
    d2h4 = -24.0*u + 84.0*u2 - 60.0*u3
    d2h5 = 60.0*u - 180.0*u2 + 120.0*u3
    acc = (np.outer(d2h0, P_A) + np.outer(d2h1, V_A) + np.outer(d2h2, A_A) +
           np.outer(d2h3, A_B) + np.outer(d2h4, V_B) + np.outer(d2h5, P_B))
           
    xp = vel[:, 0]
    yp = vel[:, 1]
    xpp = acc[:, 0]
    ypp = acc[:, 1]
    speed_sq = xp**2 + yp**2
    # Navigation convention curvature: kappa_nav = (yp * xpp - xp * ypp) / (speed_sq ^ 1.5)
    curv = (yp * xpp - xp * ypp) / (speed_sq ** 1.5)
    
    # Heading along curve in navigation degrees
    headings_deg = np.degrees(np.arctan2(xp, yp)) % 360.0
    
    return pos, vel, acc, curv, headings_deg

# Let's test a straight -> right turn junction:
# Straight: along North (heading 0), length 60m
# Right turn: 15 deg right turn, R=350m, length 91.6m
R = 349.9
kappa_R = 1.0 / R
# Junction J is at (0, 60)
# Before J: 30m before J: P_A = (0, 30), theta=0, kappa=0
# After J: 30m into turn:
# Center of right turn circle is at (R, 60) = (349.9, 60)
# angle at 30m arc: alpha = 30 / R = 0.0857 rad = 4.91 deg
alpha = 30.0 / R
P_B = np.array([349.9 - 349.9 * math.cos(alpha), 60.0 + 349.9 * math.sin(alpha)])
theta_B = math.degrees(alpha)
pos, vel, acc, curv, headings = solve_spline_transition(
    np.array([0.0, 30.0]), 0.0, 0.0,
    P_B, theta_B, kappa_R,
    L_orig=60.0, n_samples=50
)

V = 40.0
g = 9.80665
bank = np.degrees(np.arctan((V**2) * curv / g))
ds = np.sqrt(vel[:, 0]**2 + vel[:, 1]**2) / 50.0
dt = ds / V
dphi = np.gradient(bank) / dt

print(f"Straight -> Right Turn:")
print(f"  P_A: (0, 30), P_B: ({P_B[0]:.2f}, {P_B[1]:.2f})")
print(f"  Curv start: {curv[0]:.6f} (exp: 0.0), end: {curv[-1]:.6f} (exp: {kappa_R:.6f})")
print(f"  Bank start: {bank[0]:.2f} deg, end: {bank[-1]:.2f} deg")
print(f"  Max |dphi/dt|: {np.max(np.abs(dphi)):.2f} deg/s")
