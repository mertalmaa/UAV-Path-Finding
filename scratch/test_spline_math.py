import numpy as np
import math

def quintic_hermite(p0, v0, a0, p1, v1, a1, u):
    """Evaluate 2D quintic Hermite spline at parameter u in [0, 1]."""
    u = np.asarray(u)
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
    
    pos = (np.outer(h0, p0) + np.outer(h1, v0) + np.outer(h2, a0) +
           np.outer(h3, a1) + np.outer(h4, v1) + np.outer(h5, p1))
    
    # Derivatives for velocity
    dh0 = -30.0*u2 + 60.0*u3 - 30.0*u4
    dh1 = 1.0 - 18.0*u2 + 32.0*u3 - 15.0*u4
    dh2 = u - 4.5*u2 + 6.0*u3 - 2.5*u4
    dh3 = 1.5*u2 - 4.0*u3 + 2.5*u4
    dh4 = -12.0*u2 + 28.0*u3 - 15.0*u4
    dh5 = 30.0*u2 - 60.0*u3 + 30.0*u4
    vel = (np.outer(dh0, p0) + np.outer(dh1, v0) + np.outer(dh2, a0) +
           np.outer(dh3, a1) + np.outer(dh4, v1) + np.outer(dh5, p1))
           
    # Second derivative for acceleration
    d2h0 = -60.0*u + 180.0*u2 - 120.0*u3
    d2h1 = -36.0*u + 96.0*u2 - 60.0*u3
    d2h2 = 1.0 - 9.0*u + 18.0*u2 - 10.0*u3
    d2h3 = 3.0*u - 12.0*u2 + 10.0*u3
    d2h4 = -24.0*u + 84.0*u2 - 60.0*u3
    d2h5 = 60.0*u - 180.0*u2 + 120.0*u3
    acc = (np.outer(d2h0, p0) + np.outer(d2h1, v0) + np.outer(d2h2, a0) +
           np.outer(d2h3, a1) + np.outer(d2h4, v1) + np.outer(d2h5, p1))
           
    return pos, vel, acc

# Test an S-turn transition: from Left turn (R=350, phi=-25 deg) to Right turn (R=350, phi=+25 deg)
R = 350.0
kappa_left = -1.0 / R
kappa_right = +1.0 / R
V = 40.0
g = 9.80665

# Tangent vector from heading deg (North=0, East=90: T = [sin th, cos th])
# Normal vector for clockwise turn: N = [cos th, -sin th]
th0 = math.radians(0.0)
th1 = math.radians(10.0)

L_trans = 100.0
P0 = np.array([0.0, 0.0])
# P1 roughly L_trans ahead along average heading
P1 = np.array([L_trans * math.sin(th1*0.5), L_trans * math.cos(th1*0.5)])

T0 = np.array([math.sin(th0), math.cos(th0)])
N0 = np.array([math.cos(th0), -math.sin(th0)])
V0 = L_trans * T0
A0 = (L_trans ** 2) * kappa_left * N0

T1 = np.array([math.sin(th1), math.cos(th1)])
N1 = np.array([math.cos(th1), -math.sin(th1)])
V1 = L_trans * T1
A1 = (L_trans ** 2) * kappa_right * N1

u = np.linspace(0, 1, 100)
pos, vel, acc = quintic_hermite(P0, V0, A0, P1, V1, A1, u)

# Calculate curvature along spline: kappa = (x' y'' - y' x') / (x'^2 + y'^2)^1.5
xp = vel[:, 0]
yp = vel[:, 1]
xpp = acc[:, 0]
ypp = acc[:, 1]
speed_sq = xp**2 + yp**2
curv = (xp * ypp - yp * xpp) / (speed_sq ** 1.5)
# Note: with North=0, East=90 convention, kappa sign:
# Right turn: xp > 0, heading increasing clockwise.
# Let's check curv[0] and curv[-1]:
print(f"curv[0]: {curv[0]:.6f}, expected kappa_left: {kappa_left:.6f}")
print(f"curv[-1]: {curv[-1]:.6f}, expected kappa_right: {kappa_right:.6f}")

bank = np.degrees(np.arctan((V**2) * curv / g))
print(f"bank[0]: {bank[0]:.2f} deg, bank[-1]: {bank[-1]:.2f} deg")

# Time derivative of bank angle
dt = np.sqrt(speed_sq) / V / len(u)
dphi_dt = np.gradient(bank) / (np.gradient(u) * (L_trans / V))
print(f"max |dphi/dt|: {np.max(np.abs(dphi_dt)):.2f} deg/s")
