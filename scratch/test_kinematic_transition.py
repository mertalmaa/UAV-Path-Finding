import numpy as np
import math

def smooth_junction_kinematic(s_arr, phi_1_deg, phi_2_deg, V=40.0, g=9.80665, max_roll_rate=15.0):
    """
    Generate a C2 continuous bank angle, curvature, heading, and position transition.
    """
    s0 = s_arr[0]
    s1 = s_arr[-1]
    L = s1 - s0
    u = (s_arr - s0) / L
    
    # Smooth S-curve (Quintic transition from 0 to 1 with zero 1st and 2nd derivatives at ends)
    # S(u) = 10 u^3 - 15 u^4 + 6 u^5
    # S'(u) = 30 u^2 - 60 u^3 + 30 u^4
    S = 10.0 * (u**3) - 15.0 * (u**4) + 6.0 * (u**5)
    dS_du = 30.0 * (u**2) - 60.0 * (u**3) + 30.0 * (u**4)
    
    delta_phi = phi_2_deg - phi_1_deg
    phi_deg = phi_1_deg + delta_phi * S
    
    # Time derivative of bank angle
    # dphi/dt = (dphi/du) * (du/ds) * (ds/dt) = (delta_phi * dS_du) * (1/L) * V
    dphi_dt = delta_phi * dS_du * (V / L)
    
    # Max roll rate occurs at u = 0.5 where dS_du = 30(0.25) - 60(0.125) + 30(0.0625) = 7.5 - 7.5 + 1.875 = 1.875
    # So peak roll rate is 1.875 * |delta_phi| * V / L
    return phi_deg, dphi_dt

# Test for delta_phi = 50 deg (-25 to +25)
# To achieve peak roll rate <= 15 deg/s:
# L_min = 1.875 * 50 * 40 / 15 = 250 m.
# If L = 100 m, peak roll rate is: 1.875 * 50 * 40 / 100 = 37.5 deg/s.
# If L = 150 m, peak roll rate is: 1.875 * 50 * 40 / 150 = 25.0 deg/s.
for L in [60.0, 100.0, 150.0, 200.0, 250.0]:
    s = np.linspace(0, L, 100)
    phi, dphi_dt = smooth_junction_kinematic(s, -25.0, 25.0, V=40.0)
    print(f"L = {L:5.1f}m -> Peak Roll Rate = {np.max(np.abs(dphi_dt)):5.1f} deg/s, Max Bank = {np.max(np.abs(phi)):5.1f} deg")

print("\nFor 25 deg transition (0 to 25):")
for L in [40.0, 60.0, 80.0, 100.0, 125.0]:
    s = np.linspace(0, L, 100)
    phi, dphi_dt = smooth_junction_kinematic(s, 0.0, 25.0, V=40.0)
    print(f"L = {L:5.1f}m -> Peak Roll Rate = {np.max(np.abs(dphi_dt)):5.1f} deg/s, Max Bank = {np.max(np.abs(phi)):5.1f} deg")
