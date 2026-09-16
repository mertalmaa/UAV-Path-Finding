import numpy as np
import math

def solve_local_regularized_spline(W, w_smooth=100.0, w_jerk=10.0, max_dev=5.0):
    """
    Solve regularized local B-spline control points for a window of waypoints W (N x 2).
    Minimizes:
      sum || C_i - W_i ||^2 + w_smooth * sum || C_i+1 - 2*C_i + C_i-1 ||^2 + w_jerk * sum || C_i+2 - 3*C_i+1 + 3*C_i - C_i-1 ||^2
    Subject to:
      C_0 = W_0, C_1 = W_1 (clamped start position & tangent)
      C_N-2 = W_N-2, C_N-1 = W_N-1 (clamped end position & tangent)
    """
    N = len(W)
    if N <= 4:
        return W.copy()
        
    # Build linear system A * C = b
    # A is (N x N), symmetric positive definite
    A = np.eye(N)
    
    # Second difference operator D2 (N-2 x N): [1, -2, 1]
    D2 = np.zeros((N - 2, N))
    for i in range(N - 2):
        D2[i, i] = 1.0
        D2[i, i + 1] = -2.0
        D2[i, i + 2] = 1.0
    A += w_smooth * (D2.T @ D2)
    
    # Third difference operator D3 (N-3 x N): [-1, 3, -3, 1] (jerk / roll rate)
    if N >= 5 and w_jerk > 0:
        D3 = np.zeros((N - 3, N))
        for i in range(N - 3):
            D3[i, i] = -1.0
            D3[i, i + 1] = 3.0
            D3[i, i + 2] = -3.0
            D3[i, i + 3] = 1.0
        A += w_jerk * (D3.T @ D3)
        
    b_x = W[:, 0].copy()
    b_y = W[:, 1].copy()
    
    # Clamping boundary conditions: C_0, C_1, C_N-2, C_N-1
    clamp_indices = [0, 1, N - 2, N - 1]
    for idx in clamp_indices:
        A[idx, :] = 0.0
        A[idx, idx] = 1.0
        b_x[idx] = W[idx, 0]
        b_y[idx] = W[idx, 1]
        
    C_x = np.linalg.solve(A, b_x)
    C_y = np.linalg.solve(A, b_y)
    
    C = np.column_stack([C_x, C_y])
    
    # Clamp to max corridor deviation if requested
    dev = np.linalg.norm(C - W, axis=1)
    for i in range(N):
        if dev[i] > max_dev:
            C[i] = W[i] + (C[i] - W[i]) * (max_dev / dev[i])
            
    return C

# Test on a junction: Left turn to Right turn reversal
# Sample points along Left turn (R=350, phi=-25 deg) for 40m, then Right turn (R=350, phi=+25 deg) for 40m
R = 349.9
step = 1.0
s_left = np.arange(-40.0, 0.0, step)
s_right = np.arange(0.0, 41.0, step)

# Left turn: center at (-R, 0), start at (0, 0), moving North (heading 0)
th_left = s_left / R
x_left = -R + R * np.cos(th_left)
y_left = R * np.sin(th_left)

# Right turn: center at (R, 0), start at (0, 0), moving North (heading 0)
th_right = s_right / R
x_right = R - R * np.cos(th_right)
y_right = R * np.sin(th_right)

W_raw = np.vstack([np.column_stack([x_left, y_left]), np.column_stack([x_right, y_right])])
print(f"Waypoints: {len(W_raw)}")

C_smooth = solve_local_regularized_spline(W_raw, w_smooth=500.0, w_jerk=2000.0, max_dev=5.0)

# Evaluate curvature along W_raw vs C_smooth
for name, pts in [("Raw (Unsmoothed)", W_raw), ("Local Constrained B-Spline", C_smooth)]:
    dx = np.gradient(pts[:, 0], step)
    dy = np.gradient(pts[:, 1], step)
    d2x = np.gradient(dx, step)
    d2y = np.gradient(dy, step)
    speed = np.sqrt(dx**2 + dy**2)
    curv = (dy * d2x - dx * d2y) / (speed**3 + 1e-12)
    V = 40.0
    g = 9.80665
    bank = np.degrees(np.arctan((V**2) * curv / g))
    dt = step / V
    dphi = np.gradient(bank) / dt
    dev = np.linalg.norm(pts - W_raw, axis=1)
    print(f"\n{name}:")
    print(f"  Max Bank: {np.max(np.abs(bank)):.1f} deg")
    print(f"  Max Roll Rate: {np.max(np.abs(dphi)):.1f} deg/s")
    print(f"  Max Deviation: {np.max(dev):.2f} m")
