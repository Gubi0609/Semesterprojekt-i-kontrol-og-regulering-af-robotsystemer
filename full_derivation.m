%% Full Euler-Lagrange Derivation — DH → Jacobians → Lagrangian → EOM → A,B
%  =========================================================================
%
%  This script follows the same derivation path as PendulumModel.mlx:
%    1. DH parameters for 2-link rotary inverted pendulum
%    2. Forward kinematics (homogeneous transforms)
%    3. Jacobians (geometric, for each link CoM)
%    4. Inertia tensors (solid cylinders, in body frame)
%    5. Kinetic and potential energy (Lagrangian)
%    6. Euler-Lagrange equations → nonlinear EOM
%    7. Symbolic linearisation at upright equilibrium
%    8. Torque → voltage conversion
%    9. Pole placement and comparison with pole_placement.m
%
%  Convention:
%    q1 = theta (motor/arm angle)
%    q2 = alpha (pendulum angle, 0 = upright)
%    Jp is about the PIVOT (measured value)
%
%  All steps shown explicitly so you can compare with PendulumModel.mlx.

clear; clc; close all;
syms q1 q2 dq1 dq2 ddq1 ddq2 tau1 real
syms g_sym positive

%% ====================================================================
%  1. PHYSICAL PARAMETERS
%  ====================================================================
%  Using your measured values from the parameter list.

% Link 1 (encoder + arm assembly)
m1_val      = 0.095;       % total mass of link 1 [kg]
l_enc_val   = 0.0519;      % encoder cylinder length [m]
r_enc_val   = 0.01395;     % encoder cylinder radius [m]
l_arm_val   = 0.085;       % arm length (Lr) [m]
r_arm_val   = 0.00315;     % arm rod radius [m]

% Link 2 (pendulum)
m2_val      = 0.024;       % pendulum mass [kg]
l_p_val     = 0.129;       % pendulum total length [m]
r_p_val     = 0.00475;     % pendulum rod radius [m]

% Motor
Rm_val      = 7.5;         % terminal resistance [Ω]
km_val      = 0.042;       % back-EMF / torque constant
Jr_val      = 6.2e-5;      % measured arm assembly inertia [kg·m²]
Jp_val      = 1.26e-4;     % measured pendulum inertia about pivot [kg·m²]

g_val       = 9.81;

% Derived
Lr_val = l_arm_val;
hLp_val = l_p_val / 2;

fprintf('============================================================\n');
fprintf('  FULL EULER-LAGRANGE DERIVATION\n');
fprintf('  (same method as PendulumModel.mlx)\n');
fprintf('============================================================\n\n');

%% ====================================================================
%  2. DH PARAMETERS & FORWARD KINEMATICS
%  ====================================================================
%
%  The Qube rotary inverted pendulum is a 2-DOF system:
%
%  Frame 0 (base):  Motor shaft, z-axis pointing up
%  Joint 1:         Rotation about z0 by q1 (motor angle)
%  Frame 1:         At motor shaft, after q1 rotation
%                   Link 1 extends along x1 to the pivot
%  Joint 2:         Rotation about x1 by q2 (pendulum angle)
%  Frame 2:         At pivot point, pendulum hangs along z2
%
%  DH table (modified DH convention):
%    Link | a_i   | alpha_i | d_i  | theta_i
%    -----|-------|---------|------|--------
%      1  |   0   |    0    |  0   |   q1
%      2  |  Lr   |  pi/2   |  0   |   q2
%
%  Note: alpha convention means joint 2 rotates perpendicular to
%  joint 1, which is correct for the rotary inverted pendulum.

fprintf('  Step 2: DH parameters & forward kinematics\n');
fprintf('  ─────────────────────────────────────────────\n');

syms Lr Lp m1 m2_s Jr_s Jp_s real

% Homogeneous transform for DH parameters
% Using standard DH: T = Rz(theta) * Tz(d) * Tx(a) * Rx(alpha)
dh_transform = @(theta, d, a, alpha) ...
    [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
     sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
     0,           sin(alpha),             cos(alpha),            d;
     0,           0,                      0,                     1];

% Transform from base to frame 1 (after motor rotation)
T01 = dh_transform(q1, 0, 0, 0);

% Transform from frame 1 to frame 2 (at pendulum pivot)
T12 = dh_transform(q2, 0, Lr, sym(pi)/2);

% Transform from base to frame 2
T02 = simplify(T01 * T12);

fprintf('  T01 (base → arm):\n');
disp(T01);
fprintf('  T02 (base → pendulum pivot):\n');
disp(simplify(T02));

%% ====================================================================
%  3. POSITIONS OF CENTERS OF MASS
%  ====================================================================
%
%  Link 1 CoM: halfway along the arm (along x1 direction)
%    In frame 1: [Lr/2, 0, 0]
%    In frame 0: apply T01
%
%  Link 2 CoM: at distance Lp/2 from pivot, along pendulum
%    The pendulum hangs along the -z2 direction when q2=0 (upright)
%    Actually, convention: q2=0 means upright, pendulum along +z2
%    In frame 2: [0, 0, Lp/2]
%    In frame 0: apply T02

fprintf('  Step 3: Center of mass positions\n');
fprintf('  ─────────────────────────────────\n');

% CoM of link 1 in base frame
p1_com_local = [Lr/2; 0; 0; 1];
p1_com = T01 * p1_com_local;
p1_com = simplify(p1_com(1:3));

fprintf('  Link 1 CoM (base frame):\n');
disp(p1_com);

% CoM of link 2 in base frame
% When q2 = 0 (upright), pendulum points up (+z in frame 2)
% Lp/2 along pendulum from pivot
p2_com_local = [0; 0; hLp_val; 1];  % using numeric hLp for clarity
% Actually let's keep it symbolic
syms hLp real
p2_com_local = [0; 0; hLp; 1];
p2_com = T02 * p2_com_local;
p2_com = simplify(p2_com(1:3));

fprintf('  Link 2 CoM (base frame):\n');
disp(p2_com);

%% ====================================================================
%  4. JACOBIANS (for each link CoM)
%  ====================================================================
%
%  The Jacobian maps joint velocities [dq1; dq2] to Cartesian
%  velocities of each CoM:  v_com = J_v * [dq1; dq2]
%
%  For the Lagrangian we need:
%    - Linear velocity Jacobian: J_v = d(p_com)/d(q)
%    - Angular velocity Jacobian: J_w (from joint axes)

fprintf('  Step 4: Jacobians\n');
fprintf('  ─────────────────\n');

q = [q1; q2];

% Linear velocity Jacobians (3x2)
Jv1 = jacobian(p1_com, q);
Jv1 = simplify(Jv1);

Jv2 = jacobian(p2_com, q);
Jv2 = simplify(Jv2);

fprintf('  Jv1 (link 1 linear velocity Jacobian):\n');
disp(Jv1);
fprintf('  Jv2 (link 2 linear velocity Jacobian):\n');
disp(Jv2);

% Angular velocity Jacobians (3x2)
% Joint 1 rotates about z0 = [0; 0; 1]
% Joint 2 rotates about x1 (which is the x-axis of frame 1)
% In base frame, x1 = R01 * [1; 0; 0]
R01 = T01(1:3, 1:3);
z0 = [0; 0; 1];
x1 = simplify(R01 * [1; 0; 0]);

% For link 1: only joint 1 contributes
Jw1 = [z0, [0;0;0]];

% For link 2: both joints contribute
Jw2 = [z0, x1];

fprintf('  Jw1 (link 1 angular velocity Jacobian):\n');
disp(Jw1);
fprintf('  Jw2 (link 2 angular velocity Jacobian):\n');
disp(simplify(Jw2));

%% ====================================================================
%  5. INERTIA TENSORS
%  ====================================================================
%
%  Each link is modelled as a solid cylinder.
%
%  For a solid cylinder of mass m, radius r, length L:
%    About symmetry axis:    I_axial = (1/2) m r²
%    About transverse axis:  I_trans = (1/12) m (3r² + L²)
%
%  Link 1: rotates about its own axis (z0), arm extends along x1
%    In body frame (x = along arm, z = rotation axis):
%    I1 = diag(I_trans_arm, I_trans_arm, I_axial_arm)
%    BUT: we use the measured Jr instead of computing from geometry,
%    since Jr includes the encoder, hub, screws, etc.
%
%  Link 2: pendulum, extends along its own z-axis
%    In body frame (z = along pendulum):
%    I2 = diag(I_trans_pend, I_trans_pend, I_axial_pend)
%    We use measured Jp (about pivot) for the dominant terms.

fprintf('  Step 5: Inertia tensors\n');
fprintf('  ───────────────────────\n');

% Link 1: We use measured Jr for the rotation axis.
% For the off-axis terms, compute from cylinder geometry.
I1_axial = Jr_val;  % measured, includes everything
% Transverse: approximate from arm rod
I1_trans = (1/12) * m1_val * (3*r_arm_val^2 + l_arm_val^2);

% In frame 1 (arm along x, rotation about z):
% The Jacobian Jw1 gives angular velocity in base frame.
% We need inertia in base frame. For link 1, only q1 matters
% and the rotation is about z, so:
I1_body = diag([I1_trans, I1_trans, I1_axial]);

fprintf('  I1 (link 1, body frame):\n');
fprintf('    I_axial (z) = %.4e  (measured Jr)\n', I1_axial);
fprintf('    I_trans     = %.4e  (computed from geometry)\n', I1_trans);

% Link 2: We use measured Jp (pivot-frame) directly.
% For pivot-frame, the moment about the pendulum axis (z2) and
% the transverse axes at the pivot:
%   I_zz = (1/2) m r²  (about symmetry axis, same in any frame)
%   I_xx = I_yy = Jp   (about pivot, measured)
I2_axial = 0.5 * m2_val * r_p_val^2;  % tiny, ~2.7e-7
I2_trans = Jp_val;  % measured, about pivot

% In pendulum body frame (z along pendulum):
I2_body = diag([I2_trans, I2_trans, I2_axial]);

fprintf('  I2 (link 2, body frame, PIVOT-FRAME convention):\n');
fprintf('    I_trans (x,y) = %.4e  (measured Jp about pivot)\n', I2_trans);
fprintf('    I_axial (z)   = %.4e  (½mr², negligible)\n\n', I2_axial);

%% ====================================================================
%  6. KINETIC ENERGY (via Jacobians)
%  ====================================================================
%
%  T = Σ [ ½ dq' Jv_i' m_i Jv_i dq  +  ½ dq' Jw_i' R_i I_i R_i' Jw_i dq ]
%
%  This gives T = ½ dq' M(q) dq, where M(q) is the mass/inertia matrix.

fprintf('  Step 6: Kinetic energy → mass matrix M(q)\n');
fprintf('  ──────────────────────────────────────────\n');

dq = [dq1; dq2];

% Rotation matrices for expressing body inertia in base frame
R01_mat = T01(1:3, 1:3);
R02_mat = T02(1:3, 1:3);

% Link 1: translational + rotational KE
M1_trans = m1_val * (Jv1.' * Jv1);
M1_rot   = Jw1.' * R01_mat * I1_body * R01_mat.' * Jw1;

% Link 2: translational + rotational KE
M2_trans = m2_val * (Jv2.' * Jv2);
M2_rot   = Jw2.' * R02_mat * I2_body * R02_mat.' * Jw2;

% Total mass matrix (2x2, symbolic)
M_sym = simplify(M1_trans + M1_rot + M2_trans + M2_rot);

fprintf('  M(q) = \n');
disp(M_sym);

%% ====================================================================
%  7. POTENTIAL ENERGY
%  ====================================================================
%
%  V = m1*g*z_com1 + m2*g*z_com2
%
%  Link 1 CoM is at a fixed height (in the horizontal plane) → no
%  contribution to potential energy (its z doesn't change with q).
%
%  Link 2 CoM z-coordinate depends on q2.

fprintf('  Step 7: Potential energy\n');
fprintf('  ────────────────────────\n');

% z-coordinates of CoMs
z1 = p1_com(3);
z2 = p2_com(3);

V_sym = m1_val * g_sym * z1 + m2_val * g_sym * z2;
V_sym = simplify(V_sym);

fprintf('  V = '); disp(V_sym);
fprintf('\n');

%% ====================================================================
%  8. EULER-LAGRANGE EQUATIONS
%  ====================================================================
%
%  L = T - V
%
%  d/dt(∂L/∂dq_i) - ∂L/∂q_i = Q_i
%
%  Where Q = [tau1; 0] (motor torque on joint 1, no external torque on joint 2)
%
%  This gives:  M(q) * ddq + C(q,dq) * dq + G(q) = Q
%
%  We compute this symbolically using the Christoffel symbols or
%  by direct differentiation.

fprintf('  Step 8: Euler-Lagrange equations\n');
fprintf('  ─────────────────────────────────\n');

% Lagrangian
T_kin = (1/2) * dq.' * M_sym * dq;
L_sym = T_kin - V_sym;

% Generalised forces
Q = [tau1; 0];

% E-L equations: d/dt(∂L/∂dq) - ∂L/∂q = Q
% ∂L/∂dq
dL_ddq = simplify(jacobian(L_sym, dq).');  % column vector (2x1)

% ∂L/∂q
dL_dq = simplify(jacobian(L_sym, q).');    % column vector (2x1)

% d/dt(∂L/∂dq) requires chain rule:
%   d/dt(f(q,dq)) = ∂f/∂q * dq + ∂f/∂dq * ddq
ddq_vec = [ddq1; ddq2];

% Time derivative of ∂L/∂dq
dt_dL_ddq = jacobian(dL_ddq, q) * dq + jacobian(dL_ddq, dq) * ddq_vec;
dt_dL_ddq = simplify(dt_dL_ddq);

% Full E-L equation: dt_dL_ddq - dL_dq = Q
EL = simplify(dt_dL_ddq - dL_dq - Q);

fprintf('  EL equations (= 0):\n');
fprintf('  EL(1) [theta]: '); disp(EL(1));
fprintf('  EL(2) [alpha]: '); disp(EL(2));

%% ====================================================================
%  9. EXTRACT M(q), C(q,dq), G(q)
%  ====================================================================
%
%  EL = M*ddq + (everything else) = 0
%  M*ddq = Q - C*dq - G
%
%  We already have M_sym. Extract C and G:

% Gravity vector: G = -∂V/∂q
G_sym = simplify(jacobian(V_sym, q).');

fprintf('  Gravity vector G(q):\n');
disp(G_sym);

% Coriolis/centripetal: extract from EL - M*ddq - G
% C*dq = (EL + Q - G) evaluated with ddq=0... actually let's just
% collect terms. The full EOM is:
%   M * ddq + h(q, dq) = Q
% where h = C*dq + G

% Let's verify by substituting ddq=0 and tau=0:
h_sym = simplify(subs(EL + Q, [ddq1, ddq2], [0, 0]));
fprintf('  h(q,dq) = C*dq + G:\n');
disp(h_sym);

%% ====================================================================
%  10. LINEARISE AT UPRIGHT EQUILIBRIUM
%  ====================================================================
%
%  Equilibrium: q1=0, q2=0, dq1=0, dq2=0, tau1=0
%  (q2=0 means upright in our convention)
%
%  Linearised EOM:  M0 * ddq + C0 * dq + G0_lin * q = B0 * u
%
%  State vector: x = [q1; q2; dq1; dq2]
%  A = [0, I; -M0\G0_lin, -M0\C0]  (after adding motor dynamics)

fprintf('  Step 10: Linearisation at upright equilibrium\n');
fprintf('  ──────────────────────────────────────────────\n');

eq_point = [q1, 0; q2, 0; dq1, 0; dq2, 0; tau1, 0];

% Mass matrix at equilibrium
M0 = double(subs(M_sym, [q1, q2, hLp, Lr, g_sym], ...
                         [0, 0, hLp_val, Lr_val, g_val]));

fprintf('  M(q) at equilibrium:\n');
disp(M0);
fprintf('  M(1,1) = %.4e   (Jt equivalent)\n', M0(1,1));
fprintf('  M(1,2) = %.4e   (-mp·hLp·Lr equivalent)\n', M0(1,2));
fprintf('  M(2,2) = %.4e   (Jp equivalent)\n', M0(2,2));
fprintf('\n');

% Gravity linearisation: G(q) ≈ G_lin * q  (G at equilibrium is zero
% for q2=0 upright if we define V correctly, but dG/dq is nonzero)
G0 = double(subs(G_sym, [q1, q2, hLp, Lr, g_sym], ...
                         [0, 0, hLp_val, Lr_val, g_val]));
fprintf('  G at equilibrium: '); disp(G0.');

% Jacobian of G w.r.t. q (gravity stiffness)
dG_dq = jacobian(G_sym, q);
G_lin = double(subs(dG_dq, [q1, q2, hLp, Lr, g_sym], ...
                            [0, 0, hLp_val, Lr_val, g_val]));
fprintf('  dG/dq (gravity stiffness) at equilibrium:\n');
disp(G_lin);

% Coriolis at equilibrium (all velocity terms vanish)
% But we need damping-like terms from Coriolis — at eq they're zero
% since dq=0. The linearised C contribution is zero.

% Input matrix
B0_tau = [1; 0];  % torque on joint 1 only

% ── Build state-space (torque input) ────────────────────────────
% x = [q1; q2; dq1; dq2]
% dx = [dq1; dq2; ddq1; ddq2]
% M0 * [ddq1; ddq2] = -G_lin * [q1; q2] + B0_tau * tau
% [ddq1; ddq2] = -M0\G_lin * [q1; q2] + M0\B0_tau * tau

A_tau = [zeros(2), eye(2);
         -M0 \ G_lin, zeros(2)];

B_tau = [zeros(2,1);
         M0 \ B0_tau];

fprintf('============================================================\n');
fprintf('  STATE-SPACE WITH TORQUE INPUT (from full E-L derivation)\n');
fprintf('============================================================\n');
fprintf('A_τ =\n'); disp(A_tau);
fprintf('B_τ = '); fprintf('%.4f  ', B_tau); fprintf('\n\n');

%% ====================================================================
%  11. TORQUE → VOLTAGE CONVERSION
%  ====================================================================
fprintf('  Step 11: Torque → voltage conversion\n');
fprintf('  ──────────────────────────────────────\n');

km = km_val;
Rm = Rm_val;

A_volt = A_tau;
A_volt(3,3) = A_tau(3,3) - (km^2 / Rm) * B_tau(3);
A_volt(4,3) = A_tau(4,3) - (km^2 / Rm) * B_tau(4);

B_volt = (km / Rm) * B_tau;

fprintf('============================================================\n');
fprintf('  STATE-SPACE WITH VOLTAGE INPUT (from full E-L derivation)\n');
fprintf('============================================================\n');
fprintf('A_V =\n'); disp(A_volt);
fprintf('B_V = '); fprintf('%.6f  ', B_volt); fprintf('\n\n');

% Open-loop poles
ol_poles = eig(A_volt);
fprintf('Open-loop poles:\n');
for i = 1:length(ol_poles)
    if abs(imag(ol_poles(i))) < 1e-6
        fprintf('  s = %+.4f\n', real(ol_poles(i)));
    else
        fprintf('  s = %+.4f %+.4fj\n', real(ol_poles(i)), imag(ol_poles(i)));
    end
end
fprintf('\n');

%% ====================================================================
%  12. COMPARE WITH pole_placement.m (shortcut method)
%  ====================================================================
fprintf('============================================================\n');
fprintf('  COMPARISON: Full E-L vs Shortcut (pole_placement.m)\n');
fprintf('============================================================\n\n');

% Shortcut method (what pole_placement.m does)
Jt_sc  = Jr_val + m2_val * Lr_val^2;
M11_sc = Jt_sc;
M12_sc = -m2_val * hLp_val * Lr_val;
M22_sc = Jp_val;
det_sc = M11_sc * M22_sc - M12_sc^2;

A_tau_sc = [0, 0, 1, 0;
            0, 0, 0, 1;
            0, -M12_sc*m2_val*hLp_val*g_val / det_sc,  0,  0;
            0,  M11_sc*m2_val*hLp_val*g_val / det_sc,  0,  0];

B_tau_sc = [0; 0; M22_sc/det_sc; -M12_sc/det_sc];

A_volt_sc = A_tau_sc;
A_volt_sc(3,3) = A_tau_sc(3,3) - (km^2/Rm) * B_tau_sc(3);
A_volt_sc(4,3) = A_tau_sc(4,3) - (km^2/Rm) * B_tau_sc(4);
B_volt_sc = (km/Rm) * B_tau_sc;

fprintf('  A_volt (full E-L):\n'); disp(A_volt);
fprintf('  A_volt (shortcut):\n'); disp(A_volt_sc);

fprintf('  B_volt (full E-L):  '); fprintf('%.6f  ', B_volt); fprintf('\n');
fprintf('  B_volt (shortcut):  '); fprintf('%.6f  ', B_volt_sc); fprintf('\n\n');

% Element-wise difference
A_diff = A_volt - A_volt_sc;
B_diff = B_volt - B_volt_sc;
fprintf('  A difference (full - shortcut):\n'); disp(A_diff);
fprintf('  B difference (full - shortcut): '); fprintf('%.6e  ', B_diff); fprintf('\n\n');

% Identify what's different
fprintf('  Key mass matrix comparison:\n');
fprintf('    Full E-L:   M11=%.4e  M12=%.4e  M22=%.4e\n', M0(1,1), M0(1,2), M0(2,2));
fprintf('    Shortcut:   M11=%.4e  M12=%.4e  M22=%.4e\n', M11_sc, M12_sc, M22_sc);
fprintf('    Difference: M11=%+.4e  M12=%+.4e  M22=%+.4e\n', ...
        M0(1,1)-M11_sc, M0(1,2)-M12_sc, M0(2,2)-M22_sc);
fprintf('\n');
fprintf('  The difference in M11 comes from the link 1 inertia terms\n');
fprintf('  (encoder disc, arm rod transverse inertia) that the shortcut\n');
fprintf('  method lumps into Jr.\n\n');

%% ====================================================================
%  13. POLE PLACEMENT WITH BOTH MODELS
%  ====================================================================
fprintf('============================================================\n');
fprintf('  POLE PLACEMENT COMPARISON\n');
fprintf('============================================================\n\n');

poles_vc = [-3, -4, -10, -15];
poles_c  = [-5, -7, -20, -30];

for method = {'Full E-L', 'Shortcut'}
    if strcmp(method{1}, 'Full E-L')
        A_use = A_volt; B_use = B_volt;
    else
        A_use = A_volt_sc; B_use = B_volt_sc;
    end

    fprintf('── %s ──\n', method{1});
    for p_idx = 1:2
        if p_idx == 1
            poles = poles_vc; pname = 'Very conservative';
        else
            poles = poles_c; pname = 'Conservative';
        end

        K = place(A_use, B_use, poles);
        fprintf('  %s poles [%s]:\n', pname, num2str(poles));
        fprintf('    K = [%.4f, %.4f, %.4f, %.4f]\n', K);
        fprintf('         k_θ      k_α      k_θ̇      k_α̇\n');

        % Voltage at 5°
        x5 = [0; 5*pi/180; 0; 0];
        fprintf('    Vm at 5°: %.2f V\n\n', abs(-K*x5));
    end
end

fprintf('Done.\n');
