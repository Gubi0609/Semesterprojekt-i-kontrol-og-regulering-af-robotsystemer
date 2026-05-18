%% Full Euler-Lagrange Derivation — Direct Geometry → Lagrangian → EOM → A,B
%  =========================================================================
%
%  This script derives the complete equations of motion for the Qube-Servo 3
%  rotary inverted pendulum using the Euler-Lagrange method:
%
%    1. Physical parameters
%    2. Geometry: CoM positions in base frame (DIRECT, no DH)
%    3. Velocities by differentiation
%    4. Kinetic energy T and potential energy V
%    5. Lagrangian L = T - V
%    6. Euler-Lagrange equations → nonlinear EOM
%    7. Symbolic linearisation at upright equilibrium
%    8. Torque → voltage conversion
%    9. Pole placement and comparison with pole_placement.m
%
%  Convention:
%    q1 = theta (motor/arm angle)
%    q2 = alpha (pendulum angle, 0 = upright)
%    Positive q2: pendulum tilts in direction that depends on q1
%    Jp is about the PIVOT (measured value)
%    Jr is about the MOTOR SHAFT (measured value)
%
%  WHY NOT DH?
%  -----------
%  The Qube pendulum rotates about the arm's longitudinal axis.
%  Standard DH convention cannot naturally represent this because DH
%  joint axes are always z_{i-1}, and the alpha twist is about x_i.
%  You cannot make z1 point along the arm with a single DH twist.
%  Using DH here produces a model where both joints rotate about the
%  vertical axis, putting everything in the horizontal plane (z=0),
%  which kills all gravity terms.
%
%  For a 2-DOF system, direct geometry is simpler and less error-prone.
%
%  WHY NOT USE Jr/Jp DIRECTLY AS BODY-FRAME INERTIAS IN THE JACOBIAN METHOD?
%  --------------------------------------------------------------------------
%  The Jacobian-based Lagrangian separates T into translational + rotational:
%    T = ½ dq' [m*Jv'*Jv + Jw'*R*I_com*R'*Jw] dq
%  This requires inertia about the CoM (I_com), because Jv already
%  captures CoM motion. Using Jr or Jp (about rotation axis) would
%  double-count the parallel-axis contribution.
%
%  Instead, we write T directly using the measured axis-inertias.

clear; clc; close all;
syms q1 q2 dq1 dq2 ddq1 ddq2 tau1 real
syms g_sym positive

%% ====================================================================
%  1. PHYSICAL PARAMETERS
%  ====================================================================
% Link 1 (encoder + arm assembly)
m1_val   = 0.095;        % total mass of link 1 [kg]
l_arm    = 0.085;        % arm length [m]
r_arm    = 0.00315;      % arm rod radius [m]

% Link 2 (pendulum)
mp_val   = 0.024;        % pendulum mass [kg]
Lp_val   = 0.129;        % pendulum total length [m]
r_p_val  = 0.00475;      % pendulum rod radius [m]

% Motor
Rm_val   = 7.5;          % terminal resistance [Ω]
km_val   = 0.042;        % back-EMF / torque constant

% Measured inertias (axis-frame, NOT CoM-frame)
Jr_val   = 6.2e-5;       % arm assembly about motor shaft [kg·m²]
Jp_val   = 1.26e-4;      % pendulum about pivot [kg·m²]

g_val    = 9.81;

% Derived
Lr_val  = l_arm;
hLp_val = Lp_val / 2;              % CoM distance from pivot
Jt_val  = Jr_val + mp_val*Lr_val^2; % total arm-side inertia

% Pendulum CoM inertia (for rotational KE separation)
Jp_com_val = Jp_val - mp_val * hLp_val^2;

fprintf('============================================================\n');
fprintf('  FULL EULER-LAGRANGE DERIVATION\n');
fprintf('  (direct geometry, no DH)\n');
fprintf('============================================================\n\n');
fprintf('  Jr (measured, about motor shaft) = %.4e\n', Jr_val);
fprintf('  Jp (measured, about pivot)       = %.4e\n', Jp_val);
fprintf('  Jp_com = Jp - mp·hLp²            = %.4e\n', Jp_com_val);
fprintf('  Jt = Jr + mp·Lr²                 = %.4e\n', Jt_val);
fprintf('  hLp = Lp/2                        = %.4f m\n\n', hLp_val);

%% ====================================================================
%  2. GEOMETRY — CoM POSITIONS IN BASE FRAME
%  ====================================================================
%
%  Base frame: z pointing up (gravity = -g·ẑ), origin at motor shaft.
%
%  Qube geometry:
%    - The arm extends horizontally from the motor, length Lr
%    - At the arm tip, the pendulum rotates about the ARM's axis
%    - When q2=0 (upright), the pendulum points straight up (+z)
%    - q2 rotation tilts the pendulum in a plane perpendicular to
%      the arm but containing the vertical
%
%  The pendulum CoM position is obtained by rotating the upright
%  vector [0,0,hLp] about the arm axis [cos(q1),sin(q1),0] by q2.
%
%  Using Rodrigues' formula: v_rot = v·cos(q2) + (k×v)·sin(q2)
%  with k = [cos(q1), sin(q1), 0] and v = [0, 0, hLp], k·v = 0:

fprintf('  Step 2: CoM positions (direct geometry)\n');
fprintf('  ────────────────────────────────────────\n');

syms Lr hLp mp_s Jr_s Jp_s Jp_com_s real

% Link 1 CoM: midpoint of arm (stays in horizontal plane)
p1_com = [Lr/2 * cos(q1);
          Lr/2 * sin(q1);
          0];

fprintf('  Link 1 CoM (base frame):\n');
disp(p1_com);

% Link 2 CoM: pivot + rotated hLp vector
%   pivot = [Lr·cos(q1), Lr·sin(q1), 0]
%   v = [0, 0, hLp]  (upright)
%   k = [cos(q1), sin(q1), 0]  (arm axis)
%   k × v = [sin(q1)·hLp, -cos(q1)·hLp, 0]
%
%   v_rot = v·cos(q2) + (k×v)·sin(q2)
%         = [hLp·sin(q2)·sin(q1), -hLp·sin(q2)·cos(q1), hLp·cos(q2)]

p2_com = [Lr*cos(q1) + hLp*sin(q2)*sin(q1);
          Lr*sin(q1) - hLp*sin(q2)*cos(q1);
          hLp*cos(q2)];

fprintf('  Link 2 CoM (base frame):\n');
disp(p2_com);
fprintf('  Note: z-component = hLp·cos(q2) → correct gravity dependence!\n\n');

%% ====================================================================
%  3. VELOCITIES
%  ====================================================================
fprintf('  Step 3: CoM velocities\n');
fprintf('  ──────────────────────\n');

q  = [q1; q2];
dq = [dq1; dq2];

% Differentiate positions w.r.t. q to get Jacobians, then v = J·dq
Jv1 = jacobian(p1_com, q);
Jv2 = jacobian(p2_com, q);

v1 = Jv1 * dq;
v2 = Jv2 * dq;

fprintf('  Jv1 (link 1, 3×2):\n'); disp(simplify(Jv1));
fprintf('  Jv2 (link 2, 3×2):\n'); disp(simplify(Jv2));

%% ====================================================================
%  4. KINETIC ENERGY
%  ====================================================================
%
%  T = T_arm + T_pend_trans + T_pend_rot
%
%  Arm (link 1):
%    The measured Jr already includes everything (encoder, hub, arm rod,
%    screws) about the motor axis. The arm only has one DOF (q1), so:
%      T_arm = ½·Jr·dq1²
%    This is EXACT — no need for translational + rotational separation.
%
%  Pendulum translation:
%    T_pend_trans = ½·mp·|v2|²
%
%  Pendulum rotation about its own CoM:
%    The pendulum angular velocity has two components:
%      ω = dq1·ẑ + dq2·k̂   where k̂ = [cos(q1), sin(q1), 0]
%
%    The pendulum is a thin rod along the direction:
%      rod = [sin(q2)·sin(q1), -sin(q2)·cos(q1), cos(q2)]
%
%    About the rod axis:   I_axial ≈ ½·mp·r² ≈ 0  (negligible)
%    About transverse axes: I_perp = Jp_com = Jp - mp·hLp²
%
%    Rotational KE = ½·I_axial·ω_axial² + ½·I_perp·ω_perp²
%                  ≈ ½·Jp_com·(|ω|² - ω_axial²)
%
%    Since ẑ·k̂ = 0: |ω|² = dq1² + dq2²
%    ω_axial = ω·rod = dq1·cos(q2) (since k̂·rod = 0)
%    ω_perp² = dq1² + dq2² - dq1²·cos²(q2) = dq1²·sin²(q2) + dq2²
%
%    T_pend_rot = ½·Jp_com·(dq1²·sin²(q2) + dq2²)

fprintf('  Step 4: Kinetic energy\n');
fprintf('  ──────────────────────\n');

% --- Arm kinetic energy (using measured Jr directly) ---
T_arm = sym(1)/2 * Jr_s * dq1^2;

% --- Pendulum translational KE ---
v2_sq = simplify(v1.' * v1);  % wait, this should be v2
v2_sq = simplify(v2.' * v2);
T_pend_trans = sym(1)/2 * mp_s * v2_sq;

% --- Pendulum rotational KE about CoM ---
% Angular velocity components
omega_sq = dq1^2 + dq2^2;               % |ω|²
omega_axial_sq = dq1^2 * cos(q2)^2;     % (ω · rod)²
omega_perp_sq = omega_sq - omega_axial_sq;  % dq1²·sin²(q2) + dq2²
T_pend_rot = sym(1)/2 * Jp_com_s * omega_perp_sq;

% --- Total KE ---
T_total = T_arm + T_pend_trans + T_pend_rot;
T_total = simplify(expand(T_total));

fprintf('  T_arm       = ½·Jr·θ̇²\n');
fprintf('  T_pend_trans = ½·mp·|v₂|²\n');
fprintf('  T_pend_rot  = ½·Jp_com·(θ̇²·sin²α + α̇²)\n');
fprintf('  T_total (symbolic):\n');
disp(T_total);

%% ====================================================================
%  5. EXTRACT MASS MATRIX M(q) from T = ½·dq'·M(q)·dq
%  ====================================================================
fprintf('  Step 5: Mass matrix M(q)\n');
fprintf('  ────────────────────────\n');

% M(q) can be extracted from T by noting T = ½·dq'·M·dq
% M_ij = d²T / (d(dqi)·d(dqj))
M11_sym = diff(diff(T_total, dq1), dq1);
M12_sym = diff(diff(T_total, dq1), dq2);
M22_sym = diff(diff(T_total, dq2), dq2);

M_sym = [M11_sym, M12_sym; M12_sym, M22_sym];
M_sym = simplify(M_sym);

fprintf('  M(q) =\n'); disp(M_sym);

%% ====================================================================
%  6. POTENTIAL ENERGY
%  ====================================================================
fprintf('  Step 6: Potential energy\n');
fprintf('  ────────────────────────\n');

% Only the pendulum has height-dependent PE (arm is horizontal)
V_sym = mp_s * g_sym * p2_com(3);
V_sym = simplify(V_sym);

fprintf('  V = mp·g·hLp·cos(q2)\n');
fprintf('  V = '); disp(V_sym);

%% ====================================================================
%  7. EULER-LAGRANGE EQUATIONS
%  ====================================================================
%
%  d/dt(∂L/∂dq_i) - ∂L/∂q_i = Q_i
%  where Q = [tau1; 0]
%
%  Equivalently: M(q)·ddq + h(q,dq) = Q
%  where h = C(q,dq)·dq + G(q)  (Coriolis/centrifugal + gravity)

fprintf('  Step 7: Euler-Lagrange equations\n');
fprintf('  ─────────────────────────────────\n');

L = T_total - V_sym;

% ∂L/∂dq
dL_ddq1 = diff(L, dq1);
dL_ddq2 = diff(L, dq2);

% d/dt(∂L/∂dq) — substitute dq→ddq using chain rule
% Time derivative: replace q→dq, dq→ddq in the expression
ddt_dL_ddq1 = diff(dL_ddq1, q1)*dq1 + diff(dL_ddq1, q2)*dq2 ...
            + diff(dL_ddq1, dq1)*ddq1 + diff(dL_ddq1, dq2)*ddq2;
ddt_dL_ddq2 = diff(dL_ddq2, q1)*dq1 + diff(dL_ddq2, q2)*dq2 ...
            + diff(dL_ddq2, dq1)*ddq1 + diff(dL_ddq2, dq2)*ddq2;

% ∂L/∂q
dL_dq1 = diff(L, q1);
dL_dq2 = diff(L, q2);

% EL equations: d/dt(∂L/∂dq_i) - ∂L/∂q_i = Q_i
EL1 = simplify(ddt_dL_ddq1 - dL_dq1 - tau1);   % = 0
EL2 = simplify(ddt_dL_ddq2 - dL_dq2);           % = 0

fprintf('  EL(1) [θ equation] = 0:\n');
disp(EL1);
fprintf('  EL(2) [α equation] = 0:\n');
disp(EL2);

% Extract gravity vector G(q) = -∂V/∂q  (appears as ∂L/∂q from V part)
G_vec = [-diff(V_sym, q1); -diff(V_sym, q2)];
G_vec = simplify(G_vec);

fprintf('  Gravity vector G(q) = -dV/dq:\n');
disp(G_vec);

% Coriolis + centrifugal: h = M·ddq terms subtracted from EL, minus G
% We can also get h(q,dq) from the Christoffel symbols, but let's
% just identify it from the EL equations

%% ====================================================================
%  8. LINEARISATION AT UPRIGHT EQUILIBRIUM
%  ====================================================================
%
%  Equilibrium: q1=0, q2=0 (upright), dq1=0, dq2=0
%
%  The EOM is:  M(q)·ddq + C(q,dq)·dq + G(q) = [tau; 0]
%
%  Linearising:
%    M(0)·ddq + dG/dq|₀ · q = [tau; 0]
%  (Coriolis terms vanish at zero velocity)
%
%  → ddq = M⁻¹·([tau;0] - dG/dq·q)
%
%  State x = [q1, q2, dq1, dq2]:
%    A_tau = [0, 0, I, 0; 0, 0, 0, I; 0, -M⁻¹·dG/dq(1,:), 0, 0; ...]
%    B_tau = [0; 0; M⁻¹·[1;0]]

fprintf('\n  Step 8: Linearisation at upright equilibrium\n');
fprintf('  ──────────────────────────────────────────────\n');

% Substitute numeric values
subs_list = {Lr, hLp, mp_s, Jr_s, Jp_s, Jp_com_s, g_sym};
vals_list = {Lr_val, hLp_val, mp_val, Jr_val, Jp_val, Jp_com_val, g_val};

% Mass matrix at equilibrium (q2 = 0)
M_eq = double(subs(M_sym, [subs_list{:}, q1, q2, dq1, dq2], ...
                           [vals_list{:}, 0, 0, 0, 0]));

fprintf('  M(q) at equilibrium:\n');
disp(M_eq);
fprintf('  M(1,1) = %.4e   (should be Jt = Jr + mp·Lr²)\n', M_eq(1,1));
fprintf('  M(1,2) = %.4e   (should be -mp·hLp·Lr)\n', M_eq(1,2));
fprintf('  M(2,2) = %.4e   (should be Jp)\n', M_eq(2,2));

% Verify against expected values
M11_expected = Jt_val;
M12_expected = -mp_val * hLp_val * Lr_val;
M22_expected = Jp_val;
fprintf('\n  Expected (from shortcut):\n');
fprintf('    M11 = Jt        = %.4e\n', M11_expected);
fprintf('    M12 = -mp·hLp·Lr = %.4e\n', M12_expected);
fprintf('    M22 = Jp        = %.4e\n', M22_expected);

err_M11 = abs(M_eq(1,1) - M11_expected);
err_M12 = abs(M_eq(1,2) - M12_expected);
err_M22 = abs(M_eq(2,2) - M22_expected);
fprintf('\n  Errors: |ΔM11|=%.2e  |ΔM12|=%.2e  |ΔM22|=%.2e\n\n', ...
        err_M11, err_M12, err_M22);

% Gravity stiffness: dG/dq at equilibrium
dGdq = double(subs(jacobian(G_vec, q), [subs_list{:}, q1, q2], ...
                                         [vals_list{:}, 0, 0]));

fprintf('  Gravity stiffness dG/dq at equilibrium:\n');
disp(dGdq);
fprintf('  dG/dq(2,2) = %.4f  (should be -mp·g·hLp = %.4f)\n', ...
        dGdq(2,2), -mp_val*g_val*hLp_val);

% Build state-space matrices (torque input)
Minv = inv(M_eq);
A_tau = [zeros(2), eye(2);
         -Minv * dGdq, zeros(2)];
B_tau = [0; 0; Minv * [1; 0]];

fprintf('\n============================================================\n');
fprintf('  STATE-SPACE WITH TORQUE INPUT (from full E-L derivation)\n');
fprintf('============================================================\n');
fprintf('A_τ =\n');
disp(A_tau);
fprintf('B_τ = ');
fprintf('%.4f  ', B_tau);
fprintf('\n\n');

%% ====================================================================
%  9. TORQUE → VOLTAGE CONVERSION
%  ====================================================================
%
%  Motor: Vm = Rm/km · τ + km · θ̇
%  So: τ = km/Rm · Vm - km²/Rm · θ̇
%
%  Substituting into state equation:
%    B_V = (km/Rm) · B_τ
%    A_V = A_τ  but add back-EMF damping:
%      A_V(3,3) -= km²/Rm · B_τ(3)
%      A_V(4,3) -= km²/Rm · B_τ(4)

fprintf('  Step 9: Torque → voltage conversion\n');
fprintf('  ──────────────────────────────────────\n');

A_V = A_tau;
B_V = (km_val / Rm_val) * B_tau;
back_emf = km_val^2 / Rm_val;
A_V(3,3) = A_V(3,3) - back_emf * B_tau(3);
A_V(4,3) = A_V(4,3) - back_emf * B_tau(4);

fprintf('\n============================================================\n');
fprintf('  STATE-SPACE WITH VOLTAGE INPUT (from full E-L derivation)\n');
fprintf('============================================================\n');
fprintf('A_V =\n');
disp(A_V);
fprintf('B_V = ');
fprintf('%f  ', B_V);
fprintf('\n\n');

poles_ol = eig(A_V);
fprintf('Open-loop poles:\n');
for i = 1:length(poles_ol)
    fprintf('  s = %+.4f\n', poles_ol(i));
end

%% ====================================================================
%  10. COMPARISON WITH SHORTCUT (pole_placement.m)
%  ====================================================================
fprintf('\n============================================================\n');
fprintf('  COMPARISON: Full E-L vs Shortcut (pole_placement.m)\n');
fprintf('============================================================\n');

% Rebuild shortcut A, B for comparison
Rm  = Rm_val; km = km_val;
Jr  = Jr_val; Lr = Lr_val;
mp  = mp_val; Lp = Lp_val; Jp = Jp_val; g = g_val;
hLp_n = Lp/2;
Jt  = Jr + mp*Lr^2;
M11s = Jt;
M12s = -mp*hLp_n*Lr;
M22s = Jp;
det_Ms = M11s*M22s - M12s^2;

A_tau_s = [0, 0, 1, 0;
           0, 0, 0, 1;
           0, -M12s*mp*hLp_n*g / det_Ms, 0, 0;
           0,  M11s*mp*hLp_n*g / det_Ms, 0, 0];
B_tau_s = [0; 0; M22s/det_Ms; -M12s/det_Ms];

A_V_s = A_tau_s;
B_V_s = (km/Rm) * B_tau_s;
bemf  = km^2/Rm;
A_V_s(3,3) = A_V_s(3,3) - bemf * B_tau_s(3);
A_V_s(4,3) = A_V_s(4,3) - bemf * B_tau_s(4);

fprintf('\n  A_volt (full E-L):\n');   disp(A_V);
fprintf('  A_volt (shortcut):\n');     disp(A_V_s);
fprintf('  B_volt (full E-L):  '); fprintf('%f  ', B_V); fprintf('\n');
fprintf('  B_volt (shortcut):  '); fprintf('%f  ', B_V_s); fprintf('\n\n');

fprintf('  A difference (full - shortcut):\n');
disp(A_V - A_V_s);
fprintf('  B difference (full - shortcut): ');
fprintf('%.6e  ', B_V - B_V_s);
fprintf('\n\n');

% Check if they match
tol = 1e-6;
if max(abs(A_V(:) - A_V_s(:))) < tol && max(abs(B_V - B_V_s)) < tol
    fprintf('  ✓ MATCH: Full E-L and shortcut produce identical A, B matrices.\n\n');
else
    fprintf('  ✗ MISMATCH: See differences above.\n\n');
end

%% ====================================================================
%  11. POLE PLACEMENT
%  ====================================================================
fprintf('============================================================\n');
fprintf('  POLE PLACEMENT COMPARISON\n');
fprintf('============================================================\n\n');

pole_sets = {
    [-3, -4, -10, -15],   'Very conservative';
    [-5, -6, -15, -25],   'Conservative';
    [-8, -10, -20, -30],  'Moderate';
    [-10, -15, -25, -40], 'Aggressive';
};

for i = 1:size(pole_sets, 1)
    poles = pole_sets{i, 1};
    label = pole_sets{i, 2};

    fprintf('── %s: poles = [%s] ──\n', label, num2str(poles));

    % Full E-L
    try
        K_full = place(A_V, B_V, poles);
        fprintf('  Full E-L:  K = [%.4f, %.4f, %.4f, %.4f]\n', K_full);

        eig_cl = eig(A_V - B_V * K_full);
        fprintf('  CL poles:  [');
        fprintf('%.2f ', eig_cl);
        fprintf(']\n');
    catch ME
        fprintf('  Full E-L:  FAILED — %s\n', ME.message);
    end

    % Shortcut
    try
        K_short = place(A_V_s, B_V_s, poles);
        fprintf('  Shortcut:  K = [%.4f, %.4f, %.4f, %.4f]\n', K_short);

        eig_cl = eig(A_V_s - B_V_s * K_short);
        fprintf('  CL poles:  [');
        fprintf('%.2f ', eig_cl);
        fprintf(']\n');
    catch ME
        fprintf('  Shortcut:  FAILED — %s\n', ME.message);
    end

    % Compare
    if exist('K_full', 'var') && exist('K_short', 'var')
        fprintf('  Difference: [');
        fprintf('%.6f ', K_full - K_short);
        fprintf(']\n');
    end

    fprintf('\n');
    clear K_full K_short;
end

%% ====================================================================
%  12. PHYSICAL INTERPRETATION OF THE MASS MATRIX
%  ====================================================================
fprintf('============================================================\n');
fprintf('  PHYSICAL INTERPRETATION\n');
fprintf('============================================================\n\n');

fprintf('  At the upright equilibrium (q2=0), the linearised EOM is:\n');
fprintf('    [Jt,          -mp·hLp·Lr] [θ̈]   [0           ] [θ]   [τ]\n');
fprintf('    [-mp·hLp·Lr,  Jp        ] [α̈] + [-mp·g·hLp·α ] [α] = [0]\n\n');

fprintf('  M11 = Jt = Jr + mp·Lr²\n');
fprintf('    Jr: measured inertia of arm assembly about motor shaft\n');
fprintf('    mp·Lr²: pendulum mass at arm tip (parallel-axis to motor)\n');
fprintf('    → total inertia that θ̈ must overcome\n\n');

fprintf('  M12 = -mp·hLp·Lr  (coupling)\n');
fprintf('    When the arm accelerates (θ̈), it creates a torque on the\n');
fprintf('    pendulum through the lever arm hLp at distance Lr from motor.\n');
fprintf('    The minus sign reflects the geometry: arm acceleration in +θ\n');
fprintf('    tends to tilt the pendulum in -α (reaction torque).\n\n');

fprintf('  M22 = Jp  (pivot-frame)\n');
fprintf('    Total pendulum inertia about its pivot point.\n');
fprintf('    Using pivot-frame Jp directly (no +mp·hLp² needed because\n');
fprintf('    the measured Jp already includes the parallel-axis term).\n\n');

fprintf('  The fact that Jp_com = Jp - mp·hLp² = %.4e > 0 confirms\n', Jp_com_val);
fprintf('  physical consistency: the measured Jp is larger than the\n');
fprintf('  translational contribution mp·hLp² = %.4e alone.\n\n', mp_val*hLp_val^2);

fprintf('  ──────────────────────────────────────────────────\n');
fprintf('  WHY THE ORIGINAL DH-BASED DERIVATION FAILED:\n');
fprintf('  ──────────────────────────────────────────────────\n');
fprintf('  1. DH gave z_com = 0 for the pendulum → V = 0 → no gravity\n');
fprintf('     (both joints appeared to rotate about z, everything in x-y plane)\n');
fprintf('  2. Jr and Jp were used as body-frame CoM inertias, but the\n');
fprintf('     Jacobian method needs I_com. This double-counted parallel-axis\n');
fprintf('     terms, inflating M11 by %.1f%%.\n', ...
        100*(6.3284e-4 - M11_expected)/M11_expected);
fprintf('  3. Without gravity, the open-loop system had 4 poles at 0\n');
fprintf('     and was uncontrollable → place() failed.\n');
