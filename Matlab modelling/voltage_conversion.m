%% Torque → Voltage Conversion for PendulumModel.mlx
%  =================================================
%
%  Paste this section at the end of PendulumModel.mlx (after A_val/B_val
%  are computed), OR run it standalone — it recomputes A_val/B_val first
%  using the same method as PendulumModel.mlx so you can verify.
%
%  The issue: PendulumModel.mlx derives A_ss, B_ss with torque input τ.
%  The motor equation is:  τ = (km/Rm)·Vm − (km²/Rm)·θ̇
%  Substituting into ẍ = A_τ·x + B_τ·τ gives:
%
%    ẍ = A_τ·x + B_τ·[(km/Rm)·Vm − (km²/Rm)·θ̇]
%      = [A_τ − (km²/Rm)·B_τ·[0 0 1 0]] · x + (km/Rm)·B_τ · Vm
%
%  In practice this is just 3 lines:
%    B_V = (km/Rm) · B_τ
%    A_V(3,3) -= (km²/Rm) · B_τ(3)
%    A_V(4,3) -= (km²/Rm) · B_τ(4)

clear; clc; close all;

%% ====================================================================
%  1. PARAMETERS — same as PendulumModel.mlx
%  ====================================================================
syms theta_1 theta_2 dtheta_1 dtheta_2 ddtheta_1 ddtheta_2 real
syms m_1 m_2 l_1 l_2 J_1 J_2 real
syms g real
syms b_1 b_2 tau_m real

lc1 = l_1/2;
lc2 = l_2/2;

q  = [theta_1; theta_2];
dq = [dtheta_1; dtheta_2];
ddq = [ddtheta_1; ddtheta_2];

% Physical measurements
l_0_1 = 84.5e-3;
l_1_2 = 120e-3;

l_encoder = 51.9e-3;
l_arm     = 85e-3;
l_p       = 129e-3;

r_encoder = 27.9e-3 / 2;
r_arm     = 6.3e-3 / 2;
r_p       = 9.5e-3 / 2;

m_r = 0.095;
m_p = 0.024;

% Inertias — using your group's formula from PendulumModel.mlx
J_r = 1/3 * m_r * (l_encoder + l_arm)^2 + 1/4 * m_r * (r_encoder + r_arm)^2;
J_p = 1/3 * m_p * l_p^2 + 1/4 * m_p * r_p^2;

b_r = 0.1;
b_p = 0.1;
gravity_val = 9.82;

% Motor
k_m = 0.0422;
R_m = 7.5;

fprintf('============================================================\n');
fprintf('  PARAMETERS\n');
fprintf('============================================================\n');
fprintf('  J_r = %.6e  (1/3·m·L² + 1/4·m·R², from PendulumModel.mlx)\n', J_r);
fprintf('  J_p = %.6e  (1/3·m·L² + 1/4·m·R², from PendulumModel.mlx)\n', J_p);
fprintf('  b_r = %.1f,  b_p = %.1f\n', b_r, b_p);
fprintf('  km  = %.4f,  Rm  = %.1f\n\n', k_m, R_m);

%% ====================================================================
%  2. DH → Jacobians → B-matrix → EOM  (reproducing PendulumModel.mlx)
%  ====================================================================
%  This section is a direct copy of PendulumModel.mlx so you can verify
%  that A_val/B_val match before applying the voltage conversion.

% DH parameters
alpha_1 = 90;
a_1 = 0;
d_1 = 0;

alpha_2 = 0;
a_2 = l_2;
d_2 = l_1;

% DH transforms
T_0_1 = [cos(theta_1) -sin(theta_1)*cosd(alpha_1)  sin(theta_1)*sind(alpha_1)  a_1*cos(theta_1);
         sin(theta_1)  cos(theta_1)*cosd(alpha_1) -cos(theta_1)*sind(alpha_1)  a_1*sin(theta_1);
                    0               sind(alpha_1)               cosd(alpha_1)               d_1;
                    0                           0                           0                 1];

T_1_2 = [cos(theta_2) -sin(theta_2)*cosd(alpha_2)  sin(theta_2)*sind(alpha_2)  a_2*cos(theta_2);
         sin(theta_2)  cos(theta_2)*cosd(alpha_2) -cos(theta_2)*sind(alpha_2)  a_2*sin(theta_2);
                    0               sind(alpha_2)               cosd(alpha_2)               d_2;
                    0                           0                           0                 1];

T_0_2 = T_0_1 * T_1_2;

p_0   = [0;0;0];
p_0_1 = T_0_1(1:3, 4);
p_0_2 = T_0_2(1:3, 4);

R_0_1 = T_0_1(1:3, 1:3);
R_0_2 = T_0_2(1:3, 1:3);

p_0_c1 = R_0_1*[0;0;lc1] + p_0_1;
p_0_c2 = R_0_2*[-lc2;0;0] + p_0_2;

z_0   = [0;0;1];
z_0_1 = T_0_1(1:3, 3);

% Jacobians
J_P_1 = [cross(z_0, (p_0_c1 - p_0)), [0;0;0]];
J_P_2 = [cross(z_0, (p_0_c2 - p_0)), cross(z_0_1, (p_0_c2 - p_0_1))];
J_O_1 = [z_0, [0;0;0]];
J_O_2 = [z_0, z_0_1];

% Inertia tensors (PendulumModel.mlx convention)
I_1_local = diag([J_1/2, J_1/2, J_1]);
I_2_local = diag([J_2, J_2/2, J_2/2]);

% Mass matrix (B-matrix in Lagrangian form)
B_mat = simplify(m_1*J_P_1'*J_P_1 + m_2*J_P_2'*J_P_2 ...
              + J_O_1'*R_0_1*I_1_local*R_0_1'*J_O_1 ...
              + J_O_2'*R_0_2*I_2_local*R_0_2'*J_O_2);

% Potential energy
g_direction = [0; 0; -g];
E_pot_0 = -(m_1*g_direction'*p_0_c1 + m_2*g_direction'*p_0_c2);

% Coriolis
n = 2;
C_mat = sym(zeros(n,n));
for i = 1:n
    for j = 1:n
        for k = 1:n
            c_ijk = 1/2*(diff(B_mat(i,j),q(k)) + diff(B_mat(i,k),q(j)) - diff(B_mat(j,k),q(i)));
            C_mat(i,j) = C_mat(i,j) + c_ijk*dq(k);
        end
    end
end

% Gravity vector
g_q = [diff(E_pot_0, q(1)); diff(E_pot_0, q(2))];

% Friction
f_disip = [-b_1*dtheta_1; -b_2*dtheta_2];

% EOM: B·ddq + C·dq + g = [τ; 0] + f_disip
% → ddq = B⁻¹(f_disip − C·dq − g_q)  (with τ=0 in f_x)
ddq_calc = simplify(B_mat \ (f_disip - C_mat*dq - g_q));

% State-space: f(x) + g(x)·τ
f_x = [dq; ddq_calc];
g_x = [0; 0; B_mat\[1;0]];

x = [q; dq];

% Linearise at equilibrium: θ₂ = π/2 (upright in YOUR convention)
eq_point = [theta_2, dtheta_1, dtheta_2, tau_m];
eq_vals  = [pi/2,    0,        0,        0];

A_ss = jacobian(f_x, x);
A_ss = subs(A_ss, eq_point, eq_vals);

B_ss = subs(g_x, eq_point, eq_vals);

% Substitute physical values → numeric
param_syms = [l_1,   l_2,   m_1, m_2, J_1, J_2, b_1, b_2, g];
param_vals = [l_0_1, l_1_2, m_r, m_p, J_r, J_p, b_r, b_p, gravity_val];

A_val = double(subs(A_ss, param_syms, param_vals));
B_val = double(subs(B_ss, param_syms, param_vals));

fprintf('============================================================\n');
fprintf('  STATE-SPACE WITH TORQUE INPUT (from PendulumModel.mlx)\n');
fprintf('============================================================\n');
fprintf('A_τ =\n'); disp(A_val);
fprintf('B_τ =\n'); disp(B_val);

poles_tau = eig(A_val);
fprintf('Open-loop poles (torque input):\n');
for i = 1:length(poles_tau)
    if isreal(poles_tau(i))
        fprintf('  s = %+.4f\n', poles_tau(i));
    else
        fprintf('  s = %+.4f %+.4fj\n', real(poles_tau(i)), imag(poles_tau(i)));
    end
end
fprintf('\n');

%% ====================================================================
%  3. TORQUE → VOLTAGE CONVERSION
%  ====================================================================
%
%  Motor model:  Vm = Rm·i + km·θ̇   and   τ = km·i
%  Therefore:    τ = (km/Rm)·Vm − (km²/Rm)·θ̇
%
%  The state equation with torque input is:
%    ẋ = A_τ·x + B_τ·τ
%
%  Substitute τ = (km/Rm)·Vm − (km²/Rm)·θ̇:
%    ẋ = A_τ·x + B_τ·[(km/Rm)·Vm − (km²/Rm)·θ̇]
%    ẋ = [A_τ − (km²/Rm)·B_τ·eθ̇'] · x  +  (km/Rm)·B_τ · Vm
%
%  where eθ̇' = [0, 0, 1, 0] selects the θ̇ component.
%
%  In practice, only column 3 of A changes (back-EMF damping on θ̇):

fprintf('============================================================\n');
fprintf('  TORQUE → VOLTAGE CONVERSION\n');
fprintf('============================================================\n');
fprintf('  τ = (km/Rm)·Vm − (km²/Rm)·θ̇\n');
fprintf('  km = %.4f,  Rm = %.1f\n', k_m, R_m);
fprintf('  km/Rm   = %.6f  (torque per volt)\n', k_m/R_m);
fprintf('  km²/Rm  = %.6f  (back-EMF damping coefficient)\n\n', k_m^2/R_m);

A_V = A_val;
B_V = (k_m / R_m) * B_val;

back_emf = k_m^2 / R_m;
A_V(3,3) = A_V(3,3) - back_emf * B_val(3);
A_V(4,3) = A_V(4,3) - back_emf * B_val(4);

fprintf('============================================================\n');
fprintf('  STATE-SPACE WITH VOLTAGE INPUT\n');
fprintf('============================================================\n');
fprintf('A_V =\n'); disp(A_V);
fprintf('B_V =\n'); disp(B_V);

poles_V = eig(A_V);
fprintf('Open-loop poles (voltage input):\n');
for i = 1:length(poles_V)
    if isreal(poles_V(i))
        fprintf('  s = %+.4f\n', poles_V(i));
    else
        fprintf('  s = %+.4f %+.4fj\n', real(poles_V(i)), imag(poles_V(i)));
    end
end
fprintf('\n');

%% ====================================================================
%  4. COMPARISON: BEFORE vs AFTER voltage conversion
%  ====================================================================
fprintf('============================================================\n');
fprintf('  COMPARISON: TORQUE vs VOLTAGE input\n');
fprintf('============================================================\n');
fprintf('  What changed:\n');
fprintf('    B_τ(3) = %.4f  →  B_V(3) = (km/Rm)·B_τ(3) = %.4f\n', B_val(3), B_V(3));
fprintf('    B_τ(4) = %.4f  →  B_V(4) = (km/Rm)·B_τ(4) = %.4f\n', B_val(4), B_V(4));
fprintf('    A(3,3) = %.4f  →  A_V(3,3) = %.4f  (back-EMF damping added)\n', A_val(3,3), A_V(3,3));
fprintf('    A(4,3) = %.4f  →  A_V(4,3) = %.4f  (back-EMF damping added)\n\n', A_val(4,3), A_V(4,3));

fprintf('  The B gains shrink by Rm/km = %.1f×\n', R_m/k_m);
fprintf('  This is why torque-input gains come out ~%.0f× too large for voltage control.\n\n', R_m/k_m);

%% ====================================================================
%  5. CONTROLLABILITY CHECK
%  ====================================================================
Co = ctrb(A_V, B_V);
r = rank(Co);
fprintf('  Controllability matrix rank = %d (need %d)\n', r, size(A_V,1));
if r == size(A_V,1)
    fprintf('  ✓ System is controllable with voltage input.\n\n');
else
    fprintf('  ✗ System is NOT controllable!\n\n');
end

%% ====================================================================
%  6. POLE PLACEMENT WITH VOLTAGE INPUT
%  ====================================================================
fprintf('============================================================\n');
fprintf('  POLE PLACEMENT (voltage input)\n');
fprintf('============================================================\n\n');

%  NOTE on sign convention:
%  Your model uses θ₂=π/2 as upright. Make sure the sign of the
%  gravity column in A matches: positive A(4,2) means the pendulum
%  is unstable (falls away from upright), which requires a positive
%  real open-loop pole. Check the poles above to confirm.

pole_sets = {
    [-3, -4, -10, -15],   'Very conservative';
    [-5, -6, -15, -25],   'Conservative';
    [-8, -10, -20, -30],  'Moderate';
};

for i = 1:size(pole_sets, 1)
    poles = pole_sets{i, 1};
    label = pole_sets{i, 2};

    fprintf('── %s: poles = [%s] ──\n', label, num2str(poles));
    try
        K = place(A_V, B_V, poles);
        fprintf('  K = [%.4f, %.4f, %.4f, %.4f]\n', K);

        % Verify closed-loop poles
        eig_cl = eig(A_V - B_V*K);
        fprintf('  CL poles: [');
        fprintf('%.2f ', eig_cl);
        fprintf(']\n');

        % Check voltage at 10° deflection
        x_test = [0; 10*pi/180; 0; 0];
        V_test = -K * x_test;
        fprintf('  Voltage at α=10°: %.2f V', V_test);
        if abs(V_test) > 6
            fprintf('  ⚠ exceeds 6V software limit');
        end
        fprintf('\n\n');
    catch ME
        fprintf('  FAILED: %s\n\n', ME.message);
    end
end

%% ====================================================================
%  7. TRANSFER FUNCTION (voltage input)
%  ====================================================================
fprintf('============================================================\n');
fprintf('  TRANSFER FUNCTION (voltage input, θ₂ output)\n');
fprintf('============================================================\n');

C_out = [0, 1, 0, 0];
D_out = 0;

sys_V = ss(A_V, B_V, C_out, D_out);
[num_V, den_V] = ss2tf(A_V, B_V, C_out, D_out);

fprintf('  G_V(s) = ');
% Print nicely
fprintf('(');
for j = 1:length(num_V)
    if abs(num_V(j)) > 1e-10
        if j > 1, fprintf(' + '); end
        fprintf('%.4g·s^%d', num_V(j), length(num_V)-j);
    end
end
fprintf(') / (');
for j = 1:length(den_V)
    if j > 1, fprintf(' + '); end
    fprintf('%.4g·s^%d', den_V(j), length(den_V)-j);
end
fprintf(')\n\n');

fprintf('  Compare with torque-input TF from PendulumModel.mlx —\n');
fprintf('  the voltage version has the back-EMF pole baked in.\n');
