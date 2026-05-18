%% Euler-Lagrange Modelling of Rotary Inverted Pendulum
%  (Equivalent to PendulumModel.mlx + torque→voltage conversion)

clc; clear; close all;

syms theta_1 theta_2 dtheta_1 dtheta_2 ddtheta_1 ddtheta_2 real
syms m_1 m_2 l_1 l_2 J_1 J_2 real
%syms lc1 lc2 real
syms g real
syms b_1 b_2 tau_m real

syms detB

lc1 = l_1/2;
lc2 = l_2/2;

q = [theta_1; theta_2];
dq = [dtheta_1; dtheta_2];
ddq = [ddtheta_1; ddtheta_2];

%% Measurements from real life and data sheet
l_0_1 = 84.5*10^(-3);         % [m] Length from frame 0 to 1
l_1_2 = 120*10^(-3);          % [m] Length from frame 1 to 2

l_encoder = 51.9*10^(-3);     % [m] Length of encoder module
l_arm = 85*10^(-3);           % [m] Length of encoder arm (end to end)
l_p = 129*10^(-3);            % [m] Length of pendulum (end to end)

r_encoder = (27.9/2)*10^(-3); % [m] Radius of encoder module
r_arm = (6.3/2)*10^(-3);      % [m] Radius of encoder arm
r_p = (9.5/2)*10^(-3);        % [m] Radius of pendulum

m_r = 0.095;                  % [kg] Mass of encoder and arm
m_p = 0.024;                  % [kg] Mass of pendulum

% Calculated intertia
J_r = 1/3 * m_r * (l_encoder + l_arm)^2 + 1/4 * m_r * (r_encoder + r_arm)^2;
J_p = 1/3 * m_p * l_p^2 + 1/4 * m_p * r_p^2;

% Estimated friction
b_r = 0.1;
b_p = 0.1;

% Gravity
gravity = 9.82; % [m/s^2]

% Motor parameters
k_m = 0.0422; % [Nm/A] Motor constant
R_m = 7.5;    % [Ohm] Terminal resistance

%% DH Parameters and Forward Kinematics

% theta_1 *0* degrees is defined to be the motors middle-point
% theta_2 *0* degrees is defined to be the pendulum laying horizontal left
% seen from the encoders point of view. Thus *90* degrees is standing
% straight up.

% DH parameters
alpha_0 = 0;
a_0 = 0;
d_0 = 0;

alpha_1 = 90; % angle from z_i to z_i+1 measured about x_i
a_1 = 0; % Offset along i x-axis
d_1 = 0; % Offset along i z-axis

alpha_2 = 0; % degrees
a_2 = l_2;
d_2 = l_1; % Since we offset along the arm, this is the translation along z_2

% Transformation matrix for links from DH parameters (L5 robotkinematik)
T_0_1 = [cos(theta_1) -sin(theta_1)*cosd(alpha_1)  sin(theta_1)*sind(alpha_1)  a_1*cos(theta_1);
         sin(theta_1)  cos(theta_1)*cosd(alpha_1) -cos(theta_1)*sind(alpha_1)  a_1*sin(theta_1);
                    0               sind(alpha_1)               cosd(alpha_1)               d_1;
                    0                           0                           0                 1];

T_1_2 = [cos(theta_2) -sin(theta_2)*cosd(alpha_2)  sin(theta_2)*sind(alpha_2)  a_2*cos(theta_2);
         sin(theta_2)  cos(theta_2)*cosd(alpha_2) -cos(theta_2)*sind(alpha_2)  a_2*sin(theta_2);
                    0               sind(alpha_2)               cosd(alpha_2)               d_2;
                    0                           0                           0                 1];

T_0_2 = T_0_1*T_1_2;

% Position of origo in regards to base frame
p_0 = [0;0;0];
p_0_1 = T_0_1(1:3,4);
p_0_2 = T_0_2(1:3,4);
% Rotation matrices
R_0_1 = T_0_1(1:3, 1:3);
R_0_2 = T_0_2(1:3, 1:3);

% Position of link center in regards to base frame
p_0_c1 = R_0_1*[0;0;lc1]+p_0_1;
p_0_c2 = R_0_2*[-lc2; 0; 0]+p_0_2;

% Orientation of z-axis in regards to base frame
z_0 = [0;0;1];
z_0_1 = T_0_1(1:3, 3);
z_0_2 = T_0_2(1:3, 3);

%% Jacobians

% Jacobians for each joint (L12 modellering, and L10 robotkinematik)
J_P_1 = [cross(z_0, (p_0_c1 - p_0)) [0;0;0]];
J_P_2 = [cross(z_0, (p_0_c2 - p_0)) cross(z_0_1, (p_0_c2 - p_0_1))];

% To find the full expression of kinetic energy
J_O_1 = [z_0 [0;0;0]]; % No joint affects the z-axis orientation of frame 1.
J_O_2 = [z_0 z_0_1];   % Only joint 1 affects the z-axis of frame 2.

%% Inertia Tensors

% For each link, the axis of symmetry is either the z-axis (link 1)
% or the x-axis (link 2)
I_1_local = diag([J_1/2, J_1/2, J_1]);
I_2_local = diag([J_2, J_2/2, J_2/2]);

%% Mass Matrix (B-matrix)

B = simplify(m_1*J_P_1'*J_P_1 + m_2*J_P_2'*J_P_2 ...
           + J_O_1'*R_0_1*I_1_local*R_0_1'*J_O_1 ...
           + J_O_2'*R_0_2*I_2_local*R_0_2'*J_O_2);

E_kin_0 = simplify(1/2 * dq' * B * dq);

%% Potential Energy

g_direction = [0; 0; -g];
E_pot_0 = -(m_1*g_direction'*p_0_c1 + m_2*g_direction'*p_0_c2);

%% Coriolis Matrix

n = 2;
C = sym(zeros(n,n));
for i = 1:n
    for j = 1:n
        for k = 1:n
            c_ijk = 1/2 * (diff(B(i,j), q(k)) + diff(B(i,k), q(j)) - diff(B(j,k), q(i)));
            C(i,j) = C(i,j) + c_ijk * dq(k);
        end
    end
end

%% Gravity Vector and EOM

g_q = [diff(E_pot_0, q(1)); diff(E_pot_0, q(2))];

force_vec = [tau_m-b_1*dtheta_1; -b_2*dtheta_2];
f_disip = [-b_1*dtheta_1; -b_2*dtheta_2];

sys = B*ddq + C*dq + g_q == force_vec;
ddq_calculated = simplify(B\(f_disip - C * dq - g_q));

%% State-Space Formulation

x = [q; dq];
dx = [dq; ddq];

f_x = [dq; ddq_calculated];
g_x = [0; 0; B\[1;0]];

%% Linearisation at Upright Equilibrium (theta_2 = pi/2)

f_x_test = subs(f_x, [theta_2, dtheta_1, dtheta_2, tau_m], [pi/2, 0, 0, 0]);

A_ss = jacobian(f_x, x);
A_ss = subs(A_ss, [theta_2, dtheta_1, dtheta_2, tau_m], [pi/2, 0, 0, 0]);

B_ss = subs(g_x, [theta_2, dtheta_1, dtheta_2, tau_m], [pi/2, 0, 0, 0]);

C_ss = [0 1 0 0];
D_ss = 0;

%% Numeric A and B (TORQUE input)

param_syms = [l_1, l_2, m_1, m_2, J_1, J_2, b_1, b_2, g];
param_vals = [l_0_1, l_1_2, m_r, m_p, J_r, J_p, b_r, b_p, gravity];

A_tau = double(subs(A_ss, param_syms, param_vals));
B_tau = double(subs(B_ss, param_syms, param_vals));

fprintf('============================================================\n');
fprintf('  STATE-SPACE WITH TORQUE INPUT (from Euler-Lagrange)\n');
fprintf('============================================================\n');
fprintf('A_tau =\n'); disp(A_tau);
fprintf('B_tau =\n'); disp(B_tau);
fprintf('Open-loop poles (torque):\n');
disp(eig(A_tau));

%% ====================================================================
%  TORQUE → VOLTAGE CONVERSION
%  ====================================================================
%
%  The motor equation relates voltage to torque:
%
%    V_m = R_m · i  +  k_m · dtheta_1     (KVL)
%    tau = k_m · i                         (motor torque)
%
%  Solving for tau:
%    tau = (k_m / R_m) · V_m  -  (k_m^2 / R_m) · dtheta_1
%
%  The state equation with torque input is:
%    dx = A_tau · x + B_tau · tau
%
%  Substituting the motor equation:
%    dx = A_tau · x + B_tau · [(k_m/R_m)·V_m - (k_m^2/R_m)·dtheta_1]
%       = [A_tau - (k_m^2/R_m)·B_tau·[0 0 1 0]] · x  +  (k_m/R_m)·B_tau · V_m
%
%  So the voltage-input system is:
%    A_V = A_tau,  but with back-EMF damping added to column 3
%    B_V = (k_m / R_m) · B_tau

fprintf('============================================================\n');
fprintf('  TORQUE → VOLTAGE CONVERSION\n');
fprintf('============================================================\n');
fprintf('  k_m = %.4f,  R_m = %.1f\n', k_m, R_m);
fprintf('  k_m/R_m  = %.6f  (torque per volt)\n', k_m/R_m);
fprintf('  k_m^2/R_m = %.6f  (back-EMF damping)\n\n', k_m^2/R_m);

A_V = A_tau;
B_V = (k_m / R_m) * B_tau;

back_emf = k_m^2 / R_m;
A_V(3,3) = A_V(3,3) - back_emf * B_tau(3);
A_V(4,3) = A_V(4,3) - back_emf * B_tau(4);

fprintf('============================================================\n');
fprintf('  STATE-SPACE WITH VOLTAGE INPUT\n');
fprintf('============================================================\n');
fprintf('A_V =\n'); disp(A_V);
fprintf('B_V =\n'); disp(B_V);
fprintf('Open-loop poles (voltage):\n');
disp(eig(A_V));

%% Transfer Function (voltage input)

fprintf('============================================================\n');
fprintf('  TRANSFER FUNCTION (voltage input)\n');
fprintf('============================================================\n');

sys_V = ss(A_V, B_V, C_ss, D_ss);
[num_V, den_V] = ss2tf(A_V, B_V, C_ss, D_ss);
G_V = tf(num_V, den_V);
fprintf('G_V(s) =\n'); disp(G_V);

%% Controllability

Co = ctrb(A_V, B_V);
fprintf('Controllability rank = %d / %d\n\n', rank(Co), size(A_V,1));

%% Pole Placement (voltage input)

fprintf('============================================================\n');
fprintf('  POLE PLACEMENT (voltage input)\n');
fprintf('============================================================\n\n');

pole_sets = {
    [-3, -4, -10, -15],   'Very conservative';
    [-5, -6, -15, -25],   'Conservative';
    [-8, -10, -20, -30],  'Moderate';
};

for i = 1:size(pole_sets, 1)
    poles = pole_sets{i, 1};
    label = pole_sets{i, 2};
    fprintf('-- %s: poles = [%s] --\n', label, num2str(poles));
    try
        K = place(A_V, B_V, poles);
        fprintf('  K = [%.4f, %.4f, %.4f, %.4f]\n', K);
        eig_cl = eig(A_V - B_V*K);
        fprintf('  CL poles: ['); fprintf('%.2f ', eig_cl); fprintf(']\n');
        % Voltage at 10 deg deflection
        x_test = [0; 10*pi/180; 0; 0];
        V_test = -K * x_test;
        fprintf('  Voltage at 10 deg: %.2f V\n\n', V_test);
    catch ME
        fprintf('  FAILED: %s\n\n', ME.message);
    end
end
