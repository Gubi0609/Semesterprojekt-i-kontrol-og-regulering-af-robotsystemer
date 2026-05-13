%% Cart-Pole Simplification of the Qube-Servo 3 Rotary Inverted Pendulum
%  =====================================================================
%
%  This script does four things:
%    1. Maps rotary parameters onto the equivalent cart-pole model
%    2. Derives the linearised state-space and 2nd-order transfer function
%    3. Designs a PID controller for the Vm -> alpha loop
%    4. Simulates closed-loop performance (linear + nonlinear)
%       and evaluates against typical control specs
%
%  Convention: alpha = 0 is upright, positive = tilt from vertical.

clear; clc; close all;

%% ====================================================================
%  1. SYSTEM PARAMETERS  (from qube_types.h, SwingUp branch)
%  ====================================================================

% --- Motor ---
Rm  = 7.5;        % Terminal resistance [Ohm]
km  = 0.042;      % Torque / back-EMF constant [N·m/A = V·s/rad]

% --- Arm ---
Jr  = 4.0e-6;     % Rotor inertia [kg·m²]
Lr  = 0.0826;     % Arm length, shaft to pivot [m]

% --- Pendulum ---
mp  = 0.024;      % Pendulum mass [kg]
Lp  = 0.129;      % Pendulum total length [m]
Jp  = 3.33e-5;    % Pendulum inertia about pivot [kg·m²]
g   = 9.81;       % Gravity [m/s²]

% --- Derived cart-pole parameters ---
l   = Lp / 2;                  % Distance to pendulum CoM [m]
M   = Jr / Lr^2;               % Effective cart mass (arm ONLY) [kg]
I   = Jp;                      % Pendulum inertia about pivot [kg·m²]
Jt  = Jr + mp * Lr^2;          % Total arm-side inertia (for reference) [kg·m²]

fprintf('============================================================\n');
fprintf('  CART-POLE PARAMETER MAPPING\n');
fprintf('============================================================\n');
fprintf('  Cart mass        M  = Jr/Lr²    = %.6f kg\n', M);
fprintf('  Pendulum mass    m  = mp        = %.4f kg\n', mp);
fprintf('  Sanity check:  M+m  = %.6f  vs  Jt/Lr² = %.6f\n', M+mp, Jt/Lr^2);
fprintf('  Pendulum CoM    l   = Lp/2      = %.4f m\n', l);
fprintf('  Pendulum inertia I  = Jp        = %.4e kg·m²\n', I);
fprintf('\n');

%% ====================================================================
%  2. LINEARISED MODEL
%  ====================================================================
%
%  State: x = [x_cart; alpha; x_dot; alpha_dot]
%  Input: u = Vm  (motor voltage)
%
%  Motor produces force on cart:
%    F = (km / (Rm·Lr)) · Vm  -  (km² / (Rm·Lr²)) · x_dot
%      = b_v · Vm  -  b_d · x_dot

b_v = km / (Rm * Lr);          % Force per volt [N/V]
b_d = km^2 / (Rm * Lr^2);      % Back-EMF damping [N·s/m]

% Mass matrix determinant (must be > 0)
Delta = (M + mp) * (I + mp*l^2) - (mp*l)^2;
fprintf('  Δ = %.6e  (> 0: %s)\n\n', Delta, string(Delta > 0));

% State-space matrices
A = [0,  0,                             1,  0;
     0,  0,                             0,  1;
     0,  (mp*l)^2 * g / Delta,        -(I + mp*l^2) * b_d / Delta,  0;
     0,  mp*g*l*(M+mp) / Delta,        -mp*l * b_d / Delta,         0];

B = [0;
     0;
     (I + mp*l^2) * b_v / Delta;
     mp*l * b_v / Delta];

% Output: pendulum angle alpha (state 2)
C_alpha = [0 1 0 0];
D_alpha = 0;

% Full state output (for later use)
C_full = eye(4);
D_full = zeros(4, 1);

fprintf('============================================================\n');
fprintf('  STATE-SPACE MATRICES\n');
fprintf('  State: [x, α, ẋ, α̇],  Input: Vm\n');
fprintf('============================================================\n');
fprintf('\nA =\n'); disp(A);
fprintf('B = '); fprintf('%.6f  ', B); fprintf('\n\n');

% Open-loop poles
ol_poles = eig(A);
fprintf('Open-loop poles:\n');
for i = 1:length(ol_poles)
    if imag(ol_poles(i)) == 0
        fprintf('  s = %+.4f\n', real(ol_poles(i)));
    else
        fprintf('  s = %+.4f %+.4fj\n', real(ol_poles(i)), imag(ol_poles(i)));
    end
end
fprintf('\n');

% Controllability
Co = ctrb(A, B);
fprintf('Controllability rank: %d / %d  ', rank(Co), size(A,1));
if rank(Co) == size(A,1)
    fprintf('✓ Controllable\n\n');
else
    fprintf('✗ NOT controllable\n\n');
end

%% ====================================================================
%  3. TRANSFER FUNCTION  Vm -> alpha  (2nd order, x eliminated)
%  ====================================================================
%
%  From the coupled linearised EOM, eliminate x to get:
%    G(s) = b_tf / (s² - a_tf)
%
%  This is the plant the PID must stabilise.

a_tf = mp * g * l * (M + mp) / Delta;
b_tf = mp * l * b_v / Delta;

fprintf('============================================================\n');
fprintf('  TRANSFER FUNCTION  Vm → α  (2nd order)\n');
fprintf('============================================================\n');
fprintf('  G(s) = %.2f / (s² - %.2f)\n', b_tf, a_tf);
fprintf('  Unstable pole: s = +%.2f rad/s\n', sqrt(a_tf));
fprintf('  Stable pole:   s = -%.2f rad/s\n', sqrt(a_tf));
fprintf('\n');
fprintf('  Compare with full rotary model (SESSION_NOTES.md):\n');
fprintf('    G_rot(s)  = 120.2 / (s² - 425.6)\n');
fprintf('    G_cart(s) = %.1f / (s² - %.1f)\n', b_tf, a_tf);
fprintf('    Numerator error:   %.2f%%\n', abs(b_tf - 120.2)/120.2*100);
fprintf('    Denominator error: %.2f%%\n\n', abs(a_tf - 425.6)/425.6*100);

% Build the transfer function object
G = tf(b_tf, [1, 0, -a_tf]);

% Also build the 4th-order SISO from state-space (Vm -> alpha)
sys_ss = ss(A, B, C_alpha, D_alpha);
G_ss = tf(sys_ss);

fprintf('  4th-order TF from state-space (Vm → α):\n');
fprintf('  '); G_ss

%% ====================================================================
%  4. PID CONTROLLER DESIGN
%  ====================================================================
%
%  Strategy:
%    - Use the 2nd-order G(s) for design (clean, captures the key dynamics)
%    - PID: C(s) = Kp + Ki/s + Kd·s·N/(s+N)
%      (N = derivative filter coefficient, prevents pure differentiator)
%    - Negative feedback: the closed-loop is  G·C / (1 + G·C)
%
%  Design specs (typical for this system):
%    - Settling time    < 2 s    (2% band)
%    - Overshoot        < 30%
%    - Steady-state error = 0   (integral action handles this)
%    - No saturation beyond ±6V  (soft limit from existing controller)
%
%  We'll start with gains derived from the existing working LQR,
%  then also try a tuned PID for comparison.

fprintf('============================================================\n');
fprintf('  PID CONTROLLER DESIGN\n');
fprintf('============================================================\n\n');

% --- Design A: Gains mapped from working LQR state feedback ---
%
% The existing balance controller uses:
%   u = -k_alpha·α - k_alpha_dot·α̇ - k_theta·θ - k_theta_dot·θ̇
%     = -20·α - 2·α̇ - (-2)·θ - (-1)·θ̇
%
% For the reduced Vm→α loop (ignoring theta terms), this is PD:
%   u = -20·α - 2·α̇  ←→  C(s) = -(Kp + Kd·s) with Kp=20, Kd=2
%
% With integral from the existing Ki=0.3:
Kp_lqr = 20.0;
Ki_lqr = 0.3;
Kd_lqr = 2.0;
N_lqr  = 100;       % Derivative filter (high = less filtering)

C_lqr = pid(Kp_lqr, Ki_lqr, Kd_lqr, 1/N_lqr);
fprintf('Design A — LQR-mapped PID:\n');
fprintf('  Kp = %.1f,  Ki = %.1f,  Kd = %.1f,  N = %d\n', ...
        Kp_lqr, Ki_lqr, Kd_lqr, N_lqr);

% Closed-loop (negative feedback)
T_lqr = feedback(-G * C_lqr, 1);

% Step response info
info_lqr = stepinfo(T_lqr);
fprintf('  Settling time (2%%): %.3f s\n', info_lqr.SettlingTime);
fprintf('  Overshoot:          %.1f%%\n', info_lqr.Overshoot);
fprintf('  Rise time:          %.3f s\n', info_lqr.RiseTime);
fprintf('  Peak:               %.4f\n', info_lqr.Peak);
fprintf('\n');

% Stability margins
[Gm, Pm, Wcg, Wcp] = margin(-G * C_lqr);
fprintf('  Gain margin:  %.1f dB at %.1f rad/s\n', 20*log10(Gm), Wcg);
fprintf('  Phase margin: %.1f° at %.1f rad/s\n', Pm, Wcp);
fprintf('\n');

% --- Design B: Tuned PID ---
% Use slightly more aggressive gains with integral for zero SS error.
% We target:
%   - Faster settling than LQR (the LQR gains were conservative)
%   - Less overshoot
%   - Reasonable phase margin (> 30°)

% Start from LQR gains and increase Kp slightly, add more Ki
Kp_tune = 25.0;
Ki_tune = 1.0;
Kd_tune = 2.5;
N_tune  = 50;

C_tune = pid(Kp_tune, Ki_tune, Kd_tune, 1/N_tune);
fprintf('Design B — Tuned PID:\n');
fprintf('  Kp = %.1f,  Ki = %.1f,  Kd = %.1f,  N = %d\n', ...
        Kp_tune, Ki_tune, Kd_tune, N_tune);

T_tune = feedback(-G * C_tune, 1);

info_tune = stepinfo(T_tune);
fprintf('  Settling time (2%%): %.3f s\n', info_tune.SettlingTime);
fprintf('  Overshoot:          %.1f%%\n', info_tune.Overshoot);
fprintf('  Rise time:          %.3f s\n', info_tune.RiseTime);
fprintf('  Peak:               %.4f\n', info_tune.Peak);
fprintf('\n');

[Gm2, Pm2, Wcg2, Wcp2] = margin(-G * C_tune);
fprintf('  Gain margin:  %.1f dB at %.1f rad/s\n', 20*log10(Gm2), Wcg2);
fprintf('  Phase margin: %.1f° at %.1f rad/s\n', Pm2, Wcp2);
fprintf('\n');

%% ====================================================================
%  5. SIMULATION — Linear step response comparison
%  ====================================================================

figure('Name', 'PID Comparison — Linear Step Response', ...
       'Position', [100, 100, 900, 700]);

% --- Step response ---
subplot(2, 2, 1);
t_step = linspace(0, 3, 3000);
[y_lqr, ~] = step(T_lqr, t_step);
[y_tune, ~] = step(T_tune, t_step);
plot(t_step, y_lqr, 'LineWidth', 1.5); hold on;
plot(t_step, y_tune, '--', 'LineWidth', 1.5);
yline(1, ':', 'Color', [0.5 0.5 0.5]);
yline(1.02, ':', 'Color', [0.8 0.2 0.2]);
yline(0.98, ':', 'Color', [0.8 0.2 0.2]);
xlabel('Time [s]'); ylabel('α / α_{ref}');
title('Closed-Loop Step Response');
legend('LQR-mapped PID', 'Tuned PID', 'Location', 'best');
grid on;

% --- Bode of open-loop L(s) = -G·C ---
subplot(2, 2, 2);
L_lqr = -G * C_lqr;
L_tune = -G * C_tune;
bode(L_lqr, L_tune, {0.1, 1000});
legend('LQR-mapped', 'Tuned');
title('Open-Loop Bode  L(s) = -G·C');
grid on;

% --- Root locus of G(s) ---
subplot(2, 2, 3);
rlocus(G);
title('Root Locus of Plant G(s)');
grid on;

% --- Pole-zero map of closed loop ---
subplot(2, 2, 4);
pzmap(T_lqr, T_tune);
legend('LQR-mapped CL', 'Tuned CL');
title('Closed-Loop Pole-Zero Map');
grid on;

sgtitle('Cart-Pole PID Design — Qube-Servo 3', 'FontSize', 14);
saveas(gcf, 'pid_linear_analysis.png');
fprintf('Saved: pid_linear_analysis.png\n');

%% ====================================================================
%  6. SIMULATION — Nonlinear cart-pole with PID
%  ====================================================================
%  Uses RK4 integration of the full nonlinear cart-pole EOM,
%  closed-loop with each PID. This validates that the linear
%  design actually works on the nonlinear plant.

dt       = 0.001;       % 1 kHz
T_sim    = 3.0;         % seconds
N_steps  = round(T_sim / dt);
alpha0   = 10 * pi/180; % 10° initial tilt

% --- PID state structure ---
% We implement the PID in discrete time to match hardware reality.
pid_state_A = struct('integral', 0, 'prev_error', 0, 'prev_D', 0, 'first', true);
pid_state_B = struct('integral', 0, 'prev_error', 0, 'prev_D', 0, 'first', true);

% Pre-allocate logs
t_log = zeros(1, N_steps);
% Design A
alpha_A = zeros(1, N_steps); x_A = zeros(1, N_steps); Vm_A = zeros(1, N_steps);
% Design B
alpha_B = zeros(1, N_steps); x_B = zeros(1, N_steps); Vm_B = zeros(1, N_steps);

% Initial states: [x, alpha, x_dot, alpha_dot]
state_A = [0; alpha0; 0; 0];
state_B = [0; alpha0; 0; 0];

for k = 1:N_steps
    t_log(k) = (k-1) * dt;

    % --- Log ---
    alpha_A(k) = state_A(2);  x_A(k) = state_A(1);
    alpha_B(k) = state_B(2);  x_B(k) = state_B(1);

    % --- PID A (LQR-mapped) ---
    error_A = 0 - state_A(2);   % reference - alpha
    [u_A, pid_state_A] = pid_compute(error_A, dt, ...
        Kp_lqr, Ki_lqr, Kd_lqr, N_lqr, 6.0, pid_state_A);
    Vm_A(k) = u_A;

    % --- PID B (Tuned) ---
    error_B = 0 - state_B(2);
    [u_B, pid_state_B] = pid_compute(error_B, dt, ...
        Kp_tune, Ki_tune, Kd_tune, N_tune, 6.0, pid_state_B);
    Vm_B(k) = u_B;

    % --- RK4 step ---
    state_A = rk4_cart_pole(state_A, u_A, dt, M, mp, l, I, g, km, Rm, Lr);
    state_B = rk4_cart_pole(state_B, u_B, dt, M, mp, l, I, g, km, Rm, Lr);
end

% --- Performance metrics ---
fprintf('\n============================================================\n');
fprintf('  NONLINEAR SIMULATION RESULTS  (α₀ = %.0f°)\n', rad2deg(alpha0));
fprintf('============================================================\n');

for design = {'A (LQR-mapped)', 'B (Tuned)'}
    if contains(design{1}, 'A')
        a_log = alpha_A; v_log = Vm_A;
    else
        a_log = alpha_B; v_log = Vm_B;
    end

    % Settling time (2% of initial perturbation)
    band = 0.02 * alpha0;
    settled_idx = find(abs(a_log) > band, 1, 'last');
    if isempty(settled_idx)
        t_settle = 0;
    else
        t_settle = settled_idx * dt;
    end

    % Overshoot (negative overshoot past 0)
    peak_neg = min(a_log);
    overshoot_pct = abs(peak_neg) / alpha0 * 100;

    % Max voltage
    max_V = max(abs(v_log));

    fprintf('\n  Design %s:\n', design{1});
    fprintf('    Settling time (2%%):   %.3f s\n', t_settle);
    fprintf('    Max undershoot:       %.2f° (%.0f%% of α₀)\n', ...
            rad2deg(abs(peak_neg)), overshoot_pct);
    fprintf('    Max |Vm|:             %.2f V\n', max_V);
    fprintf('    Final α:              %.4f°\n', rad2deg(a_log(end)));
end

% --- Plot nonlinear results ---
figure('Name', 'Nonlinear Simulation — Cart-Pole + PID', ...
       'Position', [100, 100, 900, 600]);

subplot(2, 1, 1);
plot(t_log, rad2deg(alpha_A), 'LineWidth', 1.5); hold on;
plot(t_log, rad2deg(alpha_B), '--', 'LineWidth', 1.5);
yline(0, ':', 'Color', [0.5 0.5 0.5]);
xlabel('Time [s]'); ylabel('α [deg]');
title(sprintf('Nonlinear Cart-Pole: Pendulum Angle  (α₀ = %.0f°)', rad2deg(alpha0)));
legend('LQR-mapped PID', 'Tuned PID', 'Location', 'best');
grid on;

subplot(2, 1, 2);
plot(t_log, Vm_A, 'LineWidth', 1.5); hold on;
plot(t_log, Vm_B, '--', 'LineWidth', 1.5);
yline(6, ':', '6V limit', 'Color', [0.8 0.2 0.2]);
yline(-6, ':', '-6V limit', 'Color', [0.8 0.2 0.2]);
xlabel('Time [s]'); ylabel('Vm [V]');
title('Control Voltage');
legend('LQR-mapped PID', 'Tuned PID', 'Location', 'best');
grid on;

sgtitle('Nonlinear Closed-Loop Simulation', 'FontSize', 14);
saveas(gcf, 'pid_nonlinear_sim.png');
fprintf('\n\nSaved: pid_nonlinear_sim.png\n');

%% ====================================================================
%  7. APPROXIMATION QUALITY — Rotary vs Cart-Pole comparison
%  ====================================================================
%  Same comparison as in the Python script: run both nonlinear models
%  side-by-side with the same controller.

fprintf('\n============================================================\n');
fprintf('  APPROXIMATION ERROR: Cart-Pole vs Full Rotary\n');
fprintf('============================================================\n');
fprintf('%10s %16s %16s %14s\n', 'α₀ [deg]', 'max|Δα| [deg]', 'max|Δθ| [deg]', 'Note');
fprintf('%s\n', repmat('-', 1, 60));

for alpha0_deg = [1, 5, 10, 15, 20, 30]
    a0 = deg2rad(alpha0_deg);
    s_rot  = [0; a0; 0; 0];   % [theta, alpha, theta_dot, alpha_dot]
    s_cart = [0; a0; 0; 0];   % [x, alpha, x_dot, alpha_dot]

    max_da = 0;
    max_dt = 0;

    for k = 1:N_steps
        % Controller (same state-feedback gains for both)
        Vm_r = state_feedback(s_rot(2), s_rot(4), s_rot(1), s_rot(3));
        Vm_c = state_feedback(s_cart(2), s_cart(4), s_cart(1)/Lr, s_cart(3)/Lr);

        da = abs(s_rot(2) - s_cart(2));
        dt_eq = abs(s_rot(1) - s_cart(1)/Lr);
        max_da = max(max_da, da);
        max_dt = max(max_dt, dt_eq);

        s_rot  = rk4_rotary(s_rot, Vm_r, dt, Jr, mp, Lp, Jp, Lr, g, km, Rm);
        s_cart = rk4_cart_pole(s_cart, Vm_c, dt, M, mp, l, I, g, km, Rm, Lr);
    end

    if rad2deg(max_da) < 0.1
        note = 'excellent';
    elseif rad2deg(max_da) < 0.5
        note = 'good';
    elseif rad2deg(max_da) < 2.0
        note = 'acceptable';
    else
        note = 'poor';
    end

    fprintf('%10d %16.4f %16.4f %14s\n', ...
            alpha0_deg, rad2deg(max_da), rad2deg(max_dt), note);
end

fprintf('\nDone.\n');

%% ====================================================================
%  LOCAL FUNCTIONS
%  ====================================================================

function [u, s] = pid_compute(error, dt, Kp, Ki, Kd, N, u_lim, s)
    % Discrete PID with derivative filter and anti-windup
    P = Kp * error;

    s.integral = s.integral + error * dt;
    s.integral = max(-1.0, min(1.0, s.integral));   % anti-windup
    I_term = Ki * s.integral;

    if s.first
        D = 0;
        s.first = false;
    else
        alpha_f = dt / (1/N + dt);   % filter coefficient
        raw_D = (error - s.prev_error) / dt;
        D = alpha_f * raw_D + (1 - alpha_f) * s.prev_D;
    end
    s.prev_error = error;
    s.prev_D = D;

    u = P + I_term + Kd * D;
    u = max(-u_lim, min(u_lim, u));
end

function u = state_feedback(alpha, alpha_dot, theta, theta_dot)
    % Existing LQR-derived state feedback (for comparison sim)
    k_alpha = 20.0;  k_alpha_dot = 2.0;
    k_theta = -2.0;  k_theta_dot = -1.0;
    u = -k_alpha*alpha - k_alpha_dot*alpha_dot ...
        - k_theta*theta - k_theta_dot*theta_dot;
    u = max(-6, min(6, u));
end

function s_new = rk4_cart_pole(s, Vm, dt, M, mp, l, I, g, km, Rm, Lr)
    k1 = cart_pole_eom(s,             Vm, M, mp, l, I, g, km, Rm, Lr);
    k2 = cart_pole_eom(s + 0.5*dt*k1, Vm, M, mp, l, I, g, km, Rm, Lr);
    k3 = cart_pole_eom(s + 0.5*dt*k2, Vm, M, mp, l, I, g, km, Rm, Lr);
    k4 = cart_pole_eom(s + dt*k3,     Vm, M, mp, l, I, g, km, Rm, Lr);
    s_new = s + (dt/6) * (k1 + 2*k2 + 2*k3 + k4);
end

function ds = cart_pole_eom(s, Vm, M, mp, l, I, g, km, Rm, Lr)
    % Nonlinear cart-pole equations of motion
    % s = [x; alpha; x_dot; alpha_dot]
    x_dot     = s(3);
    alpha     = s(2);
    alpha_dot = s(4);
    Vm = max(-10, min(10, Vm));

    sa = sin(alpha);  ca = cos(alpha);

    % Motor force
    theta_dot = x_dot / Lr;
    tau = km * (Vm - km * theta_dot) / Rm;
    F = tau / Lr;

    % Mass matrix
    M11 = M + mp;
    M12 = -mp * l * ca;
    M22 = I + mp * l^2;

    rhs1 = F + mp * l * alpha_dot^2 * sa;
    rhs2 = mp * g * l * sa;

    det = M11*M22 - M12^2;
    x_dd     = ( M22*rhs1 - M12*rhs2) / det;
    alpha_dd = (-M12*rhs1 + M11*rhs2) / det;

    ds = [x_dot; alpha_dot; x_dd; alpha_dd];
end

function s_new = rk4_rotary(s, Vm, dt, Jr, mp, Lp, Jp, Lr, g, km, Rm)
    k1 = rotary_eom(s,             Vm, Jr, mp, Lp, Jp, Lr, g, km, Rm);
    k2 = rotary_eom(s + 0.5*dt*k1, Vm, Jr, mp, Lp, Jp, Lr, g, km, Rm);
    k3 = rotary_eom(s + 0.5*dt*k2, Vm, Jr, mp, Lp, Jp, Lr, g, km, Rm);
    k4 = rotary_eom(s + dt*k3,     Vm, Jr, mp, Lp, Jp, Lr, g, km, Rm);
    s_new = s + (dt/6) * (k1 + 2*k2 + 2*k3 + k4);
end

function ds = rotary_eom(s, Vm, Jr, mp, Lp, Jp, Lr, g, km, Rm)
    % Full rotary EOM (from plant.h)
    % s = [theta; alpha; theta_dot; alpha_dot]
    theta_dot = s(3);
    alpha     = s(2);
    alpha_dot = s(4);
    Vm = max(-10, min(10, Vm));

    sa = sin(alpha);  ca = cos(alpha);
    hLp = Lp / 2;
    Jt = Jr + mp * Lr^2;

    tau = km * (Vm - km * theta_dot) / Rm;

    M11 = Jt + Jp * sa^2;
    M12 = -mp * hLp * Lr * ca;
    M22 = Jp + mp * hLp^2;

    rhs1 = tau ...
         - mp * hLp * Lr * alpha_dot^2 * sa ...
         - 2 * Jp * sa * ca * theta_dot * alpha_dot;
    rhs2 = mp * hLp * g * sa ...
         + Jp * sa * ca * theta_dot^2;

    det = M11*M22 - M12^2;
    theta_dd = ( M22*rhs1 - M12*rhs2) / det;
    alpha_dd = ( M11*rhs2 - M12*rhs1) / det;

    ds = [theta_dot; alpha_dot; theta_dd; alpha_dd];
end
