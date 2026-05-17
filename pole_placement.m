%% Pole Placement for Qube-Servo 3 Inverted Pendulum
%  =================================================
%
%  This script:
%    1. Builds the linearised state-space from the full rotary EOM
%       (same physics as PendulumModel.mlx)
%    2. Applies the torque → voltage conversion (the missing step!)
%    3. Runs place() to compute state-feedback gains K
%    4. Validates with nonlinear simulation
%
%  The key fix:
%    PendulumModel.mlx derives A, B with torque input (τ).
%    The controller outputs voltage (Vm).
%    Without converting, gains come out ~178× too large because
%    the model thinks u=1 means 1 N·m, but on hardware u=1 means
%    1 V which only produces km/Rm = 0.0056 N·m.
%
%  State: x = [θ, α, θ̇, α̇]
%         θ = arm angle, α = pendulum angle (0 = upright)
%  Input: u = Vm (motor voltage)
%  Law:   u = -K·x

clear; clc; close all;

%% ====================================================================
%  1. PARAMETERS
%  ====================================================================
Rm  = 7.5;        % Motor resistance [Ω]
km  = 0.042;      % Back-EMF / torque constant [V·s/rad = N·m/A]
Jr  = 4.0e-6;     % Rotor inertia [kg·m²]
Lr  = 0.0826;     % Arm length [m]
mp  = 0.024;      % Pendulum mass [kg]
Lp  = 0.129;      % Pendulum length [m]
Jp  = 3.33e-5;    % Pendulum inertia about pivot [kg·m²]
g   = 9.81;

% Derived
hLp = Lp / 2;                     % Pendulum CoM distance
Jt  = Jr + mp * Lr^2;             % Total arm inertia

%% ====================================================================
%  2. LINEARISED STATE-SPACE (TORQUE INPUT)
%  ====================================================================
%  From the Euler-Lagrange EOM (same as PendulumModel.mlx), linearised
%  about α=0 (upright), all velocities = 0, friction = 0.
%
%  Mass matrix at equilibrium:
%    [Jt,        -mp·hLp·Lr ] [θ̈]   [τ           ]
%    [-mp·hLp·Lr, Jp+mp·hLp²] [α̈] = [mp·hLp·g·α  ]

M11 = Jt;
M12 = -mp * hLp * Lr;
M22 = Jp + mp * hLp^2;
det_M = M11 * M22 - M12^2;

fprintf('============================================================\n');
fprintf('  MASS MATRIX (linearised at upright)\n');
fprintf('============================================================\n');
fprintf('  Jt = Jr + mp·Lr² = %.4e kg·m²\n', Jt);
fprintf('  M12 = -mp·hLp·Lr = %.4e kg·m²\n', M12);
fprintf('  M22 = Jp + mp·hLp² = %.4e kg·m²\n', M22);
fprintf('  det = %.4e\n\n', det_M);

% Invert mass matrix to get accelerations:
%   θ̈ = (M22·τ − M12·mp·hLp·g·α) / det
%   α̈ = (M11·mp·hLp·g·α − M12·τ) / det

A_tau = [0, 0, 1, 0;
         0, 0, 0, 1;
         0, -M12*mp*hLp*g / det_M,  0,  0;
         0,  M11*mp*hLp*g / det_M,  0,  0];

B_tau = [0;
         0;
          M22 / det_M;
         -M12 / det_M];

fprintf('============================================================\n');
fprintf('  STATE-SPACE WITH TORQUE INPUT  (u = τ)\n');
fprintf('============================================================\n');
fprintf('A_τ =\n'); disp(A_tau);
fprintf('B_τ =\n'); disp(B_tau');
fprintf('\n');

% Show how large torque-input gains would be
ol_poles_tau = eig(A_tau);
fprintf('Open-loop poles (same regardless of input scaling):\n');
for i = 1:length(ol_poles_tau)
    fprintf('  s = %+.4f\n', real(ol_poles_tau(i)));
end
fprintf('\n');

%% ====================================================================
%  3. TORQUE → VOLTAGE CONVERSION  (the critical step!)
%  ====================================================================
%
%  Motor model:  τ = (km/Rm)·Vm − (km²/Rm)·θ̇
%
%  Substituting into ẋ = A_τ·x + B_τ·τ:
%
%    ẋ = A_τ·x + B_τ·[(km/Rm)·Vm − (km²/Rm)·θ̇]
%      = [A_τ − (km²/Rm)·B_τ·e₃ᵀ]·x + (km/Rm)·B_τ·Vm
%
%  where e₃ᵀ = [0 0 1 0] picks out θ̇.
%
%  IMPORTANT: update A BEFORE scaling B, since A uses B_torque!

A_volt = A_tau;
A_volt(3,3) = A_tau(3,3) - (km^2 / Rm) * B_tau(3);   % back-EMF on θ̈
A_volt(4,3) = A_tau(4,3) - (km^2 / Rm) * B_tau(4);   % back-EMF on α̈

B_volt = (km / Rm) * B_tau;

fprintf('============================================================\n');
fprintf('  STATE-SPACE WITH VOLTAGE INPUT  (u = Vm)\n');
fprintf('============================================================\n');
fprintf('A_V =\n'); disp(A_volt);
fprintf('B_V =\n'); disp(B_volt');
fprintf('\n');
fprintf('  Note: A(3,3) = %.4f  ← back-EMF damping on θ̈\n', A_volt(3,3));
fprintf('  Note: A(4,3) = %.4f  ← back-EMF coupling to α̈\n', A_volt(4,3));
fprintf('\n');

% Scale comparison
fprintf('  B_τ(3) = %.1f   →   B_V(3) = %.1f   (ratio: %.0f×)\n', ...
        B_tau(3), B_volt(3), B_tau(3)/B_volt(3));
fprintf('  B_τ(4) = %.1f   →   B_V(4) = %.1f   (ratio: %.0f×)\n', ...
        B_tau(4), B_volt(4), B_tau(4)/B_volt(4));
fprintf('  This is why torque-input gains are ~%.0f× too large!\n\n', Rm/km);

%% ====================================================================
%  4. CONTROLLABILITY CHECK
%  ====================================================================
Co = ctrb(A_volt, B_volt);
fprintf('Controllability rank: %d / %d  ', rank(Co), size(A_volt,1));
if rank(Co) == size(A_volt,1)
    fprintf('✓\n\n');
else
    fprintf('✗ — PROBLEM!\n\n');
end

%% ====================================================================
%  5. POLE PLACEMENT
%  ====================================================================
%
%  Guidelines for choosing poles:
%    - Must be faster than the unstable OL pole (+20.6 rad/s)
%    - Dominant poles set the transient response (ζ, ωn)
%    - Fast poles handle the coupling, should be 5-10× faster
%    - All poles must be in LHP (negative real part)
%
%  We'll try several sets and compare.

fprintf('============================================================\n');
fprintf('  POLE PLACEMENT — GAIN COMPARISON\n');
fprintf('============================================================\n\n');

% ── Set 1: Quanser-suggested poles ──────────────────────────────
% Dominant pair: ζ=0.7, ωn=4 → -2.8 ± 2.86j
% Fast pair: -30, -40
poles_quanser = [-2.8+2.86j, -2.8-2.86j, -30, -40];

% ── Set 2: Moderate — all real, well-separated ─────────────────
poles_moderate = [-10, -12, -25, -35];

% ── Set 3: Aggressive — fast response ──────────────────────────
poles_aggressive = [-15, -20, -40, -50];

% ── Set 4: Conservative — slow and safe ────────────────────────
poles_conservative = [-5, -7, -20, -30];

all_pole_sets = {poles_quanser, poles_moderate, poles_aggressive, poles_conservative};
set_names = {'Quanser-suggested', 'Moderate', 'Aggressive', 'Conservative'};

K_results = cell(size(all_pole_sets));

for i = 1:length(all_pole_sets)
    poles = all_pole_sets{i};
    K = place(A_volt, B_volt, poles);
    K_results{i} = K;

    fprintf('── %s ──\n', set_names{i});
    fprintf('  Poles: ');
    for p = poles
        if imag(p) ~= 0
            fprintf('%.1f%+.2fj  ', real(p), imag(p));
        else
            fprintf('%.0f  ', real(p));
        end
    end
    fprintf('\n');
    fprintf('  K = [%.2f, %.2f, %.2f, %.2f]\n', K(1), K(2), K(3), K(4));
    fprintf('       k_θ=%.2f  k_α=%.2f  k_θ̇=%.2f  k_α̇=%.2f\n', ...
            K(1), K(2), K(3), K(4));

    % Check CL eigenvalues match requested
    cl_eigs = eig(A_volt - B_volt * K);
    fprintf('  CL eigenvalues: ');
    for e = cl_eigs.'
        if abs(imag(e)) > 0.01
            fprintf('%.2f%+.2fj  ', real(e), imag(e));
        else
            fprintf('%.2f  ', real(e));
        end
    end
    fprintf('\n');

    % Voltage for 5° step
    x_5deg = [0; 5*pi/180; 0; 0];
    V_5deg = -K * x_5deg;
    fprintf('  Voltage for 5° tilt: %.2f V  (limit: 6V)\n', V_5deg);
    fprintf('\n');
end

% Show working hardware gains for reference
fprintf('── Reference: working hardware gains ──\n');
fprintf('  K = [-2.00, 20.00, -1.00, 2.00]\n');
fprintf('       k_θ=-2  k_α=20  k_θ̇=-1  k_α̇=2\n');
fprintf('  (sign convention: u = -K·x, so positive k_α\n');
fprintf('   means positive α produces negative voltage)\n\n');

%% ====================================================================
%  6. BONUS: What happens if you forget the conversion?
%  ====================================================================
%
%  This is what PendulumModel.mlx was doing — place() on torque model.

fprintf('============================================================\n');
fprintf('  WHAT GOES WRONG WITHOUT CONVERSION\n');
fprintf('============================================================\n\n');

K_wrong = place(A_tau, B_tau, poles_quanser);
K_right = K_results{1};

fprintf('  With torque input (WRONG for voltage controller):\n');
fprintf('    K = [%.4f, %.4f, %.4f, %.4f]\n', K_wrong);
fprintf('  With voltage input (CORRECT):\n');
fprintf('    K = [%.2f, %.2f, %.2f, %.2f]\n', K_right);
fprintf('\n');
fprintf('  Ratio K_wrong / K_right:\n');
ratio = K_wrong ./ K_right;
fprintf('    [%.1f, %.1f, %.1f, %.1f]\n', ratio);
fprintf('  → Torque gains are NOT simply km/Rm × voltage gains.\n');
fprintf('    The back-EMF damping in A changes the gain structure.\n');
fprintf('    This is why you can''t just rescale — you get wrong gains\n');
fprintf('    AND wrong dynamics.\n\n');

%% ====================================================================
%  7. NONLINEAR SIMULATION — validate the gains
%  ====================================================================

dt = 0.001;
T_sim = 3.0;
N_steps = round(T_sim / dt);
alpha0 = 10 * pi/180;

% Simulate all pole sets + hardware reference
n_controllers = length(all_pole_sets) + 1;
labels = [set_names, {'Hardware reference'}];
K_all = [K_results, {[-2, 20, -1, 2]}];  % last one is hardware

% Pre-allocate
states_log = zeros(4, N_steps, n_controllers);
Vm_log = zeros(N_steps, n_controllers);

for c = 1:n_controllers
    K = K_all{c};
    state = [0; alpha0; 0; 0];

    for k = 1:N_steps
        states_log(:, k, c) = state;

        u = -K * state;
        u = max(-6, min(6, u));
        Vm_log(k, c) = u;

        state = rk4_rotary(state, u, dt, Jt, mp, Lp, Jp, Lr, g, km, Rm);
    end
end

t = (0:N_steps-1) * dt;

% --- Performance metrics ---
fprintf('============================================================\n');
fprintf('  NONLINEAR SIMULATION  (α₀ = %.0f°, full rotary EOM)\n', rad2deg(alpha0));
fprintf('============================================================\n');
fprintf('%25s %12s %12s %10s %10s\n', 'Controller', 'Settle [s]', 'OS [%%]', '|Vm|max', 'α_final');
fprintf('%s\n', repmat('-', 1, 72));

for c = 1:n_controllers
    a_log = squeeze(states_log(2, :, c));
    v_log = Vm_log(:, c);

    % Settling time (2%)
    band = 0.02 * alpha0;
    idx = find(abs(a_log) > band, 1, 'last');
    if isempty(idx); t_s = 0; else; t_s = idx * dt; end

    % Overshoot
    peak = min(a_log);
    os = abs(peak) / alpha0 * 100;

    fprintf('%25s %12.3f %12.1f %10.2f %10.4f°\n', ...
            labels{c}, t_s, os, max(abs(v_log)), rad2deg(a_log(end)));
end

% --- Plot ---
figure('Position', [100, 100, 1000, 700]);

colors = lines(n_controllers);

subplot(3,1,1);
for c = 1:n_controllers
    a = rad2deg(squeeze(states_log(2, :, c)));
    if c == n_controllers
        plot(t, a, 'k--', 'LineWidth', 2, 'DisplayName', labels{c});
    else
        plot(t, a, 'LineWidth', 1.5, 'Color', colors(c,:), 'DisplayName', labels{c});
    end
    hold on;
end
yline(0, ':', 'HandleVisibility', 'off');
ylabel('\alpha [deg]');
title(sprintf('Pendulum angle (\\alpha_0 = %.0f°)', rad2deg(alpha0)));
legend('Location', 'best');
grid on;

subplot(3,1,2);
for c = 1:n_controllers
    th = rad2deg(squeeze(states_log(1, :, c)));
    if c == n_controllers
        plot(t, th, 'k--', 'LineWidth', 2, 'DisplayName', labels{c});
    else
        plot(t, th, 'LineWidth', 1.5, 'Color', colors(c,:), 'DisplayName', labels{c});
    end
    hold on;
end
ylabel('\theta [deg]');
title('Arm angle');
legend('Location', 'best');
grid on;

subplot(3,1,3);
for c = 1:n_controllers
    if c == n_controllers
        plot(t, Vm_log(:,c), 'k--', 'LineWidth', 2, 'DisplayName', labels{c});
    else
        plot(t, Vm_log(:,c), 'LineWidth', 1.5, 'Color', colors(c,:), 'DisplayName', labels{c});
    end
    hold on;
end
yline(6, ':', '6V', 'Color', [0.8 0.2 0.2], 'HandleVisibility', 'off');
yline(-6, ':', '-6V', 'Color', [0.8 0.2 0.2], 'HandleVisibility', 'off');
ylabel('Vm [V]');
xlabel('Time [s]');
title('Control voltage');
legend('Location', 'best');
grid on;

sgtitle('Pole Placement Comparison — Full Rotary Nonlinear Sim', 'FontSize', 14);
saveas(gcf, 'pole_placement_comparison.png');
fprintf('\nSaved: pole_placement_comparison.png\n');

%% ====================================================================
%  LOCAL FUNCTIONS
%  ====================================================================

function s_new = rk4_rotary(s, Vm, dt, Jt, mp, Lp, Jp, Lr, g, km, Rm)
    k1 = rotary_eom(s,             Vm, Jt, mp, Lp, Jp, Lr, g, km, Rm);
    k2 = rotary_eom(s + 0.5*dt*k1, Vm, Jt, mp, Lp, Jp, Lr, g, km, Rm);
    k3 = rotary_eom(s + 0.5*dt*k2, Vm, Jt, mp, Lp, Jp, Lr, g, km, Rm);
    k4 = rotary_eom(s + dt*k3,     Vm, Jt, mp, Lp, Jp, Lr, g, km, Rm);
    s_new = s + (dt/6) * (k1 + 2*k2 + 2*k3 + k4);
end

function ds = rotary_eom(s, Vm, Jt, mp, Lp, Jp, Lr, g, km, Rm)
    theta_dot = s(3);
    alpha     = s(2);
    alpha_dot = s(4);
    Vm = max(-10, min(10, Vm));

    sa = sin(alpha);  ca = cos(alpha);
    hLp = Lp / 2;

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
