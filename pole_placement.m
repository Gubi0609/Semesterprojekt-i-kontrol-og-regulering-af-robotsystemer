%% Pole Placement for Qube-Servo 3 Inverted Pendulum
%  =================================================
%
%  Convention: PIVOT-FRAME inertias throughout.
%    Jp = moment of inertia of pendulum about its pivot (measured)
%    Jr = moment of inertia of arm assembly about motor shaft (measured)
%
%  This means:
%    M22 = Jp  (NOT Jp + mp·hLp²)
%    M11 = Jt + Jp·sin²(α)  where Jt = Jr + mp·Lr²
%
%  Using pivot-frame avoids the double-counting bug where Jp_pivot
%  was plugged into a formula expecting Jp_com.
%
%  State: x = [θ, α, θ̇, α̇]
%  Input: u = Vm (motor voltage)
%  Law:   u = -K·x

clear; clc; close all;

%% ====================================================================
%  1. PARAMETERS (experimental values from 12/05/26)
%  ====================================================================
Rm  = 7.5;        % Motor resistance [Ω]
km  = 0.042;      % Back-EMF / torque constant [V·s/rad]
Jr  = 6.2e-5;     % Arm assembly inertia about motor shaft [kg·m²] (MEASURED)
Lr  = 0.0826;     % Arm length [m]
mp  = 0.024;      % Pendulum mass [kg]
Lp  = 0.129;      % Pendulum length [m]
Jp  = 1.26e-4;    % Pendulum inertia about PIVOT [kg·m²] (MEASURED)
g   = 9.81;

% Derived
hLp = Lp / 2;                     % Pendulum CoM distance
Jt  = Jr + mp * Lr^2;             % Total arm-side inertia

fprintf('============================================================\n');
fprintf('  PARAMETERS (pivot-frame convention)\n');
fprintf('============================================================\n');
fprintf('  Jr (measured)  = %.2e kg·m²\n', Jr);
fprintf('  Jp (measured)  = %.2e kg·m²  (about pivot)\n', Jp);
fprintf('  Jt = Jr+mp·Lr² = %.4e kg·m²\n', Jt);
fprintf('  hLp = Lp/2     = %.4f m\n', hLp);
fprintf('\n');

%% ====================================================================
%  2. LINEARISED STATE-SPACE (TORQUE INPUT, PIVOT-FRAME)
%  ====================================================================
%
%  Linearised at α=0 (upright), all velocities = 0:
%
%    [Jt,         -mp·hLp·Lr] [θ̈]   [τ          ]
%    [-mp·hLp·Lr,  Jp       ] [α̈] = [mp·hLp·g·α ]
%
%  NOTE: M22 = Jp (pivot), NOT Jp + mp·hLp².
%  The parallel axis term is already inside Jp.

M11 = Jt;
M12 = -mp * hLp * Lr;
M22 = Jp;                         % ← pivot-frame, no +mp·hLp²
det_M = M11 * M22 - M12^2;

fprintf('  Mass matrix (linearised at upright):\n');
fprintf('    M11 = Jt        = %.4e\n', M11);
fprintf('    M12 = -mp·hLp·Lr = %.4e\n', M12);
fprintf('    M22 = Jp        = %.4e   ← pivot-frame, no parallel axis\n', M22);
fprintf('    det = %.4e\n\n', det_M);

A_tau = [0, 0, 1, 0;
         0, 0, 0, 1;
         0, -M12*mp*hLp*g / det_M,  0,  0;
         0,  M11*mp*hLp*g / det_M,  0,  0];

B_tau = [0;
         0;
          M22 / det_M;
         -M12 / det_M];

%% ====================================================================
%  3. TORQUE → VOLTAGE CONVERSION
%  ====================================================================
A_volt = A_tau;
A_volt(3,3) = A_tau(3,3) - (km^2 / Rm) * B_tau(3);
A_volt(4,3) = A_tau(4,3) - (km^2 / Rm) * B_tau(4);

B_volt = (km / Rm) * B_tau;

fprintf('============================================================\n');
fprintf('  STATE-SPACE WITH VOLTAGE INPUT\n');
fprintf('============================================================\n');
fprintf('A =\n'); disp(A_volt);
fprintf('B = '); fprintf('%.6f  ', B_volt); fprintf('\n\n');

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

% Controllability
Co = ctrb(A_volt, B_volt);
fprintf('Controllability rank: %d / %d  ', rank(Co), size(A_volt,1));
if rank(Co) == size(A_volt,1)
    fprintf('✓\n\n');
else
    fprintf('✗ PROBLEM\n\n');
end

%% ====================================================================
%  4. POLE PLACEMENT
%  ====================================================================
fprintf('============================================================\n');
fprintf('  POLE PLACEMENT\n');
fprintf('============================================================\n\n');

% Unstable OL pole magnitude (for reference)
unstable_pole = max(real(ol_poles));
fprintf('  Unstable OL pole at s = +%.2f rad/s\n', unstable_pole);
fprintf('  → CL poles must be faster than this.\n\n');

poles_quanser      = [-2.8+2.86j, -2.8-2.86j, -30, -40];
poles_moderate     = [-10, -12, -25, -35];
poles_aggressive   = [-15, -20, -40, -50];
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
    fprintf('  K = [%.4f, %.4f, %.4f, %.4f]\n', K(1), K(2), K(3), K(4));
    fprintf('       k_θ       k_α       k_θ̇       k_α̇\n');

    % CL eigenvalues
    cl_eigs = eig(A_volt - B_volt * K);
    fprintf('  CL poles: ');
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
    fprintf('  Voltage for 5° tilt: %.2f V\n\n', abs(-K * x_5deg));
end

%% ====================================================================
%  5. NONLINEAR SIMULATION (pivot-frame EOM)
%  ====================================================================
%
%  IMPORTANT: the simulation EOM must also use the pivot-frame Jp.
%  This means M22 = Jp (not Jp + mp·hLp²), and correspondingly the
%  gravity and Coriolis terms use the pivot-frame expressions.
%
%  See rotary_eom_pivot() at the bottom of this file.

dt = 0.001;
T_sim = 3.0;
N_steps = round(T_sim / dt);
alpha0 = 10 * pi/180;

n_controllers = length(all_pole_sets);
labels = set_names;
K_all = K_results;

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

        state = rk4_pivot(state, u, dt, Jt, mp, Lp, Jp, Lr, g, km, Rm);
    end
end

t = (0:N_steps-1) * dt;

% --- Performance metrics ---
fprintf('============================================================\n');
fprintf('  NONLINEAR SIMULATION  (α₀ = %.0f°, pivot-frame EOM)\n', rad2deg(alpha0));
fprintf('============================================================\n');
fprintf('%25s %12s %12s %10s %10s\n', 'Controller', 'Settle [s]', 'OS [%%]', '|Vm|max', 'α_final');
fprintf('%s\n', repmat('-', 1, 72));

for c = 1:n_controllers
    a_log = squeeze(states_log(2, :, c));
    v_log = Vm_log(:, c);

    band = 0.02 * alpha0;
    idx = find(abs(a_log) > band, 1, 'last');
    if isempty(idx); t_s = 0; else; t_s = idx * dt; end

    peak = min(a_log);
    os = abs(peak) / alpha0 * 100;

    fprintf('%25s %12.3f %12.1f %10.2f %10.4f°\n', ...
            labels{c}, t_s, os, max(abs(v_log)), rad2deg(a_log(end)));
end

% --- Plots ---
figure('Position', [100, 100, 1000, 700]);
colors = lines(n_controllers);

subplot(3,1,1);
for c = 1:n_controllers
    plot(t, rad2deg(squeeze(states_log(2,:,c))), 'LineWidth', 1.5, ...
         'Color', colors(c,:), 'DisplayName', labels{c});
    hold on;
end
yline(0, ':', 'HandleVisibility', 'off');
ylabel('\alpha [deg]'); title(sprintf('Pendulum angle (\\alpha_0 = %.0f°)', rad2deg(alpha0)));
legend('Location', 'best'); grid on;

subplot(3,1,2);
for c = 1:n_controllers
    plot(t, rad2deg(squeeze(states_log(1,:,c))), 'LineWidth', 1.5, ...
         'Color', colors(c,:), 'DisplayName', labels{c});
    hold on;
end
ylabel('\theta [deg]'); title('Arm angle');
legend('Location', 'best'); grid on;

subplot(3,1,3);
for c = 1:n_controllers
    plot(t, Vm_log(:,c), 'LineWidth', 1.5, 'Color', colors(c,:), 'DisplayName', labels{c});
    hold on;
end
yline(6, ':', '6V', 'Color', [0.8 0.2 0.2], 'HandleVisibility', 'off');
yline(-6, ':', '-6V', 'Color', [0.8 0.2 0.2], 'HandleVisibility', 'off');
ylabel('Vm [V]'); xlabel('Time [s]'); title('Control voltage');
legend('Location', 'best'); grid on;

sgtitle(sprintf('Pole Placement — Pivot-Frame (Jr=%.1e, Jp=%.1e)', Jr, Jp), 'FontSize', 14);
saveas(gcf, 'pole_placement_pivot.png');
fprintf('\nSaved: pole_placement_pivot.png\n');

%% ====================================================================
%  LOCAL FUNCTIONS — Pivot-frame EOM
%  ====================================================================
%
%  When Jp is about the PIVOT, the standard Euler-Lagrange gives:
%
%    M11 = Jt + Jp·sin²(α)     ← NOT (Jp+mp·hLp²)·sin²
%    M12 = -mp·hLp·Lr·cos(α)   ← same as before
%    M22 = Jp                   ← NOT Jp+mp·hLp²
%
%    rhs1 = τ - 2·Jp·sin·cos·θ̇·α̇ - mp·hLp·Lr·α̇²·sin
%    rhs2 = mp·hLp·g·sin + Jp·sin·cos·θ̇²
%
%  The gravity term mp·hLp·g·sin(α) is the SAME in both conventions
%  because it comes from the potential energy V = -mp·g·hLp·cos(α),
%  which doesn't involve the inertia at all.
%
%  The Coriolis/centripetal terms involving Jp change because in the
%  CoM convention they use Jp_com everywhere, whereas in the pivot
%  convention they use Jp_pivot. Since the EOM is derived from the
%  Lagrangian L = T - V, and the kinetic energy in the pivot frame
%  uses Jp_pivot directly (no parallel axis in T), the Coriolis
%  terms naturally use Jp_pivot too.
%
%  CRITICAL: if your original EOM was derived with Jp_com and you
%  just swap in Jp_pivot, the Coriolis terms will be wrong.
%  The version below is re-derived for the pivot-frame convention.

function s_new = rk4_pivot(s, Vm, dt, Jt, mp, Lp, Jp, Lr, g, km, Rm)
    k1 = eom_pivot(s,             Vm, Jt, mp, Lp, Jp, Lr, g, km, Rm);
    k2 = eom_pivot(s + 0.5*dt*k1, Vm, Jt, mp, Lp, Jp, Lr, g, km, Rm);
    k3 = eom_pivot(s + 0.5*dt*k2, Vm, Jt, mp, Lp, Jp, Lr, g, km, Rm);
    k4 = eom_pivot(s + dt*k3,     Vm, Jt, mp, Lp, Jp, Lr, g, km, Rm);
    s_new = s + (dt/6) * (k1 + 2*k2 + 2*k3 + k4);
end

function ds = eom_pivot(s, Vm, Jt, mp, Lp, Jp, Lr, g, km, Rm)
    % Full nonlinear EOM with Jp in PIVOT frame
    % State: [theta; alpha; theta_dot; alpha_dot]
    theta_dot = s(3);
    alpha     = s(2);
    alpha_dot = s(4);
    Vm = max(-10, min(10, Vm));

    sa = sin(alpha);  ca = cos(alpha);
    hLp = Lp / 2;

    tau = km * (Vm - km * theta_dot) / Rm;

    % Mass matrix (pivot-frame Jp)
    M11 = Jt + Jp * sa^2;
    M12 = -mp * hLp * Lr * ca;
    M22 = Jp;                      % ← pivot-frame: NO + mp·hLp²

    % Right-hand side
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
