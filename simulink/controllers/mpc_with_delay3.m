function [thrust_desired, roll_torque_desired, pitch_torque_desired, yaw_torque_desired] = upright_template_mpc(R_cur, s_des, omega, omega_desired, omega_d_dot, x_d_ddot, e_x, e_v, e_R, e_Omega, control_gain, I_moment_vec, m, g, cntrl_enable, adaptive_pitch, adaptive_yaw, adaptive_x, adaptive_y, adaptive_z, weights_vec, k_tau)
% vofunction [thrust_desired, roll_torque_desired, pitch_torque_desired, yaw_torque_desired] = ...
%     upright_template_mpc(R_cur,s_des, omega, omega_desired, omega_d_dot, ...
%     x_d_ddot,e_x,e_v,e_R,e_Omega,control_gain,I_moment_vec, m, g, cntrl_enable, ...
%     adaptive_pitch, adaptive_yaw, adaptive_x,adaptive_y, adaptive_z,weights_vec, ...
%     k_tau)
%#codegen
% Monolithic "upright MPC" (template controller), after Chen et al.
% robobee3d/template/template_controllers.py :: UprightMPC2, augmented with
% a first-order torque-actuation lag model measured on THIS plant
% (system-id: pitch moment develops with ~6 ms dead time + ~6-15 ms lag;
% approximated here as a single 12 ms first-order lag on both moment axes).
%
% One QP, solved every controller step, directly outputs thrust and
% roll/pitch torques from the current state and the references
% (desired position via e_x/e_v, desired body-z vector via the s_des
% input). No attitude cascade, no heading (b_1_d) reference. Yaw is NOT
% controlled: yaw torque = 0.
%
% Template model (reduced attitude, yaw-free):
%   y  = [p; s],   s  = R*e3   (body z-axis in world frame)
%   dy = [v; ds],  ds = -R*e3hat*omega_body
%   v_dot  = T*s - g*e3  ~ T0*s + s0*utilde - g*e3   (bilinear, linearized
%                                                     about persistent T0
%                                                     and current s0)
%   ds_dot = Btau*[tauX_applied; tauY_applied]
%   tau_applied_dot = (tau_cmd - tau_applied)/tau_lag   (actuation lag)
%   u = [utilde; tauX_cmd; tauY_cmd]
%
% The QP is condensed to a dense box-constrained problem in U (nu*N vars)
% and solved with the same deterministic coordinate-descent used by the
% previous controllers (OSQP is not available under codegen here).
%
% Internally everything runs in the template's units (mm, ms, mg):
%   force:  1 mg*mm/ms^2  = 1e-3 N
%   torque: 1 mg*mm^2/ms^2 = 1e-6 N*m
%
% Unused inputs (kept for block signature compatibility): omega_desired,
% omega_d_dot, e_R, e_Omega, control_gain, adaptive_*.

% -------------------------------------------------------------------------
% Fixed dimensions for Simulink codegen
% -------------------------------------------------------------------------
R_cur = reshape(R_cur, 9, 1);
s_des = reshape(s_des, 3, 1);
omega = reshape(omega, 3, 1);
omega_desired = reshape(omega_desired, 3, 1);     %#ok<NASGU>
omega_d_dot = reshape(omega_d_dot, 3, 1);         %#ok<NASGU>
x_d_ddot = reshape(x_d_ddot, 3, 1);
e_x = reshape(e_x, 3, 1);
e_v = reshape(e_v, 3, 1);
e_R = reshape(e_R, 3, 1);                         %#ok<NASGU>
e_Omega = reshape(e_Omega, 3, 1);                 %#ok<NASGU>
control_gain = reshape(control_gain, 10, 1);      %#ok<NASGU>
I_moment_vec = reshape(I_moment_vec, 3, 1);
weights_vec = reshape(weights_vec, 13, 1);
k_tau = reshape(k_tau, 2, 1);
m = m(1);
g = g(1);
cntrl_enable = cntrl_enable(1);
adaptive_pitch = adaptive_pitch(1);               %#ok<NASGU>
adaptive_yaw = adaptive_yaw(1);                   %#ok<NASGU>
adaptive_x = adaptive_x(1);                       %#ok<NASGU>
adaptive_y = adaptive_y(1);                       %#ok<NASGU>
adaptive_z = adaptive_z(1);                       %#ok<NASGU>

% -------------------------------------------------------------------------
% MPC constants (template units: mm, ms, mg)
% -------------------------------------------------------------------------
N = 8;          % horizon steps (template default 3; 8 -> 40 ms preview,
                % enough to see through the 12 ms actuation lag)
dt = 6.45;       % [ms] prediction step (template default). 
ny = 6;
nu = 3;
nx = 2*ny + 2;  % [y_err; dy_err; tauX_applied; tauY_applied]
nU = nu*N;

controller_dt = 0.2;   % [ms] Simulink sample time of this block (2e-4 s)
% tau_lag = (1/155)*1e3*0.1;        % [ms] moment actuation lag (system-id: dead+lag)
% tau_lag = 12; 
% tau_lag = 12.9 / (-log(0.05));  % = 4.306 ms
tau_lag = 10; 

% Template-derived weights, retuned for THIS plant (probe-based scan
% matched to the takeoff-transient torque levels the proven geometric/
% delay-MPC controller used: ~1 uNm at 10-20 deg tilt wobble). With the
% vehicle's huge torque authority (15 uNm ~ 7400 rad/s^2) and the ~18 ms
% effective moment actuation delay, the template's wmom=1e-2 makes torque
% free and commands bang-bang between the rails (flips the bee at
% takeoff); stiff variants (wmom=1) still oscillated through the delay.
% These values give ~0.7 uNm per 0.2 rad tilt, ~0.4 uNm per 5 rad/s rate,
% ~0.15 uNm per 2 cm/s lateral velocity error.
% ws      = 3.0e1;    % running orientation-vector weight
% wds     = 1.5e2;    % running orientation-rate weight
% wpr     = 1.0;      % running position weight
% wpf     = 5.0;      % final position weight
% wvr     = 1.0e3;    % running velocity weight
% wvf     = 2.0e3;    % final velocity weight
% wthrust = 1.0e-1;   % specific-thrust-correction effort
% wmom    = 3.0e0;    % torque effort

ws      = weights_vec(1);   % running orientation-vector weight
wds     = weights_vec(2);   % running orientation-rate weight
wpr_xy  = weights_vec(3);   % running position weight
wpr_z   = weights_vec(4);
wpf     = weights_vec(5);   % final position weight
wvr_xy  = weights_vec(6);   % running horizontal-velocity weight
wvr_z   = weights_vec(7);   % running vertical-velocity weight
wvf_xy  = weights_vec(8);   % final horizontal-velocity weight
wvf_z   = weights_vec(9);   % final vertical-velocity weight
wthrust     = weights_vec(10); % specific-thrust-correction effort
wmom        = weights_vec(11); % roll/pitch torque magnitude effort
wdmom_roll  = weights_vec(12); % roll torque-change penalty
wdmom_pitch = weights_vec(13); % pitch torque-change penalty


% Actuator limits (SI, same as previous controllers)
thrust_min_N = 0.8e-3;   % [N]
thrust_max_N = 1.6e-3;   % [N]
roll_max_Nm  = 15e-6;   %  [N*m]
pitch_max_Nm = 10.0e-6;   % [N*m]

% -------------------------------------------------------------------------
% Unit conversion to template units
% -------------------------------------------------------------------------
m_t = m * 1.0e6;                 % kg      -> mg
g_t = g * 1.0e-3;                % m/s^2   -> mm/ms^2
ex_t = e_x * 1.0e3;              % m       -> mm
ev_t = e_v;                      % m/s == mm/ms
om_t = omega * 1.0e-3;           % rad/s   -> rad/ms
xdd_t = x_d_ddot * 1.0e-3;       % m/s^2   -> mm/ms^2
Ib_t = I_moment_vec * 1.0e12;    % kg*m^2  -> mg*mm^2

Tmin_sp = thrust_min_N * 1.0e3 / max(m_t, 1.0e-9);  % specific thrust [mm/ms^2]
Tmax_sp = thrust_max_N * 1.0e3 / max(m_t, 1.0e-9);
tau_roll_max  = roll_max_Nm  * 1.0e6;   % N*m -> mg*mm^2/ms^2
tau_pitch_max = pitch_max_Nm * 1.0e6;

% -------------------------------------------------------------------------
% Persistent controller state: thrust linearization point T0 and the
% estimated APPLIED moments (hidden actuator states behind the lag)
% -------------------------------------------------------------------------
persistent T0 tau_applied tau_cmd_prev U_prev
if isempty(T0)
    T0 = 0.0;
end
if isempty(tau_applied)
    tau_applied = zeros(2,1);
end
if isempty(tau_cmd_prev)
    tau_cmd_prev = zeros(2,1);
end
if isempty(U_prev)
    U_prev = zeros(nU,1);
end

% -------------------------------------------------------------------------
% Soft-start envelope: cntrl_enable is the model's engagement ramp (the
% Cntrl_enable saturating ramp in Open_Closed_loop_selection, 0 -> 1).
% Rather than rate-limiting outputs externally (which the QP could not see
% and would fight), the ramp scales the QP's own constraint boxes: the
% thrust ceiling follows the ramp from zero and the torque boxes open from
% 30% to 100%. While disabled the controller state is held at rest so
% engagement starts bumplessly.
% -------------------------------------------------------------------------
en = cntrl_enable;
if en < 0.0
    en = 0.0;
elseif en > 1.0
    en = 1.0;
end
if en < 0.01
    T0 = 0.0;
    tau_applied = zeros(2,1);
    tau_cmd_prev = zeros(2,1);
    U_prev = zeros(nU,1);
end
T_ceiling = en * Tmax_sp;                      % thrust ceiling rides the ramp
T_floor   = min(Tmin_sp, T_ceiling);           % floor engages once ceiling passes it
tau_roll_eff  = (0.3 + 0.7*en) * tau_roll_max; % torque boxes open 30% -> 100%
tau_pitch_eff = (0.3 + 0.7*en) * tau_pitch_max;

% -------------------------------------------------------------------------
% Current reduced-attitude state
% -------------------------------------------------------------------------
Rot = [R_cur(1), R_cur(2), R_cur(3); ...
       R_cur(4), R_cur(5), R_cur(6); ...
       R_cur(7), R_cur(8), R_cur(9)];

s0 = [R_cur(3); R_cur(6); R_cur(9)];          % R*e3, third column
sdes = s_des / max(norm(s_des), 1.0e-9);      % desired body-z, normalized

e3h = [0, -1, 0; 1, 0, 0; 0, 0, 0];           % hat([0;0;1])
Ibi = [1/max(Ib_t(1),1e-9), 0, 0; 0, 1/max(Ib_t(2),1e-9), 0; 0, 0, 1/max(Ib_t(3),1e-9)];

k_tau_roll = k_tau(1); 
k_tau_pitch = k_tau(2); 
BtauFull = -Rot * e3h * Ibi;
Btau = BtauFull(:, 1:2) * diag([k_tau_roll, k_tau_pitch]);                      % no yaw torque column

ds0 = -Rot * e3h * om_t;                      % s-rate from body omega

% Error state x0 = [e_p; e_s; e_v; e_ds; tau_applied], refs: sdes, ds_des=0
es0 = s0 - sdes;
x0 = [ex_t; es0; ev_t; ds0; tau_applied];

% Affine drift in error coordinates:
% e_v_dot = T0*e_s + s0*utilde + (T0*sdes - g*e3 - xdd_des)
d_v = T0*sdes - [0;0;g_t] - xdd_t;

% -------------------------------------------------------------------------
% Discrete error dynamics  x_{k+1} = Ad*x_k + Bd*u_k + cd
% indices: e_p 1:3, e_s 4:6, e_v 7:9, e_ds 10:12, tau_applied 13:14
% -------------------------------------------------------------------------
beta = 1.0 - exp(-dt / tau_lag);          % first-order lag step fraction (Euler)
if beta > 1.0
    beta = 1.0;
end

% Per-axis rotational damping from open-loop step ID (b in 1/s):
%   roll  (omega_x):  b_roll  = +47   real wing counter-torque (stable)
%   pitch (omega_y):  b_pitch = -30   ANTI-damped -> unstable open-loop mode
% Reduced-attitude swap (ds = -R*e3hat*omega): ds_x = omega_y (pitch) -> row 10,
% ds_y = -omega_x (roll) -> row 11. So pitch damping goes on Ad(10,10).
b_roll  =  47.0;
b_pitch = -30.0;
decay_pitch = 1.0 - b_pitch * (dt*1.0e-3);   % ds_x / state 10  (= 1.19, >1)
decay_roll  = 1.0 - b_roll  * (dt*1.0e-3);   % ds_y / state 11  (= 0.70)
% CRITICAL: do NOT clamp the pitch value down to 1. The pitch mode really
% grows, and the MPC must predict that to command torque early enough.
decay_pitch = min(max(decay_pitch, 0.0), 1.5);   % allow >1, cap for safety
decay_roll  = min(max(decay_roll,  0.0), 1.5);


Ad = eye(nx);
for i = 1:3
    Ad(i, 6+i) = dt;                           % e_p += dt*e_v
    Ad(3+i, 9+i) = dt;                         % e_s += dt*e_ds
    Ad(6+i, 3+i) = dt*T0;                      % e_v += dt*T0*e_s
end
% e_ds += dt*Btau*tau_applied  (moments act through the lag state)

% Ad(10,10) = rot_decay; 
% Ad(11,11) = rot_decay; 
% Ad(12,12) = rot_decay; 
Ad(10,10) = decay_pitch;
Ad(11,11) = decay_roll;
Ad(12,12) = 1.0;          % ds_z ~ 0 at hover; no ID data, leave undamped

Ad(10, 13) = dt*Btau(1,1);  Ad(10, 14) = dt*Btau(1,2);
Ad(11, 13) = dt*Btau(2,1);  Ad(11, 14) = dt*Btau(2,2);
Ad(12, 13) = dt*Btau(3,1);  Ad(12, 14) = dt*Btau(3,2);
% tau_applied += beta*(tau_cmd - tau_applied)
Ad(13, 13) = 1.0 - beta;
Ad(14, 14) = 1.0 - beta;

Bd = zeros(nx, nu);
Bd(7, 1) = dt*s0(1);                           % thrust correction -> e_v
Bd(8, 1) = dt*s0(2);
Bd(9, 1) = dt*s0(3);
Bd(13, 2) = beta;                              % tau commands -> lag states
Bd(14, 3) = beta;

cd = zeros(nx,1);
cd(7) = dt*d_v(1);
cd(8) = dt*d_v(2);
cd(9) = dt*d_v(3);

% -------------------------------------------------------------------------
% Stage weights: x = [e_p(3); e_s(3); e_v(3); e_ds(3); tau_app(2)]
% -------------------------------------------------------------------------
Qrun = [wpr_xy; wpr_xy; wpr_z; ...
        ws; ws; ws; ...
        wvr_xy; wvr_xy; wvr_z; ...
        wds; wds; wds; ...
        0; 0];

Qfin = [wpf; wpf; wpf; ...
        ws; ws; ws; ...
        wvf_xy; wvf_xy; wvf_z; ...
        wds; wds; wds; ...
        0; 0];
Rdiag = [wthrust; wmom; wmom];

% -------------------------------------------------------------------------
% Condense:  x_k = Phi_k*x0 + sum_j G(k,j)*u_j + w_k,  k = 1..N
% H = sum_k G_k' Q_k G_k + Rbar,  h = sum_k G_k' Q_k (Phi_k x0 + w_k)
% -------------------------------------------------------------------------
Gamma = zeros(nx*N, nU);
xfree = zeros(nx*N, 1);

for k = 1:N
    % free response (with affine drift)
    if k == 1
        xfree((k-1)*nx+1:k*nx) = Ad*x0 + cd;
    else
        xfree((k-1)*nx+1:k*nx) = Ad*xfree((k-2)*nx+1:(k-1)*nx) + cd;
    end
    % input response blocks: G(k,j) = Ad^(k-j-1)*Bd for j = 0..k-1
    for j = 1:k
        if j == k
            Gblk = Bd;
        else
            Gprev = Gamma((k-2)*nx+1:(k-1)*nx, (j-1)*nu+1:j*nu);
            Gblk = Ad*Gprev;
        end
        Gamma((k-1)*nx+1:k*nx, (j-1)*nu+1:j*nu) = Gblk;
    end
end

H = zeros(nU, nU);
h = zeros(nU, 1);
for k = 1:N
    if k == N
        Qk = Qfin;
    else
        Qk = Qrun;
    end
    Gk = Gamma((k-1)*nx+1:k*nx, :);
    xf = xfree((k-1)*nx+1:k*nx);
    GkQ = Gk' .* repmat(Qk', nU, 1);       % Gk' * diag(Qk)
    H = H + GkQ * Gk;
    h = h + GkQ * xf;
end
for k = 1:N
    base = (k-1)*nu;
    H(base+1, base+1) = H(base+1, base+1) + Rdiag(1);
    H(base+2, base+2) = H(base+2, base+2) + Rdiag(2);
    H(base+3, base+3) = H(base+3, base+3) + Rdiag(3);
end

% -------------------------------------------------------------------------
% Torque-change penalty:
%   0.5*wdmom_roll *(tau_x,0 - tau_x,prev)^2
% + 0.5*wdmom_pitch*(tau_y,0 - tau_y,prev)^2
% + sum 0.5*w*(tau_k - tau_k-1)^2 over the horizon.
% QP convention: min 0.5*U'*H*U + h'*U.
% -------------------------------------------------------------------------
for k = 1:N
    idx_roll  = (k-1)*nu + 2;
    idx_pitch = (k-1)*nu + 3;

    if k == 1
        H(idx_roll,idx_roll) = H(idx_roll,idx_roll) + wdmom_roll;
        h(idx_roll) = h(idx_roll) - wdmom_roll*tau_cmd_prev(1);

        H(idx_pitch,idx_pitch) = H(idx_pitch,idx_pitch) + wdmom_pitch;
        h(idx_pitch) = h(idx_pitch) - wdmom_pitch*tau_cmd_prev(2);
    else
        idx_roll_prev  = idx_roll  - nu;
        idx_pitch_prev = idx_pitch - nu;

        H(idx_roll,idx_roll) = H(idx_roll,idx_roll) + wdmom_roll;
        H(idx_roll_prev,idx_roll_prev) = ...
            H(idx_roll_prev,idx_roll_prev) + wdmom_roll;
        H(idx_roll,idx_roll_prev) = H(idx_roll,idx_roll_prev) - wdmom_roll;
        H(idx_roll_prev,idx_roll) = H(idx_roll_prev,idx_roll) - wdmom_roll;

        H(idx_pitch,idx_pitch) = H(idx_pitch,idx_pitch) + wdmom_pitch;
        H(idx_pitch_prev,idx_pitch_prev) = ...
            H(idx_pitch_prev,idx_pitch_prev) + wdmom_pitch;
        H(idx_pitch,idx_pitch_prev) = H(idx_pitch,idx_pitch_prev) - wdmom_pitch;
        H(idx_pitch_prev,idx_pitch) = H(idx_pitch_prev,idx_pitch) - wdmom_pitch;
    end
end

% -------------------------------------------------------------------------
% Box constraints per stage: thrust lims on utilde, actuator lims on torques
% -------------------------------------------------------------------------
Umin = zeros(nU,1);
Umax = zeros(nU,1);
for k = 1:N
    base = (k-1)*nu;
    Umin(base+1) = T_floor - T0;
    Umax(base+1) = T_ceiling - T0;
    Umin(base+2) = -tau_roll_eff;
    Umax(base+2) =  tau_roll_eff;
    Umin(base+3) = -tau_pitch_eff;
    Umax(base+3) =  tau_pitch_eff;
end

% -------------------------------------------------------------------------
% Receding-horizon warm start
%
% Previous solution:
%   U_prev = [u_0; u_1; ...; u_(N-1)]
%
% Initial guess for the new QP:
%   U_init = [u_1; u_2; ...; u_(N-1); u_(N-1)]
%
% This warm-starts the decision vector used by coordinate descent. The
% gradient is still recomputed from the current H, h, and current iterate.
% -------------------------------------------------------------------------
% U_init = zeros(nU,1);
% for k = 1:N-1
%     dst_base = (k-1)*nu;
%     src_base = k*nu;
% 
%     U_init(dst_base+1) = U_prev(src_base+1);
%     U_init(dst_base+2) = U_prev(src_base+2);
%     U_init(dst_base+3) = U_prev(src_base+3);
% end
% 
% % Repeat the previous terminal input at the end of the shifted sequence.
% last_base = (N-1)*nu;
% U_init(last_base+1) = U_prev(last_base+1);
% U_init(last_base+2) = U_prev(last_base+2);
% U_init(last_base+3) = U_prev(last_base+3);

U_init = U_prev; 

U = solve_box_qp_coordinate_descent( ...
    H, h, Umin, Umax, nU, U_init);

% Save the optimized horizon for the next MPC update.
U_prev = U;

% -------------------------------------------------------------------------
% Apply first input; advance linearization point (template: T0 += utilde0)
% and propagate the applied-moment estimate at the CONTROLLER rate
% -------------------------------------------------------------------------
utilde0 = U(1);
tau_x_t = U(2);
tau_y_t = U(3);

T0 = T0 + utilde0;
if T0 < T_floor
    T0 = T_floor;
elseif T0 > T_ceiling
    T0 = T_ceiling;
end

beta_c = 1.0 - exp(-controller_dt / tau_lag);
tau_applied(1) = tau_applied(1) + beta_c*(tau_x_t - tau_applied(1));
tau_applied(2) = tau_applied(2) + beta_c*(tau_y_t - tau_applied(2));

tau_cmd_prev(1) = tau_x_t;
tau_cmd_prev(2) = tau_y_t;

% Convert back to SI
thrust_desired = m_t * T0 * 1.0e-3;            % mg*mm/ms^2 -> N
roll_torque_desired  = tau_x_t * 1.0e-6;       % mg*mm^2/ms^2 -> N*m
pitch_torque_desired = tau_y_t * 1.0e-6;
yaw_torque_desired   = 0.0;                    % yaw not controlled

% Final SI saturations (redundant with QP boxes; cheap insurance)
thrust_desired = clamp_scalar(thrust_desired, 0.0, thrust_max_N);
roll_torque_desired = clamp_scalar(roll_torque_desired, -roll_max_Nm, roll_max_Nm);
pitch_torque_desired = clamp_scalar(pitch_torque_desired, -pitch_max_Nm, pitch_max_Nm);

end

function y = clamp_scalar(x, xmin, xmax)
%#codegen
y = x;
if y > xmax
    y = xmax;
elseif y < xmin
    y = xmin;
end
end

function U = solve_box_qp_coordinate_descent( ...
    H, h, Umin, Umax, nU, U_init)
%#codegen
% min 0.5*U'*H*U + h'*U  s.t. Umin <= U <= Umax
%
% Warm-start from the shifted solution of the previous MPC update.
U = reshape(U_init, nU, 1);

% Project the warm start into the current feasible box because the bounds
% can change with the engagement ramp and thrust linearization point.
for i = 1:nU
    if U(i) < Umin(i)
        U(i) = Umin(i);
    elseif U(i) > Umax(i)
        U(i) = Umax(i);
    end
end
% Fixed iteration count keeps generated code deterministic.
for iter = 1:20
    for i = 1:nU
        grad_i = h(i);
        for j = 1:nU
            grad_i = grad_i + H(i,j) * U(j);
        end
        U(i) = U(i) - grad_i / max(H(i,i), 1.0e-18);
        if U(i) < Umin(i)
            U(i) = Umin(i);
        elseif U(i) > Umax(i)
            U(i) = Umax(i);
        end
    end
end
end
