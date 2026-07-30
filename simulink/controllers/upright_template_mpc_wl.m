function [pdotdes, h0_wl, accdes, thrust_desired, roll_torque_desired, ...
    pitch_torque_desired, yaw_torque_desired] = upright_template_mpc_wl( ...
    R_cur, omega, p_cur, v_cur, pdes, dpdes, sdes_in, ...
    I_moment_vec, m, g, cntrl_enable, weights_vec, k_tau, ctrl_Ts, a_ref) %#ok<INUSD>
%#codegen
% Hierarchical variant of upright_template_mpc: same template QP, but the
% I/O is restructured for the MPC -> WLQP cascade of robobee3d
% (template/uprightmpc2 + template/wlcontroller):
%
%   * INPUTS are the raw trajectory reference (pdes, dpdes, sdes) plus the
%     measured state (p_cur, v_cur, R_cur, omega). The position/velocity
%     errors are formed internally instead of upstream.
%   * The PRIMARY OUTPUT is the desired momentum rate pdotdes (with its
%     companion bias term h0_wl), ready to feed wlqp.m:
%         u = wlqp(u_prev, h0_wl, pdotdes, popts, controlRate)
%     The wrench outputs (thrust/torques) are still returned for logging
%     and for running without the WLQP stage, mirroring how umpcUpdate
%     returns both uquad and accdes.
%
% LONG HORIZON + SOLVE DECIMATION
%   The prediction step is TWO wingbeats (dt = 12.9 ms at 155 Hz) and
%   N = 10 -> a 129 ms (~20 wingbeat) preview, long enough for the QP to
%   see tilt -> lateral acceleration -> position consequences, not just
%   the actuation lag. To pay for it, the QP is re-solved only every
%   SOLVE_DECIM controller steps (1 kHz, the rate the robobee3d MPC ran
%   at); between solves the outputs are held and only the applied-moment
%   lag estimate advances at the full 5 kHz.
%   NOTE the frozen linearization (s0, Btau, T0 constant over the horizon)
%   is a hover-regulation assumption; over 129 ms of aggressive transient
%   the tail of the prediction is soft. The unstable pitch mode grows
%   1.387^k along the horizon (x26 at k=10) -- by design, so the QP acts
%   early -- which is also why the solver gets 30 sweeps + warm start.
%
% The desired acceleration is recovered exactly as in
% robobee3d/template/uprightmpc2/uprightmpc2.c (umpcUpdate):
%   dq1des(1:3) = optimal next-step velocity  v1*
%   dq1des(4:6) = e3hat * R' * s1dot*   (lift template s-rate -> body omega)
%   accdes      = (dq1des - dq0)/dt
% and converted to WLQP quantities as in the .withWLQP variant:
%   pdotdes = [ m*R'*accdes_lin ;  Ib.*accdes_ang ]   (BODY frame)
%   h0_wl   = [ R'*[0;0;m*g]    ;  0;0;0 ]            (BODY frame)
% both in template units (mg, mm, ms: force 1e-3 N, torque 1e-6 N*m),
% matching the popts wrench-map fit that wlqp.m consumes.
%
% Outputs
%   pdotdes (6x1) desired momentum rate, body frame, template units
%   h0_wl   (6x1) momentum-dynamics bias (gravity in body frame), template units
%   accdes  (6x1) desired [linear acc (m/s^2); body angular acc (rad/s^2)], SI,
%                 world-frame linear part (for logging / non-WLQP consumers)
%   thrust/roll/pitch/yaw_desired : legacy wrench outputs, SI (N, N*m)
%
% Inputs
%   R_cur   (9x1) rotation matrix (body->world), row-major vectorized
%   omega   (3x1) body angular velocity [rad/s]
%   p_cur   (3x1) position [m]
%   v_cur   (3x1) world linear velocity [m/s]
%   pdes    (3x1) desired position [m]
%   dpdes   (3x1) desired velocity [m/s] (analytic derivative of pdes)
%   sdes_in (3x1) desired body-z (upright) unit vector
%   I_moment_vec, m, g, cntrl_enable, weights_vec(14), k_tau(2), ctrl_Ts:
%                 same as upright_template_mpc (ctrl_Ts kept for signature
%                 compatibility; the internal rates below are hardcoded)
%
% No attitude cascade, yaw not controlled.

% -------------------------------------------------------------------------
% Fixed dimensions for Simulink codegen
% -------------------------------------------------------------------------
R_cur = reshape(R_cur, 9, 1);
omega = reshape(omega, 3, 1);
p_cur = reshape(p_cur, 3, 1);
v_cur = reshape(v_cur, 3, 1);
pdes = reshape(pdes, 3, 1);
dpdes = reshape(dpdes, 3, 1);
sdes_in = reshape(sdes_in, 3, 1);
I_moment_vec = reshape(I_moment_vec, 3, 1);
weights_vec = reshape(weights_vec, 14, 1);
k_tau = reshape(k_tau, 2, 1);
m = m(1);
g = g(1);
cntrl_enable = cntrl_enable(1);
a_ref = reshape(a_ref,3,1); 

% Errors formed internally from the trajectory reference. The (pdes,
% dpdes, sdes) triple carries no acceleration feed-forward, so xdd_des = 0.
e_x = p_cur - pdes;
e_v = v_cur - dpdes;

% -------------------------------------------------------------------------
% MPC constants (template units: mm, ms, mg)
% -------------------------------------------------------------------------
N = 10;         % horizon steps
dt = 12.9;      % [ms] prediction step = TWO wingbeats at 155 Hz
                % N*dt = 129 ms ~ 20 wingbeats of preview
ny = 6;         %#ok<NASGU>
nu = 3;
nx = 2*6 + 2;   % [y_err; dy_err; tauX_applied; tauY_applied]
nU = nu*N;

controller_dt = 0.2;   % [ms] Simulink sample time of this block (2e-4 s)
tau_lag = 16;          % [ms] moment actuation lag (system-id: dead+lag)

SOLVE_DECIM = 5;       % controller steps per QP solve (5 x 0.2 ms = 1 ms,
                       % the rate the robobee3d template MPC ran at)

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
wdthrust    = weights_vec(14); % commanded specific-thrust-change penalty

% Actuator limits (SI). With the WLQP downstream these boxes no longer
% clip the real actuators (wlqp.m has its own umin/umax/dumax on the
% voltage-level inputs); here they bound the template model's inputs so
% the QP only asks for accelerations the vehicle can plausibly deliver.
thrust_min_N = 0.8e-3;   % [N]
thrust_max_N = 1.6e-3;   % [N]
roll_max_Nm  = 15e-6;    % [N*m]
pitch_max_Nm = 10.0e-6;  % [N*m]

% -------------------------------------------------------------------------
% Unit conversion to template units
% -------------------------------------------------------------------------
m_t = m * 1.0e6;                 % kg      -> mg
g_t = g * 1.0e-3;                % m/s^2   -> mm/ms^2
ex_t = e_x * 1.0e3;              % m       -> mm
ev_t = e_v;                      % m/s == mm/ms
om_t = omega * 1.0e-3;           % rad/s   -> rad/ms
dpdes_t = dpdes;                 % m/s == mm/ms
v_t = v_cur;                     % m/s == mm/ms
Ib_t = I_moment_vec * 1.0e12;    % kg*m^2  -> mg*mm^2

Tmin_sp = thrust_min_N * 1.0e3 / max(m_t, 1.0e-9);  % specific thrust [mm/ms^2]
Tmax_sp = thrust_max_N * 1.0e3 / max(m_t, 1.0e-9);
tau_roll_max  = roll_max_Nm  * 1.0e6;   % N*m -> mg*mm^2/ms^2
tau_pitch_max = pitch_max_Nm * 1.0e6;

% -------------------------------------------------------------------------
% Persistent controller state: thrust linearization point T0, the estimated
% APPLIED moments (hidden actuator states behind the lag), warm start, the
% solve-decimation counter, and the held WLQP outputs between solves.
% -------------------------------------------------------------------------
persistent T0 tau_applied thrust_cmd_prev_sp tau_cmd_prev U_prev ...
    solve_tick pdotdes_hold accdes_hold
if isempty(T0)
    T0 = 0.0;
end
if isempty(tau_applied)
    tau_applied = zeros(2,1);
end
if isempty(thrust_cmd_prev_sp)
    thrust_cmd_prev_sp = 0.0;
end
if isempty(tau_cmd_prev)
    tau_cmd_prev = zeros(2,1);
end
if isempty(U_prev)
    U_prev = zeros(nU,1);
end
if isempty(solve_tick)
    solve_tick = 0.0;
end
if isempty(pdotdes_hold)
    pdotdes_hold = zeros(6,1);
end
if isempty(accdes_hold)
    accdes_hold = zeros(6,1);
end

% -------------------------------------------------------------------------
% Soft-start envelope (see upright_template_mpc.m): the engagement ramp
% scales the QP's own constraint boxes so the optimizer sees the limits.
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
    thrust_cmd_prev_sp = 0.0;
    tau_cmd_prev = zeros(2,1);
    U_prev = zeros(nU,1);
    solve_tick = 0.0;
    pdotdes_hold = zeros(6,1);
    accdes_hold = zeros(6,1);
end
T_ceiling = en * Tmax_sp;                      % thrust ceiling rides the ramp
T_floor   = min(Tmin_sp, T_ceiling);           % floor engages once ceiling passes it
tau_roll_eff  = (0.3 + 0.7*en) * tau_roll_max; % torque boxes open 30% -> 100%
tau_pitch_eff = (0.3 + 0.7*en) * tau_pitch_max;

% -------------------------------------------------------------------------
% Current reduced-attitude state (cheap; needed every call for h0_wl)
% -------------------------------------------------------------------------
Rot = [R_cur(1), R_cur(2), R_cur(3); ...
       R_cur(4), R_cur(5), R_cur(6); ...
       R_cur(7), R_cur(8), R_cur(9)];

s0 = [R_cur(3); R_cur(6); R_cur(9)];          % R*e3, third column
sdes = sdes_in / max(norm(sdes_in), 1.0e-9);  % desired body-z, normalized

e3h = [0, -1, 0; 1, 0, 0; 0, 0, 0];           % hat([0;0;1])
Ibi = [1/max(Ib_t(1),1e-9), 0, 0; 0, 1/max(Ib_t(2),1e-9), 0; 0, 0, 1/max(Ib_t(3),1e-9)];

k_tau_roll = k_tau(1);
k_tau_pitch = k_tau(2);
BtauFull = -Rot * e3h * Ibi;
Btau = BtauFull(:, 1:2) * diag([k_tau_roll, k_tau_pitch]);   % no yaw torque column

ds0 = -Rot * e3h * om_t;                      % s-rate from body omega

% -------------------------------------------------------------------------
% Solve decimation gate
% -------------------------------------------------------------------------
do_solve = (solve_tick <= 0.5);
if do_solve
    solve_tick = SOLVE_DECIM - 1;
else
    solve_tick = solve_tick - 1;
end

if do_solve

% Error state x0 = [e_p; e_s; e_v; e_ds; tau_applied], refs: sdes, ds_des=0
es0 = s0 - sdes;
x0 = [ex_t; es0; ev_t; ds0; tau_applied];

% Affine drift in error coordinates (no trajectory accel feed-forward):
% e_v_dot = T0*e_s + s0*utilde + (T0*sdes - g*e3)
aref_t = a_ref * 1.0e-3; 
d_v = T0*sdes - [0;0;g_t] - aref_t;

% -------------------------------------------------------------------------
% Discrete error dynamics  x_{k+1} = Ad*x_k + Bd*u_k + cd
% indices: e_p 1:3, e_s 4:6, e_v 7:9, e_ds 10:12, tau_applied 13:14
% -------------------------------------------------------------------------
beta = 1.0 - exp(-dt / tau_lag);          % first-order lag step fraction
if beta > 1.0
    beta = 1.0;
end

% Per-axis rotational damping from open-loop step ID (b in 1/s):
%   roll  (omega_x):  b_roll  = +47   real wing counter-torque (stable)
%   pitch (omega_y):  b_pitch = -30   ANTI-damped -> unstable open-loop mode
% Reduced-attitude swap (ds = -R*e3hat*omega): ds_x = omega_y (pitch) -> row 10,
% ds_y = -omega_x (roll) -> row 11. So pitch damping goes on Ad(10,10).
b_roll  =  30.0;
b_pitch = -30.0;
decay_pitch = 1.0 - b_pitch * (dt*1.0e-3);   % ds_x / state 10  (= 1.387, >1)
decay_roll  = 1.0 - b_roll  * (dt*1.0e-3);   % ds_y / state 11  (= 0.394)
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
Ad(10,10) = decay_pitch;
Ad(11,11) = decay_roll;
Ad(12,12) = 1.0;          % ds_z ~ 0 at hover; no ID data, leave undamped
% e_ds += dt*Btau*tau_applied  (moments act through the lag state)
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
% Input-change penalties: first move compared against previous issued
% command, then successive differences along the horizon.
% QP convention: min 0.5*U'*H*U + h'*U.
% -------------------------------------------------------------------------
utilde_prev_ref = thrust_cmd_prev_sp - T0;
for k = 1:N
    idx_thrust = (k-1)*nu + 1;
    idx_roll  = (k-1)*nu + 2;
    idx_pitch = (k-1)*nu + 3;

    if k == 1
        H(idx_thrust,idx_thrust) = ...
            H(idx_thrust,idx_thrust) + wdthrust;
        h(idx_thrust) = h(idx_thrust) - wdthrust*utilde_prev_ref;

        H(idx_roll,idx_roll) = H(idx_roll,idx_roll) + wdmom_roll;
        h(idx_roll) = h(idx_roll) - wdmom_roll*tau_cmd_prev(1);

        H(idx_pitch,idx_pitch) = H(idx_pitch,idx_pitch) + wdmom_pitch;
        h(idx_pitch) = h(idx_pitch) - wdmom_pitch*tau_cmd_prev(2);
    else
        idx_thrust_prev = idx_thrust - nu;
        idx_roll_prev  = idx_roll  - nu;
        idx_pitch_prev = idx_pitch - nu;

        H(idx_thrust,idx_thrust) = ...
            H(idx_thrust,idx_thrust) + wdthrust;
        H(idx_thrust_prev,idx_thrust_prev) = ...
            H(idx_thrust_prev,idx_thrust_prev) + wdthrust;
        H(idx_thrust,idx_thrust_prev) = ...
            H(idx_thrust,idx_thrust_prev) - wdthrust;
        H(idx_thrust_prev,idx_thrust) = ...
            H(idx_thrust_prev,idx_thrust) - wdthrust;

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
% Solve (warm-started from previous solution)
% -------------------------------------------------------------------------
U = solve_box_qp_coordinate_descent(H, h, Umin, Umax, nU, U_prev);
U_prev = U;

% -------------------------------------------------------------------------
% Apply first input; advance linearization point (template: T0 += utilde0)
% -------------------------------------------------------------------------
utilde0 = U(1);
tau_x_t = U(2);
tau_y_t = U(3);

% Predicted next error state under the first input -- this is the QP's
% dy1des in condensed form, and everything the WLQP needs comes from it.
x1 = Ad*x0 + Bd*[utilde0; tau_x_t; tau_y_t] + cd;

T0 = T0 + utilde0;
if T0 < T_floor
    T0 = T_floor;
elseif T0 > T_ceiling
    T0 = T_ceiling;
end
thrust_cmd_prev_sp = T0;

tau_cmd_prev(1) = tau_x_t;
tau_cmd_prev(2) = tau_y_t;

% -------------------------------------------------------------------------
% WLQP interface outputs (the umpcUpdate accdes recipe, condensed form),
% computed at the solve rate and held between solves.
% -------------------------------------------------------------------------
% Absolute desired velocity at step 1: error state + reference
v1des_t = x1(7:9) + dpdes_t;       % [mm/ms]  world frame
ds1des_t = x1(10:12);              % [1/ms]   ds reference is 0, error = absolute

% Lift template s-rate to body angular velocity: omega_des = e3h*R'*ds_des
om1des_t = e3h * (Rot' * ds1des_t);   %#ok<NASGU> % kept for reference

% Linear accdes = (v1des - v)/dt, template units. dt is one (2-wingbeat)
% MPC step, so this is the average acceleration demanded over 12.9 ms.
% (Thrust enters x1's velocity row DIRECTLY, so this carries the command.)
acc_lin_t = (v1des_t - v_t) / dt;     % [mm/ms^2] world frame

% ANGULAR demand: with the actuation-lag state in the model, the commanded
% torque only reaches the predicted ds at step 2, so the umpcUpdate
% (ds1des - ds0)/dt recipe returns ~zero moment demand at rest tilt and
% the WLQP would get rate damping but NO attitude stiffness. The angular
% momentum rate the MPC wants IS its commanded moment -- pass it directly.
acc_ang_t = [tau_x_t / max(Ib_t(1),1e-9); ...
             tau_y_t / max(Ib_t(2),1e-9); ...
             0.0];                    % [rad/ms^2] body frame, yaw = 0

% Momentum rate for the WLQP, BODY frame (wrench map & h0 live there),
% template units: [mg*mm/ms^2 (=1e-3 N); mg*mm^2/ms^2 (=1e-6 N*m)]
pdotdes_hold = [m_t * (Rot' * acc_lin_t); tau_x_t; tau_y_t; 0.0];

% SI accdes for logging / non-WLQP consumers
accdes_hold = [acc_lin_t * 1.0e3; acc_ang_t * 1.0e6];   % [m/s^2; rad/s^2]

end % do_solve

% -------------------------------------------------------------------------
% Every call (5 kHz): propagate the applied-moment lag estimate toward the
% last issued command, refresh the gravity bias with the current attitude,
% and emit the (held) solve outputs.
% -------------------------------------------------------------------------
beta_c = 1.0 - exp(-controller_dt / tau_lag);
tau_applied(1) = tau_applied(1) + beta_c*(tau_cmd_prev(1) - tau_applied(1));
tau_applied(2) = tau_applied(2) + beta_c*(tau_cmd_prev(2) - tau_applied(2));

pdotdes = pdotdes_hold;
accdes = accdes_hold;

% Gravity bias in body frame (cf. conn_MPC_WL / h0B = R'*h0W) -- cheap, so
% it tracks the live attitude even between solves.
h0_wl = [Rot' * [0; 0; m_t*g_t]; 0; 0; 0];

% -------------------------------------------------------------------------
% Legacy wrench outputs (SI) -- logging / running without the WLQP stage
% -------------------------------------------------------------------------
thrust_desired = m_t * T0 * 1.0e-3;            % mg*mm/ms^2 -> N
roll_torque_desired  = tau_cmd_prev(1) * 1.0e-6;  % mg*mm^2/ms^2 -> N*m
pitch_torque_desired = tau_cmd_prev(2) * 1.0e-6;
yaw_torque_desired   = 0.0;                    % yaw not controlled

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
% Warm-start from the previous MPC solution.
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
% Fixed iteration count keeps generated code deterministic. 30 sweeps for
% the longer-horizon QP (nU = 30, unstable pitch mode worsens conditioning);
% the 1 kHz warm-started replan makes each solve's remaining error small.
for iter = 1:30
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
