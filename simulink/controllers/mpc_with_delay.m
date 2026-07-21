function [thrust_desired, roll_torque_desired, pitch_torque_desired, yaw_torque_desired] = mpc_fcn_nogeo(R_cur,R_desired, omega, omega_desired, omega_d_dot,x_d_ddot,e_x,e_v,e_R,e_Omega,control_gain,I_moment_vec, m, g, adaptive_roll,adaptive_pitch, adaptive_yaw, adaptive_x,adaptive_y, adaptive_z)
%#codegen
% PURE receding-horizon controller (no geometric feed-forward) with a
% measured pitch actuation model: a pure DEAD TIME followed by a FIRST-ORDER
% LAG on the applied pitch moment.
%
% Motivation (from unstable_flight13.mat system-id):
%   The command -> pitch-acceleration response is NOT a pure transport delay.
%   A pure delay best-fits at ~16 ms but describes the data poorly (R^2~0.44).
%   A dead time + first-order lag fits much better (R^2~0.55-0.76):
%       * dead time  ~ 5-8 ms  (~one wingbeat; 155-180 Hz flap -> 5.6-6.5 ms)
%       * lag tau    ~ 10-15 ms (passive wing-hinge + stroke-averaging)
%   Physically the plant (Drake co-sim) generates pitch moment by biasing the
%   mean wing stroke; the net cycle-averaged moment only develops over roughly
%   one wingbeat (dead time) and then rolls in through the passive hinge
%   response (first-order lag). The slider/piezo itself tracks in <0.5 ms, so
%   this lag is aeromechanical, not actuator-electrical.
%
% Compared to the pure-delay FIFO controller (mpc_fcn_nogeo.m), the pitch
% prediction here replaces "command becomes moment instantly after 16 ms"
% with "command becomes moment after a 6 ms dead time, then approaches the
% commanded value with a 12 ms time constant". The applied moment is an extra
% (hidden) actuator state; its current value is carried in a persistent
% variable and its history over the dead-time window in a persistent FIFO.
%
% Everything OTHER than the pitch channel is identical to the pure-MPC model:
%   * NO M_ff, NO geometric thrust feed-forward, NO PD shaping.
%   * gravity/desired-accel enter only via the affine term c_aff.
%
% control_gain are state-error weights:
%   [kx, kv, kR, kOmega, kz, kvz, kRx, kOmega_pitch, kOmega_yaw, kR_yaw]
%
% Unused-by-pure-MPC inputs (kept only for signature compatibility):
%   R_desired, omega, omega_desired, omega_d_dot

% -------------------------------------------------------------------------
% Pitch actuation model constants (dead time + first-order lag)
% -------------------------------------------------------------------------
% controller_dt is the Simulink sample time this block runs at. The dead time
% is stored as an integer number of controller samples so the FIFO indexing
% is exact: n_dead = pitch_dead_time_s / controller_dt.

controller_dt     = 2.0e-4;    % block execution period [s]
pitch_dead_time_s = 6.0e-3;    % transport dead time (~1 wingbeat) [s]
pitch_lag_tau_s   = 6.0e-3;    % first-order actuation lag time constant [s]

% Derived FIFO length. NOTE: this is the variable the prediction actually
% uses -- editing pitch_dead_time_s now changes the model (previously n_dead
% was hardcoded to 30, so setting pitch_dead_time_s = 0 silently kept a 6 ms
% dead time). Kept >= 1 because the FIFO must hold at least one sample.
n_dead = max(1, round(pitch_dead_time_s / controller_dt));

persistent pitch_command_fifo pitch_applied_moment

if isempty(pitch_command_fifo)
    % pitch_command_fifo(1)   = oldest sent command (applied on THIS step),
    % pitch_command_fifo(end) = most recently sent command.
    pitch_command_fifo = zeros(n_dead,1);
end
if isempty(pitch_applied_moment)
    % Hidden first-order-lag state: the currently applied (filtered) pitch
    % moment. Carried between calls so the prediction starts from it.
    pitch_applied_moment = 0.0;
end

% -------------------------------------------------------------------------
% Force fixed dimensions for Simulink code generation / dimension inference
% -------------------------------------------------------------------------

R_cur = reshape(R_cur, 9, 1);
R_desired = reshape(R_desired, 9, 1);         %#ok<NASGU> % unused (was M_ff)

omega = reshape(omega, 3, 1);                 %#ok<NASGU> % unused (was M_ff)
omega_desired = reshape(omega_desired, 3, 1); %#ok<NASGU> % unused (was M_ff)
omega_d_dot = reshape(omega_d_dot, 3, 1);     %#ok<NASGU> % unused (was M_ff)

x_d_ddot = reshape(x_d_ddot, 3, 1);
e_x = reshape(e_x, 3, 1);
e_v = reshape(e_v, 3, 1);
e_R = reshape(e_R, 3, 1);
e_Omega = reshape(e_Omega, 3, 1);

control_gain = reshape(control_gain, 10, 1);
I_moment_vec = reshape(I_moment_vec, 3, 1);

m = m(1);
g = g(1);

adaptive_roll = adaptive_roll(1);
adaptive_pitch = adaptive_pitch(1);
adaptive_yaw = adaptive_yaw(1);
adaptive_x = adaptive_x(1);
adaptive_y = adaptive_y(1);
adaptive_z = adaptive_z(1);

% -------------------------------------------------------------------------
% Constants and dimensions
% -------------------------------------------------------------------------

nx = 12;
nu = 4;
N = 5;

% Existing uprightmpc2 controllers in this project use a 6.5 ms prediction
% step. The block can still be called faster; this is the MPC model step.
dt = 0.0065;

% Fine-grid used to roll out the pitch actuator model (dead time + lag) over
% the horizon. One fine step == controller_dt so the FIFO history aligns
% exactly with the dead-time window.
n_sub = 163;                   % = round(N*dt/controller_dt), horizon in fine steps

thrust_min = 0.5e-3;
thrust_max = 1.6e-3;
roll_min = -1.5e-5;
roll_max = 1.5e-5;
pitch_min = -50e-7;
pitch_max = 50e-7;
yaw_min = -5e-7;
yaw_max = 5e-7;
% -------------------------------------------------------------------------
% Reconstruct current rotation matrix from row-major 9-element vector
% [R11; R12; R13; R21; R22; R23; R31; R32; R33]
% (R_desired is no longer needed -- it only fed the geometric M_ff.)
% -------------------------------------------------------------------------

Rot_cur = [R_cur(1), R_cur(2), R_cur(3); ...
           R_cur(4), R_cur(5), R_cur(6); ...
           R_cur(7), R_cur(8), R_cur(9)];

% -------------------------------------------------------------------------
% Weights from the control_gain vector (unchanged)
% -------------------------------------------------------------------------

k_x = control_gain(1);
k_v = control_gain(2);
k_R = control_gain(3);
k_Omega = control_gain(4);
k_z = control_gain(5);
k_vz = control_gain(6);
k_Rx = control_gain(7);
k_Omega_pitch = control_gain(8);
k_Omega_yaw   = control_gain(9);
k_R_yaw       = control_gain(10);

K_X = [k_x; k_x; k_z];
K_V = [k_v; k_v; k_vz];
K_R = [k_Rx; k_R; k_R_yaw];                        % roll, pitch, yaw
K_Omega = [k_Omega; k_Omega_pitch; k_Omega_yaw];   % roll, pitch, yaw

Qdiag = [positive_weight(K_X(1)); positive_weight(K_X(2)); positive_weight(K_X(3)); ...
         positive_weight(K_V(1)); positive_weight(K_V(2)); positive_weight(K_V(3)); ...
         positive_weight(K_R(1)); positive_weight(K_R(2)); positive_weight(K_R(3)); ...
         positive_weight(K_Omega(1)); positive_weight(K_Omega(2)); positive_weight(K_Omega(3))];
Qfdiag = 4.0 .* Qdiag;

% Input-effort weights. thrust, roll, pitch, yaw (must be 1/max^2 form).
Rdiag = [2.5e7; 1/(roll_max)^2; 500/(pitch_max)^2; 1/(yaw_max)^2];

e_3 = [0; 0; 1];
b_3 = Rot_cur * e_3;

I_moment = zeros(3,3);
I_moment(1,1) = max(abs(I_moment_vec(1)), 1.0e-12);
I_moment(2,2) = max(abs(I_moment_vec(2)), 1.0e-12);
I_moment(3,3) = max(abs(I_moment_vec(3)), 1.0e-12);

% -------------------------------------------------------------------------
% Delay-free discrete model for all non-pitch channels
%   x[k+1] = A x[k] + B u[k] + c
%   x = [e_x; e_v; e_R; e_Omega]
%   u = [thrust; Mx; My; Mz_internal]   (ABSOLUTE wrench, not deltas)
% The pitch rows/columns are OVERWRITTEN below with the dead-time+lag model.
% -------------------------------------------------------------------------

A = eye(nx);
A(1,4) = dt;
A(2,5) = dt;
A(3,6) = dt;
A(7,10) = dt;
A(8,11) = dt;
A(9,12) = dt;

% Tilt -> lateral acceleration coupling (small angle): tilting the thrust
% vector accelerates the vehicle sideways. Without these terms the QP treats
% attitude and translation as decoupled, so the real x<->pitch cascade
% (x error -> desired tilt -> tilt -> x acceleration -> x error) is invisible
% to the prediction; in flight it rings at ~9.8 Hz with coherence 1.0
% (stable_acrobatic1-4). Signs verified empirically from stable_acrobatic4:
% a_x = +g*pitch (phase ~0 deg at 9.8 Hz), a_y = -g*roll (phase ~180 deg).
A(4,8) = dt * g;    % e_vx_dot += +g * e_R_pitch
A(5,7) = -dt * g;   % e_vy_dot += -g * e_R_roll

B = zeros(nx, nu);
B(4,1) = dt * b_3(1) / max(abs(m), 1.0e-12);
B(5,1) = dt * b_3(2) / max(abs(m), 1.0e-12);
B(6,1) = dt * b_3(3) / max(abs(m), 1.0e-12);
B(10,2) = dt / I_moment(1,1);
B(11,3) = dt / I_moment(2,2);
B(12,4) = dt / I_moment(3,3);   % keep original yaw model convention (/6 at output)

adaptive_lateral = [adaptive_x; adaptive_y; adaptive_z];
safe_mass = max(abs(m), 1.0e-12);

% Affine term. Velocity-error rows only (rotation carries no feed-forward:
% the gyroscopic coupling is left as an unmodeled disturbance handled by the
% receding-horizon re-solve -- this is exactly what makes it "pure MPC").
c_aff = zeros(nx,1);
c_aff(4) = dt * ( -g*e_3(1) - x_d_ddot(1) + adaptive_lateral(1)/safe_mass );
c_aff(5) = dt * ( -g*e_3(2) - x_d_ddot(2) + adaptive_lateral(2)/safe_mass );
c_aff(6) = dt * ( -g*e_3(3) - x_d_ddot(3) + adaptive_lateral(3)/safe_mass );

x0 = [e_x; e_v; e_R; e_Omega];

% -------------------------------------------------------------------------
% Absolute box bounds on the wrench (no feed-forward shift). Adaptive torque
% offsets and the yaw x6 actuator scaling are retained as before.
% -------------------------------------------------------------------------

umin = zeros(nu,1);
umax = zeros(nu,1);
umin(1) = thrust_min;
umax(1) = thrust_max;
umin(2) = roll_min + adaptive_roll;
umax(2) = roll_max + adaptive_roll;
umin(3) = pitch_min + adaptive_pitch;
umax(3) = pitch_max + adaptive_pitch;
umin(4) = 6.0*(yaw_min + adaptive_yaw);
umax(4) = 6.0*(yaw_max + adaptive_yaw);

% -------------------------------------------------------------------------
% Condensed QP: min 0.5 U'HU + h'U, subject to box bounds on U
%   x_k = A^k x0 + sum_j A^{k-j} B u_j + d_k,   d_k = (sum_{i=0}^{k-1} A^i) c
% -------------------------------------------------------------------------

Phi = zeros(nx*N, nx);
Gamma = zeros(nx*N, nu*N);
Dvec = zeros(nx*N, 1);   % stacked affine offsets d_k

Apow = eye(nx);
acc = zeros(nx,1);       % running (sum_{i=0}^{k-1} A^i) c
for k = 1:N
    % Affine offset d_k accumulated BEFORE bumping Apow to A^k
    acc = acc + Apow * c_aff;          % after this: acc = sum_{i=0}^{k-1} A^i c
    Dvec = set_block_vec1(Dvec, k, acc, nx);

    Apow = A * Apow;                   % now Apow = A^k
    Phi = set_block_vec(Phi, k, Apow, nx);

    Astep = eye(nx);
    for j = k:-1:1
        Gblk = Astep * B;
        Gamma = set_block_mat(Gamma, k, j, Gblk, nx, nu);
        Astep = A * Astep;
    end
end

% -------------------------------------------------------------------------
% Overwrite pitch prediction rows/columns with the dead-time + lag model
% -------------------------------------------------------------------------
% Pitch subsystem (augmented with the actuator lag state Ma):
%   theta_dot = omega
%   omega_dot = Ma / Iyy
%   Ma_dot    = (u_delayed - Ma) / tau,   u_delayed(t) = u(t - dead_time)
%
% The system is LTI, so the predicted pitch angle/rate over the horizon are a
% linear superposition of three contributions, each rolled out on the fine
% grid by predict_pitch():
%   (Phi)   the free double integrator driven by the current theta0, omega0
%   (Dvec)  the known part: initial lag state Ma0 + FIFO commands already sent
%           during the dead-time window (+ the adaptive pitch bias)
%   (Gamma) each future decision u_j, delayed by the dead time and low-passed
% The slider/piezo mapping is treated as instantaneous (measured <0.5 ms lag).

Iyy = I_moment(2,2);
pitch_angle_state = 8;    % e_R(2)
pitch_rate_state  = 11;   % e_Omega(2)
pitch_input       = 3;    % My in u

h = controller_dt;                 % fine-grid step
a_lag = exp(-h / pitch_lag_tau_s); % first-order lag pole over one fine step

% Prediction sample indices on the fine grid (land on k*dt, clamp to horizon).
i_pred = zeros(N,1);
for k = 1:N
    idx = round(k*dt/h);
    if idx > n_sub
        idx = n_sub;
    elseif idx < 1
        idx = 1;
    end
    i_pred(k) = idx;
end

% Known applied-moment input over the horizon: the commands already sitting in
% the dead-time FIFO become active during the first n_dead fine steps, each
% carrying the current adaptive pitch bias (same effective-moment convention
% as the pure-delay controller). Beyond the dead-time window the known input
% is zero; future decisions are handled separately in Gamma.
u_known = zeros(n_sub,1);
for i = 1:n_sub
    if i <= n_dead
        u_known(i) = pitch_command_fifo(i) + adaptive_pitch;
    end
end
[theta_known, omega_known] = ...
    predict_pitch(pitch_applied_moment, u_known, h, Iyy, a_lag, i_pred, n_sub, N);

for k = 1:N
    prediction_time = k * dt;
    angle_row = (k-1)*nx + pitch_angle_state;
    rate_row  = (k-1)*nx + pitch_rate_state;

    % Free (unforced) double-integrator response to theta0/omega0 only.
    for col = 1:nx
        Phi(angle_row,col) = 0.0;
        Phi(rate_row,col)  = 0.0;
    end
    Phi(angle_row,pitch_angle_state) = 1.0;
    Phi(angle_row,pitch_rate_state)  = prediction_time;
    Phi(rate_row,pitch_rate_state)   = 1.0;

    % Known offset: lag state + FIFO history + adaptive, through dead time+lag.
    Dvec(angle_row) = theta_known(k);
    Dvec(rate_row)  = omega_known(k);

    % Remove all delay-free pitch-input coefficients at this prediction row.
    for j = 1:N
        pitch_col = (j-1)*nu + pitch_input;
        Gamma(angle_row,pitch_col) = 0.0;
        Gamma(rate_row,pitch_col)  = 0.0;
    end
end

% Future decisions: each u_j is sent over [(j-1)dt, j*dt), reaches the plant
% pitch_dead_time_s later, and then rolls in through the first-order lag.
for j = 1:N
    u_ind = zeros(n_sub,1);
    for i = (n_dead+1):n_sub
        sent_time = (i-1-n_dead) * h;          % >= 0 in this range
        jj = floor(sent_time/dt) + 1;
        if jj < 1
            jj = 1;
        elseif jj > N
            jj = N;                            % last decision held beyond horizon
        end
        if jj == j
            u_ind(i) = 1.0;
        end
    end
    [theta_g, omega_g] = predict_pitch(0.0, u_ind, h, Iyy, a_lag, i_pred, n_sub, N);
    pitch_col = (j-1)*nu + pitch_input;
    for k = 1:N
        angle_row = (k-1)*nx + pitch_angle_state;
        rate_row  = (k-1)*nx + pitch_rate_state;
        Gamma(angle_row,pitch_col) = theta_g(k);
        Gamma(rate_row,pitch_col)  = omega_g(k);
    end
end

% -------------------------------------------------------------------------
% Cost matrices and QP
% -------------------------------------------------------------------------

Qbar = zeros(nx*N, nx*N);
Rbar = zeros(nu*N, nu*N);
for k = 1:N
    if k == N
        Qbar = set_diag_block(Qbar, k, Qfdiag, nx);
    else
        Qbar = set_diag_block(Qbar, k, Qdiag, nx);
    end
    Rbar = set_diag_block(Rbar, k, Rdiag, nu);
end

% Hover-thrust regularization center (pure weight compensation, NOT the
% geometric control law).
Uref = zeros(nu*N,1);
uref_stage = [m*g; 0; 0; 0];
for k = 1:N
    base = (k-1)*nu;
    Uref(base+1:base+nu) = uref_stage;
end

H = Gamma' * Qbar * Gamma + Rbar;
h_lin = Gamma' * Qbar * (Phi * x0 + Dvec) - Rbar * Uref;

Umin = zeros(nu*N,1);
Umax = zeros(nu*N,1);
for k = 1:N
    base = (k-1)*nu;
    Umin(base+1:base+nu) = umin;
    Umax(base+1:base+nu) = umax;
end

U = solve_box_qp_coordinate_descent(H, h_lin, Umin, Umax, nu*N);

thrust_cmd       = U(1);
Mx_cmd           = U(2);
My_cmd           = U(3);
Mz_internal_cmd  = U(4);

% -------------------------------------------------------------------------
% Outputs: absolute wrench, adaptive offsets, yaw scaling, saturations.
% -------------------------------------------------------------------------

thrust_desired       = thrust_cmd;
roll_torque_desired  = Mx_cmd - adaptive_roll;
pitch_torque_desired = My_cmd - adaptive_pitch;
yaw_torque_desired   = Mz_internal_cmd/6.0 - adaptive_yaw;

thrust_desired       = clamp_scalar(thrust_desired, thrust_min, thrust_max);
roll_torque_desired  = clamp_scalar(roll_torque_desired, roll_min, roll_max);
pitch_torque_desired = clamp_scalar(pitch_torque_desired, pitch_min, pitch_max);
yaw_torque_desired   = clamp_scalar(yaw_torque_desired, yaw_min, yaw_max);

% -------------------------------------------------------------------------
% Advance the pitch actuator model by one controller step, then update the
% FIFO. The moment applied on THIS step is the command sent one dead time ago
% (the oldest FIFO entry) plus the current adaptive bias. This must happen
% AFTER optimization so the prediction above used only already-issued history.
% -------------------------------------------------------------------------

applied_now = pitch_command_fifo(1) + adaptive_pitch;
pitch_applied_moment = a_lag*pitch_applied_moment + (1.0 - a_lag)*applied_now;

for q = 1:(n_dead-1)
    pitch_command_fifo(q) = pitch_command_fifo(q+1);
end
pitch_command_fifo(n_dead) = pitch_torque_desired;

end

% =========================================================================
% Helpers
% =========================================================================

function [theta_at, omega_at] = predict_pitch(Ma_init, u_seq, h, Iyy, a_lag, i_pred, n_sub, N)
%#codegen
% Roll out the augmented pitch subsystem on the fine grid and sample the
% angle/rate at the prediction indices i_pred.
%   theta_dot = omega,  omega_dot = Ma/Iyy,  Ma <- a_lag*Ma + (1-a_lag)*u
% Each fine step holds Ma constant over [(i-1)h, i h) (exact ZOH for the
% double integrator over that sub-step), then advances the lag state.
theta_at = zeros(N,1);
omega_at = zeros(N,1);

theta = 0.0;
omega = 0.0;
Ma = Ma_init;
inv_Iyy = 1.0 / max(Iyy, 1.0e-18);

krec = 1;
for i = 1:n_sub
    theta = theta + h*omega + 0.5*h*h*(Ma*inv_Iyy);
    omega = omega + h*(Ma*inv_Iyy);
    Ma = a_lag*Ma + (1.0 - a_lag)*u_seq(i);

    if krec <= N
        if i >= i_pred(krec)
            theta_at(krec) = theta;
            omega_at(krec) = omega;
            krec = krec + 1;
        end
    end
end

% Guard: fill any unrecorded tail with the final state (should not trigger
% when i_pred(N) <= n_sub, but keeps outputs well defined for codegen).
while krec <= N
    theta_at(krec) = theta;
    omega_at(krec) = omega;
    krec = krec + 1;
end
end

function w = positive_weight(v)
%#codegen
w = abs(v);
if w < 1.0e-12
    w = 1.0e-12;
end
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

function M = set_block_vec(M, k, B, nx)
%#codegen
row0 = (k-1)*nx;
for r = 1:nx
    for c = 1:nx
        M(row0+r,c) = B(r,c);
    end
end
end

function v = set_block_vec1(v, k, col, nx)
%#codegen
row0 = (k-1)*nx;
for r = 1:nx
    v(row0+r) = col(r);
end
end

function M = set_block_mat(M, k, j, B, nx, nu)
%#codegen
row0 = (k-1)*nx;
col0 = (j-1)*nu;
for r = 1:nx
    for c = 1:nu
        M(row0+r,col0+c) = B(r,c);
    end
end
end

function M = set_diag_block(M, k, d, n)
%#codegen
idx0 = (k-1)*n;
for i = 1:n
    M(idx0+i,idx0+i) = d(i);
end
end

function U = solve_box_qp_coordinate_descent(H, h, Umin, Umax, nU)
%#codegen
U = zeros(nU,1);
for i = 1:nU
    if U(i) < Umin(i)
        U(i) = Umin(i);
    elseif U(i) > Umax(i)
        U(i) = Umax(i);
    end
end

% Fixed iteration count keeps generated code deterministic.
for iter = 1:12
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