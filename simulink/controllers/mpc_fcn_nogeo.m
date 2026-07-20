function [thrust_desired, roll_torque_desired, pitch_torque_desired, yaw_torque_desired] = mpc_fcn_nogeo(R_cur,R_desired, omega, omega_desired, omega_d_dot,x_d_ddot,e_x,e_v,e_R,e_Omega,control_gain,I_moment_vec, m, g, adaptive_roll,adaptive_pitch, adaptive_yaw, adaptive_x,adaptive_y,adaptive_z)
%#codegen
% Pure receding-horizon controller with a 16 ms pitch actuator delay.
%
% The pitch delay is represented in the prediction model, not by shifting
% the measured state. Commands already sent to the actuator are stored in a
% fixed-size FIFO at the controller execution sample time. They are known
% inputs during the first 16 ms of the prediction. Newly optimized pitch
% commands affect the predicted pitch states only after that delay.
%
% Assumptions:
%   * The MATLAB Function executes every controller_dt = 0.2 ms.
%   * pitch_delay_s is a command-to-applied-moment (actuator/input) delay.
%   * Each MPC decision is held for one prediction interval dt = 6.5 ms in
%     the horizon model, as in the original controller.
%
% State:
%   x = [e_x; e_v; e_R; e_Omega]
%
% Input:
%   u = [thrust; Mx; My; Mz_internal]
%
% control_gain:
%   [kx, kv, kR, kOmega, kz, kvz, ...
%    kRx, kOmega_pitch, kOmega_yaw, kR_yaw]

% -------------------------------------------------------------------------
% Applied pitch-command FIFO
% -------------------------------------------------------------------------

controller_dt = 2.0e-4;
pitch_delay_s = 16.0e-3;
pitch_delay_samples = 80;

persistent pitch_command_fifo

if isempty(pitch_command_fifo)
    pitch_command_fifo = zeros(pitch_delay_samples,1);
end

% -------------------------------------------------------------------------
% Force fixed dimensions for Simulink code generation
% -------------------------------------------------------------------------

R_cur = reshape(R_cur, 9, 1);
R_desired = reshape(R_desired, 9, 1);         %#ok<NASGU>

omega = reshape(omega, 3, 1);                 %#ok<NASGU>
omega_desired = reshape(omega_desired, 3, 1); %#ok<NASGU>
omega_d_dot = reshape(omega_d_dot, 3, 1);     %#ok<NASGU>

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
dt = 0.0065;

thrust_min = 0.5e-3;
thrust_max = 1.6e-3;
roll_min = -1.5e-5;
roll_max = 1.5e-5;
pitch_min = -50e-7;
pitch_max = 50e-7;
yaw_min = -5e-7;
yaw_max = 5e-7;

% -------------------------------------------------------------------------
% Current rotation and controller weights
% -------------------------------------------------------------------------

Rot_cur = [R_cur(1), R_cur(2), R_cur(3); ...
           R_cur(4), R_cur(5), R_cur(6); ...
           R_cur(7), R_cur(8), R_cur(9)];

k_x = control_gain(1);
k_v = control_gain(2);
k_R = control_gain(3);
k_Omega = control_gain(4);
k_z = control_gain(5);
k_vz = control_gain(6);
k_Rx = control_gain(7);
k_Omega_pitch = control_gain(8);
k_Omega_yaw = control_gain(9);
k_R_yaw = control_gain(10);

K_X = [k_x; k_x; k_z];
K_V = [k_v; k_v; k_vz];
K_R = [k_Rx; k_R; k_R_yaw];
K_Omega = [k_Omega; k_Omega_pitch; k_Omega_yaw];

Qdiag = [positive_weight(K_X(1)); ...
         positive_weight(K_X(2)); ...
         positive_weight(K_X(3)); ...
         positive_weight(K_V(1)); ...
         positive_weight(K_V(2)); ...
         positive_weight(K_V(3)); ...
         positive_weight(K_R(1)); ...
         positive_weight(K_R(2)); ...
         positive_weight(K_R(3)); ...
         positive_weight(K_Omega(1)); ...
         positive_weight(K_Omega(2)); ...
         positive_weight(K_Omega(3))];

Qfdiag = 4.0 .* Qdiag;

Rdiag = [2.5e7; ...
         1/(roll_max)^2; ...
         1/(pitch_max)^2; ...
         1/(yaw_max)^2];

e_3 = [0; 0; 1];
b_3 = Rot_cur * e_3;

I_moment = zeros(3,3);
I_moment(1,1) = max(abs(I_moment_vec(1)), 1.0e-12);
I_moment(2,2) = max(abs(I_moment_vec(2)), 1.0e-12);
I_moment(3,3) = max(abs(I_moment_vec(3)), 1.0e-12);

% -------------------------------------------------------------------------
% Delay-free discrete model for all non-pitch channels
% -------------------------------------------------------------------------

A = eye(nx);
A(1,4) = dt;
A(2,5) = dt;
A(3,6) = dt;
A(7,10) = dt;
A(8,11) = dt;
A(9,12) = dt;

B = zeros(nx, nu);
B(4,1) = dt * b_3(1) / max(abs(m), 1.0e-12);
B(5,1) = dt * b_3(2) / max(abs(m), 1.0e-12);
B(6,1) = dt * b_3(3) / max(abs(m), 1.0e-12);
B(10,2) = dt / I_moment(1,1);
B(11,3) = dt / I_moment(2,2);
B(12,4) = dt / I_moment(3,3);

adaptive_lateral = [adaptive_x; adaptive_y; adaptive_z];
safe_mass = max(abs(m), 1.0e-12);

c_aff = zeros(nx,1);
c_aff(4) = dt * (-g*e_3(1) - x_d_ddot(1) + adaptive_lateral(1)/safe_mass);
c_aff(5) = dt * (-g*e_3(2) - x_d_ddot(2) + adaptive_lateral(2)/safe_mass);
c_aff(6) = dt * (-g*e_3(3) - x_d_ddot(3) + adaptive_lateral(3)/safe_mass);

x0 = [e_x; e_v; e_R; e_Omega];

% -------------------------------------------------------------------------
% Absolute wrench bounds
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
% Condensed prediction model
% -------------------------------------------------------------------------
% Start with the original delay-free construction. The pitch rows are then
% replaced by an exact continuous-time double-integrator prediction that
% includes the 16 ms input delay and the known command FIFO.

Phi = zeros(nx*N, nx);
Gamma = zeros(nx*N, nu*N);
Dvec = zeros(nx*N, 1);

Apow = eye(nx);
acc = zeros(nx,1);

for k = 1:N
    acc = acc + Apow * c_aff;
    Dvec = set_block_vec1(Dvec, k, acc, nx);

    Apow = A * Apow;
    Phi = set_block_vec(Phi, k, Apow, nx);

    Astep = eye(nx);
    for j = k:-1:1
        Gblk = Astep * B;
        Gamma = set_block_mat(Gamma, k, j, Gblk, nx, nu);
        Astep = A * Astep;
    end
end

% -------------------------------------------------------------------------
% Replace pitch prediction rows with delayed-input dynamics
% -------------------------------------------------------------------------
% Pitch subsystem:
%   theta_dot = omega
%   omega_dot = My_applied / Iyy
%
% pitch_command_fifo(1) is the physical command that becomes active now.
% The FIFO contains commands already sent over the preceding 16 ms. The
% current adaptive estimate is added to each physical command so the known
% queue uses the same effective-moment convention as the original MPC.

Iyy = I_moment(2,2);
pitch_angle_state = 8;
pitch_rate_state = 11;
pitch_input = 3;

for k = 1:N
    prediction_time = k * dt;
    angle_row = (k-1)*nx + pitch_angle_state;
    rate_row = (k-1)*nx + pitch_rate_state;

    % Exact state transition for the unforced pitch double integrator.
    for col = 1:nx
        Phi(angle_row,col) = 0.0;
        Phi(rate_row,col) = 0.0;
    end
    Phi(angle_row,pitch_angle_state) = 1.0;
    Phi(angle_row,pitch_rate_state) = prediction_time;
    Phi(rate_row,pitch_rate_state) = 1.0;

    % No generic affine term acts directly on pitch in this model.
    Dvec(angle_row) = 0.0;
    Dvec(rate_row) = 0.0;

    % Known commands already in the 16 ms actuator queue.
    for q = 1:pitch_delay_samples
        interval_start = (q-1) * controller_dt;
        interval_end = q * controller_dt;
        active_end = min_scalar(prediction_time, interval_end);

        if active_end > interval_start
            active_duration = active_end - interval_start;
            effective_pitch_moment = pitch_command_fifo(q) + adaptive_pitch;

            Dvec(rate_row) = Dvec(rate_row) + ...
                effective_pitch_moment * active_duration / Iyy;

            Dvec(angle_row) = Dvec(angle_row) + ...
                effective_pitch_moment * ...
                (prediction_time*active_duration - ...
                 0.5*(active_end*active_end - interval_start*interval_start)) / Iyy;
        end
    end

    % Remove all delay-free pitch-input coefficients at this prediction row.
    for j = 1:N
        pitch_col = (j-1)*nu + pitch_input;
        Gamma(angle_row,pitch_col) = 0.0;
        Gamma(rate_row,pitch_col) = 0.0;
    end

    % Future decision j is sent over [(j-1)dt,j*dt] and reaches the plant
    % pitch_delay_s later. Its exact contribution is evaluated at k*dt.
    for j = 1:N
        sent_start = (j-1) * dt;
        applied_start = pitch_delay_s + sent_start;
        applied_end = applied_start + dt;
        active_end = min_scalar(prediction_time, applied_end);

        if active_end > applied_start
            active_duration = active_end - applied_start;
            pitch_col = (j-1)*nu + pitch_input;

            Gamma(rate_row,pitch_col) = active_duration / Iyy;
            Gamma(angle_row,pitch_col) = ...
                (prediction_time*active_duration - ...
                 0.5*(active_end*active_end - applied_start*applied_start)) / Iyy;
        end
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

Uref = zeros(nu*N,1);
uref_stage = [m*g; 0; 0; 0];

for k = 1:N
    base = (k-1)*nu;
    Uref(base+1:base+nu) = uref_stage;
end

H = Gamma' * Qbar * Gamma + Rbar;
h = Gamma' * Qbar * (Phi*x0 + Dvec) - Rbar*Uref;

Umin = zeros(nu*N,1);
Umax = zeros(nu*N,1);

for k = 1:N
    base = (k-1)*nu;
    Umin(base+1:base+nu) = umin;
    Umax(base+1:base+nu) = umax;
end

U = solve_box_qp_coordinate_descent(H, h, Umin, Umax, nu*N);

thrust_cmd = U(1);
Mx_cmd = U(2);
My_cmd = U(3);
Mz_internal_cmd = U(4);

% -------------------------------------------------------------------------
% Outputs and physical saturations
% -------------------------------------------------------------------------

thrust_desired = thrust_cmd;
roll_torque_desired = Mx_cmd - adaptive_roll;
pitch_torque_desired = My_cmd - adaptive_pitch;
yaw_torque_desired = Mz_internal_cmd/6.0 - adaptive_yaw;

thrust_desired = clamp_scalar(thrust_desired, thrust_min, thrust_max);
roll_torque_desired = clamp_scalar(roll_torque_desired, roll_min, roll_max);
pitch_torque_desired = clamp_scalar(pitch_torque_desired, pitch_min, pitch_max);
yaw_torque_desired = clamp_scalar(yaw_torque_desired, yaw_min, yaw_max);

% Store the final physical pitch command sent to the actuator. The update
% happens after optimization, so the FIFO used above contains only commands
% that were already issued before this controller call.
for q = 1:(pitch_delay_samples-1)
    pitch_command_fifo(q) = pitch_command_fifo(q+1);
end
pitch_command_fifo(pitch_delay_samples) = pitch_torque_desired;

end

% =========================================================================
% Helpers
% =========================================================================

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

function y = min_scalar(a,b)
%#codegen
y = a;
if b < a
    y = b;
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
