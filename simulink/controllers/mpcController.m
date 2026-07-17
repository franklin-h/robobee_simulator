function [thrust_desired, roll_torque_desired, pitch_torque_desired, yaw_torque_desired] = mpc_fcn(R_cur,R_desired, omega, omega_desired, omega_d_dot,x_d_ddot,e_x,e_v,e_R,e_Omega,control_gain,I_moment_vec, m, g, adaptive_roll,adaptive_pitch, adaptive_yaw, adaptive_x,adaptive_y, adaptive_z)
%#codegen
% Receding-horizon replacement for the geometric SE(3) controller.
%
% This keeps the same inputs, outputs, feed-forward terms, adaptive offsets,
% and saturations as the geometric controller. The QP uses control_gain as
% state-error weights:
%   [kx, kv, kR, kOmega, kz, kvz, kRx]
%
% The optimized variables are corrections around the nominal feed-forward
% thrust and moments:
%   [delta_thrust, delta_Mx, delta_My, delta_Mz_internal]
% where Mz_internal is converted back through the same /6 yaw scaling used
% by the original controller.

% -------------------------------------------------------------------------
% Force fixed dimensions for Simulink code generation / dimension inference
% -------------------------------------------------------------------------

R_cur = reshape(R_cur, 9, 1);
R_desired = reshape(R_desired, 9, 1);

omega = reshape(omega, 3, 1);
omega_desired = reshape(omega_desired, 3, 1);
omega_d_dot = reshape(omega_d_dot, 3, 1);

x_d_ddot = reshape(x_d_ddot, 3, 1);
e_x = reshape(e_x, 3, 1);
e_v = reshape(e_v, 3, 1);
e_R = reshape(e_R, 3, 1);
e_Omega = reshape(e_Omega, 3, 1);

control_gain = reshape(control_gain, 7, 1);
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

% Existing uprightmpc2 controllers in this project use a 5 ms prediction
% step. The block can still be called faster; this is the MPC model step.
dt = 5.0e-3;

thrust_min = 0.5e-3;
thrust_max = 1.6e-3;
roll_min = -0.6e-4;
roll_max = 0.5e-4;
pitch_min = -0.2e-6;
pitch_max = 0.2e-6;
yaw_min = -1.0e-7;
yaw_max = 0.9e-7;

% -------------------------------------------------------------------------
% Reconstruct rotation matrices from row-major 9-element vectors
% [R11; R12; R13; R21; R22; R23; R31; R32; R33]
% -------------------------------------------------------------------------

Rot_cur = [R_cur(1), R_cur(2), R_cur(3); ...
           R_cur(4), R_cur(5), R_cur(6); ...
           R_cur(7), R_cur(8), R_cur(9)];

Rot_desired = [R_desired(1), R_desired(2), R_desired(3); ...
               R_desired(4), R_desired(5), R_desired(6); ...
               R_desired(7), R_desired(8), R_desired(9)];

% -------------------------------------------------------------------------
% Weights from the original control_gain vector
% -------------------------------------------------------------------------

k_x = control_gain(1);
k_v = control_gain(2);
k_R = control_gain(3);
k_Omega = control_gain(4);
k_z = control_gain(5);
k_vz = control_gain(6);
k_Rx = control_gain(7);

K_X = [k_x; k_x; k_z];
K_V = [k_v; k_v; k_vz];
K_R = [k_Rx; k_R; k_R];
K_Omega = [k_Omega; k_Omega; k_Omega];

Qdiag = [positive_weight(K_X(1)); positive_weight(K_X(2)); positive_weight(K_X(3)); ...
         positive_weight(K_V(1)); positive_weight(K_V(2)); positive_weight(K_V(3)); ...
         positive_weight(K_R(1)); positive_weight(K_R(2)); positive_weight(K_R(3)); ...
         positive_weight(K_Omega(1)); positive_weight(K_Omega(2)); positive_weight(K_Omega(3))];
Qfdiag = 4.0 .* Qdiag;

% Same input-effort weights as target_driver_setup.m wmpc:
% wthrust = 1e-1, wmom = 1e0.
Rdiag = [1.0e-1; 1.0e0; 1.0e0; 1.0e0];

e_3 = [0; 0; 1];
b_3 = Rot_cur * e_3;

I_moment = zeros(3,3);
I_moment(1,1) = max(abs(I_moment_vec(1)), 1.0e-12);
I_moment(2,2) = max(abs(I_moment_vec(2)), 1.0e-12);
I_moment(3,3) = max(abs(I_moment_vec(3)), 1.0e-12);

% -------------------------------------------------------------------------
% Nominal feed-forward terms from the original geometric controller
% -------------------------------------------------------------------------

w_hat = hat3(omega);

M_ff = w_hat*I_moment*omega ...
    - I_moment*(w_hat*Rot_cur'*Rot_desired*omega_desired ...
    - Rot_cur'*Rot_desired*omega_d_dot);

adaptive_lateral = [adaptive_x; adaptive_y; adaptive_z];
force_ff = -adaptive_lateral + m*g*e_3 + m*x_d_ddot;
thrust_ff = force_ff' * b_3;

% -------------------------------------------------------------------------
% Linear prediction model for error dynamics
% x = [e_x; e_v; e_R; e_Omega]
% u = [delta_thrust; delta_Mx; delta_My; delta_Mz_internal]
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

x0 = [e_x; e_v; e_R; e_Omega];

force_feedback = -K_X.*e_x - K_V.*e_v;
moment_feedback = -K_R.*e_R - k_Omega*e_Omega;
u_feedback = [force_feedback' * b_3; ...
              moment_feedback(1); ...
              moment_feedback(2); ...
              moment_feedback(3)];

umin = zeros(nu,1);
umax = zeros(nu,1);
umin(1) = thrust_min - thrust_ff;
umax(1) = thrust_max - thrust_ff;
umin(2) = roll_min + adaptive_roll - M_ff(1);
umax(2) = roll_max + adaptive_roll - M_ff(1);
umin(3) = pitch_min + adaptive_pitch - M_ff(2);
umax(3) = pitch_max + adaptive_pitch - M_ff(2);
umin(4) = 6.0*(yaw_min + adaptive_yaw) - M_ff(3);
umax(4) = 6.0*(yaw_max + adaptive_yaw) - M_ff(3);

% -------------------------------------------------------------------------
% Condensed QP: min 0.5 U'HU + h'U, subject to box bounds on U
% -------------------------------------------------------------------------

Phi = zeros(nx*N, nx);
Gamma = zeros(nx*N, nu*N);

Apow = eye(nx);
for k = 1:N
    Apow = A * Apow;
    Phi = set_block_vec(Phi, k, Apow, nx);

    Astep = eye(nx);
    for j = k:-1:1
        Gblk = Astep * B;
        Gamma = set_block_mat(Gamma, k, j, Gblk, nx, nu);
        Astep = A * Astep;
    end
end

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
for k = 1:N
    base = (k-1)*nu;
    Uref(base+1:base+nu) = u_feedback;
end

H = Gamma' * Qbar * Gamma + Rbar;
h = Gamma' * Qbar * Phi * x0 - Rbar * Uref;

Umin = zeros(nu*N,1);
Umax = zeros(nu*N,1);
for k = 1:N
    base = (k-1)*nu;
    Umin(base+1:base+nu) = umin;
    Umax(base+1:base+nu) = umax;
end

U = solve_box_qp_coordinate_descent(H, h, Umin, Umax, nu*N);

delta_thrust = U(1);
delta_Mx = U(2);
delta_My = U(3);
delta_Mz_internal = U(4);

% -------------------------------------------------------------------------
% Outputs, adaptive offsets, yaw scaling, and final saturations
% -------------------------------------------------------------------------

thrust_desired = thrust_ff + delta_thrust;
roll_torque_desired = M_ff(1) + delta_Mx - adaptive_roll;
pitch_torque_desired = M_ff(2) + delta_My - adaptive_pitch;
yaw_torque_desired = (M_ff(3) + delta_Mz_internal)/6.0 - adaptive_yaw;

thrust_desired = clamp_scalar(thrust_desired, thrust_min, thrust_max);
roll_torque_desired = clamp_scalar(roll_torque_desired, roll_min, roll_max);
pitch_torque_desired = clamp_scalar(pitch_torque_desired, pitch_min, pitch_max);
yaw_torque_desired = clamp_scalar(yaw_torque_desired, yaw_min, yaw_max);

end

function w = positive_weight(v)
%#codegen
w = abs(v);
if w < 1.0e-12
    w = 1.0e-12;
end
end

function S = hat3(v)
%#codegen
S = zeros(3,3);
S(1,2) = -v(3);
S(2,1) =  v(3);
S(1,3) =  v(2);
S(3,1) = -v(2);
S(2,3) = -v(1);
S(3,2) =  v(1);
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
