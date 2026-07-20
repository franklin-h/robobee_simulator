function [thrust_desired, roll_torque_desired, pitch_torque_desired, yaw_torque_desired] = mpc_fcn_nogeo(R_cur,R_desired, omega, omega_desired, omega_d_dot,x_d_ddot,e_x,e_v,e_R,e_Omega,control_gain,I_moment_vec, m, g, adaptive_roll,adaptive_pitch, adaptive_yaw, adaptive_x,adaptive_y, adaptive_z)
%#codegen
% PURE receding-horizon controller (no geometric feed-forward).
%
% Same I/O as mpc_fcn.m so it drops into the same Simulink block, but the
% GEOMETRIC PART of the controller is removed:
%   * NO M_ff  (gyroscopic omega x I*omega + attitude-transport feed-forward)
%   * NO geometric thrust feed-forward thrust_ff = force_ff' * b_3
%   * NO PD term u_feedback / Uref shaping
%
% This mirrors the "uprightmpc2" pure-MPC philosophy in this repo
% (template/template_controllers.py): tracking is done ENTIRELY by the
% horizon cost driving the error state to zero, feedback is implicit via
% re-solving from the measured state each call, and the ONLY thing carried
% by the model is gravity -- baked into an affine term c (the analog of
% c0(g) in uprightmpc2), NOT fed forward as a wrench.
%
% The QP therefore optimizes the ABSOLUTE wrench directly:
%   u = [thrust; Mx; My; Mz_internal]
% (Mz_internal keeps the original /6 yaw actuator scaling.)
%
% control_gain are state-error weights:
%   [kx, kv, kR, kOmega, kz, kvz, kRx, kOmega_pitch, kOmega_yaw, kR_yaw]
%
% Unused-by-pure-MPC inputs (kept only for signature compatibility):
%   R_desired, omega, omega_desired, omega_d_dot
% They defined the geometric M_ff, which no longer exists here. The attitude
% reference now enters solely through e_R / e_Omega (computed upstream).

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

% Existing uprightmpc2 controllers in this project use a 5 ms prediction
% step. The block can still be called faster; this is the MPC model step.
dt = 0.0065;

thrust_min = 0.5e-3;
thrust_max = 1.6e-3;
% roll_min = -0.6e-4;
% roll_max = 0.5e-4;
% roll_min = -0.2e-6; 
% roll_max = 0.2e-6; 
roll_min = -1.5e-5; 
roll_max = 1.5e-5; 
% pitch_min = -0.125e-5; 
% pitch_max = 0.125e-5; 
pitch_min = -60e-7; 
pitch_max = 60e-7; 
% pitch_min = -0.2e-6;
% pitch_max = 0.2e-6;
% pitch_min = -0.6e-4; 
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
Rdiag = [2.5e7; 1/(roll_max)^2; 1/(pitch_max)^2; 1/(yaw_max)^2];

e_3 = [0; 0; 1];
b_3 = Rot_cur * e_3;

I_moment = zeros(3,3);
I_moment(1,1) = max(abs(I_moment_vec(1)), 1.0e-12);
I_moment(2,2) = max(abs(I_moment_vec(2)), 1.0e-12);
I_moment(3,3) = max(abs(I_moment_vec(3)), 1.0e-12);

% -------------------------------------------------------------------------
% Linear prediction model for error dynamics (affine)
%   x[k+1] = A x[k] + B u[k] + c
%   x = [e_x; e_v; e_R; e_Omega]
%   u = [thrust; Mx; My; Mz_internal]   (ABSOLUTE wrench, not deltas)
%
% The affine term c carries what the geometric feed-forward used to cancel:
% gravity, desired linear acceleration, and the (adaptive) lateral force
% estimate. The MPC then discovers the thrust needed to hold position.
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
B(12,4) = dt / I_moment(3,3);   % keep original yaw model convention (/6 at output)

adaptive_lateral = [adaptive_x; adaptive_y; adaptive_z];

% Affine term. Velocity-error rows only (rotation carries no feed-forward:
% the gyroscopic coupling is left as an unmodeled disturbance handled by the
% receding-horizon re-solve -- this is exactly what makes it "pure MPC").
%   e_v_dot = (1/m) thrust b_3 - g e_3 - x_d_ddot + (1/m) adaptive_lateral
% At equilibrium this reproduces the old thrust_ff exactly:
%   thrust = (m g e_3 + m x_d_ddot - adaptive_lateral)' b_3
c_aff = zeros(nx,1);
c_aff(4) = dt * ( -g*e_3(1) - x_d_ddot(1) + adaptive_lateral(1)/max(abs(m),1.0e-12) );
c_aff(5) = dt * ( -g*e_3(2) - x_d_ddot(2) + adaptive_lateral(2)/max(abs(m),1.0e-12) );
c_aff(6) = dt * ( -g*e_3(3) - x_d_ddot(3) + adaptive_lateral(3)/max(abs(m),1.0e-12) );

x0 = [e_x; e_v; e_R; e_Omega];

% -------------------------------------------------------------------------
% Absolute box bounds on the wrench (no feed-forward shift). Adaptive torque
% offsets and the yaw x6 actuator scaling are retained as before so the
% adaptive-disturbance mechanism keeps working; it is separate from the
% geometric control law that was removed.
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
% geometric control law). Keeps the R-penalty on absolute thrust from
% biasing the steady-state thrust below the weight and causing altitude sag.
Uref = zeros(nu*N,1);
uref_stage = [m*g; 0; 0; 0];
for k = 1:N
    base = (k-1)*nu;
    Uref(base+1:base+nu) = uref_stage;
end

H = Gamma' * Qbar * Gamma + Rbar;
h = Gamma' * Qbar * (Phi * x0 + Dvec) - Rbar * Uref;

Umin = zeros(nu*N,1);
Umax = zeros(nu*N,1);
for k = 1:N
    base = (k-1)*nu;
    Umin(base+1:base+nu) = umin;
    Umax(base+1:base+nu) = umax;
end

U = solve_box_qp_coordinate_descent(H, h, Umin, Umax, nu*N);

thrust_cmd       = U(1);
Mx_cmd           = U(2);
My_cmd           = U(3);
Mz_internal_cmd  = U(4);

% -------------------------------------------------------------------------
% Outputs: absolute wrench, adaptive offsets, yaw scaling, saturations.
% (No M_ff added back -- there is no geometric feed-forward anymore.)
% -------------------------------------------------------------------------

thrust_desired       = thrust_cmd;
roll_torque_desired  = Mx_cmd - adaptive_roll;
pitch_torque_desired = My_cmd - adaptive_pitch;
yaw_torque_desired   = Mz_internal_cmd/6.0 - adaptive_yaw;

thrust_desired       = clamp_scalar(thrust_desired, thrust_min, thrust_max);
roll_torque_desired  = clamp_scalar(roll_torque_desired, roll_min, roll_max);
pitch_torque_desired = clamp_scalar(pitch_torque_desired, pitch_min, pitch_max);
yaw_torque_desired   = clamp_scalar(yaw_torque_desired, yaw_min, yaw_max);

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
