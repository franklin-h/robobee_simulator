function [thrust_desired, roll_torque_desired, pitch_torque_desired, yaw_torque_desired] = fcn(R_cur,R_desired, omega, omega_desired, omega_d_dot,x_d_ddot,e_x,e_v,e_R,e_Omega,control_gain,I_moment_vec, m, g, adaptive_roll,adaptive_pitch, adaptive_yaw, adaptive_x,adaptive_y, adaptive_z)
%#codegen

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
% Preinitialize scalar outputs
% -------------------------------------------------------------------------

thrust_desired = 0.0;
roll_torque_desired = 0.0;
pitch_torque_desired = 0.0;
yaw_torque_desired = 0.0;

% -------------------------------------------------------------------------
% Reconstruct rotation matrices from 9-element vectors
% Original ordering:
% [R11; R12; R13; R21; R22; R23; R31; R32; R33]
% -------------------------------------------------------------------------

Rot_cur = [R_cur(1), R_cur(2), R_cur(3); ...
           R_cur(4), R_cur(5), R_cur(6); ...
           R_cur(7), R_cur(8), R_cur(9)];

Rot_desired = [R_desired(1), R_desired(2), R_desired(3); ...
               R_desired(4), R_desired(5), R_desired(6); ...
               R_desired(7), R_desired(8), R_desired(9)];

% -------------------------------------------------------------------------
% Gains
% -------------------------------------------------------------------------

k_x = control_gain(1);
k_v = control_gain(2);
k_R = control_gain(3);
k_Omega = control_gain(4);
k_z = control_gain(5);
k_vz = control_gain(6);
k_Rx = control_gain(7);

e_3 = [0; 0; 1];

I_moment = zeros(3,3);
I_moment(1,1) = I_moment_vec(1);
I_moment(2,2) = I_moment_vec(2);
I_moment(3,3) = I_moment_vec(3);

% -------------------------------------------------------------------------
% Angular velocity hat map
% -------------------------------------------------------------------------

w_hat = zeros(3,3);

w_hat(1,2) = -omega(3);
w_hat(2,1) =  omega(3);
w_hat(1,3) =  omega(2);
w_hat(3,1) = -omega(2);
w_hat(2,3) = -omega(1);
w_hat(3,2) =  omega(1);

K_X = [k_x; k_x; k_z];
K_V = [k_v; k_v; k_vz];

% Different attitude gain
K_R = [k_Rx; k_R; k_R];

% -------------------------------------------------------------------------
% Geometric controller in SE(3)
% -------------------------------------------------------------------------

M = -K_R.*e_R ...
    - k_Omega*e_Omega ...
    + w_hat*I_moment*omega ...
    - I_moment*(w_hat*Rot_cur'*Rot_desired*omega_desired ...
    - Rot_cur'*Rot_desired*omega_d_dot);

% Adaptive torques
roll_torque_desired = M(1) - adaptive_roll;
pitch_torque_desired = M(2) - adaptive_pitch;
yaw_torque_desired = M(3)/6 - adaptive_yaw;

% -------------------------------------------------------------------------
% Geometric lateral control in SE(3)
% -------------------------------------------------------------------------

adaptive_lateral = [adaptive_x; adaptive_y; adaptive_z];

force_vec = -K_X.*e_x ...
            - K_V.*e_v ...
            - adaptive_lateral ...
            + m*g*e_3 ...
            + m*x_d_ddot;

f = force_vec' * Rot_cur * e_3;

% Force scalar output
thrust_desired = f(1);

% -------------------------------------------------------------------------
% Saturations
% -------------------------------------------------------------------------

if thrust_desired >= 1.6e-3
    thrust_desired = 1.6e-3;
elseif thrust_desired <= 0.5e-3
    thrust_desired = 0.5e-3;
end

% vastly increased bandwidth 
if roll_torque_desired >= 0.5e-4
    roll_torque_desired = 0.5e-4;
elseif roll_torque_desired <= -0.6e-4
    roll_torque_desired = -0.6e-4;
end

if pitch_torque_desired >= 0.2e-6
    pitch_torque_desired = 0.2e-6;
elseif pitch_torque_desired <= -0.2e-6
    pitch_torque_desired = -0.2e-6;
end

if yaw_torque_desired >= 0.9e-7
    yaw_torque_desired = 0.9e-7;
elseif yaw_torque_desired <= -1.0e-7
    yaw_torque_desired = -1.0e-7;
end

end