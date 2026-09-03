function wrenchBias = wlqp_vertical_wrench_bias( ...
    Rot_corrected_vec, v_filtered_W, c_drag_N_per_mps)
%#codegen
%WLQP_VERTICAL_WRENCH_BIAS Velocity-dependent aero corrections to the map.
%
% Inputs match signals available in
% updated_target_driver_2026_withVariants_MPC_andwlqp2.slx:
%   Rot_corrected_vec   9x1 body-to-world rotation matrix, row-major
%   v_filtered_W        3x1 filtered world-frame velocity [m/s]
%   c_drag_N_per_mps    scalar flapping counter-force (drag) coefficient
%                       [N/(m/s)], acts on BODY-X vs body-x airspeed
%                       (ff20 measured 0.72e-3; c_vertical in the setup)
%
% Modeled velocity dependence of the aero wrench (ff20 force balance):
%
%   Fx_actual = Fx_map - c_drag * v_x_body     (counter-force, dominant)
%   Fz_actual = Fz_map - c_z    * v_z_body     (small lift discount)
%
% wrenchBias is the signed correction to the static wrench map, used in
% wlqp.m as w0_corrected = w0 + wrenchBias. Body-frame template wrench
% units: force in 1e-3 N, moment in 1e-6 N*m.

% Body-z lift discount measured from the ff20 per-axis residuals; ~7x
% smaller than the drag coefficient (do NOT reuse c_drag here).
c_z_N_per_mps = 0.10e-3;

Rot_corrected_vec = reshape(Rot_corrected_vec, 9, 1);
v_filtered_W = reshape(v_filtered_W, 3, 1);
c_drag_N_per_mps = c_drag_N_per_mps(1);

R_WB = [Rot_corrected_vec(1), Rot_corrected_vec(2), Rot_corrected_vec(3); ...
        Rot_corrected_vec(4), Rot_corrected_vec(5), Rot_corrected_vec(6); ...
        Rot_corrected_vec(7), Rot_corrected_vec(8), Rot_corrected_vec(9)];

% World velocity in the body frame.
v_B = R_WB' * v_filtered_W;

wrenchBias = zeros(6, 1, 'like', v_filtered_W);
wrenchBias(1) = -1.0e3 * c_drag_N_per_mps * v_B(1);   % counter-force, body-x
wrenchBias(3) = -1.0e3 * c_z_N_per_mps  * v_B(3);     % lift discount, body-z

end
