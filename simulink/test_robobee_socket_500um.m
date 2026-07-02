function log = test_robobee_socket_500um(duration_s, dt_s, stroke_gain_um_per_v)
%TEST_ROBOBEE_SOCKET_500UM Codegen-safe 500 um p-p TCP socket test.
%#codegen
%
% log columns:
%   1  time_s
%   2  x_m
%   3  y_m
%   4  z_m
%   5  roll_rad
%   6  pitch_rad
%   7  yaw_rad
%   8  left_voltage_v
%   9  right_voltage_v
%   10 bias_voltage_v
%   11 target_stroke_p2p_um
%
% Start the server separately before running generated code:
%   start_robobee_server(4242)
% or:
%   bazel run //apps:robobee_simulink_server -- --server_port=4242
%
% Example MEX build and run:
%   codegen test_robobee_socket_500um -args {0.05, 1/(180*20), 1.5}
%   log = test_robobee_socket_500um_mex(0.05, 1/(180*20), 1.5);

if nargin < 1 || isempty(duration_s)
    duration_s = 0.05;
end
if nargin < 2 || isempty(dt_s)
    dt_s = 1 / (180 * 20);
end
if nargin < 3 || isempty(stroke_gain_um_per_v)
    stroke_gain_um_per_v = 1.5;
end

target_stroke_p2p_um = 500;
target_stroke_amplitude_um = target_stroke_p2p_um / 2;

effective_voltage_v = target_stroke_amplitude_um / stroke_gain_um_per_v;
left_voltage_v = effective_voltage_v;
right_voltage_v = effective_voltage_v;
bias_voltage_v = 0;

num_steps = max(1, ceil(duration_s / dt_s));
log = zeros(num_steps, 11);

for k = 1:num_steps
    pose = robobee_tcp_step_codegen( ...
        dt_s, left_voltage_v, right_voltage_v, bias_voltage_v);

    log(k, 1:7) = pose;
    log(k, 8) = left_voltage_v;
    log(k, 9) = right_voltage_v;
    log(k, 10) = bias_voltage_v;
    log(k, 11) = target_stroke_p2p_um;
end
end
