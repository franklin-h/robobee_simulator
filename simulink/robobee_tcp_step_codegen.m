function pose = robobee_tcp_step_codegen(dt_s, left_voltage_v, right_voltage_v, bias_voltage_v)
%ROBOBEE_TCP_STEP_CODEGEN Codegen-safe TCP step for the RoboBee server.
%#codegen
%
% pose = [time_s, x_m, y_m, z_m, roll_rad, pitch_rad, yaw_rad, ...
%         thrust_z_N, roll_torque_Nm, pitch_torque_Nm, yaw_torque_Nm]
%
% Elements 8-11 are the net aerodynamic wrench about the robot COM in the
% body frame (z-axis thrust and roll/pitch/yaw torques).
%
% The native C client owns a persistent socket connection to
% 127.0.0.1:4242. It connects on the first call and reuses that connection
% until the process exits or robobee_tcp_reset_c is called from custom code.

pose = zeros(1, 11);

if coder.target('MATLAB')
    error('robobee_tcp_step_codegen:CodegenOnly', ...
        'robobee_tcp_step_codegen requires code generation. Use robobee_tcp_step for interpreted MATLAB.');
else
    source_dir = coder.const(fileparts(mfilename('fullpath')));
    coder.updateBuildInfo('addIncludePaths', source_dir);
    coder.updateBuildInfo('addSourceFiles', 'robobee_tcp_client.c', source_dir);
    coder.cinclude('robobee_tcp_client.h');

    status = int32(0);
    status = coder.ceval( ...
        'robobee_tcp_step_c', ...
        double(dt_s), ...
        double(left_voltage_v), ...
        double(right_voltage_v), ...
        double(bias_voltage_v), ...
        coder.wref(pose));

    if status ~= 0
        pose(:) = NaN;
        pose(1) = double(status);
    end
end
end
