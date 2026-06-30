function y = robobee_tcp_step(dt_s, left_voltage_v, right_voltage_v, host, port)
%ROBOBEE_TCP_STEP Step the Bazel-built Drake RoboBee server once.
%
% y = [time_s; x_m; y_m; z_m; roll_rad; pitch_rad; yaw_rad]
%
% This helper keeps one persistent TCP connection open. Use it from a
% MATLAB Function block running in interpreted mode, or call it from MATLAB
% while prototyping the Simulink interface. Call robobee_tcp_step('reset') to
% close the persistent socket.

persistent client

if nargin >= 1 && (isstring(dt_s) || ischar(dt_s))
    if strcmp(string(dt_s), "reset")
        client = [];
        y = [];
        return
    end
end

if nargin < 4 || isempty(host)
    host = "127.0.0.1";
end
if nargin < 5 || isempty(port)
    port = 4242;
end

if isempty(client)
    client = tcpclient(host, port, "Timeout", 10);
end

request = double([dt_s, left_voltage_v, right_voltage_v]);
write(client, request, "double");

response = read(client, 7, "double");
if numel(response) ~= 7
    error("robobee_tcp_step:ShortRead", ...
        "Expected 7 doubles from RoboBee server, received %d.", ...
        numel(response));
end

y = reshape(double(response(2:7)), 6, 1);
end
