function [time,x_m,y_m,z_m,alpha_rad,beta_rad,gamma_rad] = robobee_tcp_step(dt_s, ...
    left_voltage_v, right_voltage_v, bias_voltage_v, host, port)
%ROBOBEE_TCP_STEP Step the Bazel-built Drake RoboBee server once.
%
% y = [x_m; y_m; z_m; alpha_rad; beta_rad; gamma_rad]
% where alpha, beta, gamma correspond to roll, pitch, yaw from the server.
%
% Request payload = [dt_s, left_voltage_v, right_voltage_v, bias_voltage_v].
%
% This helper keeps one persistent TCP connection open. Use it from a
% MATLAB Function block running in interpreted mode, or call it from MATLAB
% while prototyping the Simulink interface. Call robobee_tcp_step('reset') to
% close the persistent socket.

persistent client

host_was_supplied_as_fourth_arg = false;
port_was_supplied_as_fifth_arg = false;

if nargin >= 1 && (isstring(dt_s) || ischar(dt_s))
    if strcmp(string(dt_s), "reset")
        client = [];
        y = [];
        return
    end
end

if nargin < 4 || isempty(bias_voltage_v)
    bias_voltage_v = 0;
elseif isstring(bias_voltage_v) || ischar(bias_voltage_v)
    if nargin >= 5
        port = host;
        port_was_supplied_as_fifth_arg = true;
    end
    host = bias_voltage_v;
    host_was_supplied_as_fourth_arg = true;
    bias_voltage_v = 0;
end

if (nargin < 5 && ~host_was_supplied_as_fourth_arg) || isempty(host)
    host = "127.0.0.1";
end
if (nargin < 6 && ~port_was_supplied_as_fifth_arg) || isempty(port)
    port = 4242;
end

if isempty(client)
    client = tcpclient(host, port, "Timeout", 10);
end

request = double([dt_s, left_voltage_v, right_voltage_v, bias_voltage_v]);
write(client, request, "double");

response = read(client, 7, "double");
if numel(response) ~= 7
    error("robobee_tcp_step:ShortRead", ...
        "Expected 7 doubles from RoboBee server, received %d.", ...
        numel(response));
end

time       = response(1); 
x_m        = response(2);
y_m        = response(3);
z_m        = response(4);
alpha_rad  = response(5);  % roll
beta_rad   = response(6);  % pitch
gamma_rad  = response(7);  % yaw

% y = double([time; x_m; y_m; z_m; alpha_rad; beta_rad; gamma_rad]');
end