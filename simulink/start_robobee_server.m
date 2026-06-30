function start_robobee_server(port)
%START_ROBOBEE_SERVER Build and launch the Drake RoboBee TCP server.
%
% Intended for a Simulink model InitFcn callback:
%   start_robobee_server(4242)

if nargin < 1 || isempty(port)
    port = 4242;
end

repo_root = fileparts(fileparts(mfilename("fullpath")));
old_dir = pwd;
cleanup = onCleanup(@() cd(old_dir));
cd(repo_root);

build_status = system("bazel build //apps:robobee_simulink_server");
if build_status ~= 0
    error("start_robobee_server:BuildFailed", ...
        "Could not build //apps:robobee_simulink_server.");
end

server_cmd = sprintf( ...
    "./bazel-bin/apps/robobee_simulink_server --server_port=%d > /tmp/robobee_simulink_server.log 2>&1 &", ...
    port);
launch_status = system(server_cmd);
if launch_status ~= 0
    error("start_robobee_server:LaunchFailed", ...
        "Could not launch RoboBee server.");
end

pause(1.0);
robobee_tcp_step("reset");
end
