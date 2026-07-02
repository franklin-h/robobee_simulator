function robobee_tcp_reset_codegen()
%ROBOBEE_TCP_RESET_CODEGEN Close the native RoboBee TCP client socket.
%#codegen

if coder.target('MATLAB')
    return
else
    source_dir = coder.const(fileparts(mfilename('fullpath')));
    coder.updateBuildInfo('addIncludePaths', source_dir);
    coder.updateBuildInfo('addSourceFiles', 'robobee_tcp_client.c', source_dir);
    coder.cinclude('robobee_tcp_client.h');
    coder.ceval('robobee_tcp_reset_c');
end
end
