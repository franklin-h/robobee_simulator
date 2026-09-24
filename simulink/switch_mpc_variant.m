function switch_mpc_variant(variant, mdl)
%SWITCH_MPC_VARIANT Point the MPC wrapper block at the reduced or full-attitude MPC.
%
%   switch_mpc_variant('full')      % full_attitude_mpc_wl   (yaw in the QP)
%   switch_mpc_variant('template')  % upright_template_mpc_wl (reduced attitude)
%   switch_mpc_variant(variant, mdl) for a model other than the default.
%
% The wrapper is the MATLAB Function block whose script forwards its inputs
% to one of the two controllers (identical signatures). Besides swapping the
% call, the weights_vec and k_tau input ports are set to inherited size so
% the block accepts the 14- or 18-element weights and the 2- or 3-element
% k_tau that the setup script defines. The model is saved in place.

if nargin < 2
    mdl = 'updated_target_driver_2026_withVariants_MPC_andwlqp2';
end
switch lower(variant)
    case 'full'
        target = 'full_attitude_mpc_wl';
    case 'template'
        target = 'upright_template_mpc_wl';
    otherwise
        error('switch_mpc_variant: variant must be ''full'' or ''template''');
end
names = {'upright_template_mpc_wl', 'full_attitude_mpc_wl'};

load_system(mdl);
rt = sfroot;
charts = rt.find('-isa', 'Stateflow.EMChart');
nswapped = 0;
for i = 1:numel(charts)
    ch = charts(i);
    if ~strcmp(bdroot(ch.Path), mdl)
        continue;
    end
    % Replace only on CODE lines (a block whose comments merely mention a
    % controller, e.g. desTraj, is left alone).
    lines = splitlines(ch.Script);
    hit = false;
    for L = 1:numel(lines)
        code = regexprep(lines{L}, '%.*$', '');
        for j = 1:numel(names)
            if contains(code, names{j})
                lines{L} = strrep(lines{L}, names{j}, target);
                hit = true;
            end
        end
    end
    if ~hit
        continue;
    end
    ch.Script = strjoin(lines, newline);
    for pname = {'weights_vec', 'k_tau'}
        d = ch.find('-isa', 'Stateflow.Data', 'Name', pname{1});
        for k = 1:numel(d)
            d(k).Props.Array.Size = '-1';
        end
    end
    fprintf('switch_mpc_variant: %s -> calls %s (weights_vec/k_tau ports inherited)\n', ch.Path, target);
    nswapped = nswapped + 1;
end
if nswapped == 0
    error('switch_mpc_variant: no MATLAB Function block calling an MPC controller found in %s', mdl);
end
save_system(mdl);
fprintf('switch_mpc_variant: saved %s\n', mdl);
end
