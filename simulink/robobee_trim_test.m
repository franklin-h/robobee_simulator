function robobee_trim_test(t_ol, t_cl)
% ROBOBEE_TRIM_TEST  Quick tuning diagnostic for the RoboBee Drake sim.
%   Runs an open-loop trim check (all feedback off) and a closed-loop
%   flight, then prints tilt / position summaries. Uses the current base
%   workspace values of params_vec and control_gain.
%
%   t_ol : open-loop sim duration (default 0.1 s)
%   t_cl : closed-loop sim duration (default 0.3 s)

if nargin < 1 || isempty(t_ol), t_ol = 0.1; end
if nargin < 2 || isempty(t_cl), t_cl = 0.3; end

mdl = 'updated_target_driver_2026_withVariants';
pv  = evalin('base','params_vec');
cg  = evalin('base','control_gain');

fprintf('\n=== params_vec: nu=%.3f eta=%.3f | gains k_R=%.3g k_Om=%.3g k_Rx=%.3g k_z=%.3g k_vz=%.3g ===\n', ...
    pv(8), pv(7), cg(3), cg(4), cg(7), cg(5), cg(6));

% ---- Open-loop trim (all attitude+lateral feedback off) ----
in = Simulink.SimulationInput(mdl);
in = in.setModelParameter('StopTime', num2str(t_ol));
in = in.setVariable('params_vec', pv);
in = in.setVariable('control_gain', zeros(size(cg)));
o  = sim(in);
[til, zbx, zby] = local_tilt(o);
fprintf('[OPEN-LOOP %gs]  max tilt=%.1f deg | end pitch(zb_x)=%+.3f roll(zb_y)=%+.3f\n', ...
    t_ol, max(til), zbx(end), zby(end));

% ---- Closed-loop flight ----
in = Simulink.SimulationInput(mdl);
in = in.setModelParameter('StopTime', num2str(t_cl));
in = in.setVariable('params_vec', pv);
in = in.setVariable('control_gain', cg);
o  = sim(in);
[til, ~, ~] = local_tilt(o);
t   = o.logsout.getElement(1).Values.Time;
pos = o.logsout.getElement(3).Values.Data;
fi  = find(til > 90, 1);
if isempty(fi), ft = 'NO FLIP'; else, ft = sprintf('%.3fs', t(fi)); end
fprintf('[CLOSED-LOOP %gs] max tilt=%.1f deg | flip=%s | end pos=[%+.4f %+.4f %+.4f] (z target 0.03)\n', ...
    t_cl, max(til), ft, pos(end,1), pos(end,2), pos(end,3));
fprintf('   tilt trace: ');
for k = 1:floor((numel(t)-1)/10):numel(t), fprintf('%.0f ', til(k)); end
fprintf('\n');
end

function [tilt, zbx, zby] = local_tilt(o)
Rm  = o.logsout.getElement(1).Values.Data;   % [R11 R12 R13 R21 R22 R23 R31 R32 R33]
tilt = acosd(max(min(Rm(:,9),1),-1));
zbx  = Rm(:,3);   % world-X component of body z-axis (pitch lean)
zby  = Rm(:,6);   % world-Y component of body z-axis (roll lean)
end
