2% Plot commanded vs actual roll torque from a virtualBee flight log.
% Actual torque is reconstructed as Ixx * d(omega_x)/dt, with omega_x taken
% from the observer output obs_out1 (first element). Note this is the NET
% roll torque (delivered minus aero counter-torque), not delivered torque.

logfile = fullfile(fileparts(mfilename('fullpath')), ...
    'virtualBee flight logs', 'geometric.mat');

Ixx    = 1.95e-9;  % kg m^2, matches target_driver_setup_2022_Ctrl_V_2_4_geometric.m
fflap  = 155;      % Hz, wingbeat frequency (smoothing window = 1 wingbeat)
t_zoom = 0.3;      % s, end of the detail window in the lower panel

L  = load(logfile);
ds = L.data;

% obs_out1 is named; roll_desired_output is unnamed in the Dataset, so
% locate both by name/block path instead of hard-coded indices.
el_w = [];
el_c = [];
for i = 1:ds.numElements
    el = ds.getElement(i);
    if strcmp(el.Name, 'obs_out1')
        el_w = el;
    end
    try
        p = el.BlockPath;
        if p.getLength > 0 && endsWith(p.getBlock(p.getLength), 'roll_desired_output')
            el_c = el;
        end
    catch
    end
end
assert(~isempty(el_w), 'obs_out1 not found in %s', logfile);
assert(~isempty(el_c), 'roll_desired_output not found in %s', logfile);

t       = el_w.Values.Time;
wx      = el_w.Values.Data(:,1);   % body roll rate [rad/s]
tau_cmd = el_c.Values.Data;        % commanded roll torque [Nm]

tau_act_raw = Ixx * gradient(wx, t);
win         = max(3, round((1/fflap) / mean(diff(t))));  % ~1 wingbeat
tau_act_s   = movmean(tau_act_raw, win);

[~, logname] = fileparts(logfile);

fig = figure('Position', [100 100 1400 900]);
tl  = tiledlayout(2, 1, 'TileSpacing', 'compact');

nexttile; hold on; grid on;
plot(t, tau_act_raw*1e6, 'Color', [0.9 0.65 0.6], 'LineWidth', 0.25);
h2 = plot(t, tau_act_s*1e6, 'Color', [0.75 0.2 0.1], 'LineWidth', 1.1);
h1 = plot(t, tau_cmd*1e6, 'b', 'LineWidth', 1.1);
xlabel('t [s]'); ylabel('\tau_x [\muNm]');
title(sprintf('full flight (%.0f s)', t(end)));
legend([h1 h2], ...
    {'commanded \tau_{roll} (roll\_desired\_output)', ...
     sprintf('actual \\tau_{roll} = I_{xx}d\\omega_x/dt (obs\\_out1(1), %.1f ms movmean)', ...
             win*mean(diff(t))*1e3)}, ...
    'Location', 'northeast');

nexttile; hold on; grid on;
m = t <= t_zoom;
plot(t(m), tau_act_raw(m)*1e6, 'Color', [0.9 0.65 0.6], 'LineWidth', 0.25);
plot(t(m), tau_act_s(m)*1e6, 'Color', [0.75 0.2 0.1], 'LineWidth', 1.2);
plot(t(m), tau_cmd(m)*1e6, 'b', 'LineWidth', 1.2);
xlabel('t [s]'); ylabel('\tau_x [\muNm]');
title(sprintf('launch transient (0 - %g s)', t_zoom)); xlim([0 t_zoom]);

title(tl, sprintf('%s — commanded vs actual roll torque (I_{xx} = %.3g kg m^2)', ...
    logname, Ixx), 'Interpreter', 'tex');

outpng = strrep(logfile, '.mat', '_roll_torque_cmd_vs_actual.png');
exportgraphics(fig, outpng, 'Resolution', 150);
fprintf('saved %s\n', outpng);

ss = t > 4;
fprintf('steady state (t>4s): mean cmd = %+.4f uNm, mean actual = %+.4f uNm\n', ...
    mean(tau_cmd(ss))*1e6, mean(tau_act_s(ss))*1e6);
fprintf('transient peaks: cmd %+.2f uNm, actual (1-wingbeat avg) %+.2f uNm\n', ...
    max(abs(tau_cmd))*1e6, max(abs(tau_act_s))*1e6);
