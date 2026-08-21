% Plot requested vs actual roll torque from a REAL RoboBee flight log.
%
% Hardware counterpart to plot_roll_torque_cmd_vs_actual.m (which reads
% Simulink Dataset logs from the simulator). Reads the post-processed
% workspace logs in "Robobee flight logs/" -- the ones saved after running
% target_driver_exec_2022_Ctrl_V_2_4_plotxyz.m, so the named variables
% (roll_desired_output, omega_x, Ixx, time_vicon, ...) are already present.
% Falls back to raw target logs that contain only `yout`.
%
% Requested torque : roll_desired_output, the geometric controller's desired
%                    roll moment [Nm].
% Actual torque    : Ixx * d(omega_x)/dt, the NET roll moment on the body
%                    (delivered actuator moment MINUS aero counter-torque).
%                    Set add_counter_torque=true to estimate delivered.
%
% VICON LATENCY: omega_x is derived from Vicon, so it is stale by the sense
% pipeline latency, while roll_desired_output is generated on the target and
% is not. Plotting both against time_vicon (as plotxyz.m does) therefore
% inflates any apparent command->response delay. vicon_latency_s shifts the
% Vicon-derived trace earlier to undo this. Default 0 = no compensation,
% matching plotxyz.m; the script prints the empirical lag so you can set it.

logfile = fullfile(fileparts(mfilename('fullpath')), 'Robobee flight logs', ...
    '20220512_BBee_adaptive_landing_hovering1.mat');

vicon_latency_s   = 0;      % [s] sense latency to remove from Vicon traces
smooth_ms         = 10;     % [ms] smoothing window for the differentiation
add_counter_torque = false; % add b_roll*Ixx*omega_x to estimate DELIVERED torque
b_roll            = 47.0;   % [1/s] roll damping -- from SIMULATOR step ID,
                            % not measured on hardware. Provisional.
Ixx_default       = 1.42e-9;% [kg m^2] used only if the log has no Ixx

%% ---- load ---------------------------------------------------------------
% These logs contain a saved SimulinkRealTime.target object, so a bulk
% load() fails outright; whos('-file') cannot enumerate them fully either,
% and asking load() for a name that is absent also errors. So every variable
% is fetched individually via getvar() below, which returns a default when
% the variable is missing or unreadable.
[t,       have_t  ] = getvar(logfile, 'time_vicon', []);
[tau_cmd, have_cmd] = getvar(logfile, 'roll_desired_output', []);
[wx,      have_wx ] = getvar(logfile, 'omega_x', []);

if ~(have_t && have_cmd && have_wx)
    % Raw target log: column mapping per target_driver_exec_..._plotxyz.m
    [yout, have_yout] = getvar(logfile, 'yout', []);
    if ~have_yout
        error('%s has neither the named signals nor yout.', logfile);
    end
    t = yout(:,1);  wx = yout(:,12);  tau_cmd = yout(:,55);
end

[Ixx, from_log] = getvar(logfile, 'Ixx', Ixx_default);

t = t(:); wx = wx(:); tau_cmd = tau_cmd(:);
dt = mean(diff(t));

% Flight window: prefer the log's own start_delay/running_time.
t0 = getvar(logfile, 'start_delay',  t(1));
t1 = getvar(logfile, 'running_time', t(end));
t0 = max(t0, t(1));  t1 = min(t1, t(end));

%% ---- actual torque from d(omega_x)/dt -----------------------------------
% Vicon updates at fs_vicon (500 Hz here) inside a 10 kHz log, so the
% wingbeat ripple is not recoverable -- this smoothing is for differentiation
% noise, not for wingbeat averaging as in the simulator script.
win = max(3, round((smooth_ms*1e-3)/dt));
wx_s        = movmean(wx, win);
tau_act     = movmean(Ixx * gradient(wx_s, t), win);
tau_act_lbl = 'actual net \tau_{roll} = I_{xx}d\omega_x/dt';

if add_counter_torque
    tau_act     = tau_act + Ixx * b_roll * wx_s;
    tau_act_lbl = 'actual delivered \tau_{roll} = I_{xx}(d\omega_x/dt + b\omega_x)';
end

t_act = t - vicon_latency_s;   % Vicon-derived -> shift to physical time

%% ---- empirical command->response lag (report only) ----------------------
m   = t >= t0 & t <= t1;
a   = tau_cmd(m) - mean(tau_cmd(m));
b   = tau_act(m) - mean(tau_act(m));
lags_ms   = -5:0.5:40;
score     = zeros(size(lags_ms));
for i = 1:numel(lags_ms)
    n = round(lags_ms(i)*1e-3/dt);
    if n >= 0
        aa = a(1:end-n);  bb = b(1+n:end);       % actual lags command by n
    else
        aa = a(1-n:end);  bb = b(1:end+n);
    end
    score(i) = (aa'*bb) / sqrt((aa'*aa)*(bb'*bb));
end
[best_corr, k] = max(score);

%% ---- plot --------------------------------------------------------------
[~, logname] = fileparts(logfile);

fig = figure('Position', [100 100 1400 900]);
tl  = tiledlayout(2, 1, 'TileSpacing', 'compact');

nexttile; hold on; grid on;
h1 = plot(t, tau_cmd*1e6, 'b', 'LineWidth', 0.9);
h2 = plot(t_act, tau_act*1e6, 'Color', [0.75 0.2 0.1], 'LineWidth', 0.9);
xlabel('t [s]'); ylabel('\tau_x [\muNm]');
title(sprintf('full log (%.1f - %.1f s)', t(1), t(end)));
legend([h1 h2], {'requested \tau_{roll} (roll\_desired\_output)', ...
    sprintf('%s (%g ms smooth)', tau_act_lbl, smooth_ms)}, 'Location', 'northeast');

nexttile; hold on; grid on;
plot(t(m), tau_cmd(m)*1e6, 'b', 'LineWidth', 1.0);
plot(t_act(m), tau_act(m)*1e6, 'Color', [0.75 0.2 0.1], 'LineWidth', 1.0);
xlabel('t [s]'); ylabel('\tau_x [\muNm]');
title(sprintf('flight window (%.2f - %.2f s)', t0, t1)); xlim([t0 t1]);

title(tl, sprintf('%s — requested vs actual roll torque (I_{xx} = %.3g kg m^2, Vicon latency removed: %g ms)', ...
    logname, Ixx, vicon_latency_s*1e3), 'Interpreter', 'tex');

outpng = fullfile(fileparts(logfile), [logname '_roll_torque_cmd_vs_actual.png']);
exportgraphics(fig, outpng, 'Resolution', 150);
fprintf('saved %s\n', outpng);

%% ---- summary -----------------------------------------------------------
if from_log, src = 'from log'; else, src = 'default'; end
fprintf('Ixx = %.3g kg m^2 (%s)\n', Ixx, src);
fprintf('flight window %.2f-%.2f s, log dt = %.2g s\n', t0, t1, dt);
fprintf('requested: mean %+.4f uNm, rms %.4f uNm, peak %.4f uNm\n', ...
    mean(tau_cmd(m))*1e6, sqrt(mean(tau_cmd(m).^2))*1e6, max(abs(tau_cmd(m)))*1e6);
fprintf('actual   : mean %+.4f uNm, rms %.4f uNm, peak %.4f uNm\n', ...
    mean(tau_act(m))*1e6, sqrt(mean(tau_act(m).^2))*1e6, max(abs(tau_act(m)))*1e6);
fprintf('rms ratio actual/requested = %.3f\n', ...
    sqrt(mean(tau_act(m).^2)) / sqrt(mean(tau_cmd(m).^2)));
fprintf('best command->response lag = %+.1f ms (corr %.3f)\n', lags_ms(k), best_corr);
if abs(best_corr) < 0.3
    fprintf('  ** corr < 0.3: the lag estimate is NOT meaningful for this log.\n');
    fprintf('     See the band table below -- if command and response occupy\n');
    fprintf('     different bands there is no shared dynamics to align.\n');
else
    fprintf('  -> consider setting vicon_latency_s if that exceeds the ~0-1 ms\n');
    fprintf('     actuation delay measured open loop.\n');
end

% Band power split. Command and response must share a band for the
% comparison to mean anything: a command that is all DC trim and a response
% that is all high-frequency disturbance are simply unrelated signals.
edges = [0 1 5 16 40 100 500];
Fs    = 1/dt;
sigs  = {tau_cmd(m), wx(m)};
P     = zeros(2, numel(edges)-1);
for s = 1:2
    x = sigs{s} - mean(sigs{s});
    N = numel(x);  X = abs(fft(x)).^2;  fx = (0:N-1)*Fs/N;  h = 1:floor(N/2);
    for i = 1:numel(edges)-1
        kk = h(fx(h) >= edges(i) & fx(h) < edges(i+1));
        P(s,i) = sum(X(kk));
    end
    P(s,:) = P(s,:) / sum(P(s,:)) * 100;
end
fprintf('\n%-12s %10s %10s\n', 'band [Hz]', 'cmd %', 'omega_x %');
for i = 1:numel(edges)-1
    fprintf('%4g-%-7g %10.2f %10.2f\n', edges(i), edges(i+1), P(1,i), P(2,i));
end

function [val, found] = getvar(file, name, default)
% Fetch one variable from a .mat file, tolerating absent/unreadable entries.
val = default; found = false;
ws = warning('off', 'all');
try
    tmp = load(file, name);
    if isfield(tmp, name)
        val = tmp.(name); found = true;
    end
catch
end
warning(ws);
end
