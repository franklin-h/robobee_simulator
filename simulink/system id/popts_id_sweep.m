%% popts_id_sweep.m
% Open-loop sweep that produces the data set for fit_popts.m: the quadratic
% wrench map w(u) used by the WLQP (wlqp.m), fit in the WLQP's NATIVE input
% coordinates u = [Vmean; uoffs; udiff; h2].
%
% RELATION TO system_id_sweep.m
%   Same plant, same serial run/reset machinery, same steady-state averaging.
%   Differences:
%     * Points are generated in u-space and converted to plant drive commands
%       through wlqp2driver.m -- the SAME conversion block that sits between
%       wlqp.m and the plant in closed loop. Fitting through the identical
%       converter makes the popts signs/scales consistent by construction.
%     * Records the full 6-component wrench (Fx, Fy, Fz, taux, tauy, tauz).
%       REQUIRES the extended server that logs force_x_N/force_y_N (7-column
%       CSV) -- rebuild/relaunch bazel-bin servers from after that change.
%     * Adds a cross-term campaign (Latin-hypercube batch over the full 4-D
%       u box). Per-axis slices alone leave the uoffs*udiff, uoffs*h2 and
%       udiff*h2 quadratic cross-terms structurally unobservable (their
%       regressor columns are identically zero), which would silently pin 3
%       of the 15 coefficients per wrench row to the ridge prior.
%
% OUTPUT (popts_id_sweep_results_<stamp>.mat), ready for fit_popts.m:
%   U            n x 4   u-space commands [Vmean, uoffs, udiff, h2]
%   W_SI         n x 6   steady-state wrench [Fx Fy Fz taux tauy tauz], N / N*m,
%                        controller axes (+x forward, +y left, +z up)
%   W_template   n x 6   same wrench in template units (mg, mm, ms):
%                        forces *1e3 (-> mg*mm/ms^2), torques *1e6 (-> mg*mm^2/ms^2)
%                        -- the units wlqp.m / popts operate in
%   W_std_SI     n x 6   per-point std over the averaging window (noise weight)
%   campaign     n x 1   1=Vmean x uoffs, 2=Vmean x udiff, 3=Vmean x h2, 4=LHS cross
%   drv          n x 6   realized plant commands [drv_amp, drv_pitch_left,
%                        drv_pitch_right, drv_roll, a2, required_rail_V]
%   results, meta        full per-run records (incl. traces) and settings
%
% Author: generated for the RoboBee simulator WLQP/popts workflow.

clc; close all;

%% ------------------------------------------------------------------------
% 0) Resolve paths and populate the full model parameter workspace.
% -------------------------------------------------------------------------
% NOTE: the base setup script runs "clear all", so NOTHING defined above this
% point survives it. All configuration below is declared AFTER the setup call.
this_dir_bootstrap = fileparts(mfilename('fullpath'));
simulink_dir_bootstrap = fileparts(this_dir_bootstrap);
cd(simulink_dir_bootstrap);
fprintf('Loading base parameters from target_driver_setup_2022_Ctrl_V_2_4.m ...\n');
try
    run(fullfile(simulink_dir_bootstrap, 'target_driver_setup_2022_Ctrl_V_2_4.m'));
catch base_setup_err
    fprintf('  base setup complete (ignoring expected trailing error: %s)\n', ...
        base_setup_err.message);
end

mdl          = 'updated_target_driver_2026_withVariants';
this_dir     = fileparts(mfilename('fullpath'));
simulink_dir = fileparts(this_dir);
repo_root    = fileparts(simulink_dir);
addpath(simulink_dir);   % wlqp.m / wlqp2driver.m live here
addpath(this_dir);
if ~bdIsLoaded(mdl)
    load_system(mdl);
end

%% ------------------------------------------------------------------------
% 1) USER SETTINGS  (edit these)
% -------------------------------------------------------------------------
PORT            = 4242;      % Drake server TCP port
SYSID_DURATION  = 0.3;       % [s] per-run open-loop hold (transient + averaging)
BIAS_V          = 200;       % max_drv_bias (PZT bias rail), held FIXED across the
                             %   sweep so it is not a hidden 5th input to the fit

AVG_CYCLES       = 30;       % flap cycles spanned by the averaging window
TRANSIENT_CYCLES = 6;        % initial flap cycles dropped before averaging
SPIKE_MAD_FACTOR = Inf;      % gross-spike guard (Inf = disabled)
FREQ_HZ          = 155;        % flap frequency from the base setup

USE_FIXED_AIRFRAME    = true;   % welded airframe -> logged wrench is pure aero
FORCE_SERVER_RELAUNCH = false;

% ---- u-space fit box -----------------------------------------------------
% This box is the region popts will be TRUSTED in: make it match (or slightly
% exceed) the umin/umax box you will give wlqp.m in closed loop. The WLQP
% linearizes the map at every step, so fit quality inside this box matters far
% more than range coverage outside it.
%
% Driver-units sanity (via wlqp2driver): drv_pitch = Vmean*uoffs, so
% uoffs 0.12 @ 165 V ~ 20 V pitch bias; drv_roll = Vmean*udiff/2, so
% udiff 0.12 @ 165 V ~ 10 V differential. h2 == a2 directly.
VMEAN_LIST = [90 110 130 150 170 190];                       % common amplitude [V]
UOFFS_LIST = [-0.23 -0.18 -0.14 -0.10 -0.05 0 0.05 0.10 0.14 0.18];   % pitch channel
UDIFF_LIST = [-0.12 -0.09 -0.06 -0.03 0 0.03 0.06 0.09 0.12];   % roll channel
H2_LIST    = [-0.25 -0.2 -0.15 -0.1 0 0.1 0.15 0.2 0.25];       % yaw channel

% Cross-term campaign: space-filling points over the FULL 4-D box. These are
% what make the 3 off-axis quadratic cross-terms identifiable.
N_CROSS    = 48;
CROSS_SEED = 42;             % rng seed -> the batch is reproducible

% Which campaigns to include.
DO_UOFFS_SWEEP = true;    % Vmean x uoffs  (thrust + pitch)
DO_UDIFF_SWEEP = true;    % Vmean x udiff  (thrust + roll)
DO_H2_SWEEP    = true;    % Vmean x h2     (thrust + yaw)
DO_CROSS_SWEEP = true;    % LHS batch      (cross-terms; strongly recommended)

DRIVER_SIGNS = [1; 1; 1]; % wlqp2driver signs for [pitch; roll; yaw]. MUST match
                          %   what the closed-loop model uses. Fitting and flying
                          %   through the same conversion is the whole point.

SAVE_TRACES    = true;
COM_WRENCH_CSV = '/tmp/robobee_com_wrench.csv';

if abs(FREQ_HZ - f) > eps
    warning(['FREQ_HZ (%g) differs from the frequency the base setup derived its ', ...
             'filters/delays at (%g).'], FREQ_HZ, f);
end

%% ------------------------------------------------------------------------
% 2) Configure the model for open-loop system ID.
% -------------------------------------------------------------------------
assignin('base', 'Plant',           2);   % Drake simulation variant
assignin('base', 'control_flag',    1);   % open loop
assignin('base', 'closedloop_flag', 0);
assignin('base', 'landing_flag',    0);
assignin('base', 'f',               FREQ_HZ);
assignin('base', 'drv_bias',        BIAS_V);
assignin('base', 'max_drv_bias',    BIAS_V);

set_param(mdl, 'SolverType', 'Fixed-step');
set_param(mdl, 'Solver',     'FixedStepDiscrete');
set_param(mdl, 'FixedStep',  'dt_s');

dt_s_val = evalin('base', 'dt_s');
fprintf('Model: open-loop Drake simulation, dt = %.4g s, run = %.3g s (%d steps/run)\n', ...
    dt_s_val, SYSID_DURATION, round(SYSID_DURATION/dt_s_val));
min_dur = (AVG_CYCLES + TRANSIENT_CYCLES) / FREQ_HZ;
if SYSID_DURATION < min_dur * 1.1
    warning(['SYSID_DURATION (%.3g s) is short for a %d-cycle window + %d-cycle ', ...
             'transient at %g Hz (needs >= ~%.3g s).'], ...
             SYSID_DURATION, AVG_CYCLES, TRANSIENT_CYCLES, FREQ_HZ, min_dur*1.25);
end

%% ------------------------------------------------------------------------
% 3) Build the u-space point list.
% -------------------------------------------------------------------------
% U columns: [Vmean, uoffs, udiff, h2]; campaign: 1=uoffs, 2=udiff, 3=h2, 4=cross
U = zeros(0, 4);
campaign = zeros(0, 1);

if DO_UOFFS_SWEEP
    for vm = VMEAN_LIST
        for uo = UOFFS_LIST
            U(end+1, :) = [vm, uo, 0, 0]; campaign(end+1, 1) = 1; %#ok<SAGROW>
        end
    end
end
if DO_UDIFF_SWEEP
    for vm = VMEAN_LIST
        for ud = UDIFF_LIST
            U(end+1, :) = [vm, 0, ud, 0]; campaign(end+1, 1) = 2; %#ok<SAGROW>
        end
    end
end
if DO_H2_SWEEP
    for vm = VMEAN_LIST
        for h2 = H2_LIST
            U(end+1, :) = [vm, 0, 0, h2]; campaign(end+1, 1) = 3; %#ok<SAGROW>
        end
    end
end
if DO_CROSS_SWEEP
    % Latin hypercube over the full box: one stratified sample per dimension,
    % randomly paired. Deterministic via CROSS_SEED.
    rng(CROSS_SEED);
    lo = [min(VMEAN_LIST), min(UOFFS_LIST), min(UDIFF_LIST), min(H2_LIST)];
    hi = [max(VMEAN_LIST), max(UOFFS_LIST), max(UDIFF_LIST), max(H2_LIST)];
    strata = ((1:N_CROSS)' - rand(N_CROSS, 4)) / N_CROSS;   % n x 4 in (0,1)
    lhs01 = zeros(N_CROSS, 4);
    for d = 1:4
        lhs01(:, d) = strata(randperm(N_CROSS), d);
    end
    Ucross = lo + lhs01 .* (hi - lo);
    U = [U; Ucross];
    campaign = [campaign; 4*ones(N_CROSS, 1)];
end

if isempty(U)
    error('popts_id_sweep:noPoints', 'No campaigns enabled - nothing to sweep.');
end

% Drop duplicate points (the Vmean/0/0/0 baselines shared by campaigns 1-3).
[~, keep] = unique(U, 'rows', 'stable');
U = U(keep, :);
campaign = campaign(keep);
n_runs = size(U, 1);
fprintf('Sweep: %d unique u-space points (uoffs=%d, udiff=%d, h2=%d, cross=%d).\n', ...
    n_runs, nnz(campaign == 1), nnz(campaign == 2), nnz(campaign == 3), ...
    nnz(campaign == 4));

% ---- convert every u point to plant drive commands via wlqp2driver --------
% drv columns: [drv_amp, drv_pitch_left, drv_pitch_right, drv_roll, a2, rail_V]
drv = zeros(n_runs, 6);
for i = 1:n_runs
    [amp_i, dpl_i, dpr_i, roll_i, a2_i, rail_i] = ...
        wlqp2driver(U(i, :).', DRIVER_SIGNS);
    drv(i, :) = [amp_i, dpl_i, dpr_i, roll_i, a2_i, rail_i];
end
n_over = nnz(drv(:, 6) > BIAS_V);
if n_over > 0
    fprintf(['NOTE: %d/%d points need a rail above BIAS_V=%g V (max %.1f V). ', ...
             'drv_bias stays fixed at BIAS_V (same convention as ', ...
             'system_id_sweep.m); required rail is recorded in drv(:,6).\n'], ...
        n_over, n_runs, BIAS_V, max(drv(:, 6)));
end

%% ------------------------------------------------------------------------
% 4) Make sure the correct Drake server is up.
% -------------------------------------------------------------------------
if USE_FIXED_AIRFRAME
    server_target = '//apps:robobee_simulink_server_fixed';
    server_binary = 'bazel-bin/apps/robobee_simulink_server_fixed';
else
    server_target = '//apps:robobee_simulink_server';
    server_binary = 'bazel-bin/apps/robobee_simulink_server';
end

if FORCE_SERVER_RELAUNCH && is_server_up('127.0.0.1', PORT)
    fprintf('Stopping any server on port %d (FORCE_SERVER_RELAUNCH)...\n', PORT);
    system('pkill -f robobee_simulink_server');
    pause(1.0);
end

if is_server_up('127.0.0.1', PORT)
    fprintf(['A server is already listening on 127.0.0.1:%d - reusing it.\n', ...
             '  >> It must be a build that logs force_x_N/force_y_N (7-column ', ...
             'CSV)\n     AND the %s build for valid data.\n'], PORT, server_binary);
else
    fprintf('Launching %s on port %d ...\n', server_binary, PORT);
    launch_drake_server(repo_root, server_target, server_binary, PORT);
    if ~is_server_up('127.0.0.1', PORT)
        error('popts_id_sweep:noServer', ...
            ['Drake server not reachable on port %d after launch.\n', ...
             'Start it manually with:\n  bazel run %s -- --server_port=%d'], ...
            PORT, server_target, PORT);
    end
end
fprintf('Drake server is reachable on 127.0.0.1:%d.\n', PORT);

%% ------------------------------------------------------------------------
% 5) Run the sweep (strictly serial; reset the plant between points).
% -------------------------------------------------------------------------
results = struct('campaign', {}, 'u', {}, 'drv_amp', {}, 'drv_roll', {}, ...
    'drv_pitch_left', {}, 'drv_pitch_right', {}, 'a2_openloop', {}, ...
    'rail_required_V', {}, 'wrench_SI', {}, 'wrench_std_SI', {}, ...
    'n_avg', {}, 'n_outliers', {}, 't_end', {}, 'sim_seconds', {}, ...
    'wrench_trace', {});

stamp    = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
out_file = fullfile(this_dir, sprintf('popts_id_sweep_results_%s.mat', stamp));

reset_plant(mdl);   % clean slate before the first run
sweep_t0 = tic;

%%
for i = 1:n_runs
    amp    = drv(i, 1);  pitchL = drv(i, 2);  pitchR = drv(i, 3);
    roll   = drv(i, 4);  a2     = drv(i, 5);

    % ---- run one open-loop hold ----
    in = Simulink.SimulationInput(mdl);
    in = in.setVariable('drv_amp',         amp);
    in = in.setVariable('drv_roll',        roll);
    in = in.setVariable('drv_pitch_left',  pitchL);
    in = in.setVariable('drv_pitch_right', pitchR);
    in = in.setVariable('a2_openloop',     a2);
    in = in.setVariable('drv_bias',        BIAS_V);
    in = in.setModelParameter('StopTime',  num2str(SYSID_DURATION, '%.6g'));

    % Run + read, retrying once on a truncated wrench CSV. A short file means
    % the steady-state "average" spans raw flap ripple (worst observed: 13
    % samples -> phantom +/-15 mN*mm torque) and poisons the popts fit -- see
    % the matching data-quality guard in fit_popts.m.
    for attempt = 1:2
        run_t0 = tic;
        sim(in);                   % output is captured via the server wrench CSV
        sim_seconds = toc(run_t0);

        % ---- reset socket -> server flushes+closes this run's CSV ----
        reset_plant(mdl);
        W = read_wrench_csv6(COM_WRENCH_CSV);
        if W(end, 1) >= 0.95 * SYSID_DURATION, break; end
        fprintf('  truncated CSV on run %d (t_end %.3f s < %.3f s) -- retrying\n', ...
            i, W(end, 1), 0.95 * SYSID_DURATION);
    end
    if W(end, 1) < 0.95 * SYSID_DURATION
        warning('popts_id_sweep:truncatedRun', ...
            ['Run %d still truncated after retry (t_end %.3f s of %.3g s); ', ...
             'fit_popts.m will drop it.'], i, W(end, 1), SYSID_DURATION)
    end

    % ---- steady-state average over an integer number of flap cycles ----
    t = W(:, 1);
    dt_data   = median(diff(t));
    per_cycle = 1 / (FREQ_HZ * dt_data);
    window    = max(1, round(AVG_CYCLES * per_cycle));
    start_idx = max(1, round(TRANSIENT_CYCLES * per_cycle) + 1);
    lo_idx = max(start_idx, size(W, 1) - window + 1);
    % CSV cols: 2=Fz 3=taux 4=tauy 5=tauz 6=Fx 7=Fy -> reorder to the wlqp/popts
    % wrench convention w = [Fx Fy Fz taux tauy tauz]
    seg = W(lo_idx:end, [6 7 2 3 4 5]);
    [mean_w, std_w, n_rej] = cycle_mean(seg, SPIKE_MAD_FACTOR);
    n_used = size(seg, 1);

    r.campaign = campaign(i);
    r.u = U(i, :);
    r.drv_amp = amp;  r.drv_roll = roll;
    r.drv_pitch_left = pitchL;  r.drv_pitch_right = pitchR;
    r.a2_openloop = a2;  r.rail_required_V = drv(i, 6);
    r.wrench_SI = mean_w;         % [Fx Fy Fz taux tauy tauz], N / N*m
    r.wrench_std_SI = std_w;
    r.n_avg = n_used;  r.n_outliers = n_rej;
    r.t_end = t(end);  r.sim_seconds = sim_seconds;
    if SAVE_TRACES, r.wrench_trace = W; else, r.wrench_trace = []; end
    results(i) = r;

    fprintf(['[%3d/%3d] c%d Vm=%3g uo=%+6.3f ud=%+6.3f h2=%+5.2f | ', ...
             'F=[%+6.3f %+6.3f %6.3f] mN  M=[%+7.3f %+7.3f %+7.3f] mNmm | %.1fs\n'], ...
        i, n_runs, campaign(i), U(i,1), U(i,2), U(i,3), U(i,4), ...
        1e3*mean_w(1), 1e3*mean_w(2), 1e3*mean_w(3), ...
        1e6*mean_w(4), 1e6*mean_w(5), 1e6*mean_w(6), sim_seconds);

    save(out_file, 'results', 'U', 'campaign', 'drv', '-v7.3');  % checkpoint
end

fprintf('Sweep finished: %d runs in %.1f s.\n', n_runs, toc(sweep_t0));

%% ------------------------------------------------------------------------
% 6) Assemble the fit-ready arrays and save.
% -------------------------------------------------------------------------
W_SI     = reshape([results.wrench_SI],     6, []).';   % n x 6
W_std_SI = reshape([results.wrench_std_SI], 6, []).';   % n x 6

% Template units (mg, mm, ms) -- what wlqp.m/popts consume:
%   force:  1 N = 1e3 mg*mm/ms^2      torque: 1 N*m = 1e6 mg*mm^2/ms^2
W_template = W_SI .* [1e3 1e3 1e3 1e6 1e6 1e6];

% Outlier filter: same spirit as system_id_sweep's yaw guard -- drop points
% whose steady-state yaw torque is non-physically large (saturated waveform).
YAW_TORQUE_MAX_MNMM = 1000;                     % [mN*mm]
keep = abs(1e6 * W_SI(:, 6)) < YAW_TORQUE_MAX_MNMM;
if ~all(keep)
    fprintf('Filter: dropped %d of %d points with |yaw| >= %g mN*mm.\n', ...
        nnz(~keep), numel(keep), YAW_TORQUE_MAX_MNMM);
end

meta = struct('model', mdl, 'sysid_duration_s', SYSID_DURATION, ...
    'avg_cycles', AVG_CYCLES, 'transient_cycles', TRANSIENT_CYCLES, ...
    'bias_v', BIAS_V, 'freq_hz', FREQ_HZ, 'dt_s', dt_s_val, ...
    'timestamp', stamp, 'port', PORT, 'driver_signs', DRIVER_SIGNS, ...
    'cross_seed', CROSS_SEED, 'n_cross', N_CROSS, ...
    'u_box_lo', [min(VMEAN_LIST), min(UOFFS_LIST), min(UDIFF_LIST), min(H2_LIST)], ...
    'u_box_hi', [max(VMEAN_LIST), max(UOFFS_LIST), max(UDIFF_LIST), max(H2_LIST)], ...
    'wrench_order', '[Fx Fy Fz taux tauy tauz], controller axes (+x fwd, +y left)', ...
    'u_order', '[Vmean uoffs udiff h2]', ...
    'template_units', 'forces mg*mm/ms^2 (=1e-3 N), torques mg*mm^2/ms^2 (=1e-6 N*m)');
meta.n_kept  = nnz(keep);
meta.n_total = numel(keep);

U_fit          = U(keep, :);
W_template_fit = W_template(keep, :);
W_SI_fit       = W_SI(keep, :);
W_std_fit      = W_std_SI(keep, :);
campaign_fit   = campaign(keep);

save(out_file, 'results', 'U', 'campaign', 'drv', 'W_SI', 'W_std_SI', ...
    'W_template', 'keep', 'U_fit', 'W_template_fit', 'W_SI_fit', ...
    'W_std_fit', 'campaign_fit', 'meta', '-v7.3');
fprintf('Saved %d points (%d after filter) to:\n  %s\n', n_runs, nnz(keep), out_file);
fprintf(['Next: run fit_popts.m on this file. It should fit W_template_fit ', ...
         'against U_fit\n(15 quadratic coefficients per wrench row, ', ...
         'funapprox row-major packing).\n']);

%% ------------------------------------------------------------------------
% 7) Quick-look plots: each per-axis campaign vs its dominant wrench channels.
% -------------------------------------------------------------------------
figure('Name', 'popts ID sweep summary', 'Color', 'w');
chan_names = {'F_x [mN]', 'F_y [mN]', 'F_z [mN]', ...
              '\tau_x [mN\cdotmm]', '\tau_y [mN\cdotmm]', '\tau_z [mN\cdotmm]'};
scales = [1e3 1e3 1e3 1e6 1e6 1e6];
xvars  = {2, 3, 4};   % campaign 1 -> uoffs, 2 -> udiff, 3 -> h2
xnames = {'uoffs', 'udiff', 'h2'};
for c = 1:3
    idx = campaign_fit == c;
    for k = 1:6
        subplot(3, 6, (c-1)*6 + k); hold on; grid on;
        if any(idx)
            scatter(U_fit(idx, xvars{c}), scales(k)*W_SI_fit(idx, k), 14, ...
                U_fit(idx, 1), 'filled');   % color by Vmean
        end
        if c == 1, title(chan_names{k}, 'FontWeight', 'normal'); end
        if k == 1, ylabel(sprintf('campaign %d', c)); end
        xlabel(xnames{c});
    end
end
sgtitle('Steady-state wrench vs u (per-axis campaigns, color = Vmean)');

%% ========================================================================
% Local functions
% =========================================================================
function reset_plant(mdl)
%RESET_PLANT Drop the persistent Drake TCP socket so the next sim() gets a
% fresh simulation (pose reset, wrench CSV truncated).
    sfun = [mdl '_sfun'];
    clear(sfun);
    clear mex; %#ok<CLMEX>
    pause(0.2);
end

function launch_drake_server(repo_root, target, binary, port)
%LAUNCH_DRAKE_SERVER Build (if needed) and launch a Drake TCP server in the
% background from the repo root, then wait until it accepts connections.
    old_dir = cd(repo_root);
    restore = onCleanup(@() cd(old_dir));
    if system(sprintf('bazel build %s', target)) ~= 0
        error('popts_id_sweep:buildFailed', 'bazel build %s failed.', target);
    end
    system(sprintf('./%s --server_port=%d > /tmp/robobee_simulink_server.log 2>&1 &', ...
        binary, port));
    for k = 1:50
        if is_server_up('127.0.0.1', port), break; end
        pause(0.1);
    end
end

function [m, s, n_rej] = cycle_mean(X, mad_factor)
%CYCLE_MEAN Column-wise mean/std over an integer-flap-cycle window.
    n_ch = size(X, 2);
    m = zeros(1, n_ch);
    s = zeros(1, n_ch);
    n_rej = 0;
    for c = 1:n_ch
        col = X(:, c);
        if isfinite(mad_factor) && mad_factor > 0
            bad = isoutlier(col, 'median', 'ThresholdFactor', mad_factor);
        else
            bad = false(size(col));
        end
        n_rej = n_rej + nnz(bad);
        col(bad) = NaN;
        m(c) = mean(col, 'omitnan');
        s(c) = std(col, 'omitnan');
    end
end

function tf = is_server_up(host, port)
%IS_SERVER_UP True if a TCP client can connect to host:port.
    tf = false;
    try
        c = tcpclient(host, port, 'Timeout', 2); %#ok<NASGU>
        clear c;
        tf = true;
    catch
        tf = false;
    end
end

function W = read_wrench_csv6(csv_path)
%READ_WRENCH_CSV6 Read the server COM-wrench log, requiring the 7-column format
% (time_s, thrust_z_N, roll/pitch/yaw torque, force_x_N, force_y_N). Polls
% briefly until the file size is stable (server flushes on client disconnect).
    prev = -1; stable = 0;
    for k = 1:60
        d = dir(csv_path);
        if isempty(d), pause(0.05); continue; end
        if d.bytes == prev, stable = stable + 1; else, stable = 0; end
        if stable >= 3 && d.bytes > 0, break; end
        prev = d.bytes; pause(0.05);
    end
    W = readmatrix(csv_path);
    % The server can cut the FINAL row mid-write in its last column (Fy),
    % leaving a truncated float (e.g. "9.6814" for "9.6814e-05", ~1e5x off).
    % The time column is intact so t_end checks pass, and one bad sample in
    % the averaging window poisons the Fy mean (Fy R2 0.90 -> 0.04 on the
    % 2026-08-19/20 sweeps). Always drop the last row; it costs one dt.
    if size(W, 1) > 1
        W = W(1:end-1, :);
    end
    if isempty(W) || size(W, 2) < 7
        error('popts_id_sweep:badCsv', ...
            ['Wrench CSV %s has %d columns; the popts fit needs the 7-column ', ...
             'format with force_x_N/force_y_N. Rebuild + relaunch the server ', ...
             '(bazel build //apps:robobee_simulink_server_fixed) from a source ', ...
             'tree that includes the Fx/Fy logging change.'], ...
            csv_path, size(W, 2));
    end
    W = W(all(isfinite(W), 2), :);
end
