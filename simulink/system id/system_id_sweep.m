%% system_id_sweep.m
% Open-loop system-identification parameter sweep for the Drake simulation
% plant driven through updated_target_driver_2026_withVariants.
%
% WHAT THIS DOES
%   Runs the Simulink model repeatedly in OPEN LOOP (Plant==2, the Drake TCP
%   simulation variant), stepping through a matrix of open-loop drive commands
%   and recording the net aerodynamic wrench (thrust + roll/pitch/yaw torque)
%   the plant produces for each command. The result is the same "measured
%   force/torque vs. commanded voltage" data set that the hardware campaigns in
%   Patrick_Bee_systemID_20231103.xlsx produced, and that
%   data_optimize_full_optimization.m consumes to fit the voltage->wrench model.
%
% SWEEP STRUCTURE (inspired by Patrick_Bee_systemID_20231103.xlsx)
%   Each Excel campaign holds most channels fixed and sweeps one axis at a time:
%     * roll/thrust campaign : drv_amp x drv_roll     (pitch = 0, a2 = 0)
%     * pitch campaign       : drv_amp x drv_pitch    (roll  = 0, a2 = 0)
%   Rather than a full 5-D grid (mostly redundant and expensive), we build the
%   union of those per-axis campaigns and drop duplicate command points. An
%   optional yaw campaign (drv_amp x a2) is included for completeness.
%
% HOW THE PLANT IS DRIVEN / RESET
%   The model's "MATLAB Function" TCP block keeps ONE persistent socket to the
%   Drake server (apps/robobee_simulink_server). The server builds a FRESH
%   simulation (pose reset to the default context, t = 0, wrench CSV truncated)
%   every time a new client connects. So to give every sweep point identical
%   initial conditions we drop the socket between runs by unloading the model
%   sim-target MEX (clear mex). The next sim() reconnects and gets a clean sim.
%
%   The server continuously writes the net COM wrench to
%   /tmp/robobee_com_wrench.csv (columns: time_s, thrust_z_N, roll_torque_Nm,
%   pitch_torque_Nm, yaw_torque_Nm, force_x_N, force_y_N; the last two are in
%   controller axes, +x forward / +y left). Because a reconnect truncates that
%   file, after each run the file contains exactly that run's wrench trace.
%
% PREREQUISITES
%   * MATLAB with the model on the path (this script adds the simulink folder).
%   * The Drake server must be reachable on 127.0.0.1:PORT. If it is not, this
%     script builds+launches it with bazel. Use the WELDED-airframe build
%     (robobee_simulink_server_fixed) for system ID; see USE_FIXED_AIRFRAME.
%   * Runs are strictly SERIAL: the server accepts a single client at a time,
%     so parsim / parallel sweeps are NOT supported here.
%
% OUTPUT
%   * results          : struct array, one entry per command point (commanded
%                        voltages, steady-state mean/std wrench, wrench trace).
%   * openloop_data    : struct with the field names expected by
%                        data_optimize_full_optimization.m so the fit can be run
%                        directly on this simulated data set.
%   * A summary table is printed and a summary figure is drawn.
%   * Everything is saved to system_id_sweep_results_<timestamp>.mat.
%
% Author: generated for the RoboBee simulator system-ID workflow.

clc; close all;

%% ------------------------------------------------------------------------
% 0) Resolve paths and populate the full model parameter workspace.
% -------------------------------------------------------------------------
% target_driver_setup_2022_Ctrl_V_2_4.m defines every gain / filter / mapping
% the model needs and already selects open-loop simulation. It ends with a line
% that references the model by a bare identifier and therefore throws; the throw
% is expected and harmless because the workspace is fully populated by then.
% NOTE: that setup script runs "clear all", so NOTHING defined above this point
% survives it. All configuration below is declared AFTER the setup call.
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

% Re-establish everything the sweep needs (the workspace was just cleared).
mdl          = 'updated_target_driver_2026_withVariants';
this_dir     = fileparts(mfilename('fullpath'));
simulink_dir = fileparts(this_dir);
repo_root    = fileparts(simulink_dir);
addpath(simulink_dir);
addpath(this_dir);
if ~bdIsLoaded(mdl)
    load_system(mdl);
end

%% ------------------------------------------------------------------------
% 1) USER SETTINGS  (edit these)
% -------------------------------------------------------------------------
PORT            = 4242;      % Drake server TCP port
SYSID_DURATION  = 0.3;      % [s] per-run open-loop hold time (transient + averaging window)
BIAS_V          = 200;       % max_drv_bias (PZT bias rail); Excel campaigns used 300

% Steady-state wrench averaging (mirrors tools/plot_com_wrench_moving_avg.py):
% the wrench is a periodic flap signal riding on the DC offset we want, so we
% average over a trailing window sized to an INTEGER number of flap cycles -
% that cancels the ripple exactly and leaves the DC value, after dropping an
% initial transient. No ripple-peak rejection is needed.
AVG_CYCLES       = 30;       % flap cycles spanned by the averaging window
TRANSIENT_CYCLES = 6;        % initial flap cycles dropped before averaging
SPIKE_MAD_FACTOR = Inf;        % guard: drop only gross (> N scaled-MAD) non-physical
                             %   samples before averaging; set Inf to disable.
                             %   High enough that real flap ripple is never touched.
FREQ_HZ         = f;         % flap frequency; default = base setup value (leave as-is
                             %   unless you also re-run setup at the new frequency,
                             %   because the filters/delays were derived at that f)

% Plant server. For open-loop system ID you want the WELDED airframe
% (robobee_simulink_server_fixed): the body is held fixed so the logged COM
% wrench is pure aerodynamics. The free-flight build lets the body tumble under
% open-loop commands, which corrupts the measured wrench.
USE_FIXED_AIRFRAME    = true;   % true -> robobee_simulink_server_fixed (recommended)
FORCE_SERVER_RELAUNCH = false;  % true -> kill any server on PORT and launch the chosen build

% Command ranges: full usable ranges from Patrick_Bee_systemID_20231103.xlsx /
% OL_20230927 (155 Hz). Excludes the 5 rows that sheet flags "DO NOT USE"
% (a2 = -4, -2, -1, -0.5, and the 190/-0.2 point -> excessive negative yaw).
AMP_LIST   = [155 165 180 190];                                % drv_amp  (peak-to-peak drive command)
% AMP_LIST = [50, 75, 100, 125, 150, 175, 200]; 
ROLL_LIST  = [-4 -2 0 2 4 6 8 10];                             % drv_roll (differential -> roll)
% ROLL_LIST = [-20 -15 -10 -5 0 5 10 20]; 
PITCH_LIST = [-70 -50 -20 -15 -10 -5 0 5 10 15 20 25 30 40 50 60 70];  % drv_pitch_left = drv_pitch_right (offset -> pitch)
A2_LIST    = [-0.3 -0.2 -0.15 -0.1 0 0.1 0.15 0.2 0.3];        % a2_openloop (2nd-harmonic coeff -> yaw)

% Which per-axis campaigns to include.
DO_ROLL_SWEEP  = true;    % drv_amp x drv_roll   (thrust + roll identification)
DO_PITCH_SWEEP = true;    % drv_amp x drv_pitch  (pitch identification)
DO_A2_SWEEP    = true;    % drv_amp x a2         (yaw identification)

SAVE_TRACES = true;       % keep the full per-run wrench trace in results
COM_WRENCH_CSV = '/tmp/robobee_com_wrench.csv';   % written by the Drake server

if abs(FREQ_HZ - f) > eps
    warning(['FREQ_HZ (%g) differs from the frequency the base setup derived its ', ...
             'filters/delays at (%g). Re-run target_driver_setup at the new ', ...
             'frequency for a fully consistent model.'], FREQ_HZ, f);
end

%% ------------------------------------------------------------------------
% 2) Configure the model for open-loop system ID.
% -------------------------------------------------------------------------
% (target_driver_setup already sets Plant=2, control_flag=1, closedloop_flag=0,
%  landing_flag=0; we re-assert the essentials and the campaign-level constants.)
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
per_cycle_nom = 1 / (FREQ_HZ * dt_s_val);
fprintf(['Averaging: mean over a %d-cycle window (~%d samples) after a %d-cycle ', ...
         'transient, at %g Hz.\n'], AVG_CYCLES, round(AVG_CYCLES*per_cycle_nom), ...
         TRANSIENT_CYCLES, FREQ_HZ);
min_dur = (AVG_CYCLES + TRANSIENT_CYCLES) / FREQ_HZ;
if SYSID_DURATION < min_dur * 1.1
    warning(['SYSID_DURATION (%.3g s) is short for a %d-cycle window + %d-cycle ', ...
             'transient at %g Hz (needs >= ~%.3g s incl. flush margin).'], ...
             SYSID_DURATION, AVG_CYCLES, TRANSIENT_CYCLES, FREQ_HZ, min_dur*1.25);
end

%% ------------------------------------------------------------------------
% 3) Build the command matrix (the "nested for loops", deduplicated).
% -------------------------------------------------------------------------
% Columns: [drv_amp, drv_roll, drv_pitch_left, drv_pitch_right, a2_openloop, campaign]
% campaign: 1 = roll/thrust, 2 = pitch, 3 = yaw
cmd = zeros(0, 6);

if DO_ROLL_SWEEP
    for amp = AMP_LIST
        for roll = ROLL_LIST
            cmd(end+1, :) = [amp, roll, 0, 0, 0, 1]; %#ok<SAGROW>
        end
    end
end
if DO_PITCH_SWEEP
    for amp = AMP_LIST
        for pitch = PITCH_LIST
            cmd(end+1, :) = [amp, 0, pitch, pitch, 0, 2]; %#ok<SAGROW>
        end
    end
end
if DO_A2_SWEEP
    for amp = AMP_LIST
        for a2 = A2_LIST(A2_LIST ~= 0)
            cmd(end+1, :) = [amp, 0, 0, 0, a2, 3]; %#ok<SAGROW>
        end
    end
end

if isempty(cmd)
    error('system_id_sweep:noPoints', 'No campaigns enabled - nothing to sweep.');
end

% Drop duplicate command points (e.g. the amp/0/0/0/0 rows shared by campaigns).
[~, keep] = unique(cmd(:, 1:5), 'rows', 'stable');
cmd = cmd(keep, :);
n_runs = size(cmd, 1);
fprintf('Sweep: %d unique command points (roll=%d, pitch=%d, yaw=%d).\n', ...
    n_runs, DO_ROLL_SWEEP*numel(AMP_LIST)*numel(ROLL_LIST), ...
    DO_PITCH_SWEEP*numel(AMP_LIST)*numel(PITCH_LIST), ...
    DO_A2_SWEEP*numel(AMP_LIST)*nnz(A2_LIST ~= 0));

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
             '  >> Make sure it is the %s build for valid system-ID data\n', ...
             '     (set FORCE_SERVER_RELAUNCH=true to guarantee this).\n'], ...
             PORT, server_binary);
else
    fprintf('Launching %s on port %d ...\n', server_binary, PORT);
    launch_drake_server(repo_root, server_target, server_binary, PORT);
    if ~is_server_up('127.0.0.1', PORT)
        error('system_id_sweep:noServer', ...
            ['Drake server not reachable on port %d after launch.\n', ...
             'Start it manually with:\n', ...
             '  bazel run %s -- --server_port=%d'], PORT, server_target, PORT);
    end
end
fprintf('Drake server is reachable on 127.0.0.1:%d.\n', PORT);

%% ------------------------------------------------------------------------
% 5) Run the sweep (strictly serial; reset the plant between points).
% -------------------------------------------------------------------------
results = struct('campaign', {}, 'drv_amp', {}, 'drv_roll', {}, ...
    'drv_pitch_left', {}, 'drv_pitch_right', {}, 'a2_openloop', {}, ...
    'left_voltage_p2p', {}, 'right_voltage_p2p', {}, ...
    'left_voltage_offset', {}, 'right_voltage_offset', {}, 'a2_yaw', {}, ...
    'thrust_z_N', {}, 'roll_torque_Nm', {}, 'pitch_torque_Nm', {}, ...
    'yaw_torque_Nm', {}, 'thrust_std_N', {}, 'roll_std_Nm', {}, ...
    'pitch_std_Nm', {}, 'yaw_std_Nm', {}, 'n_avg', {}, 'n_outliers', {}, ...
    't_end', {}, 'sim_seconds', {}, 'wrench_trace', {});

stamp    = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
out_file = fullfile(this_dir, sprintf('system_id_sweep_results_%s.mat', stamp));

reset_plant(mdl);   % clean slate before the first run
sweep_t0 = tic;

%%
for i = 1:n_runs
    amp    = cmd(i, 1);  roll = cmd(i, 2);
    pitchL = cmd(i, 3);  pitchR = cmd(i, 4);
    a2     = cmd(i, 5);  campaign = cmd(i, 6);

    % ---- run one open-loop hold ----
    in = Simulink.SimulationInput(mdl);
    in = in.setVariable('drv_amp',         amp);
    in = in.setVariable('drv_roll',        roll);
    in = in.setVariable('drv_pitch_left',  pitchL);
    in = in.setVariable('drv_pitch_right', pitchR);
    in = in.setVariable('a2_openloop',     a2);
    in = in.setVariable('drv_bias',        BIAS_V);
    in = in.setModelParameter('StopTime',  num2str(SYSID_DURATION, '%.6g'));

    run_t0 = tic;
    sim(in);                       % output is captured via the server wrench CSV
    sim_seconds = toc(run_t0);

    % ---- reset socket -> server flushes+closes this run's CSV and is ready
    %      to build a fresh sim for the next point ----
    reset_plant(mdl);
    W = read_wrench_csv(COM_WRENCH_CSV);

    % ---- steady-state average: mean over a trailing window that spans an
    % integer number of flap cycles (plot_com_wrench_moving_avg.py method) ----
    t = W(:, 1);
    dt_data   = median(diff(t));                       % actual logged spacing
    per_cycle = 1 / (FREQ_HZ * dt_data);               % samples per flap cycle
    window    = max(1, round(AVG_CYCLES * per_cycle));
    start_idx = max(1, round(TRANSIENT_CYCLES * per_cycle) + 1);
    lo = max(start_idx, size(W, 1) - window + 1);      % last `window` samples,
    seg = W(lo:end, 2:5);                               %   not reaching the transient
    [mean_w, std_w, n_rej] = cycle_mean(seg, SPIKE_MAD_FACTOR);
    n_used = size(seg, 1);

    % ---- commanded voltages (same mapping the hardware post-processing uses) ----
    left_p2p   = amp + 2 * roll;
    right_p2p  = amp - 2 * roll;
    left_off   = pitchL;
    right_off  = pitchR;

    r.campaign = campaign;
    r.drv_amp = amp;  r.drv_roll = roll;
    r.drv_pitch_left = pitchL;  r.drv_pitch_right = pitchR;  r.a2_openloop = a2;
    r.left_voltage_p2p = left_p2p;      r.right_voltage_p2p = right_p2p;
    r.left_voltage_offset = left_off;   r.right_voltage_offset = right_off;
    r.a2_yaw = a2;
    r.thrust_z_N = mean_w(1);  r.roll_torque_Nm = mean_w(2);
    r.pitch_torque_Nm = mean_w(3);  r.yaw_torque_Nm = mean_w(4);
    r.thrust_std_N = std_w(1);  r.roll_std_Nm = std_w(2);
    r.pitch_std_Nm = std_w(3);  r.yaw_std_Nm = std_w(4);
    r.n_avg = n_used;  r.n_outliers = n_rej;
    r.t_end = t(end);  r.sim_seconds = sim_seconds;
    if SAVE_TRACES, r.wrench_trace = W; else, r.wrench_trace = []; end
    results(i) = r; %#ok<SAGROW>

    fprintf(['[%3d/%3d] cmp%d amp=%3g roll=%+5g pitch=%+5g a2=%+4g | ', ...
             'Ft=%6.3f mN  Mr=%+7.3f  Mp=%+7.3f  My=%+7.3f mNmm | rej=%d | %.1fs\n'], ...
        i, n_runs, campaign, amp, roll, pitchL, a2, ...
        1e3*mean_w(1), 1e3*1e3*mean_w(2), 1e3*1e3*mean_w(3), 1e3*1e3*mean_w(4), ...
        n_rej, sim_seconds);

    save(out_file, 'results', 'cmd', '-v7.3');   % incremental checkpoint
end

fprintf('Sweep finished: %d runs in %.1f s.\n', n_runs, toc(sweep_t0));

%% ------------------------------------------------------------------------
% 6) Assemble the open-loop data set, apply the yaw-torque filter, and save the
%    full + filtered results to out_file (consumed by section 7 and the fit).
% -------------------------------------------------------------------------
% Standalone re-processing: if the sweep vars are not in the workspace (running
% this section on its own), load results/cmd/meta from a saved sweep .mat. Set
% data_file to choose which file; a fresh sweep above otherwise supplies them.
loaded_from_file = false;
data_file = "Drake Model/system_id_sweep_results_20260714_171606"; 
if ~exist('results', 'var') || ~exist('cmd', 'var')
    if ~exist('data_file', 'var') || isempty(data_file)
        error('system_id_sweep:noData', ['results/cmd are not in the workspace. ', ...
            'Set data_file to a saved sweep .mat to re-process it in section 6.']);
    end
    Lraw    = load(data_file, 'results', 'cmd', 'meta');
    results = Lraw.results;
    cmd     = Lraw.cmd;
    if isfield(Lraw, 'meta'), meta = Lraw.meta; else, meta = struct(); end
    if isfield(meta, 'freq_hz'), FREQ_HZ = meta.freq_hz; else, FREQ_HZ = 155; end
    out_file = data_file;              % re-save the filtered set into the same file
    loaded_from_file = true;
    fprintf('Section 6: loaded %d runs from %s (re-processing).\n', numel(results), data_file);
end
n_runs = numel(results);

openloop_data = struct();
openloop_data.left_voltage_p2p     = [results.left_voltage_p2p];
openloop_data.right_voltage_p2p    = [results.right_voltage_p2p];
openloop_data.left_voltage_offset  = [results.left_voltage_offset];
openloop_data.right_voltage_offset = [results.right_voltage_offset];
openloop_data.a2_yaw               = [results.a2_yaw];
openloop_data.y_thrust_average     = [results.thrust_z_N];       % [N]
openloop_data.y_torque_x_average   = [results.roll_torque_Nm];   % [N*m]
openloop_data.y_torque_y_average   = [results.pitch_torque_Nm];  % [N*m]
openloop_data.y_torque_z_average   = [results.yaw_torque_Nm];    % [N*m]
openloop_data.freq                 = FREQ_HZ * ones(1, n_runs);
openloop_data.campaign             = cmd(:, 6).';

if ~loaded_from_file
    meta = struct('model', mdl, 'sysid_duration_s', SYSID_DURATION, ...
        'avg_cycles', AVG_CYCLES, 'bias_v', BIAS_V, 'freq_hz', FREQ_HZ, ...
        'dt_s', dt_s_val, 'timestamp', stamp, 'port', PORT);
end

% ---- filter: drop trials with non-physically large steady-state yaw torque.
% Open-loop points that saturate or tumble can log |tau_yaw| far beyond anything
% the real bee produces; keeping them distorts the downstream fit and the plots.
YAW_TORQUE_MAX_MNMM = 1000;                        % [mN*mm] rejection threshold
yaw_mNmm = 1e6 * openloop_data.y_torque_z_average; % N*m -> mN*mm (1 N*m = 1e6 mN*mm)
keep     = abs(yaw_mNmm) < YAW_TORQUE_MAX_MNMM;
if ~all(keep)
    drop = find(~keep);
    fprintf('Filter: dropped %d of %d trials with |yaw torque| >= %g mN*mm:\n', ...
        numel(drop), numel(keep), YAW_TORQUE_MAX_MNMM);
    for di = drop(:).'
        fprintf('  run %2d (campaign %d, amp %g): yaw = %.1f mN*mm\n', ...
            di, cmd(di, 6), results(di).drv_amp, yaw_mNmm(di));
    end
end

% Filtered dataset: openloop_data (all fields 1 x n_runs) plus the matching
% command matrix cmd_f. out_file stores BOTH the full run record (results, cmd)
% and the filtered set (openloop_data, cmd_f) that data_optimize + section 7 use.
openloop_data = structfun(@(v) v(keep), openloop_data, 'UniformOutput', false);
cmd_f = cmd(keep, :);
meta.yaw_torque_max_mNmm = YAW_TORQUE_MAX_MNMM;
meta.n_kept  = nnz(keep);
meta.n_total = numel(keep);

save(out_file, 'results', 'cmd', 'openloop_data', 'cmd_f', 'meta', '-v7.3');
fprintf('Saved full results/cmd (%d) + filtered openloop_data/cmd_f (%d) to:\n  %s\n', ...
    n_runs, nnz(keep), out_file);

%% ------------------------------------------------------------------------
% 7) Load the filtered results from file and plot (summary table + figure).
%    Decoupled from the sweep: set plot_file to any saved sweep .mat and run
%    this section on its own to re-plot without re-running the sweep.
% -------------------------------------------------------------------------
if ~exist('plot_file', 'var') || isempty(plot_file)
    if exist('out_file', 'var')
        plot_file = out_file;              % default: the file section 6 just wrote
    else
        error('system_id_sweep:noPlotFile', ...
            'Set plot_file to a saved sweep .mat before running section 7.');
    end
end
S  = load(plot_file);
od = S.openloop_data;                      % filtered wrench + campaign (1 x n_keep)
if isfield(S, 'cmd_f')
    cmd_f = S.cmd_f;                        % filtered command matrix (n_keep x 6)
elseif isfield(S, 'cmd') && size(S.cmd, 1) == numel(od.campaign)
    cmd_f = S.cmd;                          % file whose cmd already matches the filter
else
    error('system_id_sweep:noCmdF', ['%s has no filtered command matrix (cmd_f) ', ...
        'matching openloop_data (%d rows). Re-run the sweep to regenerate it.'], ...
        plot_file, numel(od.campaign));
end
fprintf('Loaded %d filtered trials from:\n  %s\n', numel(od.campaign), plot_file);

summary = table( cmd_f(:,6), cmd_f(:,1), cmd_f(:,2), cmd_f(:,3), cmd_f(:,5), ...
    1e3*od.y_thrust_average.', 1e6*1e3*od.y_torque_x_average.', ...
    1e6*1e3*od.y_torque_y_average.', 1e6*1e3*od.y_torque_z_average.', ...
    'VariableNames', {'campaign','drv_amp','drv_roll','drv_pitch','a2', ...
                      'Ft_mN','Mroll_uNmm','Mpitch_uNmm','Myaw_uNmm'});
disp(summary);

figure('Name', 'System-ID sweep summary', 'Color', 'w');
labels = {'Thrust F_T [mN]', 'Roll \tau_R [mN\cdotmm]', ...
          'Pitch \tau_P [mN\cdotmm]', 'Yaw \tau_Y [mN\cdotmm]'};
vals = [1e3*od.y_thrust_average.', 1e6*od.y_torque_x_average.', ...
        1e6*od.y_torque_y_average.', 1e6*od.y_torque_z_average.'];
cmap = lines(3);
campaign_names = {'roll', 'pitch', 'yaw'};
for k = 1:4
    subplot(2, 2, k); hold on; grid on;
    leg = {};
    for c = 1:3
        idx = cmd_f(:, 6) == c;
        if any(idx)
            plot(find(idx), vals(idx, k), 'o-', 'Color', cmap(c, :), ...
                'MarkerFaceColor', cmap(c, :));
            leg{end+1} = campaign_names{c}; %#ok<SAGROW>
        end
    end
    xlabel('Run index'); ylabel(labels{k});
    if k == 1 && ~isempty(leg), legend(leg, 'Location', 'best'); end
end
sgtitle('Open-loop system-ID sweep (mean steady-state wrench per command)');

fprintf(['\nNext step: fit the voltage->wrench model by pointing\n', ...
         'data_optimize_full_optimization.m at the saved openloop_data\n', ...
         '(load ''%s'').\n'], plot_file);

%% ========================================================================
% Local functions
% =========================================================================
function reset_plant(mdl)
%RESET_PLANT Drop the persistent Drake TCP socket so the next sim() gets a
% fresh simulation (pose reset, wrench CSV truncated). Unloading the model
% sim-target MEX destroys the native client's static socket; the server then
% sees the disconnect, flushes/closes the current wrench CSV, and rebuilds a
% clean simulation on the next connect.
    sfun = [mdl '_sfun'];
    clear(sfun);
    clear mex; %#ok<CLMEX>  % ensure the sim-target library is fully unloaded
    pause(0.2);            % give the server time to notice the disconnect + flush
end

function launch_drake_server(repo_root, target, binary, port)
%LAUNCH_DRAKE_SERVER Build (if needed) and launch a Drake TCP server in the
% background from the repo root, then wait until it accepts connections.
    old_dir = cd(repo_root);
    restore = onCleanup(@() cd(old_dir));
    if system(sprintf('bazel build %s', target)) ~= 0
        error('system_id_sweep:buildFailed', 'bazel build %s failed.', target);
    end
    system(sprintf('./%s --server_port=%d > /tmp/robobee_simulink_server.log 2>&1 &', ...
        binary, port));
    for k = 1:50   % up to ~5 s for the listen socket to come up
        if is_server_up('127.0.0.1', port), break; end
        pause(0.1);
    end
end

function [m, s, n_rej] = cycle_mean(X, mad_factor)
%CYCLE_MEAN Column-wise mean/std over an integer-flap-cycle window. Because the
% window spans whole flap cycles the periodic ripple cancels, so the plain mean
% is the unbiased DC value - NO ripple rejection is applied. The only guard is
% an optional, very high MAD threshold that drops gross (orders-of-magnitude)
% non-physical spikes; set mad_factor = Inf to disable it entirely.
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
        clear c;   % probe connection is closed immediately
        tf = true;
    catch
        tf = false;
    end
end

function W = read_wrench_csv(csv_path)
%READ_WRENCH_CSV Read the server COM-wrench log, polling briefly until its size
% is stable (the server flushes on the client disconnect that reset_plant
% triggers). Columns: time_s, thrust_z_N, roll_torque_Nm, pitch_torque_Nm,
% yaw_torque_Nm, and (newer servers) force_x_N, force_y_N in controller axes
% (+x forward, +y left).
    prev = -1; stable = 0;
    for k = 1:60
        d = dir(csv_path);
        if isempty(d), pause(0.05); continue; end
        if d.bytes == prev, stable = stable + 1; else, stable = 0; end
        if stable >= 3 && d.bytes > 0, break; end
        prev = d.bytes; pause(0.05);
    end
    W = readmatrix(csv_path);
    if isempty(W) || size(W, 2) < 5
        error('system_id_sweep:badCsv', ...
            'Wrench CSV %s is empty or malformed (got %d cols).', ...
            csv_path, size(W, 2));
    end
    W = W(all(isfinite(W), 2), :);   % drop any partially-written trailing row
end
