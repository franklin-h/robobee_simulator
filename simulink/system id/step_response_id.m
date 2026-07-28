%% step_response_id.m
% Open-loop STEP-RESPONSE system ID for the torque axes (roll & pitch).
%
% WHAT THIS DOES
%   Flies the Drake plant OPEN LOOP (free-flight build) at the hover-ish trim
%   (drv_amp/drv_roll/drv_pitch below), lets it take off, then injects a step
%   into the roll or pitch voltage bias at ID_STEP_TIME via the RollStep_ID /
%   PitchStep_ID blocks added to Open_Closed_loop_selection. Each step run is
%   paired with a no-step BASELINE run; because every run reconnects to a
%   freshly-reset deterministic sim, the baseline-subtracted rate response
%   isolates the step's effect even though the trim flight drifts.
%
%   Per axis it reports, from the measured torque step (server wrench CSV)
%   and the measured body-rate response (model logsout):
%     * torque step size       dTau        [Nm]     (cycle-averaged CSV diff)
%     * actuation dead time    Td_act      [ms]     (CSV torque onset - cmd)
%     * total delay to CONTROLLER-VISIBLE rate Td_ctl [ms] (obs omega onset)
%     * first-order rate fit:  dOmega(t) = K*(1 - exp(-(t-Td)/tau))
%         - K   : steady rate change  [rad/s]
%         - tau : time constant -> damping b = 1/tau [1/s]
%         - K/tau: initial rate slope [rad/s^2]
%     * effectiveness  k_eff = I * (K/tau) / dTau   (directly comparable to
%       the controller's k_tau_roll / k_tau_pitch)
%
% SIGN CONVENTION NOTE: everything is reported SIGNED. If k_eff comes out
%   negative on an axis, the plant torque opposes the controller's assumed
%   direction on that axis (this is the pitch -1 issue) - fix the sign at the
%   source rather than patching downstream.
%
% PREREQUISITES
%   * updated_target_driver_2026_withVariants.slx with the ID step blocks
%     (RollStep_ID / PitchStep_ID, added 2026-07-26) and logging marks
%     obs_out1/obs_out6/obs_out7/RollStepSum_out/PitchStepSumL_out.
%   * FREE-FLIGHT server build (robobee_simulink_server, NOT _fixed): the body
%     must rotate. This script kills any welded server on the port.
%
% Author: generated for the RoboBee simulator system-ID workflow (2026-07-26).

clc; close all;

%% ------------------------------------------------------------------------
% 0) Base parameter workspace (setup script does `clear all` - run it FIRST).
% -------------------------------------------------------------------------
this_dir_bootstrap     = fileparts(mfilename('fullpath'));
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
addpath(simulink_dir); addpath(this_dir);
if ~bdIsLoaded(mdl), load_system(mdl); end

%% ------------------------------------------------------------------------
% 1) USER SETTINGS
% -------------------------------------------------------------------------
PORT           = 4242;
COM_WRENCH_CSV = '/tmp/robobee_com_wrench.csv';

% Open-loop trim that produces the ~vertical takeoff (user-validated):
OL_AMP        = 150;
OL_ROLL       = -1e-3;
OL_PITCH      = 9.05;    % drv_pitch_left = drv_pitch_right
OL_A2         = 0;

ID_STEP_TIME  = 0.10;    % [s] step insertion (airborne by here per test flight)
SIM_DUR       = 0.28;    % [s] 0.10 trim + 0.18 response window
FIT_WIN       = 0.045;    % [s] analysis window after the step. Keep SHORT: in
                         % free flight the step run diverges chaotically from
                         % the baseline run, so late samples are common-mode
                         % divergence, not linear step response.

ROLL_STEPS    = [-2  2];      % [V] added to drv_roll
PITCH_STEPS   = [-1.5  1.5];  % [V] added to BOTH drv_pitch_left/right

FREQ_HZ       = f;       % flap frequency from setup (for cycle averaging)
AVG_CYCLES    = 4;       % cycles per averaging window for the torque step

FORCE_SERVER_RELAUNCH = true;   % guarantee the FREE-FLIGHT build is serving

%% ------------------------------------------------------------------------
% 2) Model + workspace configuration (open loop, Drake plant).
% -------------------------------------------------------------------------
assignin('base', 'Plant',           2);
assignin('base', 'control_flag',    1);   % open loop
assignin('base', 'closedloop_flag', 0);
assignin('base', 'landing_flag',    0);
% ID injection defaults (no step) - individual runs override via setVariable.
assignin('base', 'id_step_time',  ID_STEP_TIME);
assignin('base', 'id_roll_step',  0);
assignin('base', 'id_pitch_step', 0);

set_param(mdl, 'SolverType', 'Fixed-step');
set_param(mdl, 'Solver',     'FixedStepDiscrete');
set_param(mdl, 'FixedStep',  'dt_s');
dt_s_val = evalin('base', 'dt_s');

Ivec = evalin('base', 'I_moment_vec');   % [Ixx Iyy Izz]

%% ------------------------------------------------------------------------
% 3) Free-flight Drake server.
% -------------------------------------------------------------------------
server_target = '//apps:robobee_simulink_server';        % FREE FLIGHT
server_binary = 'bazel-bin/apps/robobee_simulink_server';
if FORCE_SERVER_RELAUNCH
    fprintf('Ensuring the FREE-FLIGHT server is the one on port %d ...\n', PORT);
    system('pkill -f robobee_simulink_server'); pause(1.0);
end
if ~is_server_up('127.0.0.1', PORT)
    fprintf('Launching %s on port %d ...\n', server_binary, PORT);
    launch_drake_server(repo_root, server_target, server_binary, PORT);
end
if ~is_server_up('127.0.0.1', PORT)
    error('step_response_id:noServer', ...
        'Drake server not reachable on port %d. Start manually: bazel run %s -- --server_port=%d', ...
        PORT, server_target, PORT);
end
fprintf('Free-flight Drake server reachable on 127.0.0.1:%d.\n', PORT);

%% ------------------------------------------------------------------------
% 4) Build the run list: baseline first, then each step run.
%    axis: 0 = baseline, 1 = roll, 2 = pitch
% -------------------------------------------------------------------------
runs = struct('axis', 0, 'step_V', 0);      % baseline
for v = ROLL_STEPS,  runs(end+1) = struct('axis', 1, 'step_V', v); end %#ok<SAGROW>
for v = PITCH_STEPS, runs(end+1) = struct('axis', 2, 'step_V', v); end %#ok<SAGROW>
n_runs = numel(runs);
fprintf('Campaign: 1 baseline + %d roll steps + %d pitch steps (%d runs).\n', ...
    numel(ROLL_STEPS), numel(PITCH_STEPS), n_runs);

reset_plant(mdl);
raw = cell(n_runs, 1);

for i = 1:n_runs
    ax = runs(i).axis; sv = runs(i).step_V;
    in = Simulink.SimulationInput(mdl);
    in = in.setVariable('drv_amp',         OL_AMP);
    in = in.setVariable('drv_roll',        OL_ROLL);
    in = in.setVariable('drv_pitch_left',  OL_PITCH);
    in = in.setVariable('drv_pitch_right', OL_PITCH);
    in = in.setVariable('a2_openloop',     OL_A2);
    in = in.setVariable('id_step_time',    ID_STEP_TIME);
    in = in.setVariable('id_roll_step',    sv * (ax == 1));
    in = in.setVariable('id_pitch_step',   sv * (ax == 2));
    in = in.setModelParameter('StopTime',  num2str(SIM_DUR, '%.6g'));

    label = "baseline";
    if ax == 1, label = sprintf("roll %+g V", sv); end
    if ax == 2, label = sprintf("pitch %+g V", sv); end
    fprintf('[%d/%d] %-12s ... ', i, n_runs, label);

    t0 = tic;
    out = sim(in);
    reset_plant(mdl);                      % flush + close this run's CSV
    W = read_wrench_csv(COM_WRENCH_CSV);

    L = out.logsout;
    r = struct();
    r.axis = ax; r.step_V = sv; r.label = char(label);
    r.om_raw = squeeze_ts(L, 'obs_out1');  % Euler-derivative omega (pre-averaging)
    r.om_obs = squeeze_ts(L, 'obs_out7');  % filtered omega (what the controller sees)
    r.bias_roll  = squeeze_ts(L, 'RollStepSum_out');
    r.bias_pitch = squeeze_ts(L, 'PitchStepSumL_out');
    r.wrench = W;                          % [t thrust rollT pitchT yawT]
    raw{i} = r;
    fprintf('done (%.1f s, wrench %d rows)\n', toc(t0), size(W, 1));
end

%% ------------------------------------------------------------------------
% 5) Analysis: baseline-subtract, fit FOPDT per run, report per axis.
% -------------------------------------------------------------------------
% Physics used: in free flight the CSV logs the TOTAL aero torque, which is
% the applied control step plus the flapping counter-torque reacting to the
% body rate:   dTq(t) = tau0 - c * dOmega(t)
% Regressing baseline-subtracted torque against baseline-subtracted rate over
% a short window therefore yields BOTH the applied step tau0 (gain) and the
% rotational damping c in one shot, without trusting a shape fit on the rate
% trace (which is contaminated by chaotic divergence from the baseline).
base_run = raw{1};
res = struct('label', {}, 'axis', {}, 'step_V', {}, 'tau0_Nm', {}, ...
    'gain_uNm_per_V', {}, 'c_damp', {}, 'b_damp', {}, 'tau_ms', {}, ...
    'K_pred', {}, 'Td_act_ms', {}, 'Td_ctl_ms', {}, 'Td_raw_ms', {}, 'r2', {});

fprintf('\n===================== STEP-RESPONSE ID RESULTS =====================\n');
fprintf('%-12s %10s %10s %8s %8s %9s %9s %9s %6s\n', 'run', 'tau0[uNm]', ...
    'uNm/V', 'b[1/s]', 'tau[ms]', 'K[rad/s]', 'TdAct[ms]', 'TdCtl[ms]', 'R2');

for i = 2:n_runs
    r = raw{i};
    ax = r.axis;
    om_col   = ax;                 % roll -> omega_x (1), pitch -> omega_y (2)
    tq_col   = 2 + ax;             % CSV col: 3 = roll torque, 4 = pitch torque
    I_ax     = Ivec(ax);

    % --- baseline-subtracted, one-flap-cycle-averaged torque trace ---
    Wt = r.wrench(:, 1); Tq = r.wrench(:, tq_col);
    Wb = base_run.wrench;
    Tq_b = interp1(Wb(:, 1), Wb(:, tq_col), Wt, 'linear', 'extrap');
    dTq  = Tq - Tq_b;
    cyc  = 1 / FREQ_HZ;
    wsamp = max(1, round(cyc / median(diff(Wt))));
    dTq_f = movmean(dTq, wsamp);
    pre   = Wt > ID_STEP_TIME - AVG_CYCLES*cyc & Wt < ID_STEP_TIME;
    dTq_f = dTq_f - mean(dTq_f(pre));                  % zero pre-step

    % --- baseline-subtracted rate (raw omega for physics, obs for delay) ---
    [t_r, om_r] = align_sub(r.om_raw, base_run.om_raw, om_col);
    [t_o, om_o] = align_sub(r.om_obs, base_run.om_obs, om_col);
    om_on_W = interp1(t_r, movmean(om_r, wsamp), Wt, 'linear', 'extrap');
    om_on_W = om_on_W - mean(om_on_W(pre));

    % --- joint regression dTq = tau0 - c*omega over the short window ---
    m = Wt > ID_STEP_TIME + 0.5*cyc & Wt < ID_STEP_TIME + FIT_WIN;
    A = [ones(nnz(m), 1), -om_on_W(m)];
    th = A \ dTq_f(m);
    tau0 = th(1); c = th(2);
    rr = dTq_f(m) - A*th;
    r2 = 1 - var(rr)/max(var(dTq_f(m)), eps);
    b_damp = c / I_ax;                     % [1/s]; NEGATIVE = that axis is
                                           % aerodynamically anti-damped over
                                           % this window (or velocity coupling
                                           % masquerading as rate feedback)
    if c > 0
        tau_ms = 1e3 * I_ax / c;           % rate time constant [ms]
        K_pred = tau0 / c;                 % predicted steady rate [rad/s]
    else
        tau_ms = NaN; K_pred = NaN;        % no stable first-order equivalent
    end

    % --- delays from onset crossings (20% of tau0 / of early rate slope) ---
    on = find(Wt > ID_STEP_TIME & abs(dTq_f) > 0.2*abs(tau0), 1);
    Td_act = NaN; if ~isempty(on), Td_act = (Wt(on) - ID_STEP_TIME)*1e3; end
    Td_ctl = onset_delay(t_o, om_o, ID_STEP_TIME, FIT_WIN, sign(tau0/I_ax));
    Td_raw = onset_delay(t_r, om_r, ID_STEP_TIME, FIT_WIN, sign(tau0/I_ax));

    res(end+1) = struct('label', r.label, 'axis', ax, 'step_V', r.step_V, ...
        'tau0_Nm', tau0, 'gain_uNm_per_V', tau0*1e6/r.step_V, 'c_damp', c, ...
        'b_damp', b_damp, 'tau_ms', tau_ms, 'K_pred', K_pred, ...
        'Td_act_ms', Td_act, 'Td_ctl_ms', Td_ctl, 'Td_raw_ms', Td_raw, ...
        'r2', r2); %#ok<SAGROW>

    fprintf('%-12s %10.3f %10.3f %8.1f %8.1f %9.2f %9.1f %9.1f %6.2f\n', ...
        r.label, tau0*1e6, tau0*1e6/r.step_V, b_damp, tau_ms, K_pred, ...
        Td_act, Td_ctl, r2);
end

% per-axis medians (the numbers to put in the controller model)
fprintf('---------------------------------------------------------------------\n');
for ax = 1:2
    sel = [res.axis] == ax;
    if ~any(sel), continue; end
    nm = 'ROLL '; if ax == 2, nm = 'PITCH'; end
    fprintf(['%s median: gain = %+.3f uNm/V   damping b = %.0f 1/s   ', ...
             'Td act/raw/ctl = %.1f / %.1f / %.1f ms\n'], ...
        nm, median([res(sel).gain_uNm_per_V]), median([res(sel).b_damp]), ...
        median([res(sel).Td_act_ms]), median([res(sel).Td_raw_ms]), ...
        median([res(sel).Td_ctl_ms]));
end
fprintf('=====================================================================\n');
fprintf(['Controller mapping:\n', ...
         '  * b_rot (per axis)  <- damping b above\n', ...
         '  * tau_lag           <- Td(ctl): total delay the controller sees\n', ...
         '  * k_tau (per axis)  <- tau0 / (torque the inverse map claims for\n', ...
         '    the same voltage step). A NEGATIVE gain means that axis''s sign\n', ...
         '    convention is flipped - fix it at the source, not with -1 gains.\n', ...
         'Low R2 (<0.5) means the tau0 - c*omega model did not explain the\n', ...
         'torque trace; treat that run''s numbers as suspect.\n']);

stamp    = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
out_file = fullfile(this_dir, sprintf('step_response_id_results_%s.mat', stamp));
save(out_file, 'raw', 'res', 'runs', 'OL_AMP', 'OL_ROLL', 'OL_PITCH', ...
    'ID_STEP_TIME', 'SIM_DUR', 'ROLL_STEPS', 'PITCH_STEPS', '-v7.3');
fprintf('Saved %s\n', out_file);

% quick-look figure
fig = figure('Color', 'w', 'Position', [80 80 1000 640]);
for ax = 1:2
    subplot(2, 2, ax); hold on; grid on;
    for i = 2:n_runs
        if raw{i}.axis ~= ax, continue; end
        [tt, oo] = align_sub(raw{i}.om_obs, base_run.om_obs, ax);
        plot(tt, oo, 'LineWidth', 1.1, 'DisplayName', raw{i}.label);
    end
    xline(ID_STEP_TIME, 'k:', 'step'); xlabel('t [s]'); ylabel('\Delta\omega [rad/s]');
    if ax == 1, title('roll: \Delta\omega_x (obs)'); else, title('pitch: \Delta\omega_y (obs)'); end
    legend('Location', 'best');
    subplot(2, 2, 2 + ax); hold on; grid on;
    for i = 2:n_runs
        if raw{i}.axis ~= ax, continue; end
        Wt = raw{i}.wrench(:, 1); tq = raw{i}.wrench(:, 2 + ax);
        Wb = base_run.wrench;
        plot(Wt, movmean(tq - interp1(Wb(:,1), Wb(:,2+ax), Wt, 'linear', 'extrap'), ...
            max(1, round(1/(FREQ_HZ*median(diff(Wt)))))) * 1e6, 'LineWidth', 1.1, ...
            'DisplayName', raw{i}.label);
    end
    xline(ID_STEP_TIME, 'k:'); xlabel('t [s]'); ylabel('\Delta\tau [uNm] (1-cycle avg)');
    if ax == 1, title('roll torque step (CSV)'); else, title('pitch torque step (CSV)'); end
end
saveas(fig, fullfile(this_dir, sprintf('step_response_id_%s.png', stamp)));
fprintf('Figure saved.\n');

%% ========================================================================
% Local functions
% =========================================================================
function ts = squeeze_ts(logsout, name)
%SQUEEZE_TS Extract a logged signal as struct('t', 'v') with channels in cols.
    el = logsout.getElement(name);
    v  = squeeze(el.Values.Data);
    if size(v, 1) ~= numel(el.Values.Time), v = v'; end
    ts = struct('t', el.Values.Time, 'v', v);
end

function [t, d] = align_sub(ts_a, ts_b, col)
%ALIGN_SUB Baseline-subtract one channel of two logged signals on a's grid.
    t  = ts_a.t;
    vb = interp1(ts_b.t, ts_b.v(:, col), t, 'linear', 'extrap');
    d  = ts_a.v(:, col) - vb;
end

function Td_ms = onset_delay(t, y, t_step, fit_win, expect_sign)
%ONSET_DELAY Effective dead time via the tangent-intercept method: fit a line
% to the rate while it rises through 30-70% of its window peak and take the
% line's zero crossing relative to the step time. This separates dead time
% from rise (a 15%-threshold crossing would lump rise time into the delay).
% expect_sign orients the response along the applied torque's direction.
    pre = t > t_step - 0.05 & t < t_step;
    y   = (y - mean(y(pre))) * expect_sign;
    m   = t >= t_step & t <= t_step + fit_win;
    tt  = t(m); yy = y(m);
    [pk, ipk] = max(yy);
    Td_ms = NaN;
    if pk <= 0, return; end
    seg = find(yy(1:ipk) > 0.3*pk & yy(1:ipk) < 0.7*pk);
    if numel(seg) < 3, return; end
    p = polyfit(tt(seg), yy(seg), 1);
    if p(1) <= 0, return; end
    Td_ms = (-p(2)/p(1) - t_step) * 1e3;
end

function reset_plant(mdl)
    sfun = [mdl '_sfun']; clear(sfun);
    clear mex; %#ok<CLMEX>
    pause(0.2);
end

function launch_drake_server(repo_root, target, binary, port)
    old_dir = cd(repo_root);
    restore = onCleanup(@() cd(old_dir));
    % Build only if the binary is missing (bazel may not be on MATLAB's PATH;
    % GUI-launched MATLAB inherits a minimal shell environment).
    if ~isfile(fullfile(repo_root, binary))
        bazel = find_bazel();
        if isempty(bazel)
            error('step_response_id:noBazel', ...
                ['%s not built and bazel not found on PATH. Build it in a ', ...
                 'terminal first:  bazel build %s'], binary, target);
        end
        if system(sprintf('"%s" build %s', bazel, target)) ~= 0
            error('step_response_id:buildFailed', 'bazel build %s failed.', target);
        end
    end
    system(sprintf('./%s --server_port=%d > /tmp/robobee_simulink_server.log 2>&1 &', ...
        binary, port));
    for k = 1:50
        if is_server_up('127.0.0.1', port), break; end
        pause(0.1);
    end
end

function p = find_bazel()
    p = '';
    for cand = {'/opt/homebrew/bin/bazel', '/usr/local/bin/bazel', '/usr/bin/bazel'}
        if isfile(cand{1}), p = cand{1}; return; end
    end
    [ok, out] = system('which bazel');
    if ok == 0, p = strtrim(out); end
end

function tf = is_server_up(host, port)
    tf = false;
    try
        c = tcpclient(host, port, 'Timeout', 2); %#ok<NASGU>
        clear c; tf = true;
    catch
        tf = false;
    end
end

function W = read_wrench_csv(csv_path)
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
        error('step_response_id:badCsv', 'Wrench CSV %s empty/malformed.', csv_path);
    end
    W = W(all(isfinite(W), 2), :);
end
