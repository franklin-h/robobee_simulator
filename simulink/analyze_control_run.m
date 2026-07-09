% Read-only analysis of the saved hover run.
clearvars;
close all;
clc;

matFile = fullfile(fileparts(mfilename('fullpath')), 'control_run.mat');
vars = whos('-file', matFile);
fprintf('MAT file: %s\n', matFile);
fprintf('Variables: %d\n', numel(vars));
for k = 1:numel(vars)
    fprintf('  %-32s %-16s %s\n', vars(k).name, vars(k).class, mat2str(vars(k).size));
end

S = load(matFile);
if isfield(S, 'yout')
    yout = S.yout;
elseif isfield(S, 'data') && isa(S.data, 'Simulink.SimulationData.Dataset')
    fprintf('\nDataset elements in data:\n');
    namesInDataset = S.data.getElementNames;
    if isstring(namesInDataset)
        namesInDataset = cellstr(namesInDataset);
    end
    if ischar(namesInDataset)
        namesInDataset = {namesInDataset};
    end
    for k = 1:S.data.numElements
        elem = S.data{k};
        elemName = '';
        if k <= numel(namesInDataset)
            elemName = namesInDataset{k};
        end
        if isempty(elemName) && isprop(elem, 'Name')
            elemName = elem.Name;
        end
        fprintf('  %2d: %s (%s)\n', k, elemName, class(elem));
        describeDatasetElement(elem);
    end
    summarizeDataset(S.data);
    return;
else
    error('control_run.mat contains neither yout nor data Dataset.');
end
fprintf('\nyout size: %d x %d\n', size(yout, 1), size(yout, 2));
if size(yout, 2) < 91
    warning('Expected at least 91 yout columns from target_driver_exec_2022_Ctrl_V_2_4.m.');
end

time = yout(:,1);
dt = diff(time);
validDt = dt(isfinite(dt) & dt > 0);
fprintf('time range: %.6f to %.6f s, samples=%d\n', min(time), max(time), numel(time));
fprintf('dt median/min/max: %.6g / %.6g / %.6g s\n', median(validDt), min(validDt), max(validDt));

names = {
    'x', 6; 'y', 7; 'z', 8;
    'roll_alpha', 9; 'pitch_beta', 10; 'yaw_gamma', 11;
    'omega_x', 12; 'omega_y', 13; 'omega_z', 14;
    'omega_x_filt', 15; 'omega_y_filt', 16; 'omega_z_filt', 17;
    'eOmega_x', 27; 'eOmega_y', 28; 'eOmega_z', 29;
    'drv_amp_closedloop', 30; 'drv_pitch_left_closedloop', 31; 'drv_pitch_right_closedloop', 32;
    'drv_roll_closedloop', 33; 'a2_closedloop', 34;
    'eR_x', 38; 'eR_y', 39; 'eR_z', 40;
    'vx_avg', 41; 'vy_avg', 42; 'vz_avg', 43;
    'x_avg', 44; 'y_avg', 45; 'z_avg', 46;
    'norm_ex_x_or_alt_z', 47; 'norm_ex_y_or_alt_vz', 48; 'norm_ex_z', 49;
    'drv_amp_output', 50; 'drv_pitch_left_output', 51; 'drv_pitch_right_output', 52;
    'drv_roll_output', 53; 'a2_output', 54;
    'roll_desired', 55; 'pitch_desired', 56; 'thrust_desired', 57; 'yaw_desired', 64;
    'norm_ev_x', 80; 'norm_ev_y', 81; 'norm_ev_z', 82;
    'r_ref_x', 83; 'r_ref_y', 84; 'r_ref_z', 85;
    'v_ref_x', 86; 'v_ref_y', 87; 'v_ref_z', 88;
    };

fprintf('\nSignal summary over full log:\n');
for k = 1:size(names, 1)
    label = names{k, 1};
    col = names{k, 2};
    if col <= size(yout, 2)
        sig = yout(:, col);
        fprintf('  %-28s col %2d: first=% .5g final=% .5g min=% .5g max=% .5g rms=% .5g\n', ...
            label, col, sig(1), sig(end), min(sig), max(sig), rms(sig));
    end
end

fields = {'m','g','Ixx','Iyy','Izz','dt_s','sampling_f','f','T','control_flag','Plant', ...
    'drv_amp','drv_roll','drv_pitch_left','drv_pitch_right','a2_openloop', ...
    'Thurst_limit','Torque_roll_limit','Torque_pitch_limit','Torque_yaw_limit', ...
    'k_x','k_z','k_v','k_vz','k_R','k_Omega','c_1_adaptive','c_2_adaptive', ...
    'upp_bound','upp_bound_eR','upp_bound_x','upp_bound_y','upp_bound_z'};
fprintf('\nSaved scalar/vector parameters:\n');
for k = 1:numel(fields)
    if isfield(S, fields{k})
        val = S.(fields{k});
        if isnumeric(val) || islogical(val)
            fprintf('  %-24s = %s\n', fields{k}, mat2str(val, 8));
        else
            fprintf('  %-24s = %s\n', fields{k}, class(val));
        end
    end
end

if size(yout, 2) >= 57
    gain_voltage = 0.01;
    fprintf('\nVoltage-command derived quantities:\n');
    fprintf('  closed-loop raw bias/VL/VR final: %.3f %.3f %.3f V\n', ...
        yout(end,35)/gain_voltage, yout(end,36)/gain_voltage, yout(end,37)/gain_voltage);
    fprintf('  closed-loop raw bias/VL/VR min:   %.3f %.3f %.3f V\n', ...
        min(yout(:,35)/gain_voltage), min(yout(:,36)/gain_voltage), min(yout(:,37)/gain_voltage));
    fprintf('  closed-loop raw bias/VL/VR max:   %.3f %.3f %.3f V\n', ...
        max(yout(:,35)/gain_voltage), max(yout(:,36)/gain_voltage), max(yout(:,37)/gain_voltage));
end

if size(yout, 2) >= 85
    posErr = yout(:,44:46) - yout(:,83:85);
    fprintf('\nReference tracking using averaged position columns:\n');
    fprintf('  final pos error [x y z] m: %s\n', mat2str(posErr(end,:), 6));
    fprintf('  max abs pos error [x y z] m: %s\n', mat2str(max(abs(posErr), [], 1), 6));
    fprintf('  rms pos error [x y z] m: %s\n', mat2str(rms(posErr), 6));
end

if size(yout, 2) >= 54
    cmdCols = [50 51 52 53 54 57 64];
    cmdNames = {'drv_amp_out','pitch_left_out','pitch_right_out','roll_out','a2_out','thrust_des','yaw_des'};
    fprintf('\nPotential saturation / flatline checks:\n');
    for i = 1:numel(cmdCols)
        sig = yout(:, cmdCols(i));
        fprintf('  %-16s unique-ish=%d longest flat run=%d samples\n', ...
            cmdNames{i}, numel(unique(round(sig, 8))), longestFlatRun(sig));
    end
end

function n = longestFlatRun(x)
tol = 1e-10;
n = 1;
run = 1;
for ii = 2:numel(x)
    if abs(x(ii) - x(ii-1)) <= tol
        run = run + 1;
    else
        n = max(n, run);
        run = 1;
    end
end
n = max(n, run);
end

function describeDatasetElement(elem)
try
    fprintf('      source: %s, port %s\n', string(elem.BlockPath), mat2str(elem.PortIndex));
catch
end
try
    fprintf('      propagated name: %s\n', string(elem.PropagatedName));
catch
end
try
    vals = elem.Values;
catch
    return;
end
try
    if isa(vals, 'timeseries')
        fprintf('      timeseries: time %d, data %s, first t %.6g, final t %.6g\n', ...
            numel(vals.Time), mat2str(size(vals.Data)), vals.Time(1), vals.Time(end));
    elseif isa(vals, 'timetable')
        fprintf('      timetable: %d rows, %d variables\n', height(vals), width(vals));
    else
        fprintf('      Values: %s\n', class(vals));
    end
catch err
    fprintf('      could not summarize Values: %s\n', err.message);
end
end

function summarizeDataset(ds)
fprintf('\nDataset signal ranges:\n');
for k = 1:ds.numElements
    elem = ds{k};
    try
        vals = elem.Values;
        [t, d] = valuesToTimeAndMatrix(vals);
    catch err
        fprintf('  %2d: skipped (%s)\n', k, err.message);
        continue;
    end
    label = elementLabel(elem, k);
    if isempty(d)
        fprintf('  %2d %-48s empty\n', k, label);
        continue;
    end
    fprintf('  %2d %-48s t=[%.4g %.4g] n=%d dims=%d\n', k, label, t(1), t(end), numel(t), size(d, 2));
    maxComponentsToPrint = min(size(d, 2), 9);
    for c = 1:maxComponentsToPrint
        sig = d(:, c);
        fprintf('       c%-2d first=% .5g final=% .5g min=% .5g max=% .5g rms=% .5g\n', ...
            c, sig(1), sig(end), min(sig), max(sig), rms(sig));
    end
    if size(d, 2) > maxComponentsToPrint
        fprintf('       ... %d more components omitted\n', size(d, 2) - maxComponentsToPrint);
    end
end
end

function [t, d] = valuesToTimeAndMatrix(vals)
if isa(vals, 'timeseries')
    t = vals.Time(:);
    raw = vals.Data;
elseif isa(vals, 'timetable')
    t = seconds(vals.Properties.RowTimes);
    raw = vals.Variables;
else
    error('unsupported Values class %s', class(vals));
end

sz = size(raw);
if isempty(raw)
    d = [];
elseif sz(1) == numel(t)
    d = reshape(raw, numel(t), []);
elseif sz(end) == numel(t)
    order = [ndims(raw), 1:ndims(raw)-1];
    d = reshape(permute(raw, order), numel(t), []);
else
    d = reshape(raw, numel(t), []);
end
end

function label = elementLabel(elem, idx)
label = elem.Name;
try
    if isempty(label)
        label = elem.PropagatedName;
    end
catch
end
try
    source = string(elem.BlockPath);
    if strlength(source) > 0
        if isempty(label)
            label = char(source);
        else
            label = sprintf('%s @ %s', label, source);
        end
    end
catch
end
if isempty(label)
    label = sprintf('element_%02d', idx);
end
if strlength(string(label)) > 48
    label = char(extractAfter(string(label), max(0, strlength(string(label)) - 48)));
end
end
