function [a_x, a_y, T_x0, T_y0, envelope_fit] = ...
    fit_torque_thrust_envelope(data_file, make_plot)
%FIT_TORQUE_THRUST_ENVELOPE Fit roll/pitch torque authority versus thrust.
%
%   [a_x,a_y,T_x0,T_y0,fit] = FIT_TORQUE_THRUST_ENVELOPE(data_file)
%   loads a system_id_sweep result and fits
%
%       |tau_x|max = a_x * max(T - T_x0, 0)
%       |tau_y|max = a_y * max(T - T_y0, 0).
%
%   The sweep contains several roll or pitch commands at each drive amplitude.
%   For every amplitude, this function selects the largest observed absolute
%   torque in the applicable campaign. The measured thrust from that same run
%   is the thrust coordinate of the envelope point. It then fits
%
%       tau_max = a*T + b,       T_0 = -b/a.
%
%   Inputs
%     data_file  system_id_sweep_results_*.mat file. If omitted or empty, the
%                newest matching file beside this function is used.
%     make_plot  true (default) to plot the envelope points and affine fits.
%
%   Outputs use SI units:
%     a_x, a_y        [m]     torque/thrust slopes
%     T_x0, T_y0      [N]     zero-authority thrust intercepts
%     envelope_fit             details, R^2, selected points, and separate
%                              positive/negative authority fits
%
%   A positive slope is required for the stated max(T-T_0,0) model. If the
%   measured authority instead decreases with thrust, this function retains
%   the signed affine slope and warns. In that case the same fitted line over
%   its positive portion is
%
%       tau_max = (-a) * max(T_0 - T, 0).
%
%   IMPORTANT: the selected extrema are maxima OBSERVED in the supplied sweep.
%   They are true actuator limits only if the roll and pitch command ranges
%   drove the actuator to its boundary at every amplitude.

    if nargin < 1 || isempty(data_file)
        data_file = newest_sweep_file(fileparts(mfilename('fullpath')));
    end
    if nargin < 2 || isempty(make_plot)
        make_plot = true;
    end

    data_file = char(data_file);
    validateattributes(make_plot, {'logical', 'numeric'}, ...
        {'scalar'}, mfilename, 'make_plot', 2);
    make_plot = logical(make_plot);

    [T, tau_x, tau_y, drive_amp, campaign, command] = load_sweep_data(data_file);

    x_points = observed_envelope(T, tau_x, drive_amp, campaign == 1, ...
        command(:, 2), 'roll');
    y_points = observed_envelope(T, tau_y, drive_amp, campaign == 2, ...
        command(:, 3), 'pitch');

    fit_x = fit_affine_envelope(x_points.thrust_N, x_points.torque_abs_Nm, 'roll');
    fit_y = fit_affine_envelope(y_points.thrust_N, y_points.torque_abs_Nm, 'pitch');

    a_x  = fit_x.a_m;
    a_y  = fit_y.a_m;
    T_x0 = fit_x.T0_N;
    T_y0 = fit_y.T0_N;

    envelope_fit = struct();
    envelope_fit.source_file = data_file;
    envelope_fit.model = '|tau|max = a*max(T-T0,0)';
    envelope_fit.units = struct('thrust', 'N', 'torque', 'N*m', ...
        'slope', 'm');
    envelope_fit.symmetric.x = fit_x;
    envelope_fit.symmetric.y = fit_y;
    envelope_fit.symmetric.x.points = x_points;
    envelope_fit.symmetric.y.points = y_points;

    % Positive and negative authority can differ. Fit their magnitudes
    % separately and retain them as diagnostics without changing the four
    % requested symmetric outputs above.
    envelope_fit.asymmetric.x.positive = signed_envelope_fit( ...
        T, tau_x, drive_amp, campaign == 1, +1, 'roll positive');
    envelope_fit.asymmetric.x.negative = signed_envelope_fit( ...
        T, tau_x, drive_amp, campaign == 1, -1, 'roll negative');
    envelope_fit.asymmetric.y.positive = signed_envelope_fit( ...
        T, tau_y, drive_amp, campaign == 2, +1, 'pitch positive');
    envelope_fit.asymmetric.y.negative = signed_envelope_fit( ...
        T, tau_y, drive_amp, campaign == 2, -1, 'pitch negative');

    print_results(data_file, x_points, y_points, fit_x, fit_y, envelope_fit);
    check_slope(fit_x, 'x');
    check_slope(fit_y, 'y');

    if make_plot
        plot_fits(x_points, y_points, fit_x, fit_y);
    end
end

function data_file = newest_sweep_file(this_dir)
    files = dir(fullfile(this_dir, 'system_id_sweep_results_*.mat'));
    if isempty(files)
        error('fit_torque_thrust_envelope:noData', ...
            'No system_id_sweep_results_*.mat file exists in %s.', this_dir);
    end
    [~, newest] = max([files.datenum]);
    data_file = fullfile(files(newest).folder, files(newest).name);
end

function [T, tau_x, tau_y, drive_amp, campaign, command] = ...
    load_sweep_data(data_file)
% Prefer system_id_sweep's filtered openloop_data/cmd_f pair. Fall back to
% the full results/cmd checkpoint format so partially processed sweeps work.
    if ~isfile(data_file)
        error('fit_torque_thrust_envelope:fileNotFound', ...
            'Sweep result does not exist: %s', data_file);
    end

    file_vars = {whos('-file', data_file).name};
    if all(ismember({'openloop_data', 'cmd_f'}, file_vars))
        S = load(data_file, 'openloop_data', 'cmd_f');
        od = S.openloop_data;
        command = S.cmd_f;
        required = {'y_thrust_average', 'y_torque_x_average', ...
            'y_torque_y_average'};
        if ~all(isfield(od, required))
            error('fit_torque_thrust_envelope:badOpenloopData', ...
                'openloop_data in %s is missing a required wrench field.', data_file);
        end
        T     = od.y_thrust_average(:);
        tau_x = od.y_torque_x_average(:);
        tau_y = od.y_torque_y_average(:);
        if size(command, 2) < 6
            error('fit_torque_thrust_envelope:badCommandMatrix', ...
                'cmd_f must have at least six columns.');
        end
        drive_amp = command(:, 1);
        campaign  = command(:, 6);
    elseif all(ismember({'results', 'cmd'}, file_vars))
        S = load(data_file, 'results', 'cmd');
        required = {'thrust_z_N', 'roll_torque_Nm', 'pitch_torque_Nm'};
        if ~all(isfield(S.results, required))
            error('fit_torque_thrust_envelope:badResults', ...
                'results in %s is missing a required wrench field.', data_file);
        end
        T     = [S.results.thrust_z_N].';
        tau_x = [S.results.roll_torque_Nm].';
        tau_y = [S.results.pitch_torque_Nm].';
        command = S.cmd;
        if size(command, 2) < 6
            error('fit_torque_thrust_envelope:badCommandMatrix', ...
                'cmd must have at least six columns.');
        end
        drive_amp = command(:, 1);
        campaign  = command(:, 6);
    else
        error('fit_torque_thrust_envelope:unsupportedFile', ...
            ['%s must contain openloop_data with cmd_f, or results with cmd. ', ...
             'Generate it with system_id_sweep.m.'], data_file);
    end

    n = numel(T);
    if any([numel(tau_x), numel(tau_y), numel(drive_amp), ...
            numel(campaign), size(command, 1)] ~= n)
        error('fit_torque_thrust_envelope:lengthMismatch', ...
            'Wrench arrays and command matrix in %s do not have matching lengths.', ...
            data_file);
    end

    valid = isfinite(T) & isfinite(tau_x) & isfinite(tau_y) & ...
        isfinite(drive_amp) & isfinite(campaign);
    if ~all(valid)
        warning('fit_torque_thrust_envelope:nonfiniteRows', ...
            'Ignoring %d rows containing non-finite sweep data.', nnz(~valid));
        T = T(valid); tau_x = tau_x(valid); tau_y = tau_y(valid);
        drive_amp = drive_amp(valid); campaign = campaign(valid);
        command = command(valid, :);
    end
end

function points = observed_envelope(T, tau, drive_amp, use_row, axis_command, axis_name)
    amps = unique(drive_amp(use_row), 'sorted');
    if numel(amps) < 2
        error('fit_torque_thrust_envelope:notEnoughGroups', ...
            'The %s campaign needs at least two distinct drive amplitudes.', axis_name);
    end

    n = numel(amps);
    thrust_N = zeros(n, 1);
    torque_abs_Nm = zeros(n, 1);
    torque_signed_Nm = zeros(n, 1);
    selected_command = zeros(n, 1);
    source_index = zeros(n, 1);
    for k = 1:n
        candidates = find(use_row & drive_amp == amps(k));
        [torque_abs_Nm(k), local_index] = max(abs(tau(candidates)));
        source_index(k) = candidates(local_index);
        thrust_N(k) = T(source_index(k));
        torque_signed_Nm(k) = tau(source_index(k));
        selected_command(k) = axis_command(source_index(k));
    end

    points = table(amps, thrust_N, torque_abs_Nm, torque_signed_Nm, ...
        selected_command, source_index, 'VariableNames', ...
        {'drive_amp', 'thrust_N', 'torque_abs_Nm', 'torque_signed_Nm', ...
         'axis_command', 'source_index'});
end

function fit = fit_affine_envelope(T, tau_mag, label)
    T = T(:);
    tau_mag = tau_mag(:);
    if numel(T) < 2 || numel(unique(T)) < 2
        error('fit_torque_thrust_envelope:unfitData', ...
            '%s envelope needs at least two distinct thrust values.', label);
    end

    X = [T, ones(size(T))];
    coeff = X \ tau_mag;
    a = coeff(1);
    b = coeff(2);
    scale = max([abs(b), max(abs(tau_mag)), eps]);
    if abs(a) * max([max(abs(T)), 1]) <= 100 * eps(scale)
        error('fit_torque_thrust_envelope:zeroSlope', ...
            '%s envelope slope is numerically zero, so T_0 is undefined.', label);
    end

    predicted = X * coeff;
    residual = tau_mag - predicted;
    sse = sum(residual.^2);
    sst = sum((tau_mag - mean(tau_mag)).^2);
    if sst > 0
        r2 = 1 - sse / sst;
    else
        r2 = NaN;
    end

    fit = struct('a_m', a, 'b_Nm', b, 'T0_N', -b/a, 'R2', r2, ...
        'n_points', numel(T), 'predicted_torque_Nm', predicted, ...
        'residual_Nm', residual);
end

function result = signed_envelope_fit(T, tau, drive_amp, use_row, sign_wanted, label)
% Extract max positive torque or magnitude of the most-negative torque at
% every amplitude. Groups without the requested sign are omitted.
    amps = unique(drive_amp(use_row), 'sorted');
    selected_T = zeros(0, 1);
    selected_mag = zeros(0, 1);
    selected_amp = zeros(0, 1);
    for k = 1:numel(amps)
        candidates = find(use_row & drive_amp == amps(k) & sign_wanted*tau > 0);
        if isempty(candidates)
            continue;
        end
        [mag, local_index] = max(sign_wanted * tau(candidates));
        source_index = candidates(local_index);
        selected_T(end+1, 1) = T(source_index); %#ok<AGROW>
        selected_mag(end+1, 1) = mag; %#ok<AGROW>
        selected_amp(end+1, 1) = amps(k); %#ok<AGROW>
    end

    result = struct('a_m', NaN, 'b_Nm', NaN, 'T0_N', NaN, 'R2', NaN, ...
        'n_points', numel(selected_T), 'drive_amp', selected_amp, ...
        'thrust_N', selected_T, 'torque_magnitude_Nm', selected_mag);
    if numel(selected_T) >= 2 && numel(unique(selected_T)) >= 2
        fitted = fit_affine_envelope(selected_T, selected_mag, label);
        result.a_m = fitted.a_m;
        result.b_Nm = fitted.b_Nm;
        result.T0_N = fitted.T0_N;
        result.R2 = fitted.R2;
    end
end

function print_results(data_file, x_points, y_points, fit_x, fit_y, all_fits)
    fprintf('\nTorque-thrust envelope source:\n  %s\n', data_file);
    fprintf('\nObserved roll envelope points:\n');
    disp(display_table(x_points));
    fprintf('Observed pitch envelope points:\n');
    disp(display_table(y_points));

    fprintf('Symmetric affine fits (SI units):\n');
    fprintf('  a_x  = %+.9g m\n', fit_x.a_m);
    fprintf('  T_x0 = %+.9g N  (%+.6g mN)\n', fit_x.T0_N, 1e3*fit_x.T0_N);
    fprintf('  b_x  = %+.9g N*m, R^2 = %.5f\n', fit_x.b_Nm, fit_x.R2);
    fprintf('  a_y  = %+.9g m\n', fit_y.a_m);
    fprintf('  T_y0 = %+.9g N  (%+.6g mN)\n', fit_y.T0_N, 1e3*fit_y.T0_N);
    fprintf('  b_y  = %+.9g N*m, R^2 = %.5f\n', fit_y.b_Nm, fit_y.R2);

    fprintf('\nAsymmetric diagnostic fits:\n');
    print_signed_fit('a_x^+', all_fits.asymmetric.x.positive);
    print_signed_fit('a_x^-', all_fits.asymmetric.x.negative);
    print_signed_fit('a_y^+', all_fits.asymmetric.y.positive);
    print_signed_fit('a_y^-', all_fits.asymmetric.y.negative);
    fprintf('\n');
end

function out = display_table(points)
    out = table(points.drive_amp, 1e3*points.thrust_N, ...
        1e6*points.torque_abs_Nm, 1e6*points.torque_signed_Nm, ...
        points.axis_command, 'VariableNames', ...
        {'drive_amp', 'thrust_mN', 'torque_abs_uNm', ...
         'torque_signed_uNm', 'axis_command'});
end

function print_signed_fit(name, fit)
    if isfinite(fit.a_m)
        fprintf('  %-5s = %+ .9g m, T_0 = %+ .6g mN, R^2 = %.5f (%d points)\n', ...
            name, fit.a_m, 1e3*fit.T0_N, fit.R2, fit.n_points);
    else
        fprintf('  %-5s: not enough signed envelope points to fit (%d points)\n', ...
            name, fit.n_points);
    end
end

function check_slope(fit, axis_name)
    if fit.a_m <= 0
        warning('fit_torque_thrust_envelope:nonpositiveSlope', ...
            ['a_%s = %.6g m is not positive, so the data do not support ', ...
             '|tau_%s|max = a_%s*max(T-T_%s0,0). The fitted positive ', ...
             'decreasing-authority form is |tau_%s|max = %.6g*max(T_%s0-T,0).'], ...
            axis_name, fit.a_m, axis_name, axis_name, axis_name, axis_name, ...
            -fit.a_m, axis_name);
    end
end

function plot_fits(x_points, y_points, fit_x, fit_y)
    figure('Name', 'Torque-thrust actuator envelope', 'Color', 'w');
    tiledlayout(1, 2, 'Padding', 'compact', 'TileSpacing', 'compact');
    plot_one_axis(nexttile, x_points, fit_x, 'Roll', 'x');
    plot_one_axis(nexttile, y_points, fit_y, 'Pitch', 'y');
end

function plot_one_axis(ax, points, fit, title_text, subscript)
    hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');
    scatter(ax, 1e3*points.thrust_N, 1e6*points.torque_abs_Nm, ...
        55, points.drive_amp, 'filled', 'DisplayName', 'observed maxima');

    span = max(points.thrust_N) - min(points.thrust_N);
    if span <= 0, span = max(abs(points.thrust_N)) * 0.1; end
    T_plot = linspace(max(0, min(points.thrust_N)-0.15*span), ...
        max(points.thrust_N)+0.15*span, 200).';
    tau_line = fit.a_m*T_plot + fit.b_Nm;
    plot(ax, 1e3*T_plot, 1e6*tau_line, 'k-', 'LineWidth', 1.5, ...
        'DisplayName', 'affine fit');
    yline(ax, 0, ':', 'HandleVisibility', 'off');
    xline(ax, 1e3*fit.T0_N, '--', 'T_0', 'HandleVisibility', 'off');

    xlabel(ax, 'Measured thrust T [mN]');
    ylabel(ax, sprintf('|\\tau_%s|_{max} [\\muN\\cdotm]', subscript));
    title(ax, sprintf('%s: a = %.4g m, T_0 = %.4g mN, R^2 = %.3f', ...
        title_text, fit.a_m, 1e3*fit.T0_N, fit.R2));
    legend(ax, 'Location', 'best');
    cb = colorbar(ax);
    cb.Label.String = 'drive amplitude';
end
