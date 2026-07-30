%% fit_popts.m
% Fit the 6x15 quadratic wrench map ("popts") consumed by wlqp.m from a
% popts_id_sweep.m data set.
%
% MODEL (funapprox, per wrench component i = 1..6):
%   w_i(u) = a0 + a1'*u + 0.5*u'*A2*u,   u = [Vmean; uoffs; udiff; h2]
% with A2 symmetric, packed row-major upper-triangular. popts row i is
%   [a0, a1(1:4), A2(1,1), A2(1,2), A2(1,3), A2(1,4), A2(2,2), ..., A2(4,4)]
% and the 6 rows are concatenated into the 90-element popts vector -- exactly
% the layout wlqp.m / funapprox.c unpack.
%
% METHOD
%   Linear least squares in the coefficients. Inputs are centered/scaled
%   before fitting (raw regressors mix Vmean^2 ~ 3e4 with uoffs*h2 ~ 3e-2 and
%   condition terribly); the normalized-space coefficients are transformed
%   back to raw-u coefficients EXACTLY (quadratics are closed under affine
%   input maps). Optional ridge (RIDGE_LAMBDA) and per-point noise weighting
%   (USE_NOISE_WEIGHTS, from the sweep's steady-state std) are available but
%   off by default -- the sweep design is full-rank and well conditioned.
%
% VALIDATION (all printed / plotted below)
%   * random holdout R^2 per wrench channel (fit on 80%, test on 20%),
%     then a final refit on ALL points
%   * round-trip through wlqp.m itself: the fitted popts is evaluated via
%     wlqp.m's w0 output, guaranteeing the packing matches the consumer
%   * Jacobian sign table at the hover point (dFz/dVmean, dtaux/dudiff, ...)
%   * hover trim: iterate wlqp.m with pdotdes = 0 until u converges; report
%     the trim u and the wrench there vs. the weight m*g
%   * overlay plots: fit vs. data along each per-axis campaign slice
%
% OUTPUT
%   popts_fit_<stamp>.mat : popts (90x1 double), Popts (6x15), R2, meta
%   popts_fit_<stamp>.txt : paste-able "popts = single([...]);" literal
%
% Author: generated for the RoboBee simulator WLQP/popts workflow.

clc; close all;

%% ------------------------------------------------------------------------
% 0) Settings and data file.
% -------------------------------------------------------------------------
this_dir     = fileparts(mfilename('fullpath'));
simulink_dir = fileparts(this_dir);
addpath(simulink_dir);                 % wlqp.m lives here

% Data file: set explicitly, or leave '' to use the newest sweep result.
DATA_FILE = '';
if isempty(DATA_FILE)
    d = dir(fullfile(this_dir, 'popts_id_sweep_results_*.mat'));
    if isempty(d)
        error('fit_popts:noData', ...
            'No popts_id_sweep_results_*.mat in %s. Run popts_id_sweep.m first.', ...
            this_dir);
    end
    [~, newest] = max([d.datenum]);
    DATA_FILE = fullfile(this_dir, d(newest).name);
end

RIDGE_LAMBDA      = 0;       % ridge on non-intercept coeffs (normalized space);
                             %   0 = plain LS. Try ~1e-6..1e-3 only if noisy.
USE_NOISE_WEIGHTS = false;   % weight rows by 1/std from the sweep window. The
                             %   window std is flap RIPPLE, not estimator noise,
                             %   so this is off by default.
HOLDOUT_FRAC      = 0.2;     % random holdout fraction for the R^2 report
HOLDOUT_SEED      = 7;
M_KG              = 1.0e-4;  % vehicle mass for the hover-trim check [kg]
G_SI              = 9.81;    % [m/s^2]
CONTROL_RATE_HZ   = 5000;    % wlqp rate used in the trim iteration

WCHAN = {'F_x', 'F_y', 'F_z', '\tau_x', '\tau_y', '\tau_z'};

%% ------------------------------------------------------------------------
% 1) Load the sweep data (fit-ready, template units).
% -------------------------------------------------------------------------
S = load(DATA_FILE);
need = {'U_fit', 'W_template_fit', 'campaign_fit', 'meta'};
for k = 1:numel(need)
    if ~isfield(S, need{k})
        error('fit_popts:badFile', '%s is missing "%s" (older sweep format?).', ...
            DATA_FILE, need{k});
    end
end
U  = S.U_fit;                 % n x 4  [Vmean uoffs udiff h2]
W  = S.W_template_fit;        % n x 6  [Fx Fy Fz taux tauy tauz], template units
cp = S.campaign_fit(:);
n  = size(U, 1);
fprintf('Loaded %d points from %s\n', n, DATA_FILE);
fprintf('  u box: [%s] .. [%s]\n', num2str(min(U), '%g '), num2str(max(U), '%g '));

wts = ones(n, 6);
if USE_NOISE_WEIGHTS && isfield(S, 'W_std_fit')
    % Same unit conversion the sweep applied to the means.
    std_t = S.W_std_fit .* [1e3 1e3 1e3 1e6 1e6 1e6];
    wts = 1 ./ max(std_t, eps);
    wts = wts ./ mean(wts, 1);         % normalize so lambda keeps its meaning
end

%% ------------------------------------------------------------------------
% 2) Build the regressor in NORMALIZED input space.
% -------------------------------------------------------------------------
mu = mean(U, 1);
sg = std(U, 0, 1);
sg(sg < eps) = 1;                      % guard: constant column (degenerate sweep)
Z  = (U - mu) ./ sg;

Phi = quad_regressor(Z);               % n x 15, funapprox term order
fprintf('Regressor: rank %d/15, cond %.3g (normalized space)\n', ...
    rank(Phi), cond(Phi));
if rank(Phi) < 15
    warning(['Regressor is rank-deficient: some quadratic terms are not ', ...
             'excited by this sweep (did you disable the cross campaign?). ', ...
             'Those coefficients will be ridge/pinv-pinned near 0.']);
end

%% ------------------------------------------------------------------------
% 3) Holdout report, then final fit on all points.
% -------------------------------------------------------------------------
rng(HOLDOUT_SEED);
test  = rand(n, 1) < HOLDOUT_FRAC;
train = ~test;

R2_holdout = nan(1, 6);
for i = 1:6
    th = solve_ls(Phi(train, :), W(train, i), wts(train, i), RIDGE_LAMBDA);
    yhat = Phi(test, :) * th;
    R2_holdout(i) = 1 - sum((W(test, i) - yhat).^2) / ...
                        max(sum((W(test, i) - mean(W(test, i))).^2), eps);
end

theta = zeros(15, 6);                  % final fit: all points
R2_all = nan(1, 6);
for i = 1:6
    theta(:, i) = solve_ls(Phi, W(:, i), wts(:, i), RIDGE_LAMBDA);
    yhat = Phi * theta(:, i);
    R2_all(i) = 1 - sum((W(:, i) - yhat).^2) / ...
                    max(sum((W(:, i) - mean(W(:, i))).^2), eps);
end

fprintf('\n%-8s %12s %12s\n', 'channel', 'R2(holdout)', 'R2(all)');
for i = 1:6
    fprintf('%-8s %12.4f %12.4f\n', WCHAN{i}, R2_holdout(i), R2_all(i));
end

%% ------------------------------------------------------------------------
% 4) Transform normalized-space coefficients back to raw u (exact).
% -------------------------------------------------------------------------
% w = b0 + b1'*z + 0.5*z'*B2*z with z = D*(u - mu), D = diag(1./sg)
%   => A2 = D*B2*D,  a1 = D*b1 - A2*mu,  a0 = b0 - b1'*D*mu + 0.5*mu'*A2*mu
D = diag(1 ./ sg);
Popts = zeros(6, 15);
for i = 1:6
    [b0, b1, B2] = unpack_coeffs(theta(:, i));
    A2 = D * B2 * D;
    a1 = D * b1 - A2 * mu';
    a0 = b0 - b1' * D * mu' + 0.5 * mu * A2 * mu';
    Popts(i, :) = pack_coeffs(a0, a1, A2);
end
popts = reshape(Popts', [], 1);        % row-major 6x15 -> 90x1 (wlqp.m layout)

%% ------------------------------------------------------------------------
% 5) Round-trip check THROUGH wlqp.m: guarantees the packing matches the
%    consumer, not just this script's conventions.
% -------------------------------------------------------------------------
max_rt = 0;
for k = round(linspace(1, n, min(n, 25)))
    [~, w0] = wlqp(U(k, :)', zeros(6, 1), zeros(6, 1), popts, CONTROL_RATE_HZ, nan(4,1));
    w_here = eval_quad(Popts, U(k, :)');
    max_rt = max(max_rt, max(abs(w0 - w_here)));
end
fprintf('\nRound-trip vs wlqp.m w0 output: max |diff| = %.3g (should be ~eps)\n', max_rt);

%% ------------------------------------------------------------------------
% 6) Physical sanity: Jacobian signs at hover, and hover trim via wlqp.m.
% -------------------------------------------------------------------------
u_hover = [mean(U(:, 1)); 0; 0; 0];
J = quad_jacobian(Popts, u_hover);      % 6 x 4, dw/du
fprintf('\nJacobian at u = [%g 0 0 0] (rows w, cols u):\n', u_hover(1));
fprintf('%-8s %12s %12s %12s %12s\n', '', 'dVmean', 'duoffs', 'dudiff', 'dh2');
for i = 1:6
    fprintf('%-8s %12.4g %12.4g %12.4g %12.4g\n', WCHAN{i}, J(i, :));
end
fprintf(['Check: dF_z/dVmean > 0; the dominant torque channels ', ...
         '(d\\tau_y/duoffs, d\\tau_x/dudiff, d\\tau_z/dh2) should match the ', ...
         'signs your static tests found.\n']);

% Hover trim: drive wlqp.m to the input whose wrench supports the weight.
mg_template = (M_KG * 1e6) * (G_SI * 1e-3);   % mg * mm/ms^2
h0 = [0; 0; mg_template; 0; 0; 0];
u_trim = u_hover;
for k = 1:3000
    u_prev = u_trim;
    u_trim = wlqp(u_trim, zeros(6, 1), h0, popts, CONTROL_RATE_HZ);
    if max(abs(u_trim - u_prev)) < 1e-9, break; end
end
w_trim = eval_quad(Popts, u_trim);
fprintf(['\nHover trim (pdotdes = 0, weight %.3f): u = [%.1f, %+.4f, %+.4f, %+.4f] ', ...
         'after %d iters\n  wrench there = [%s]  (want [0 0 %.3f 0 0 0])\n'], ...
    mg_template, u_trim(1), u_trim(2), u_trim(3), u_trim(4), k, ...
    num2str(w_trim', '%+.4f '), mg_template);

%% ------------------------------------------------------------------------
% 7) Save popts (.mat + paste-able single literal).
% -------------------------------------------------------------------------
stamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
fit_meta = struct('data_file', DATA_FILE, 'timestamp', stamp, ...
    'ridge_lambda', RIDGE_LAMBDA, 'use_noise_weights', USE_NOISE_WEIGHTS, ...
    'holdout_frac', HOLDOUT_FRAC, 'R2_holdout', R2_holdout, 'R2_all', R2_all, ...
    'mu', mu, 'sg', sg, 'u_order', '[Vmean uoffs udiff h2]', ...
    'wrench_order', '[Fx Fy Fz taux tauy tauz], template units', ...
    'sweep_meta', S.meta);

mat_file = fullfile(this_dir, sprintf('popts_fit_%s.mat', stamp));
save(mat_file, 'popts', 'Popts', 'fit_meta');

txt_file = fullfile(this_dir, sprintf('popts_fit_%s.txt', stamp));
fid = fopen(txt_file, 'w');
fprintf(fid, '%% popts fit from %s (R2_all: %s)\n', DATA_FILE, ...
    num2str(R2_all, '%.4f '));
fprintf(fid, 'popts = single([%s]);\n', sprintf('%.10g, ', popts(1:end-1)) + ...
    string(sprintf('%.10g', popts(end))));
fclose(fid);
fprintf('\nSaved:\n  %s\n  %s\n', mat_file, txt_file);

%% ------------------------------------------------------------------------
% 8) Overlay plots: fit vs data along each per-axis campaign.
% -------------------------------------------------------------------------
figure('Name', 'popts fit vs data', 'Color', 'w');
xcol   = [2 3 4];                       % campaign 1->uoffs, 2->udiff, 3->h2
xnames = {'uoffs', 'udiff', 'h2'};
for c = 1:3
    idx = find(cp == c);
    for i = 1:6
        subplot(3, 6, (c-1)*6 + i); hold on; grid on;
        if ~isempty(idx)
            scatter(U(idx, xcol(c)), W(idx, i), 12, U(idx, 1), 'filled');
            % fitted curves at each Vmean in the campaign, other channels 0
            for vm = unique(U(idx, 1))'
                xs = linspace(min(U(idx, xcol(c))), max(U(idx, xcol(c))), 60);
                uu = repmat([vm; 0; 0; 0], 1, numel(xs));
                uu(xcol(c), :) = xs;
                wf = zeros(1, numel(xs));
                for kk = 1:numel(xs), wtmp = eval_quad(Popts, uu(:, kk)); wf(kk) = wtmp(i); end
                plot(xs, wf, '-', 'LineWidth', 0.8);
            end
        end
        if c == 1, title(WCHAN{i}, 'FontWeight', 'normal'); end
        if i == 1, ylabel(sprintf('vs %s', xnames{c})); end
    end
end
sgtitle('Quadratic wrench-map fit (lines) vs sweep data (dots, color = Vmean)');

figure('Name', 'popts fit residuals', 'Color', 'w');
for i = 1:6
    subplot(2, 3, i);
    histogram(W(:, i) - Phi * theta(:, i), 20);
    grid on; title(sprintf('%s residual (R^2 = %.3f)', WCHAN{i}, R2_all(i)));
end

%% ========================================================================
% Local functions
% =========================================================================
function Phi = quad_regressor(U)
%QUAD_REGRESSOR n x 15 design matrix in funapprox term order:
% [1, u1..u4, (r,c) upper-tri row-major with 0.5*u_r^2 on the diagonal and
%  u_r*u_c off it] so that Phi*[a0; a1; packed(A2)] = a0 + a1'u + 0.5u'A2u.
    n = size(U, 1);
    Phi = zeros(n, 15);
    Phi(:, 1) = 1;
    Phi(:, 2:5) = U;
    k = 6;
    for r = 1:4
        for c = r:4
            if r == c
                Phi(:, k) = 0.5 * U(:, r).^2;
            else
                Phi(:, k) = U(:, r) .* U(:, c);
            end
            k = k + 1;
        end
    end
end

function th = solve_ls(Phi, y, w, lambda)
%SOLVE_LS (Weighted, optionally ridge-regularized) least squares.
% Ridge is not applied to the intercept column.
    sw = sqrt(w(:));
    A = Phi .* sw;
    b = y(:) .* sw;
    if lambda > 0
        R = lambda * eye(size(Phi, 2));
        R(1, 1) = 0;
        th = (A' * A + R) \ (A' * b);
    else
        th = A \ b;
    end
end

function [a0, a1, A2] = unpack_coeffs(th)
%UNPACK_COEFFS 15-vector (funapprox order) -> a0, a1 (4x1), symmetric A2 (4x4).
    a0 = th(1);
    a1 = th(2:5);
    A2 = zeros(4);
    k = 6;
    for r = 1:4
        for c = r:4
            A2(r, c) = th(k);
            A2(c, r) = th(k);
            k = k + 1;
        end
    end
end

function row = pack_coeffs(a0, a1, A2)
%PACK_COEFFS a0, a1, symmetric A2 -> 15-vector (funapprox row-major packing).
    row = zeros(1, 15);
    row(1) = a0;
    row(2:5) = a1(:)';
    k = 6;
    for r = 1:4
        for c = r:4
            row(k) = A2(r, c);
            k = k + 1;
        end
    end
end

function w = eval_quad(Popts, u)
%EVAL_QUAD Evaluate the 6-component quadratic wrench map at u (4x1).
    w = zeros(6, 1);
    for i = 1:6
        [a0, a1, A2] = unpack_coeffs(Popts(i, :)');
        w(i) = a0 + a1' * u + 0.5 * (u' * A2 * u);
    end
end

function J = quad_jacobian(Popts, u)
%QUAD_JACOBIAN dw/du (6x4) of the quadratic wrench map at u.
    J = zeros(6, 4);
    for i = 1:6
        [~, a1, A2] = unpack_coeffs(Popts(i, :)');
        J(i, :) = (a1 + A2 * u)';
    end
end
