%% plot_popts_taux_surface.m
% Reads a popts_id_sweep results file and plots the roll-torque map as a
% 3-D surface:
%   x-axis: Vmean  (mean drive voltage, u(1))
%   y-axis: udiff  (differential command, u(3))
%   z-axis: tau_x  (steady-state roll torque, uN*m, controller axes)
%
% Uses the Vmean x udiff campaign (campaign == 2), where uoffs = h2 = 0, so
% the surface is a true 2-D slice of the wrench map w(u). Points flagged
% bad by the sweep's `keep` mask are excluded (when present).
%
% Companion to popts_id_sweep.m / fit_popts.m / plot_popts_fz_surface.m.

clc; close all;

%% ------------------------------------------------------------------------
% Settings
% -------------------------------------------------------------------------
RESULTS_FILE = '';      % explicit path to a popts_id_sweep_results_*.mat;
                        % leave '' to use the newest one in this folder
SAVE_PNG     = true;    % save <results-stem>_taux_surface.png next to the .mat
REFINE       = true;    % also draw an interpolated fine surface under the
                        % raw grid (natural-neighbor, display only)

%% ------------------------------------------------------------------------
% Locate and load results
% -------------------------------------------------------------------------
this_dir = fileparts(mfilename('fullpath'));
if isempty(RESULTS_FILE)
    cand = dir(fullfile(this_dir, 'popts_id_sweep_results_*.mat'));
    assert(~isempty(cand), 'No popts_id_sweep_results_*.mat found in %s', this_dir);
    [~, newest] = max([cand.datenum]);
    RESULTS_FILE = fullfile(cand(newest).folder, cand(newest).name);
end
fprintf('Reading %s\n', RESULTS_FILE);
S = load(RESULTS_FILE);
assert(all(isfield(S, {'U','W_SI','campaign'})), ...
    '%s does not contain U / W_SI / campaign (is this a completed sweep file?)', RESULTS_FILE);

keep = true(size(S.campaign));
if isfield(S, 'keep'), keep = logical(S.keep(:)); end

% Vmean x udiff campaign only (uoffs = h2 = 0)
sel = S.campaign(:) == 2 & keep;
assert(nnz(sel) >= 4, 'Too few valid campaign-2 (Vmean x udiff) points: %d', nnz(sel));

vmean   = S.U(sel, 1);
udiff   = S.U(sel, 3);
taux_uNm = S.W_SI(sel, 4) * 1e6;   % N*m -> uN*m

%% ------------------------------------------------------------------------
% Assemble the grid (campaign 2 is generated as a regular grid; average any
% repeats, leave holes as NaN)
% -------------------------------------------------------------------------
vg = unique(vmean);
ug = unique(udiff);
[~, iv] = ismember(vmean, vg);
[~, iu] = ismember(udiff, ug);
Tx_grid  = accumarray([iu, iv], taux_uNm, [numel(ug), numel(vg)], @mean, NaN);
[VG, UG] = meshgrid(vg, ug);

%% ------------------------------------------------------------------------
% Plot
% -------------------------------------------------------------------------
fig = figure('Name', 'popts sweep: tau_x(Vmean, udiff)', 'Color', 'w', ...
             'Position', [100 100 900 700]);
hold on;

if REFINE && nnz(sel) >= 8
    Fint = scatteredInterpolant(vmean, udiff, taux_uNm, 'natural', 'none');
    vf = linspace(min(vg), max(vg), 60);
    uf = linspace(min(ug), max(ug), 60);
    [VF, UF] = meshgrid(vf, uf);
    surf(VF, UF, Fint(VF, UF), 'EdgeColor', 'none', 'FaceAlpha', 0.85);
    % raw grid drawn as a mesh on top so the real measurement lattice is visible
    mesh(VG, UG, Tx_grid, 'EdgeColor', [0.25 0.25 0.25], 'FaceColor', 'none');
else
    surf(VG, UG, Tx_grid, 'FaceAlpha', 0.9);
end

plot3(vmean, udiff, taux_uNm, 'ko', 'MarkerFaceColor', 'r', 'MarkerSize', 5, ...
      'DisplayName', 'measured points');

grid on; box on;
xlabel('V_{mean}  [V]');
ylabel('u_{diff}');
zlabel('\tau_x  [\muN\cdotm]');
title({'Steady-state roll torque \tau_x over (V_{mean}, u_{diff})', ...
       sprintf('%s   (campaign 2: u_{offs} = h_2 = 0, n = %d)', ...
               strrep(nameOnly(RESULTS_FILE), '_', '\_'), nnz(sel))});
colormap(parula);
cb = colorbar; cb.Label.String = '\tau_x  [\muN\cdotm]';
view(-35, 25);

%% ------------------------------------------------------------------------
% Save
% -------------------------------------------------------------------------
if SAVE_PNG
    [pdir, stem] = fileparts(RESULTS_FILE);
    png = fullfile(pdir, [stem '_taux_surface.png']);
    exportgraphics(fig, png, 'Resolution', 150);
    fprintf('Saved %s\n', png);
end

function nm = nameOnly(p)
[~, nm, ext] = fileparts(p);
nm = [nm ext];
end
