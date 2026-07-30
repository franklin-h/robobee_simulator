%% plot_popts_fz_surface.m
% Reads a popts_id_sweep results file and plots the vertical-force map as a
% 3-D surface:
%   x-axis: Vmean  (mean drive voltage, u(1))
%   y-axis: uoffs  (offset command, u(2))
%   z-axis: Fz     (steady-state vertical force, mN, controller axes +z up)
%
% Uses the Vmean x uoffs campaign (campaign == 1), where udiff = h2 = 0, so
% the surface is a true 2-D slice of the wrench map w(u). Points flagged
% bad by the sweep's `keep` mask are excluded (when present).
%
% Companion to popts_id_sweep.m / fit_popts.m.

clc; close all;

%% ------------------------------------------------------------------------
% Settings
% -------------------------------------------------------------------------
RESULTS_FILE = '';      % explicit path to a popts_id_sweep_results_*.mat;
                        % leave '' to use the newest one in this folder
SAVE_PNG     = true;    % save <results-stem>_fz_surface.png next to the .mat
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

% Vmean x uoffs campaign only (udiff = h2 = 0)
sel = S.campaign(:) == 1 & keep;
assert(nnz(sel) >= 4, 'Too few valid campaign-1 (Vmean x uoffs) points: %d', nnz(sel));

vmean = S.U(sel, 1);
uoffs = S.U(sel, 2);
fz_mN = S.W_SI(sel, 3) * 1e3;   % N -> mN

%% ------------------------------------------------------------------------
% Assemble the grid (campaign 1 is generated as a regular grid; average any
% repeats, leave holes as NaN)
% -------------------------------------------------------------------------
vg = unique(vmean);
ug = unique(uoffs);
[~, iv] = ismember(vmean, vg);
[~, iu] = ismember(uoffs, ug);
Fz_grid  = accumarray([iu, iv], fz_mN, [numel(ug), numel(vg)], @mean, NaN);
[VG, UG] = meshgrid(vg, ug);

%% ------------------------------------------------------------------------
% Plot
% -------------------------------------------------------------------------
fig = figure('Name', 'popts sweep: Fz(Vmean, uoffs)', 'Color', 'w', ...
             'Position', [100 100 900 700]);
hold on;

if REFINE && nnz(sel) >= 8
    Fint = scatteredInterpolant(vmean, uoffs, fz_mN, 'natural', 'none');
    vf = linspace(min(vg), max(vg), 60);
    uf = linspace(min(ug), max(ug), 60);
    [VF, UF] = meshgrid(vf, uf);
    surf(VF, UF, Fint(VF, UF), 'EdgeColor', 'none', 'FaceAlpha', 0.85);
    % raw grid drawn as a mesh on top so the real measurement lattice is visible
    mesh(VG, UG, Fz_grid, 'EdgeColor', [0.25 0.25 0.25], 'FaceColor', 'none');
else
    surf(VG, UG, Fz_grid, 'FaceAlpha', 0.9);
end

plot3(vmean, uoffs, fz_mN, 'ko', 'MarkerFaceColor', 'r', 'MarkerSize', 5, ...
      'DisplayName', 'measured points');

grid on; box on;
xlabel('V_{mean}  [V]');
ylabel('u_{offs}');
zlabel('F_z  [mN]');
title({'Steady-state vertical force F_z over (V_{mean}, u_{offs})', ...
       sprintf('%s   (campaign 1: u_{diff} = h_2 = 0, n = %d)', ...
               strrep(nameOnly(RESULTS_FILE), '_', '\_'), nnz(sel))});
colormap(parula);
cb = colorbar; cb.Label.String = 'F_z  [mN]';
view(-35, 25);

%% ------------------------------------------------------------------------
% Save
% -------------------------------------------------------------------------
if SAVE_PNG
    [pdir, stem] = fileparts(RESULTS_FILE);
    png = fullfile(pdir, [stem '_fz_surface.png']);
    exportgraphics(fig, png, 'Resolution', 150);
    fprintf('Saved %s\n', png);
end

function nm = nameOnly(p)
[~, nm, ext] = fileparts(p);
nm = [nm ext];
end
