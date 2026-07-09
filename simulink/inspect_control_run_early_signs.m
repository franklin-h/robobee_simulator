% Inspect early-time signs in control_run.mat.
clearvars;
close all;
clc;

thisDir = fileparts(mfilename('fullpath'));
S = load(fullfile(thisDir, 'control_run.mat'));
data = S.data;

t = data{1}.Values.Time(:);
R = data{1}.Values.Data;
Rd = data{2}.Values.Data;
pos = data{3}.Values.Data;
ref = data{4}.Values.Data;
vel = data{43}.Values.Data;
omegaRaw = data{39}.Values.Data;
omegaFiltered = data{40}.Values.Data;
eOmega = data{46}.Values.Data;
eRraw = data{47}.Values.Data;
ePos = data{48}.Values.Data;
eVel = data{49}.Values.Data;
thrust = data{52}.Values.Data;
rollTorque = data{53}.Values.Data;
pitchTorque = data{54}.Values.Data;
yawTorque = data{55}.Values.Data;
voltageCmd = data{56}.Values.Data;

tilt = acosd(max(min(R(:,9), 1), -1));
desiredTilt = acosd(max(min(Rd(:,9), 1), -1));
idx = unique([1, 2, 3, 4, 5, 6, 11, 21, 51, 101, 151, 201, 241, 251, 301]);
idx = idx(idx <= numel(t));

fprintf('Early sign table from control_run.mat\n');
fprintf('%7s %8s %8s %8s %8s %8s %8s %9s %9s %9s %10s %10s %10s %8s %8s %8s %8s %8s\n', ...
    't', 'tilt', 'tiltD', 'z', 'zref', 'ePz', 'eVz', 'omX', 'omY', 'omZ', 'thrust', 'rollT', 'pitchT', 'amp', 'pL', 'rollV', 'pR', 'a2');
for k = idx
    fprintf('%7.4f %8.2f %8.2f %8.4f %8.4f %8.3f %8.3f %9.2f %9.2f %9.2f %10.3g %10.3g %10.3g %8.2f %8.2f %8.2f %8.2f %8.3f\n', ...
        t(k), tilt(k), desiredTilt(k), pos(k,3), ref(k,3), ePos(k,3), eVel(k,3), ...
        omegaFiltered(k,1), omegaFiltered(k,2), omegaFiltered(k,3), ...
        thrust(k), rollTorque(k), pitchTorque(k), voltageCmd(k,1), voltageCmd(k,2), voltageCmd(k,3), voltageCmd(k,4), voltageCmd(k,5));
end

fprintf('\nInitial vectors:\n');
fprintf('  pos-ref = %s\n', mat2str(pos(1,:) - ref(1,:), 6));
fprintf('  normalized position error = %s\n', mat2str(squeeze(ePos(1,:)), 6));
fprintf('  normalized velocity error = %s\n', mat2str(squeeze(eVel(1,:)), 6));
fprintf('  normalized eR raw size %s first = %s\n', mat2str(size(eRraw)), mat2str(squeeze(eRraw(:,:,1)).', 6));
fprintf('  normalized eOmega = %s\n', mat2str(squeeze(eOmega(1,:)), 6));
