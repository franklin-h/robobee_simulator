% Compact hover diagnostic for control_run.mat.
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

thrust = data{52}.Values.Data;
rollTorque = data{53}.Values.Data;
pitchTorque = data{54}.Values.Data;
yawTorque = data{55}.Values.Data;
voltageCmd = data{56}.Values.Data;

tilt = acosd(max(min(R(:,9), 1), -1));
desiredTilt = acosd(max(min(Rd(:,9), 1), -1));
flipIdx = find(tilt > 90, 1, 'first');

fprintf('control_run.mat hover report\n');
fprintf('  duration: %.4f s, samples: %d, dt median: %.6g s\n', t(end) - t(1), numel(t), median(diff(t)));
fprintf('  reference start/end: [% .4f % .4f % .4f] -> [% .4f % .4f % .4f] m\n', ref(1,:), ref(end,:));
fprintf('  position start/end:  [% .4f % .4f % .4f] -> [% .4f % .4f % .4f] m\n', pos(1,:), pos(end,:));
fprintf('  position error end:  [% .4f % .4f % .4f] m\n', pos(end,:) - ref(end,:));
fprintf('  velocity end:        [% .4f % .4f % .4f] m/s\n', vel(end,:));
fprintf('  tilt max/end:        %.2f / %.2f deg\n', max(tilt), tilt(end));
fprintf('  desired tilt max/end %.2f / %.2f deg\n', max(desiredTilt), desiredTilt(end));
if isempty(flipIdx)
    fprintf('  tilt > 90 deg:       never\n');
else
    fprintf('  tilt > 90 deg:       %.4f s\n', t(flipIdx));
end

fprintf('\nCommand ranges and clamp fractions:\n');
printCmd('thrust N', thrust, 0.5e-3, 1.6e-3);
printCmd('roll torque N*m', rollTorque, -0.6e-4, 0.5e-4);
printCmd('pitch torque N*m', pitchTorque, -0.2e-6, 0.2e-6);
printCmd('yaw torque N*m', yawTorque, -1.0e-7, 0.9e-7);

fprintf('\nVoltage command vector from Force_to_Voltage/Open_Closed_loop path:\n');
labels = {'drv_amp','drv_pitch_left','drv_roll','drv_pitch_right','a2'};
for i = 1:size(voltageCmd, 2)
    fprintf('  %-16s first=% .5g final=% .5g min=% .5g max=% .5g\n', ...
        labels{i}, voltageCmd(1,i), voltageCmd(end,i), min(voltageCmd(:,i)), max(voltageCmd(:,i)));
end

function printCmd(name, x, lo, hi)
tol = 1e-10;
fracLo = mean(x <= lo + tol);
fracHi = mean(x >= hi - tol);
fprintf('  %-18s first=% .5g final=% .5g min=% .5g max=% .5g at_lo=%5.1f%% at_hi=%5.1f%%\n', ...
    name, x(1), x(end), min(x), max(x), 100*fracLo, 100*fracHi);
end
