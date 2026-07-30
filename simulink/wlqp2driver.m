function [drv_amp, drv_pitch_left, drv_pitch_right, drv_roll, a2, ...
    drv_bias, vleft, vright] = wlqp2driver(u, signs)
%#codegen
%WLQP2DRIVER Decompose the WLQP output u into wing-driver signals.
%
%   [drv_amp, drv_pitch_left, drv_pitch_right, drv_roll, a2, drv_bias] = ...
%       wlqp2driver(u)
%
% u = [Vmean; uoffs; udiff; h2] is the wlqp.m output:
%   Vmean  common stroke amplitude (mean wing drive voltage)  -> thrust
%   uoffs  fractional DC stroke-mean offset (both wings)      -> pitch
%   udiff  fractional left/right amplitude split              -> roll
%   h2     second-harmonic waveform coefficient               -> yaw
%
% Conversion (convertU / wltest.m convention, robobee3d wlcontroller):
%   vleft  = Vmean*(1 + udiff)         left wing p2p drive
%   vright = Vmean*(1 - udiff)         right wing p2p drive
%   drv_amp   = (vleft+vright)/2       = Vmean   (common amplitude)
%   drv_roll  = (vleft-vright)/4       = Vmean*udiff/2
%   drv_pitch = Vmean*uoffs            pitch bias voltage, same both wings
%   a2        = h2                     second-harmonic coefficient
%   drv_bias  = max(vl,vr) + 2*|drv_pitch|   HV rail headroom (derived,
%                                            not a control DOF)
%
% The driver reconstructs the wing voltages as
%   vleft = drv_amp + 2*drv_roll,  vright = drv_amp - 2*drv_roll.
%
% signs (optional, 3x1, default [1;1;1]) multiplies the [pitch; roll; yaw]
% channels. The torque signs of the popts wrench fit are empirical
% (wltest.m flips uoffs; convertU notes "drv_pch>0 => -momy,
% vR>vL => -momx"), so verify each channel against the plant with a
% static/trim test and set signs accordingly rather than editing the map.

u = u(:);
one = ones(1, 'like', u);

if nargin < 2 || isempty(signs)
    signs = [1; 1; 1] * one;
end
signs = signs(:);

Vmean = u(1);
uoffs = signs(1) * u(2);
udiff = signs(2) * u(3);
h2    = signs(3) * u(4);

vleft  = Vmean * (1 + udiff);
vright = Vmean * (1 - udiff);

drv_amp  = 0.5 * (vleft + vright);           % = Vmean
drv_roll = 0.25 * (vleft - vright);          % = Vmean*udiff/2

drv_pch = Vmean * uoffs;                     % pitch bias voltage
drv_pitch_left  = drv_pch;                   % WLQP pitch offset is symmetric
drv_pitch_right = drv_pch;

a2 = h2;

drv_bias = max(vleft, vright) + 2 * abs(drv_pch);

end
