function [Rref, omref, alpref] = famp_reference(sdes_seq, psi0, psidot, dt, N)
%#codegen
%FAMP_REFERENCE Build the full-attitude reference preview R*_k, omega*_k, alpha*_k.
%
%   [Rref, omref, alpref] = famp_reference(sdes_seq, psi0, psidot, dt, N)
%
% The trajectory generator (desTraj) supplies only the reduced attitude
% reference sdes (desired body-z, world frame) as a 3x(N+1) preview plus a
% heading reference (psi0 [rad], psidot [rad/ms]). This lifts that to a
% full rotation per stage:
%
%   R*_k = Rmin(e3 -> sdes_k) * Rz(psi_k),      psi_k = psi0 + k*dt*psidot
%
% Rmin(e3 -> s) is the minimal (geodesic) rotation taking e3 onto s, so the
% body-z reference is sdes_k exactly, and for an upright vehicle the body-x
% heading is exactly psi_k (for a tilt in the heading direction it is also
% exact; sideways tilt perturbs the heading only at second order, same as
% the atan2(R21, R11) heading the yaw PID used).
%
% Rmin is singular only at s = -e3 (inverted). Along a pitch great circle
% (the template flip, sdes = [-sin a; 0; cos a]) Rmin(e3 -> s) = Ry(-a),
% which is continuous THROUGH the inverted point, and the fallback used
% exactly there (a half turn about e2) is that same limit. Roll flips would
% see a heading jump at the inverted instant; acceptable.
%
% omega*_k is the finite-difference body rate of the reference sequence,
% expressed in the frame of R*_k (what f_R in famp_linearize expects), and
% alpha*_k its finite difference. Both are zero for a held reference.
%
% Inputs
%   sdes_seq (3 x Np)  desired body-z preview, column k+1 = sdes(t + k dt);
%                      missing columns (Np < N+1) repeat the last one.
%   psi0     heading reference now [rad]
%   psidot   heading reference rate [rad/ms]
%   dt       prediction step [ms]
%   N        horizon length
% Outputs
%   Rref   3x3x(N+1),  omref 3x(N+1) [rad/ms],  alpref 3x(N+1) [rad/ms^2]

Np = size(sdes_seq, 2);
Rref   = zeros(3, 3, N+1);
omref  = zeros(3, N+1);
alpref = zeros(3, N+1);

for k = 1:N+1
    if k <= Np
        s = sdes_seq(:, k);
    else
        s = sdes_seq(:, Np);
    end
    s = s / max(norm(s), 1.0e-9);
    psi_k = psi0 + (k-1) * dt * psidot;
    Rref(:, :, k) = rot_from_sdes(s, psi_k);
end

for k = 1:N
    Rk  = Rref(:, :, k);
    Rk1 = Rref(:, :, k+1);
    omref(:, k) = famp_so3_log(Rk' * Rk1) / dt;
end
omref(:, N+1) = omref(:, N);

for k = 1:N
    alpref(:, k) = (omref(:, k+1) - omref(:, k)) / dt;
end
alpref(:, N+1) = zeros(3, 1);
end

% =========================================================================
function R = rot_from_sdes(s, psi)
%#codegen
% R = Rmin(e3 -> s) * Rz(psi)
c = s(3);                                   % e3 . s
v = [-s(2); s(1); 0.0];                     % e3 x s
if c > -1.0 + 1.0e-9
    V = [   0.0, -v(3),  v(2); ...
           v(3),   0.0, -v(1); ...
          -v(2),  v(1),   0.0];
    Rmin = eye(3) + V + (V*V) / (1.0 + c);
else
    Rmin = [-1.0, 0.0, 0.0; 0.0, 1.0, 0.0; 0.0, 0.0, -1.0];   % half turn about e2
end
cp = cos(psi);
sp = sin(psi);
Rz = [cp, -sp, 0.0; sp, cp, 0.0; 0.0, 0.0, 1.0];
R = Rmin * Rz;
end
