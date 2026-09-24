function phi = famp_so3_log(R)
%#codegen
%FAMP_SO3_LOG Rotation vector of a rotation matrix (SO(3) logarithm).
%
%   phi = famp_so3_log(R) returns phi in R^3 with R = expm(hat(phi)),
%   on the principal branch ||phi|| <= pi. Used by the full-attitude MPC
%   (full_attitude_mpc_wl.m) for the attitude error e_R = log(R*' R) and
%   for the reference angular velocity omega* = log(R*_k' R*_{k+1}) / dt.
%
%   Branches
%     theta < 1e-6        : first-order series, phi = vee(R - R')/2 * (1 + th^2/6)
%     otherwise, theta<pi : phi = theta / (2 sin theta) * vee(R - R')
%     theta near pi       : axis from the dominant column of (R + I), which
%                           is well conditioned where vee(R - R') vanishes.
%                           The sign is fixed so that phi is consistent with
%                           the off-diagonal terms of R (continuity through
%                           the antipodal region is not guaranteed; an
%                           error of ~pi is a tumble the MPC cannot fix).

% Clamp the trace so acos never sees a value outside [-1, 1] from round-off.
c = 0.5 * (R(1,1) + R(2,2) + R(3,3) - 1.0);
if c > 1.0
    c = 1.0;
elseif c < -1.0
    c = -1.0;
end
theta = acos(c);

w = [R(3,2) - R(2,3); R(1,3) - R(3,1); R(2,1) - R(1,2)];   % vee(R - R')

if theta < 1.0e-6
    phi = 0.5 * w * (1.0 + theta*theta/6.0);
elseif theta < pi - 1.0e-3
    phi = (theta / (2.0 * sin(theta))) * w;
else
    % Near pi: R + I = 2 n n' (1 - cos) / ... -> columns of R + I are
    % parallel to the axis n. Pick the largest one for conditioning.
    S = R + eye(3);
    nrm = [norm(S(:,1)), norm(S(:,2)), norm(S(:,3))];
    [~, j] = max(nrm);
    n = S(:, j) / max(nrm(j), 1.0e-12);
    % Choose the sign of n so that it agrees with vee(R - R') = 2 sin(th) n
    if (n' * w) < 0.0
        n = -n;
    end
    phi = theta * n;
end
end
