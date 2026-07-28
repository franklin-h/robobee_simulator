function [u, w0] = wlqp(u0, h0, pdotdes, popts, controlRate, umin, umax, dumax, Qdiag)
%#codegen
%WLQP MATLAB translation of the wrench-linearized QP controller (WLQP).
%
% Translated from robobee3d/template/wlcontroller:
%   cpp/wlqp.cpp + cpp/wlcontroller.cpp + cpp/funapprox.hpp (Avik De, 2020)
% Momentum reference dynamics: https://github.com/avikde/robobee3d/pull/166
%
% One call = one control update (the caller carries the state, i.e. passes
% the previous output back in as u0 — same as wltest.m does with the MEX):
%
%   u = wlqp(u0, h0, pdotdes, popts, controlRate)
%
% Inputs
%   u0          4x1  previous input  [Vmean; uoffs; udiff; h2]
%   h0          6x1  momentum-dynamics bias term (e.g. [Rb'*[0;0;m*g]; 0;0;0])
%   pdotdes     6x1  desired momentum rate (wrench units)
%   popts       90x1 quadratic wrench-map fit coefficients, row-major 6x15
%                    (same vector passed to wlControllerUpdate / wltest.m)
%   controlRate scalar, Hz. Sets the rate limit dumax = [5e3;10;10;10]/controlRate
%   umin, umax  (optional) 4x1 absolute input limits.
%                    Defaults [90;-0.5;-0.2;-0.1], [240;0.5;0.2;0.1] (wlqp.c).
%                    Pass umin(1) = NaN to disable input limits (wlqp.cpp).
%   dumax       (optional) 4x1 per-step rate limit; overrides the
%                    controlRate-derived default (setLimits() in wlqp.cpp)
%   Qdiag       (optional) 6x1 wrench error weights, default [1;1;1;.1;.1;.1]
%
% Outputs
%   u           4x1  new input, u = u0 + du with du from the QP
%   w0          6x1  wrench map evaluated at u0
%
% The QP is  min_du 0.5*du'*P*du + q'*du,  L <= du <= U   with
%   P = A1'*diag(Qdiag)*A1,  q = A1'*(Qdiag.*(w0 - h0 - pdotdes)),
%   A1 = dw/du at u0. The C version solves it with embedded OSQP capped at
% 40 iterations; here a projected Gauss-Seidel loop is used instead (exact
% coordinate minimization, no toolbox needed, codegen-compatible).

nu = 4;
nw = 6;

u0 = u0(:);
h0 = h0(:);
pdotdes = pdotdes(:);

one = ones(1, 'like', u0);

if nargin < 6 || isempty(umin)
    umin = [90; -0.5; -0.2; -0.1] * one;
end
if nargin < 7 || isempty(umax)
    umax = [240; 0.5; 0.2; 0.1] * one;
end
if nargin < 8 || isempty(dumax)
    dumax = ([5.0e3; 10; 10; 10] / controlRate) * one;
end
if nargin < 9 || isempty(Qdiag)
    Qdiag = [1; 1; 1; 0.1; 0.1; 0.1] * one;
end
umin = umin(:); umax = umax(:); dumax = dumax(:); Qdiag = Qdiag(:);

% ----------------------------------------------------------------------
% Wrench map and Jacobian at u0 (funapprox: w_i = a0 + a1'*u + 0.5*u'*A2*u)
% ----------------------------------------------------------------------

% popts is row-major 6x15: row i = [a0, a1(1:4), upperTri(A2) row-major(10)]
poptsm = reshape(popts(:), 15, nw)';

w0 = zeros(nw, 1, 'like', u0);
A1 = zeros(nw, nu, 'like', u0);
for i = 1:nw
    a0i = poptsm(i, 1);
    a1i = poptsm(i, 2:5)';
    % Fill symmetric A2 from row-major upper-triangular values
    A2 = zeros(nu, nu, 'like', u0);
    kk = 6;
    for r = 1:nu
        for c = r:nu
            A2(r, c) = poptsm(i, kk);
            A2(c, r) = poptsm(i, kk);
            kk = kk + 1;
        end
    end
    w0(i) = a0i + a1i' * u0 + 0.5 * (u0' * A2 * u0);
    A1(i, :) = (a1i + A2 * u0)';
end

% ----------------------------------------------------------------------
% Build QP
% ----------------------------------------------------------------------

a0v = w0 - h0 - pdotdes;
P = A1' * (Qdiag .* A1);        % A1'*diag(Qdiag)*A1
q = A1' * (Qdiag .* a0v);

L = -dumax;
U = dumax;

% Input limits (not just rate limits)
if ~isnan(umin(1))
    for i = 1:nu
        if u0(i) < umin(i)
            L(i) = 0;   % do not reduce further
        elseif u0(i) > umax(i)
            U(i) = 0;   % do not increase further
        end
    end
end

% ----------------------------------------------------------------------
% Solve box QP with projected Gauss-Seidel (40 sweeps, like OSQP max_iter)
% ----------------------------------------------------------------------

du = zeros(nu, 1, 'like', u0);
for sweep = 1:40
    for i = 1:nu
        % Gradient of the cost wrt du(i), excluding the P(i,i)*du(i) term
        r = q(i) + P(i, :) * du - P(i, i) * du(i);
        if P(i, i) > eps(one)
            du(i) = min(max(-r / P(i, i), L(i)), U(i));
        elseif r > 0
            du(i) = L(i);
        elseif r < 0
            du(i) = U(i);
        end
    end
end

u = u0 + du;

end
