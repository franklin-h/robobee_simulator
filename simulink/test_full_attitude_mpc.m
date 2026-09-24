% TEST_FULL_ATTITUDE_MPC  Offline checks for the full-attitude MPC.
%
% Run from simulink/ :   test_full_attitude_mpc
%
% 1. famp_linearize: Ac, Bc, cc match central finite differences of the
%    NONLINEAR error dynamics (independent implementation via expm/logm) at a
%    generic operating point with a moving reference.
% 2. famp_reference: body-z of R*_k equals sdes_k, heading equals psi_k,
%    omega* recovers a commanded spin rate, and the pitch-flip preview is
%    continuous through the inverted attitude.
% 3. Zero-yaw regression: with the yaw weights zeroed, full_attitude_mpc_wl
%    reproduces upright_template_mpc_wl's thrust/roll/pitch requests at
%    small tilt (the template is the reduced-attitude special case).
% 4. Yaw sanity: heading error produces a restoring yaw torque inside the
%    box; a commanded spin rate produces the rate-plant feed-forward
%    tau_z ~ b_yaw * psi_dot (0.35e-6 N*m*s * 1.26 rad/s ~ 0.44 uN*m, the
%    value measured in yaw_spin runs).
% 5. (optional) MATLAB Coder compile of full_attitude_mpc_wl, when a Coder
%    license is available, to catch codegen violations before Simulink does.

clearvars; clc;
addpath('controllers');
rng(7);
nfail = 0;

fprintf('=== 1. famp_linearize vs finite differences ===\n');
% Template units: mm, ms, mg
g_t   = 9.81e-3;
J     = [1.95e-9; 1.69e-9; 0.3e-9] * 1e12;
Dw    = [0.0; 0.0; 0.35e-6] * 1e9;
Ktau  = [0.1; 1.1; 1.0];
Lam   = 1 ./ ([4.31; 4.31; 4.31]);
Kdrag = (0.70e-3 / 85.655e-6) * 1e-3;
T0    = 1.05 * g_t;

Rref   = expm(hat(0.3*randn(3,1)));
omref  = 1.0e-3 * randn(3,1);              % ~1 rad/s
alpref = 0.5e-6 * randn(3,1);
aref   = 1.0e-3 * randn(3,1);
vref   = 0.2 * randn(3,1);

eR0  = 0.25 * randn(3,1);
eom0 = 5.0e-3 * randn(3,1);                % ~5 rad/s
ev0  = 0.5 * randn(3,1);                   % ~0.5 m/s
ep0  = 30 * randn(3,1);
tap0 = 0.5 * randn(3,1);
x0   = [ep0; eR0; ev0; eom0; tap0];
R0   = Rref * expm(hat(eR0));
om0  = eom0 + omref;
v0   = ev0 + vref;

[Ac, Bc, cc, eR0_out, eom0_out] = famp_linearize(R0, om0, v0, vref, T0, ...
    Rref, omref, alpref, aref, J, Dw, Ktau, Lam, g_t, Kdrag);

nfail = nfail + check('eR0 = log(Rref'' R0)', norm(eR0_out - eR0), 1e-9);
nfail = nfail + check('eom0 = om0 - omref',   norm(eom0_out - eom0), 1e-12);

F = @(x, nu) nonlinear_error_dynamics(x, nu, Rref, omref, alpref, aref, vref, ...
    T0, J, Dw, Ktau, Lam, g_t, Kdrag);

% Column scaling so that the tolerance is meaningful per state group.
% 4-point central stencils (O(h^4)) keep the nested logm differentiation
% inside F from dominating the comparison.
xs = [10*ones(3,1); 0.1*ones(3,1); 0.1*ones(3,1); 1e-3*ones(3,1); 0.1*ones(3,1)];
Afd = zeros(15,15);
for i = 1:15
    Afd(:,i) = fd4(@(z) F(z, zeros(4,1)), x0, i, 1e-3 * xs(i));
end
Bfd = zeros(15,4);
us = [1e-3; 0.1; 0.1; 0.1];
for i = 1:4
    Bfd(:,i) = fd4(@(z) F(x0, z), zeros(4,1), i, 1e-3 * us(i));
end
cfd = F(x0, zeros(4,1)) - Afd*x0;

% Compare each row block relative to its own scale, and report per block
blocks = {'e_p', 1:3; 'e_R', 4:6; 'e_v', 7:9; 'e_om', 10:12; 'tau_app', 13:15};
rowscale = abs(Afd) * xs + 1e-12;
errA_rows = abs(Ac - Afd) * xs ./ rowscale;
errc_rows = abs(cc - cfd) ./ (abs(cfd) + 1e-12 + 1e-3*max(abs(cfd)));
for b = 1:size(blocks,1)
    r = blocks{b,2};
    fprintf('     %-8s  Ac row err %.2e   cc row err %.2e\n', blocks{b,1}, max(errA_rows(r)), max(errc_rows(r)));
end
errA = max(errA_rows);
errB = max(abs(Bc(:) - Bfd(:))) / max(abs(Bfd(:)));   % matrix-relative (Bc is mostly zeros)
errc = max(errc_rows);
nfail = nfail + check('Ac vs FD (row-relative)', errA, 1e-5);
nfail = nfail + check('Bc vs FD (matrix-relative)', errB, 1e-8);
nfail = nfail + check('cc vs FD (relative)',     errc, 1e-5);

% Structural expectations
nfail = nfail + check('Ac(13:15,13:15) = -Lam', norm(Ac(13:15,13:15) + diag(Lam)), 1e-12);
nfail = nfail + check('Bc(13:15,2:4) = Lam',    norm(Bc(13:15,2:4) - diag(Lam)), 1e-12);
nfail = nfail + check('Ac(10:12,13:15) = J^-1 Ktau', norm(Ac(10:12,13:15) - diag(Ktau./J)), 1e-12);

% Special case from the .tex: upright, zero rate, hover reference -> d_R = d_w = 0,
% d_v = (T0 - g) e3
[Ac0, ~, cc0] = famp_linearize(eye(3), zeros(3,1), zeros(3,1), zeros(3,1), T0, ...
    eye(3), zeros(3,1), zeros(3,1), zeros(3,1), J, Dw, Ktau, Lam, g_t, 0.0);
nfail = nfail + check('hover: d_R = d_w = 0', norm([cc0(4:6); cc0(10:12)]), 1e-12);
nfail = nfail + check('hover: d_v = (T0-g) e3', norm(cc0(7:9) - [0;0;T0-g_t]), 1e-12);
nfail = nfail + check('hover: H_w = I, H_R = 0', norm(Ac0(4:6,10:12) - eye(3)) + norm(Ac0(4:6,4:6)), 1e-9);
nfail = nfail + check('hover: G_R = -T0 e3^', norm(Ac0(7:9,4:6) + T0*hat([0;0;1])), 1e-12);

fprintf('\n=== 2. famp_reference ===\n');
N = 10; dt = 12.9;
psi0 = 0.7; psidot = 1.26e-3;               % 1.26 rad/s spin
sdes_seq = repmat([0;0;1], 1, N+1);
[Rr, omr, alr] = famp_reference(sdes_seq, psi0, psidot, dt, N);
e_s = 0; e_psi = 0;
for k = 1:N+1
    e_s   = max(e_s, norm(Rr(:,3,k) - sdes_seq(:,k)));
    e_psi = max(e_psi, abs(atan2(Rr(2,1,k), Rr(1,1,k)) - wrap(psi0 + (k-1)*dt*psidot)));
end
nfail = nfail + check('upright spin: R*e3 = sdes', e_s, 1e-12);
nfail = nfail + check('upright spin: heading = psi_k', e_psi, 1e-12);
nfail = nfail + check('upright spin: omega* = psidot e3', max(vecnorm(omr - repmat([0;0;psidot],1,N+1))), 1e-12);
nfail = nfail + check('upright spin: alpha* = 0', max(abs(alr(:))), 1e-12);

% tilted, in the heading direction -> heading preserved exactly
th = 0.5; s = [sin(th)*cos(psi0); sin(th)*sin(psi0); cos(th)];
Rt = famp_reference(repmat(s,1,2), psi0, 0, dt, 1);
nfail = nfail + check('tilt toward heading: R*e3 = sdes', norm(Rt(:,3,1) - s), 1e-12);
nfail = nfail + check('tilt toward heading: heading exact', abs(atan2(Rt(2,1,1), Rt(1,1,1)) - psi0), 1e-12);
nfail = nfail + check('R* orthonormal', norm(Rt(:,:,1)'*Rt(:,:,1) - eye(3)), 1e-12);

% pitch-flip preview through inverted: omega* stays ~constant (continuous R*)
ang = linspace(0.9*pi, 1.1*pi, N+1);
sflip = [-sin(ang); zeros(1,N+1); cos(ang)];
[~, omf] = famp_reference(sflip, 0.0, 0, dt, N);
rate_expected = -(ang(2)-ang(1))/dt;        % Ry(-a): body-y rate = -da/dt
nfail = nfail + check('pitch flip through inverted: omega*_y continuous', ...
    max(abs(omf(2,1:N) - rate_expected)), 1e-9);
nfail = nfail + check('pitch flip through inverted: omega*_x,z = 0', max(abs([omf(1,:), omf(3,:)])), 1e-9);

fprintf('\n=== 3. Zero-yaw regression against upright_template_mpc_wl ===\n');
m = 85.655e-6; g = 9.81;
I_vec = [1.95e-9; 1.69e-9; 0.3e-9];
w14 = [1.0e3; 1.0e3; 1.5; 1.0; 3.0; 1.0e2; 3.0e2; 1.0e2; 8.0e2; 1.0e1; 1.0e-1; 1e-1; 1e-1; 1e4];
wyaw_default = [5.0e1; 3.0e6; 1.0e2; 1.0e2];   % = in-file defaults of full_attitude_mpc_wl
w18 = [w14; wyaw_default];   % zero heading error in every case -> template special case
kt2 = [0.1; 1.1];
kt3 = [0.1; 1.1; 1.0];

cases = struct('name', {}, 'Rot', {}, 'om', {}, 'p', {}, 'v', {}, 'tol', {});
cases(end+1) = struct('name', 'hover, 3 deg tilt, 2 cm offset, v = 0', ...
    'Rot', expm(hat([0.03; -0.05; 0.0])), 'om', [0.3; -0.2; 0.0], ...
    'p', [0.02; -0.01; 0.015], 'v', zeros(3,1), 'tol', 3e-2);
cases(end+1) = struct('name', 'hover, 3 deg tilt, heading 40 deg, v = 0', ...
    'Rot', expm(hat([0;0;0.7])) * expm(hat([0.03; -0.05; 0.0])), 'om', [0.3; -0.2; 0.0], ...
    'p', [0.02; -0.01; 0.015], 'v', zeros(3,1), 'tol', 3e-2);
cases(end+1) = struct('name', 'forward flight 0.3 m/s, 10 deg pitch (drag active)', ...
    'Rot', expm(hat([0.0; 0.17; 0.0])), 'om', [0.0; 0.5; 0.0], ...
    'p', [0.0; 0.0; 0.0], 'v', [0.3; 0.0; 0.0], 'tol', 1e-1);

for c = 1:numel(cases)
    cs = cases(c);
    args = {reshape(cs.Rot', 9, 1), cs.om, cs.p, cs.v, zeros(3,1), zeros(3,1), [0;0;1], ...
            I_vec, m, g, 1.0};
    clear upright_template_mpc_wl full_attitude_mpc_wl
    for it = 1:15    % 3 QP solves each (SOLVE_DECIM = 5)
        [pd_old, h0_old, acc_old, T_old, rx_old, ry_old] = upright_template_mpc_wl(args{:}, w14, kt2, 2e-4, zeros(3,1), zeros(3,1));
        [pd_new, h0_new, acc_new, T_new, rx_new, ry_new] = full_attitude_mpc_wl(args{:}, w18, kt3, 2e-4, zeros(3,1), zeros(3,1));
    end
    sc = max(abs(pd_old(1:5))) + 1e-9;
    err = max(abs(pd_new(1:5) - pd_old(1:5))) / sc;
    fprintf('  %-52s  pdotdes old = [%s]\n  %-52s  pdotdes new = [%s]\n', cs.name, num2str(pd_old(1:5)', '%9.4f'), '', num2str(pd_new(1:5)', '%9.4f'));
    nfail = nfail + check(sprintf('  pdotdes(1:5) rel. mismatch (%s)', cs.name), err, cs.tol);
    nfail = nfail + check('  h0_wl identical', norm(h0_new - h0_old), 1e-12);
    nfail = nfail + check('  thrust identical-ish', abs(T_new - T_old)/max(abs(T_old),1e-9), cs.tol);
    nfail = nfail + check('  yaw request small with zero heading error', abs(pd_new(6)), 5e-2);
end

fprintf('\n=== 4. Yaw sanity ===\n');
w18y = [w14; wyaw_default];
% (a) +0.1 rad heading error, hover, heading reference held at 0
clear full_attitude_mpc_wl
Rot = expm(hat([0;0;0.1]));
args = {reshape(Rot', 9, 1), zeros(3,1), zeros(3,1), zeros(3,1), zeros(3,1), zeros(3,1), [0;0;1], I_vec, m, g, 1.0};
% first call latches the heading reference to the current heading; hand it a
% valid reference of 0 so the error is +0.1 rad
for it = 1:15
    pd = full_attitude_mpc_wl(args{:}, w18y, kt3, 2e-4, zeros(3,1), [0.0; 0.0; 1.0]);
end
fprintf('  heading error +0.1 rad -> tau_z = %+.4f uN*m (box +-0.8)\n', pd(6));
nfail = nfail + check('restoring sign (tau_z < 0 for +e_psi)', -pd(6) > 0, true);
nfail = nfail + check('inside yaw box', abs(pd(6)) <= 0.8 + 1e-9, true);
nfail = nfail + check('stiffness in the designed range (2..8 uN*m/rad)', abs(pd(6))/0.1 >= 2 && abs(pd(6))/0.1 <= 8, true);
nfail = nfail + check('roll/pitch requests unaffected by pure yaw error', max(abs(pd(4:5))), 1e-3);

% (b) commanded spin 1.26 rad/s about the current heading -> rate-plant feed-forward
clear full_attitude_mpc_wl
args = {reshape(eye(3)', 9, 1), zeros(3,1), zeros(3,1), zeros(3,1), zeros(3,1), zeros(3,1), [0;0;1], I_vec, m, g, 1.0};
for it = 1:15
    pd = full_attitude_mpc_wl(args{:}, w18y, kt3, 2e-4, zeros(3,1), [0.0; 1.26; 1.0]);
end
tau_ff = 0.35e-6 * 1.26 * 1e6;   % [uN*m]
fprintf('  spin 1.26 rad/s -> tau_z = %+.4f uN*m (full rate-plant feed-forward b*psidot = %.3f)\n', pd(6), tau_ff);
nfail = nfail + check('spin feed-forward has the spin sign', pd(6) > 0, true);
nfail = nfail + check('spin feed-forward 10..100%% of b_yaw*psidot', pd(6) >= 0.1*tau_ff && pd(6) <= 1.0*tau_ff, true);

fprintf('\n=== 5. MATLAB Coder compile (optional) ===\n');
vv = ver;
if any(strcmp({vv.Name}, 'MATLAB Coder'))
    outdir = fullfile(tempdir, 'famp_codegen');
    try
        cfg = coder.config('mex');
        cfg.GenerateReport = false;
        codegen('-config', cfg, 'full_attitude_mpc_wl', '-args', ...
            {zeros(9,1), zeros(3,1), zeros(3,1), zeros(3,1), zeros(3,1), zeros(3,1), ...
             coder.typeof(zeros(3,1), [3 11], [0 1]), zeros(3,1), 0, 0, 0, ...
             zeros(18,1), zeros(3,1), 0, zeros(3,1), zeros(3,1)}, ...
            '-d', outdir, '-o', fullfile(outdir, 'full_attitude_mpc_wl_mex'));
        fprintf('  codegen OK -> %s\n', outdir);
        nfail = nfail + check('codegen compiles', 0, 1);
    catch ME
        fprintf('  codegen FAILED: %s\n', ME.message);
        nfail = nfail + 1;
    end
else
    fprintf('  (skipped: MATLAB Coder not installed; the Simulink build is the codegen check)\n');
end

fprintf('\n%d failure(s)\n', nfail);
if nfail > 0
    error('test_full_attitude_mpc: %d failure(s)', nfail);
end

% =========================================================================
function xdot = nonlinear_error_dynamics(x, nu, Rref, omref, alpref, aref, vref, ...
    T0, J, Dw, Ktau, Lam, g, Kdrag)
% Independent implementation of the nonlinear error dynamics in the .tex,
% using expm/logm. Used only to check famp_linearize.
eR = x(4:6); ev = x(7:9); eom = x(10:12); tap = x(13:15);
R  = Rref * expm(hat(eR));
om = eom + omref;
v  = ev + vref;
T  = T0 + nu(1);
tau = nu(2:4);
e3 = [0;0;1];
% attitude error rate: d/dh log( exp(-h w*^) exp(eR^) exp(h w^) ) at h = 0,
% 4-point stencil so the derivative is accurate to ~1e-11
q = @(h) vee(real(logm(expm(-h*hat(omref)) * expm(hat(eR)) * expm(h*hat(om)))));
h = 1e-3;
eRdot = (-q(2*h) + 8*q(h) - 8*q(-h) + q(-2*h)) / (12*h);
b1 = R(:,1);
aD = -Kdrag * (b1'*v) * b1;
evdot = T*R*e3 - g*e3 + aD - aref;
omdot = (Ktau.*tap - cross(om, J.*om) - Dw.*om) ./ J - alpref;
tapdot = Lam .* (tau - tap);
xdot = [ev; eRdot; evdot; omdot; tapdot];
end

function col = fd4(fun, x, i, h)
% i-th column of the Jacobian of fun at x, 4-point central stencil
d = zeros(size(x)); d(i) = h;
col = (-fun(x+2*d) + 8*fun(x+d) - 8*fun(x-d) + fun(x-2*d)) / (12*h);
end

function S = hat(v)
S = [0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0];
end

function v = vee(S)
v = [S(3,2); S(1,3); S(2,1)];
end

function a = wrap(a)
a = atan2(sin(a), cos(a));
end

function nf = check(name, val, tol)
if islogical(tol)
    ok = (val == tol);
    fprintf('  [%s] %s\n', pf(ok), name);
else
    ok = val <= tol;
    fprintf('  [%s] %-58s %.3e (tol %.1e)\n', pf(ok), name, val, tol);
end
nf = double(~ok);
end

function s = pf(ok)
if ok, s = 'PASS'; else, s = 'FAIL'; end
end
