function [pdotdes, h0_wl, accdes, thrust_desired, roll_torque_desired, ...
    pitch_torque_desired, yaw_torque_desired] = full_attitude_mpc_wl( ...
    R_cur, omega, p_cur, v_cur, pdes, dpdes, sdes_in, ...
    I_moment_vec, m, g, cntrl_enable, weights_vec, k_tau, ctrl_Ts, a_ref, yaw_ref) %#ok<INUSD>
%#codegen
% FULL-ATTITUDE MPC for the MPC -> WLQP cascade. Drop-in replacement for
% upright_template_mpc_wl.m (identical signature and outputs) that keeps the
% full rotation R in the predicted state instead of the reduced attitude
% s = R e3, so yaw is optimized by the QP together with roll and pitch.
% Formulation: mpc_full_template.tex, "MPC: Full Attitude Control".
%
% STATE / INPUT (template units mm, ms, mg; torque 1e-6 N*m)
%   x  = [e_p(3); e_R(3); e_v(3); e_om(3); tau_app(3)]   (15)
%   nu = [dT; tau_x; tau_y; tau_z]                        (4)
%   e_p = p - p*,   e_R = log(R*' R)  (rotation vector, ref-body frame),
%   e_v = v - v*,   e_om = omega - omega*,   tau_app = lagged torque command.
%
% MODEL (per stage k, built by famp_linearize.m, discretized here)
%   The linearization point is the CURRENT attitude R0 / rate / velocity
%   (frozen over the horizon, as in the template MPC) expressed in the
%   coordinates of each stage's reference R*_k, so a moving reference
%   (flip preview, heading spin) produces stage-dependent A_k, c_k.
%   Discretization is forward Euler except for two rows handled exactly
%   under a held input:
%     * tau_app rows: tau_{k+1} = (1-beta) tau_k + beta tau_cmd
%     * e_om rows   : each axis i uses its own effective step
%                       dt_eff_i = (1 - exp(-b_i dt/J_i)) / (b_i/J_i)
%                     and self-decay exp(-b_i dt/J_i). This matters for YAW:
%                     the free plant is a heavily damped RATE plant
%                     (b_yaw/J_z ~ 1/ms >> 1/dt), for which plain Euler would
%                     give a decay of about -12 and a diverging prediction.
%                     With b_i = 0 it reduces to Euler, i.e. the template.
%
% REFERENCE
%   desTraj supplies sdes (3x1 held or 3x(N+1) preview) and yaw_ref =
%   [psi_des; dpsi_des; valid]. famp_reference.m lifts these to
%   R*_k = Rmin(e3 -> sdes_k) Rz(psi_k) with psi_k = psi_ref + k dt psi_dot_ref,
%   and finite-differences omega*_k, alpha*_k. The heading reference
%   psi_ref keeps the template's behaviour: latched to the measured heading
%   at engagement, and (HEADING_FOLLOW) slewed toward yaw_ref(1) at at most
%   PSI_SLEW_MAX when yaw_ref(3) > 0.5.
%
% YAW
%   tau_z is a QP decision variable: pdotdes(6) = tau_z* (held between
%   solves) + yaw_int. yaw_int is the same heading integrator as before
%   (k_i_yaw), kept because the popts Mz row carries a DC bias of the size
%   of a typical yaw command; it plays for yaw the role T0 plays for thrust
%   and is NOT part of the QP model or of the tau_app estimator.
%   The heading PD is gone; its job is the QP's.
%
% WEIGHTS  weights_vec is 14 (template layout, yaw weights from the
%   in-file defaults below) or 18:
%   [ws wds wpr_xy wpr_z wpf wvr_xy wvr_z wvf_xy wvf_z wthrust wmom
%    wdmom_roll wdmom_pitch wdthrust | w_yaw w_dyaw wmom_yaw wdmom_yaw]
%   ws/wds weight e_R(1:2)/e_om(1:2) [rad, rad/ms]: for small tilt these are
%   numerically the template's e_s / e_ds, so tuned values carry over.
% k_tau is 2 ([roll pitch], yaw gain 1) or 3.
%
% OUTPUTS are identical in meaning and units to upright_template_mpc_wl.m:
%   pdotdes (6x1) desired momentum rate, BODY frame, template units
%   h0_wl   (6x1) gravity bias, BODY frame, template units
%   accdes  (6x1) [world lin acc (m/s^2); body ang acc (rad/s^2)]
%   thrust/roll/pitch/yaw_desired  SI wrench (logging / non-WLQP use)

% -------------------------------------------------------------------------
% Fixed dimensions for Simulink codegen
% -------------------------------------------------------------------------
R_cur = reshape(R_cur, 9, 1);
omega = reshape(omega, 3, 1);
p_cur = reshape(p_cur, 3, 1);
v_cur = reshape(v_cur, 3, 1);
pdes = reshape(pdes, 3, 1);
dpdes = reshape(dpdes, 3, 1);
sdes_in = reshape(sdes_in, 3, []);   % 3x1 held or 3x(N+1) preview
I_moment_vec = reshape(I_moment_vec, 3, 1);
m = m(1);
g = g(1);
cntrl_enable = cntrl_enable(1);
a_ref = reshape(a_ref, 3, 1);
if nargin < 16
    yaw_ref = [0.0; 0.0; 0.0];
end
yaw_ref = reshape(yaw_ref, 3, 1);

wv = weights_vec(:);
ktv = k_tau(:);

e_x = p_cur - pdes;
e_v = v_cur - dpdes;

% -------------------------------------------------------------------------
% MPC constants (template units: mm, ms, mg)
% -------------------------------------------------------------------------
N = 10;         % horizon steps
dt = 12.9;      % [ms] prediction step = TWO wingbeats at 155 Hz
nu = 4;
nx = 15;        % [e_p; e_R; e_v; e_om; tau_app]
nU = nu*N;

controller_dt = 0.2;   % [ms] Simulink sample time of this block (2e-4 s)
% Per-axis torque actuation lag [ms] (roll, pitch, yaw). Roll/pitch as in the
% template (system-id: dead+lag). Yaw is unidentified; the free plant
% responded with ~no lag to map torque, so this is conservative.
tau_lag = [12.9 / (-log(0.05)); 12.9 / (-log(0.05)); 12.9 / (-log(0.05))];

SOLVE_DECIM = 5;       % controller steps per QP solve (5 x 0.2 ms = 1 ms)

ws      = wv(1);   % running attitude (roll/pitch) weight      [1/rad^2]
wds     = wv(2);   % running body-rate (roll/pitch) weight
wpr_xy  = wv(3);   % running position weight
wpr_z   = wv(4);
wpf     = wv(5);   % final position weight
wvr_xy  = wv(6);   % running horizontal-velocity weight
wvr_z   = wv(7);   % running vertical-velocity weight
wvf_xy  = wv(8);   % final horizontal-velocity weight
wvf_z   = wv(9);   % final vertical-velocity weight
wthrust     = wv(10); % specific-thrust-correction effort
wmom        = wv(11); % roll/pitch torque magnitude effort
wdmom_roll  = wv(12); % roll torque-change penalty
wdmom_pitch = wv(13); % pitch torque-change penalty
wdthrust    = wv(14); % commanded specific-thrust-change penalty
% Yaw weights. The yaw axis is a RATE plant to the QP (see D_omega below), so
% the familiar roll/pitch numbers do not transfer. Sizing (offline probes,
% test_full_attitude_mpc.m, box +-0.8 uN*m):
%   * wmom_yaw sets how cheap yaw is as a lever for OTHER errors. The one
%     such lever left in the model is real physics: in forward flight
%     yawing redirects the drag force sideways (A_DR). A 1 cm lateral error
%     at 0.5 m/s asks ~0.1 uN*m of yaw at wmom_yaw = 100 and rails the box
%     at wmom_yaw <= 1. (A second, spurious lever through the e_R(1:2)
%     coordinates is removed by the tilt cost, see the stage-weights note.)
%     Given the unidentified free-flight h2 couplings, keep wmom_yaw >= ~30.
%   * w_yaw / wmom_yaw sets the heading stiffness: 50/100 gives ~4.5 uN*m/rad
%     (rails at ~0.18 rad error), i.e. a ~14 rad/s first-order heading loop
%     on the rate plant K ~ 2.9 rad/s per uN*m.
%   * w_dyaw sets the spin feed-forward tau_z -> b_yaw*psi_dot through the
%     e_om(3) = omega_z - omega*_z cost. e_om is in rad/ms, hence the large
%     number: 3e6 recovers ~20% of the 0.44 uN*m needed at 1.26 rad/s, the
%     rest comes from the heading error (~5 deg lag) or the integrator.
if numel(wv) >= 18
    w_yaw     = wv(15);   % running heading-error weight e_R(3)   [1/rad^2]
    w_dyaw    = wv(16);   % running yaw-rate error weight e_om(3) [1/(rad/ms)^2]
    wmom_yaw  = wv(17);   % yaw torque magnitude effort
    wdmom_yaw = wv(18);   % yaw torque-change penalty
else
    w_yaw     = 5.0e1;
    w_dyaw    = 3.0e6;
    wmom_yaw  = 1.0e2;
    wdmom_yaw = 1.0e2;
end

k_tau_roll  = ktv(1);
k_tau_pitch = ktv(2);
if numel(ktv) >= 3
    k_tau_yaw = ktv(3);
else
    k_tau_yaw = 1.0;
end

% Rotational damping D_omega, torque per rate, SI [N*m*s] (roll, pitch, yaw).
% Roll/pitch: 0, as in the current template configuration (oscillations are
% treated as a modeling/tuning problem, not hidden by fudge damping).
% Yaw: the free plant is a rate plant, omega_z ~ K * tau_z with
% K = 2.4..3.3 rad/s per uN*m of MAP torque (yaw_spin3/4, spin8/9), i.e.
% b_yaw = 1/K ~ 0.35e-6 N*m*s with k_tau_yaw = 1 (delivered fraction is
% lumped into b). Jz/b_yaw ~ 0.9 ms: within one 12.9 ms step the yaw rate
% has fully settled to tau_z/b_yaw (see the discretization note above).
D_omega_SI = [0.0; 0.0; 0.35e-6];

% Actuator limits (SI). Boxes on the template model's inputs; the WLQP has
% its own voltage-level limits downstream.
thrust_min_N = 0.55e-3;   % [N]
thrust_max_N = 1.6e-3;    % [N]
roll_max_Nm  = 10e-6;     % [N*m]
pitch_max_Nm = 9e-6;      % [N*m]
yaw_max_Nm   = 0.8e-6;    % [N*m] the h2 channel spans about +-1.0e-6 N*m

% Heading integrator (see YAW in the header). k_i_yaw = 0 disables it; the
% previous template setting. Turn on (0.3) once the QP yaw loop is verified.
k_i_yaw     = 0.0;   % [1e-6 N*m / (rad*s)]
tau_int_max = 0.4;   % [1e-6 N*m]

HEADING_FOLLOW = true;
PSI_SLEW_MAX   = 6.0;   % [rad/s]

cx_hat = 0.70e-3;       % [N/(m/s)] cycle-averaged forward drag

% -------------------------------------------------------------------------
% Unit conversion to template units
% -------------------------------------------------------------------------
m_t = m * 1.0e6;                 % kg      -> mg
g_t = g * 1.0e-3;                % m/s^2   -> mm/ms^2
ex_t = e_x * 1.0e3;              % m       -> mm
ev_t = e_v;                      % m/s == mm/ms
om_t = omega * 1.0e-3;           % rad/s   -> rad/ms
dpdes_t = dpdes;                 % m/s == mm/ms
v_t = v_cur;                     % m/s == mm/ms
Ib_t = I_moment_vec * 1.0e12;    % kg*m^2  -> mg*mm^2
aref_t = a_ref * 1.0e-3;         % m/s^2   -> mm/ms^2
D_t = D_omega_SI * 1.0e9;        % N*m*s   -> 1e-6 N*m per rad/ms
Kdrag_t = (cx_hat / max(m, 1.0e-12)) * 1.0e-3;   % [1/ms]
Ktau = [k_tau_roll; k_tau_pitch; k_tau_yaw];
Lam = 1.0 ./ tau_lag;            % [1/ms]

Tmin_sp = thrust_min_N * 1.0e3 / max(m_t, 1.0e-9);  % specific thrust [mm/ms^2]
Tmax_sp = thrust_max_N * 1.0e3 / max(m_t, 1.0e-9);
tau_max = [roll_max_Nm; pitch_max_Nm; yaw_max_Nm] * 1.0e6;   % -> 1e-6 N*m

% -------------------------------------------------------------------------
% Persistent controller state
% -------------------------------------------------------------------------
persistent T0 tau_applied thrust_cmd_prev_sp tau_cmd_prev U_prev ...
    solve_tick pdotdes_hold accdes_hold psi_des_hold psi_latched yaw_int
if isempty(T0)
    T0 = 0.0;
end
if isempty(psi_des_hold)
    psi_des_hold = 0.0;
end
if isempty(psi_latched)
    psi_latched = false;
end
if isempty(yaw_int)
    yaw_int = 0.0;
end
if isempty(tau_applied)
    tau_applied = zeros(3,1);
end
if isempty(thrust_cmd_prev_sp)
    thrust_cmd_prev_sp = 0.0;
end
if isempty(tau_cmd_prev)
    tau_cmd_prev = zeros(3,1);
end
if isempty(U_prev)
    U_prev = zeros(nU,1);
end
if isempty(solve_tick)
    solve_tick = 0.0;
end
if isempty(pdotdes_hold)
    pdotdes_hold = zeros(6,1);
end
if isempty(accdes_hold)
    accdes_hold = zeros(6,1);
end

% -------------------------------------------------------------------------
% Soft-start envelope
% -------------------------------------------------------------------------
en = cntrl_enable;
if en < 0.0
    en = 0.0;
elseif en > 1.0
    en = 1.0;
end
if en < 0.01
    T0 = 0.0;
    tau_applied = zeros(3,1);
    thrust_cmd_prev_sp = 0.0;
    tau_cmd_prev = zeros(3,1);
    U_prev = zeros(nU,1);
    solve_tick = 0.0;
    pdotdes_hold = zeros(6,1);
    accdes_hold = zeros(6,1);
    psi_latched = false;
    yaw_int = 0.0;
end
T_ceiling = en * Tmax_sp;
T_floor   = min(Tmin_sp, T_ceiling);
tau_eff   = (0.3 + 0.7*en) * tau_max;          % torque boxes open 30% -> 100%

% -------------------------------------------------------------------------
% Current attitude
% -------------------------------------------------------------------------
Rot = [R_cur(1), R_cur(2), R_cur(3); ...
       R_cur(4), R_cur(5), R_cur(6); ...
       R_cur(7), R_cur(8), R_cur(9)];

% -------------------------------------------------------------------------
% Heading reference (every call, before the solve so the preview sees it).
% psi is the world heading of body-x. Latched at engagement; with
% HEADING_FOLLOW slewed toward yaw_ref(1) when yaw_ref(3) > 0.5.
% -------------------------------------------------------------------------
psi = atan2(Rot(2,1), Rot(1,1));
if ~psi_latched
    psi_des_hold = psi;
    psi_latched = true;
end
psi_dot_ref = 0.0;                              % [rad/s]
dt_s = controller_dt * 1.0e-3;                  % [s] per call
if HEADING_FOLLOW && yaw_ref(3) > 0.5
    psi_tgt  = yaw_ref(1);
    d_psi    = atan2(sin(psi_tgt - psi_des_hold), cos(psi_tgt - psi_des_hold));
    step_max = PSI_SLEW_MAX * dt_s;
    if d_psi > step_max
        psi_des_hold = psi_des_hold + step_max;
        psi_dot_ref  = PSI_SLEW_MAX;
    elseif d_psi < -step_max
        psi_des_hold = psi_des_hold - step_max;
        psi_dot_ref  = -PSI_SLEW_MAX;
    else
        psi_des_hold = psi_tgt;
        psi_dot_ref  = yaw_ref(2);
    end
    psi_des_hold = atan2(sin(psi_des_hold), cos(psi_des_hold));
end
e_psi = atan2(sin(psi - psi_des_hold), cos(psi - psi_des_hold));   % wrapped

% -------------------------------------------------------------------------
% Solve decimation gate
% -------------------------------------------------------------------------
do_solve = (solve_tick <= 0.5);
if do_solve
    solve_tick = SOLVE_DECIM - 1;
else
    solve_tick = solve_tick - 1;
end

if do_solve

% -------------------------------------------------------------------------
% Full-attitude reference preview R*_k, omega*_k, alpha*_k  (k = 0..N)
% -------------------------------------------------------------------------
[Rref, omref, alpref] = famp_reference(sdes_in, psi_des_hold, ...
                                       psi_dot_ref * 1.0e-3, dt, N);

% -------------------------------------------------------------------------
% Stage models. Linearization point: current (Rot, om_t, v_t, T0), expressed
% in each stage's reference coordinates. Discretize per the header note.
% -------------------------------------------------------------------------
beta = 1.0 - exp(-dt ./ tau_lag);          % lag step fractions (3x1)
bJ   = D_t ./ max(Ib_t, 1.0e-9);           % b_i / J_i  [1/ms]
decay_om = zeros(3,1);
dt_eff   = zeros(3,1);
for i = 1:3
    if abs(bJ(i)) < 1.0e-9
        decay_om(i) = 1.0;
        dt_eff(i)   = dt;
    else
        decay_om(i) = exp(-bJ(i) * dt);
        dt_eff(i)   = (1.0 - decay_om(i)) / bJ(i);
    end
end

AdSeq = zeros(nx, nx, N);
cdSeq = zeros(nx, N);
CtSeq = zeros(3, 3, N);     % tilt output: z_k = CtSeq(:,:,k) * e_R,k + dtSeq(:,k)
dtSeq = zeros(3, N);
eR0 = zeros(3,1);
eom0 = zeros(3,1);
Bd = zeros(nx, nu);
e3 = [0.0; 0.0; 1.0];
e3h = [0.0, -1.0, 0.0; 1.0, 0.0, 0.0; 0.0, 0.0, 0.0];   % hat(e3)
for k = 1:N
    [Ac, Bc, cc, eRk, eomk, Jrk] = famp_linearize( ...
        Rot, om_t, v_t, dpdes_t, T0, Rref(:,:,k), omref(:,k), alpref(:,k), ...
        aref_t, Ib_t, D_t, Ktau, Lam, g_t, Kdrag_t);
    if k == 1
        eR0 = eRk;
        eom0 = eomk;
        Bd = dt * Bc;
        Bd(13:15, :) = 0.0;
        Bd(13, 2) = beta(1);
        Bd(14, 3) = beta(2);
        Bd(15, 4) = beta(3);
    end
    % Linearized physical tilt error in the stage-k reference frame,
    %   z = exp(e_R^) e3 - e3  ~  Ct e_R + dtilt,  Ct = -Q0 e3^ Jr(e_R0)
    % with Q0 = R*_k' R0. This, not e_R(1:2), carries the ws weight (see
    % "tilt cost" below). At e_R0 = 0 it is z = [-e_R(2); e_R(1); 0].
    Q0k = Rref(:,:,k)' * Rot;
    Ctk = -Q0k * e3h * Jrk;
    CtSeq(:,:,k) = Ctk;
    dtSeq(:,k)   = Q0k*e3 - e3 - Ctk*eRk;
    Ad = eye(nx) + dt * Ac;
    cd = dt * cc;
    % e_om rows: exact self-decay, effective step for everything else
    for i = 1:3
        r = 9 + i;
        Ad(r, :) = dt_eff(i) * Ac(r, :);
        Ad(r, r) = decay_om(i);
        cd(r)    = dt_eff(i) * cc(r);
    end
    % tau_app rows: exact held-input lag update
    Ad(13:15, :) = 0.0;
    Ad(13, 13) = 1.0 - beta(1);
    Ad(14, 14) = 1.0 - beta(2);
    Ad(15, 15) = 1.0 - beta(3);
    cd(13:15) = 0.0;
    AdSeq(:, :, k) = Ad;
    cdSeq(:, k) = cd;
end

% Error state x0 relative to the stage-0 reference
x0 = [ex_t; eR0; ev_t; eom0; tau_applied];

% -------------------------------------------------------------------------
% Stage weights: x = [e_p; e_R; e_v; e_om; tau_app]
%
% TILT COST. The attitude weight ws is NOT put on the coordinates e_R(1:2):
% a rotation vector's x/y components are not the physical tilt once the
% tilt error is nonzero, and with the Jacobians frozen at e_R0 a body-yaw
% rotation (which leaves R e3 untouched) moves e_R(1:2) at first order
% along the QP's trajectory. With yaw being a cheap rate axis the QP
% exploited that whenever roll/pitch railed (offline probe: yaw torque
% pinned at the box for a pure tilt error; with Jr := I it vanished).
% Instead ws weights the linearized tilt output z_k = Ct_k e_R,k + dtilt_k
% (built above): its composition with the yaw kinematics is exactly zero,
%   Ct_k * Jr^-1 e3 = -Q0 e3^ e3 = 0,
% and at e_R0 = 0 it reduces to ws (e_R(1)^2 + e_R(2)^2), the template cost.
% Heading keeps its own diagonal weight w_yaw on e_R(3).
% -------------------------------------------------------------------------
Qrun = [wpr_xy; wpr_xy; wpr_z; ...
        0; 0; w_yaw; ...
        wvr_xy; wvr_xy; wvr_z; ...
        wds; wds; w_dyaw; ...
        0; 0; 0];
Qfin = [wpf; wpf; wpf; ...
        0; 0; w_yaw; ...
        wvf_xy; wvf_xy; wvf_z; ...
        wds; wds; w_dyaw; ...
        0; 0; 0];
Rdiag = [wthrust; wmom; wmom; wmom_yaw];
Wd    = [wdthrust; wdmom_roll; wdmom_pitch; wdmom_yaw];

% -------------------------------------------------------------------------
% Condense:  x_k = Phi_k x0 + sum_j G(k,j) u_j + w_k,  k = 1..N
% -------------------------------------------------------------------------
Gamma = zeros(nx*N, nU);
xfree = zeros(nx*N, 1);
for k = 1:N
    Adk = AdSeq(:, :, k);
    if k == 1
        xfree(1:nx) = Adk*x0 + cdSeq(:, 1);
    else
        xfree((k-1)*nx+1:k*nx) = Adk*xfree((k-2)*nx+1:(k-1)*nx) + cdSeq(:, k);
    end
    for j = 1:k
        if j == k
            Gblk = Bd;
        else
            Gprev = Gamma((k-2)*nx+1:(k-1)*nx, (j-1)*nu+1:j*nu);
            Gblk = Adk*Gprev;
        end
        Gamma((k-1)*nx+1:k*nx, (j-1)*nu+1:j*nu) = Gblk;
    end
end

H = zeros(nU, nU);
h = zeros(nU, 1);
for k = 1:N
    if k == N
        Qk = Qfin;
    else
        Qk = Qrun;
    end
    Gk = Gamma((k-1)*nx+1:k*nx, :);
    xf = xfree((k-1)*nx+1:k*nx);
    GkQ = Gk' .* repmat(Qk', nU, 1);       % Gk' * diag(Qk)
    H = H + GkQ * Gk;
    h = h + GkQ * xf;
    % tilt cost  ws * |Ct_k e_R,k + dtilt_k|^2  (see the note above)
    CG = CtSeq(:,:,k) * Gk(4:6, :);                    % 3 x nU
    zf = CtSeq(:,:,k) * xf(4:6) + dtSeq(:,k);          % free-response tilt
    H = H + ws * (CG' * CG);
    h = h + ws * (CG' * zf);
end
for k = 1:N
    base = (k-1)*nu;
    for i = 1:nu
        H(base+i, base+i) = H(base+i, base+i) + Rdiag(i);
    end
end

% -------------------------------------------------------------------------
% Input-change penalties (first move vs previous issued command, then
% successive differences).  QP: min 0.5 U'HU + h'U
% -------------------------------------------------------------------------
u_prev_ref = [thrust_cmd_prev_sp - T0; tau_cmd_prev];
for k = 1:N
    for i = 1:nu
        idx = (k-1)*nu + i;
        if k == 1
            H(idx, idx) = H(idx, idx) + Wd(i);
            h(idx) = h(idx) - Wd(i)*u_prev_ref(i);
        else
            idp = idx - nu;
            H(idx, idx) = H(idx, idx) + Wd(i);
            H(idp, idp) = H(idp, idp) + Wd(i);
            H(idx, idp) = H(idx, idp) - Wd(i);
            H(idp, idx) = H(idp, idx) - Wd(i);
        end
    end
end

% -------------------------------------------------------------------------
% Box constraints per stage
% -------------------------------------------------------------------------
Umin = zeros(nU,1);
Umax = zeros(nU,1);
for k = 1:N
    base = (k-1)*nu;
    Umin(base+1) = T_floor - T0;
    Umax(base+1) = T_ceiling - T0;
    for i = 1:3
        Umin(base+1+i) = -tau_eff(i);
        Umax(base+1+i) =  tau_eff(i);
    end
end

% -------------------------------------------------------------------------
% Solve (warm-started)
% -------------------------------------------------------------------------
U = solve_box_qp_coordinate_descent(H, h, Umin, Umax, nU, U_prev);
U_prev = U;

% -------------------------------------------------------------------------
% Apply first input; advance thrust linearization point
% -------------------------------------------------------------------------
utilde0 = U(1);
tau0 = U(2:4);

x1 = AdSeq(:, :, 1)*x0 + Bd*[utilde0; tau0] + cdSeq(:, 1);

T0 = T0 + utilde0;
if T0 < T_floor
    T0 = T_floor;
elseif T0 > T_ceiling
    T0 = T_ceiling;
end
thrust_cmd_prev_sp = T0;
tau_cmd_prev = tau0;

% -------------------------------------------------------------------------
% WLQP interface outputs (umpcUpdate accdes recipe, condensed form)
% -------------------------------------------------------------------------
v1des_t = x1(7:9) + dpdes_t + dt*aref_t;       % [mm/ms] world frame
acc_lin_t = (v1des_t - v_t) / dt;              % [mm/ms^2] world frame

% Angular demand: the commanded moment IS the desired angular momentum rate
% (the lag state means (om1des - om0)/dt would carry no attitude stiffness).
acc_ang_t = tau0 ./ max(Ib_t, 1.0e-9);         % [rad/ms^2] body frame

pdotdes_hold = [m_t * (Rot' * acc_lin_t); tau0];
accdes_hold = [acc_lin_t * 1.0e3; acc_ang_t * 1.0e6];   % [m/s^2; rad/s^2]

end % do_solve

% -------------------------------------------------------------------------
% Every call (5 kHz): applied-moment lag estimate, heading integrator,
% gravity bias, held outputs.
% -------------------------------------------------------------------------
for i = 1:3
    beta_c = 1.0 - exp(-controller_dt / tau_lag(i));
    tau_applied(i) = tau_applied(i) + beta_c*(tau_cmd_prev(i) - tau_applied(i));
end

% Heading integrator with conditional anti-windup (as in the template).
tau_yaw_max = tau_max(3);
yaw_int_step = -en * k_i_yaw * e_psi * dt_s;   % [1e-6 N*m]
tau_pre = pdotdes_hold(6) + yaw_int;
saturating = (tau_pre >  tau_yaw_max && yaw_int_step > 0.0) || ...
             (tau_pre < -tau_yaw_max && yaw_int_step < 0.0);
if ~saturating
    yaw_int = clamp_scalar(yaw_int + yaw_int_step, -tau_int_max, tau_int_max);
end
tau_z_t = clamp_scalar(pdotdes_hold(6) + yaw_int, -tau_yaw_max, tau_yaw_max);

pdotdes = pdotdes_hold;
pdotdes(6) = tau_z_t;

accdes = accdes_hold;
accdes(6) = tau_z_t / max(Ib_t(3), 1e-9) * 1.0e6;   % rad/ms^2 -> rad/s^2

h0_wl = [Rot' * [0; 0; m_t*g_t]; 0; 0; 0];

% -------------------------------------------------------------------------
% Legacy wrench outputs (SI)
% -------------------------------------------------------------------------
thrust_desired = m_t * T0 * 1.0e-3;               % mg*mm/ms^2 -> N
roll_torque_desired  = tau_cmd_prev(1) * 1.0e-6;  % mg*mm^2/ms^2 -> N*m
pitch_torque_desired = tau_cmd_prev(2) * 1.0e-6;
yaw_torque_desired   = tau_z_t * 1.0e-6;

thrust_desired = clamp_scalar(thrust_desired, 0.0, thrust_max_N);
roll_torque_desired = clamp_scalar(roll_torque_desired, -roll_max_Nm, roll_max_Nm);
pitch_torque_desired = clamp_scalar(pitch_torque_desired, -pitch_max_Nm, pitch_max_Nm);

end

function y = clamp_scalar(x, xmin, xmax)
%#codegen
y = x;
if y > xmax
    y = xmax;
elseif y < xmin
    y = xmin;
end
end

function U = solve_box_qp_coordinate_descent( ...
    H, h, Umin, Umax, nU, U_init)
%#codegen
% min 0.5*U'*H*U + h'*U  s.t. Umin <= U <= Umax, warm-started projected
% coordinate descent with a fixed sweep count (deterministic codegen).
U = reshape(U_init, nU, 1);
for i = 1:nU
    if U(i) < Umin(i)
        U(i) = Umin(i);
    elseif U(i) > Umax(i)
        U(i) = Umax(i);
    end
end
for iter = 1:30
    for i = 1:nU
        grad_i = h(i);
        for j = 1:nU
            grad_i = grad_i + H(i,j) * U(j);
        end
        U(i) = U(i) - grad_i / max(H(i,i), 1.0e-18);
        if U(i) < Umin(i)
            U(i) = Umin(i);
        elseif U(i) > Umax(i)
            U(i) = Umax(i);
        end
    end
end
end
