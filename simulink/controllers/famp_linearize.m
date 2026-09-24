function [Ac, Bc, cc, eR0, eom0, Jr] = famp_linearize( ...
    R0, om0, v0, vref, T0, Rref, omref, alpref, aref, J, Dw, Ktau, Lam, g, Kdrag)
%#codegen
%FAMP_LINEARIZE Continuous-time full-attitude error model  xdot = Ac x + Bc nu + cc
%
% Implements Section 1 of mpc_full_template.tex ("MPC: Full Attitude
% Control") including the actuator-lag, rotational-damping and forward-drag
% augmentation. Everything is in the MPC's TEMPLATE UNITS (mm, ms, mg):
% force 1e-3 N, torque 1e-6 N*m, inertia 1e-12 kg*m^2, rate rad/ms.
%
% State (15)   x  = [e_p(3); e_R(3); e_v(3); e_omega(3); tau_app(3)]
% Input (4)    nu = [dT; tau_x; tau_y; tau_z]      (dT = T - T0, specific)
%
%   e_p   = p - p*                    [mm]
%   e_R   = log(R*' R)                [rad]     rotation vector, ref-body frame
%   e_v   = v - v*                    [mm/ms]   world frame
%   e_om  = omega - omega*            [rad/ms]  body minus ref-body components
%   tau_app                           [1e-6 N*m] first-order-lagged command
%
% Inputs
%   R0     (3x3)  current attitude, body->world (linearization point)
%   om0    (3x1)  current body angular velocity            [rad/ms]
%   v0     (3x1)  current world velocity                   [mm/ms]
%   vref   (3x1)  reference velocity v* (for e_v0 = v0 - vref)
%   T0     (1x1)  operating specific thrust                [mm/ms^2]
%   Rref   (3x3)  attitude reference R* at this stage
%   omref  (3x1)  reference angular velocity omega* (ref-body frame) [rad/ms]
%   alpref (3x1)  reference angular acceleration alpha_ref  [rad/ms^2]
%   aref   (3x1)  reference linear acceleration a_ref (world) [mm/ms^2]
%   J      (3x1)  principal inertias                        [mg*mm^2]
%   Dw     (3x1)  rotational damping, torque per rate       [1e-6 N*m / (rad/ms)]
%   Ktau   (3x1)  delivered-torque gains (dimensionless)
%   Lam    (3x1)  1 / actuator lag time constants           [1/ms]
%   g      (1x1)  gravity                                   [mm/ms^2]
%   Kdrag  (1x1)  c_x / m forward-drag coefficient          [1/ms]
%
% Outputs
%   Ac (15x15), Bc (15x4), cc (15x1)  as in the .tex "complete augmented
%                                     matrices"
%   eR0  (3x1)  e_R at the operating point  = log(Rref' R0)
%   eom0 (3x1)  e_omega at the operating point = om0 - omref
%   Jr   (3x3)  right Jacobian Jr(eR0): d(local body rotation)/d(e_R), used
%               by the controller to build the linearized TILT output
%               (see full_attitude_mpc_wl.m, "tilt cost")
%
% Conventions and the two places this deviates from the .tex, on purpose:
%   * K_tau sits in the BODY input row (Ac(10:12,13:15) = J^-1 K_tau) and the
%     lag state stores the UNSCALED command (Bc(13:15,2:4) = Lam). The .tex
%     notes both placements are equivalent; this one keeps tau_app equal to
%     the "applied moment" estimator that upright_template_mpc_wl.m already
%     propagates at the controller rate, so that persistent state carries
%     over unchanged.
%   * H_R = d f_R / d e_R is evaluated by central finite differences of the
%     exact kinematics f_R (3 extra evaluations of a closed-form function),
%     instead of the analytic derivative of Jr^-1. Cheap, and it keeps the
%     drift term d_R exactly consistent with f_R.
%
% Row-by-row (see the .tex appendix):
%   e_p'   = e_v
%   e_R'   = H_R e_R + H_w e_om + d_R,      f_R = Jr(e_R)^-1 [e_om + w* - exp(-e_R^) w*]
%   e_v'   = (G_R + A_DR) e_R + A_Dv e_v + R0 e3 dT + d_v + a_D,aff
%   e_om'  = (A_w - J^-1 D) e_om + J^-1 K_tau tau_app + d_w,D
%   tau_app' = Lam (tau - tau_app)

e1 = [1.0; 0.0; 0.0];
e3 = [0.0; 0.0; 1.0];

% -------------------------------------------------------------------------
% Operating point in error coordinates
% -------------------------------------------------------------------------
Q    = Rref' * R0;
eR0  = famp_so3_log(Q);
eom0 = om0 - omref;
ev0  = v0 - vref;

[Jr, Jri] = so3_right_jacobian(eR0);

% -------------------------------------------------------------------------
% Attitude kinematics row
% -------------------------------------------------------------------------
fR0 = f_R(eR0, eom0, omref);
Hw  = Jri;
HR  = zeros(3, 3);
hfd = 1.0e-6;
for i = 1:3
    d = zeros(3, 1);
    d(i) = hfd;
    HR(:, i) = (f_R(eR0 + d, eom0, omref) - f_R(eR0 - d, eom0, omref)) / (2.0*hfd);
end
dR = fR0 - HR*eR0 - Hw*eom0;

% -------------------------------------------------------------------------
% Velocity row: thrust direction/magnitude and forward drag
% -------------------------------------------------------------------------
GR = -T0 * R0 * hat3(e3) * Jr;                         % d(T R e3)/d e_R
dv = T0*(R0*e3) - g*e3 - aref - GR*eR0;

b1   = R0 * e1;                                        % body-forward axis (world)
vxB  = b1' * v0;                                       % forward airspeed (no wind)
aD0  = -Kdrag * vxB * b1;
ADv  = -Kdrag * (b1 * b1');
PR   = -R0 * hat3(e1) * Jr;                            % d b1 / d e_R
ADR  = -Kdrag * (vxB*eye(3) + b1*v0') * PR;
aDaff = aD0 - ADR*eR0 - ADv*ev0;

% -------------------------------------------------------------------------
% Angular-velocity row: gyroscopic term, damping, lagged torque
% -------------------------------------------------------------------------
Jm  = diag(J);
Ji  = diag(1.0 ./ max(J, 1.0e-12));
Jom = J .* om0;
Aw  = Ji * (hat3(Jom) - hat3(om0)*Jm);
AwD = Aw - Ji*diag(Dw);
dw  = -Ji*cross3(om0, Jom) - alpref - Aw*eom0;
dwD = dw - Ji*(Dw .* omref);

% -------------------------------------------------------------------------
% Assemble
% -------------------------------------------------------------------------
Ac = zeros(15, 15);
Bc = zeros(15, 4);
cc = zeros(15, 1);

Ac(1:3, 7:9)     = eye(3);
Ac(4:6, 4:6)     = HR;
Ac(4:6, 10:12)   = Hw;
Ac(7:9, 4:6)     = GR + ADR;
Ac(7:9, 7:9)     = ADv;
Ac(10:12, 10:12) = AwD;
Ac(10:12, 13:15) = Ji * diag(Ktau);
Ac(13:15, 13:15) = -diag(Lam);

Bc(7:9, 1)       = R0 * e3;
Bc(13:15, 2:4)   = diag(Lam);

cc(4:6)   = dR;
cc(7:9)   = dv + aDaff;
cc(10:12) = dwD;
end

% =========================================================================
function f = f_R(eR, eom, omref)
%#codegen
% Exact attitude-error kinematics (boxed equation, .tex Sec. A.1.2):
%   e_R' = Jr(e_R)^-1 [ e_om + w* - exp(-e_R^) w* ]
[~, Jri] = so3_right_jacobian(eR);
Qt = so3_exp(-eR);                    % exp(-e_R^) = Q'
f  = Jri * (eom + omref - Qt*omref);
end

function [Jr, Jri] = so3_right_jacobian(phi)
%#codegen
% Right Jacobian of SO(3) and its inverse (Eade, "Derivative of the
% exponential map", convention Jr(phi) = D exp(-phi)):
%   Jr    = I - (1-cos th)/th^2 phi^ + (th - sin th)/th^3 phi^2
%   Jr^-1 = I + 1/2 phi^ + (1/th^2 - (1+cos th)/(2 th sin th)) phi^2
th  = norm(phi);
ph  = hat3(phi);
ph2 = ph * ph;
if th < 1.0e-5
    % series: (1-cos)/th^2 -> 1/2, (th-sin)/th^3 -> 1/6,
    %         1/th^2 - (1+cos)/(2 th sin) -> 1/12
    a  = 0.5 - th*th/24.0;
    b  = 1.0/6.0 - th*th/120.0;
    c  = 1.0/12.0 + th*th/720.0;
else
    a  = (1.0 - cos(th)) / (th*th);
    b  = (th - sin(th)) / (th*th*th);
    c  = 1.0/(th*th) - (1.0 + cos(th)) / (2.0*th*sin(th));
end
Jr  = eye(3) - a*ph + b*ph2;
Jri = eye(3) + 0.5*ph + c*ph2;
end

function R = so3_exp(phi)
%#codegen
% Rodrigues formula.
th = norm(phi);
ph = hat3(phi);
if th < 1.0e-8
    R = eye(3) + ph + 0.5*(ph*ph);
else
    R = eye(3) + (sin(th)/th)*ph + ((1.0 - cos(th))/(th*th))*(ph*ph);
end
end

function S = hat3(v)
%#codegen
S = [   0.0, -v(3),  v(2); ...
       v(3),   0.0, -v(1); ...
      -v(2),  v(1),   0.0];
end

function c = cross3(a, b)
%#codegen
c = [a(2)*b(3) - a(3)*b(2); ...
     a(3)*b(1) - a(1)*b(3); ...
     a(1)*b(2) - a(2)*b(1)];
end
