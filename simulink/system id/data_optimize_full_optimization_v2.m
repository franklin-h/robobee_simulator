%% data_optimize_full_optimization_v2.m
% Alternate of data_optimize_full_optimization.m adapted to the Drake system_id_sweep
% data set. Differences from the original:
%   1) valid_roll_index / valid_pitch_index / valid_yaw_index are DERIVED from the
%      openloop_data.campaign labels the sweep writes (1=roll, 2=pitch, 3=yaw),
%      instead of hardcoded indices that were tuned for the old hardware ordering.
%   2) The yaw fit is restricted to the yaw (a2) campaign for a clean gamma_3/mu.
%   3) A per-axis diagnostic (corr + RMS of estimated vs measured) is printed so
%      the fit quality per axis is explicit.
%
% CONSISTENCY WITH THE SWEEP MAP (checked against the sweep data):
%   * Roll  : Tr ~ (eta*V_L^2 - V_R^2) tracks drv_roll well  (corr ~ +0.95, right sign).
%   * Yaw   : Ty ~ (A2 + mu)          tracks a2 well          (corr ~ +0.98, right sign).
%   * Thrust: ~amp^2 at zero offset; the model has NO pitch-offset term, but the
%             plant's thrust DROPS as drv_pitch grows -> unmodeled in the pitch band.
%   * Pitch : the plant's pitch torque is a NONLINEAR HUMP in drv_pitch (peaks near
%             ~15-30, then decreases). This linear-in-offset model cannot represent
%             the full curve. We therefore fit pitch only over the ~monotonic region
%             PITCH_FIT_RANGE (a linearization about hover); widen/narrow as needed.
%             Extending the pitch model (e.g. a quadratic offset term) is required to
%             capture the whole range.

clear all;
close all;

%% Get RoboBee physical parameters

% Body Parameters
m_a = 25e-6;
J_phi = 51.1e-12;
T = 2666;
r_cp = 1.42*9.56e-3;
b1 = 2.03e-6;
k_a = 300;
k_t = 28.2e-6;

% Wing Parameters
rho = 1.2041; % kg/m^3 at 20deg C

AR = 3;
R = 12e-3; % m
rhat1 = 0.49;
rhat2 = 0.929*rhat1^0.732;

beta = R^4/AR*rhat2^2;
CL_max = 1.8;
CD_0 = 0.4;
CD_max = 3.4;

% approximate lift/drag coefficients
alpha = pi/4;
C_L = CL_max*sin(2*alpha);
C_D = (CD_max+CD_0)/2-(CD_max-CD_0)/2*cos(2*alpha);

%% Get Data

folder_name = 'Drake Model';
% open_loop_test_file_name = strcat(folder_name, '/Open_loop_test_PBee_20230927.mat')
open_loop_test_file_name = strcat(folder_name, '/system_id_sweep_results_20260724_144836.mat')
loaded = load(open_loop_test_file_name);
if isfield(loaded, 'openloop_data')
    openloop_data = loaded.openloop_data;   % system_id_sweep bundle (struct saved as one variable)
else
    openloop_data = loaded;                 % legacy file: fields saved as top-level variables
end


freq= 155;

experiment_valid_index = 1:length(openloop_data.left_voltage_offset);

% ---- Campaign-derived fit subsets (from the sweep's openloop_data.campaign) ----
% campaign: 1 = roll/thrust, 2 = pitch, 3 = yaw. Each sub-fit uses only the trials
% from its own campaign, so the indices track any sweep automatically.
campaign_lbl = openloop_data.campaign(:).';           % 1 x N
pitch_offset = openloop_data.left_voltage_offset(:).'; % V_off_L per trial (= drv_pitch)

valid_roll_index = find(campaign_lbl == 1);
valid_yaw_index  = find(campaign_lbl == 3);

% Pitch: the plant's pitch torque is a nonlinear hump vs drv_pitch, so fit the
% linear model only over the ~monotonic region about hover (adjust as needed).
PITCH_FIT_RANGE  = [-20 25];                            % [drv_pitch units]
valid_pitch_index = find(campaign_lbl == 2 & ...
    pitch_offset >= PITCH_FIT_RANGE(1) & pitch_offset <= PITCH_FIT_RANGE(2));

if isempty(valid_roll_index) || isempty(valid_pitch_index) || isempty(valid_yaw_index)
    error(['One of the campaign subsets is empty. Check that openloop_data.campaign ', ...
           'labels roll=1, pitch=2, yaw=3 (re-run the sweep if this field is missing).']);
end
fprintf('Fit subsets: roll=%d, pitch=%d (|offset|<=%g), yaw=%d trials\n', ...
    numel(valid_roll_index), numel(valid_pitch_index), PITCH_FIT_RANGE(2), numel(valid_yaw_index));

% load 'open_loop_data_all160';
% V_off_L V_off_R V_p2p_L V_p2p_R A1 A2 freq Ft Tr Tp Ty
V_off_L = (openloop_data.left_voltage_offset)';
V_off_R = (openloop_data.right_voltage_offset)';
V_p2p_L = (openloop_data.left_voltage_p2p)';
V_p2p_R = (openloop_data.right_voltage_p2p)';
A1 = 1.1;
A2 = (openloop_data.a2_yaw)';

Ft = (openloop_data.y_thrust_average)';
Tr = (openloop_data.y_torque_x_average)';
Tp = -(openloop_data.y_torque_y_average)';
Ty = (openloop_data.y_torque_z_average)';


% Remove data (for testing)
% nrmv = 1:38;
% V_off_L(nrmv) = []; V_off_R(nrmv) = []; V_p2p_L(nrmv) = []; V_p2p_R(nrmv) = [];
% A1(nrmv) = []; A2(nrmv) = []; freq(nrmv) = [];
% Ft(nrmv) = []; Tr(nrmv) = []; Tp(nrmv) = []; Ty(nrmv) = [];

V_amp_L = V_p2p_L / 2;
V_amp_R = V_p2p_R / 2;

% 05282021 - Frequency same across all tests
omega = 2*pi * freq(1);

N = length(Ft);

%% Optimization

[H_mag,H_phase] = get_transfer_function(m_a,J_phi,T,r_cp,b1,k_a,k_t);
    delta_1 = A1*omega*H_mag(omega);
    delta_2 = 2*omega*H_mag(2*omega);

N_opt = 1e3;
RMS_init = 1e6;

B = delta_1.^2 + delta_2.^2.*A2;

eta_lims = [0 10.0];
eta_test = linspace(eta_lims(1),eta_lims(2),N_opt);
FTTR_RMS_min = RMS_init;
TP_RMS_min = RMS_init;
TY_RMS_min = RMS_init;

% v2: fit thrust + roll on the ROLL campaign only (offset=0, a2=0 so the thrust
% model's assumptions hold). Pitch-campaign trials would bias the thrust fit -
% the plant's thrust droops with pitch offset, which this model does not capture.
ir = valid_roll_index;
Br = B(ir); VLr = V_amp_L(ir); VRr = V_amp_R(ir); Ftr = Ft(ir); Trr = Tr(ir);
nr = numel(ir);
for i = 1:N_opt

    % 1: Thrust
    BU = Br .* (eta_test(i) * VLr.^2 + VRr.^2);
    G1 = BU \ Ftr;
    FT_RMS = sqrt(sum((Ftr-BU*G1).^2)/nr);

    % 2: Roll Torque
    BGU = G1*Br .* (eta_test(i)*VLr.^2 - VRr.^2);
    G2 = BGU \ Trr;
    TR_RMS = sqrt(sum((Trr-BGU*G2).^2)/nr);

    % Choose eta
    if (FT_RMS * TR_RMS) < FTTR_RMS_min
        FTTR_RMS_min = FT_RMS * TR_RMS;
        gamma_1 = G1;
        gamma_2 = G2;
        eta = eta_test(i);
        FT_RMS_final = FT_RMS;
        TR_RMS_final = TR_RMS;
    end
end

gamma_1_multi = gamma_1;
gamma_2_multi = gamma_2;
eta_multi = eta;

%%
% % OLD OPTIMIZATION
% u1 = B.*[V_amp_L.^2, V_amp_R.^2];
% opt_gamma = inv(u1'*u1)*u1'*Ft;
% 
% % gamma_1 = opt_gamma(2);
% % eta = opt_gamma(1)/opt_gamma(2);
% 
% u2 = gamma_1*B.*[eta*V_amp_L.^2, -V_amp_R.^2];
% opt_beta = inv(u2'*u2)*u2'*Tr;


if false   % v2: bypass the fragile symbolic optimization below (kept for reference)
% Weighted sum optimization with the necessary condition
scale = 1e3;
% valid_roll_index = [1:15, 20:21, 23:24];
% valid_roll_index = [1:20];
% valid_roll_index = [1:60];
% valid_roll_index = [1:15];

Ft_new = Ft(valid_roll_index)*scale;
Tr_new = Tr(valid_roll_index)*scale^2;

syms u11 u12 

AA = B(valid_roll_index).*[V_amp_L(valid_roll_index).^2, V_amp_R(valid_roll_index).^2];
BB = B(valid_roll_index).*[V_amp_L(valid_roll_index).^2, -V_amp_R(valid_roll_index).^2];


Du1 = [u11, 0 ; 0, u12];
Du1_simple = [u11; u12];


% ADJUST WEIGHTING PARAMETER
% weighting parameter for 1st stage optimization (Thrust_error^2 + alpha Troll_error^2)
% alpha_opt = 2e-2;
alpha_opt = 0.1;


u2_opt = (Du1*(BB')*BB*Du1)^(-1)*Du1*BB'*Tr_new;
u2_opt_simple = (Du1_simple.'*(BB.')*BB*Du1_simple)^(-1)*(Du1_simple.')*BB.'*Tr_new;

Du2 = [u2_opt(1), 0; 0, u2_opt(2)];
Du2_simple = u2_opt_simple;
optimality_cond = simplify(-(Ft_new'*AA+alpha_opt*Tr_new'*BB*Du2)+[u11, u12]*(AA.'*AA+alpha_opt*Du2'*(BB.')*BB*Du2));
optimality_cond_simple = -(Ft_new'*AA+alpha_opt*Tr_new'*BB*Du2_simple)+[u11, u12]*(AA.'*AA+alpha_opt*Du2_simple'*((BB.')*BB)*Du2_simple);


S_thrust_opt=solve(optimality_cond==0, [u11, u12]);
S_thrust_opt_simple=solve(optimality_cond_simple==0, [u11, u12]);

Du1_opt = [S_thrust_opt.u11, 0 ; 0, S_thrust_opt.u12];
Du1_opt_simple = [S_thrust_opt_simple.u11; S_thrust_opt_simple.u12];

u2_opt_final = double((Du1_opt*(BB.')*BB*Du1_opt)^(-1)*Du1_opt*BB'*Tr_new);
u2_opt_final_simple = double((Du1_opt_simple'*(BB.')*BB*Du1_opt_simple)^(-1)*(Du1_opt_simple'*BB.'*Tr_new));

gamma_1_NLP = double(S_thrust_opt.u12);
eta_NLP = double(S_thrust_opt.u11/S_thrust_opt.u12);

gamma_1_NLP_simple = double(S_thrust_opt_simple.u12);
eta_NLP_simple = double(S_thrust_opt_simple.u11/S_thrust_opt_simple.u12);


% FT_RMS_NLP = sqrt(sum((Ft-AA*[gamma_1_NLP*eta_NLP; gamma_1_NLP]).^2)/N);
% TR_RMS_NLP = sqrt(sum((Tr-BB*[gamma_1_NLP*eta_NLP*u2_opt_final(1); gamma_1_NLP*u2_opt_final(2)]).^2)/N);
% 
% FT_RMS_NLP_simple = sqrt(sum((Ft-AA*[gamma_1_NLP_simple*eta_NLP_simple; gamma_1_NLP_simple]).^2)/N);
% TR_RMS_NLP_simple = sqrt(sum((Tr-BB*[gamma_1_NLP_simple*eta_NLP_simple*u2_opt_final_simple; gamma_1_NLP_simple*u2_opt_final_simple]).^2)/N);


% Multiplicative Error with one gamma_2

gamma_2_1 = gamma_2;
gamma_2_2 = gamma_2;


% Weighted sum parameters with two gamma_2

gamma_1 = gamma_1_NLP/scale;
eta = eta_NLP;
gamma_2_1 = u2_opt_final(1)/scale;
gamma_2_2 = u2_opt_final(2)/scale;

% Weighted sum parameters with one gamma2
gamma_1 = gamma_1_NLP_simple/scale;
eta = eta_NLP_simple;
gamma_2_1 = u2_opt_final_simple/scale;
gamma_2_2 = u2_opt_final_simple/scale;
end   % if false  (v2: end of bypassed symbolic optimization)

% v2 final thrust/roll gains: taken from the robust least-squares eta-sweep on
% the roll campaign (the symbolic necessary-condition solve above is skipped -
% it failed to return an explicit solution on this data set). Single gamma_2.
gamma_1   = gamma_1_multi;
eta       = eta_multi;
gamma_2_1 = gamma_2_multi;
gamma_2_2 = gamma_2_multi;


% eta = 1.0911;
% gamma_1 = 2.7628e-09;
% gamma_2 = 0.0023;

% 3: Pitch Torque
nu_lims = [-20 20];
nu_test = linspace(nu_lims(1),nu_lims(2),N_opt);

d3_lims = [-10 10]*H_mag(0);
% d3_lims = [0 10]*H_mag(0);
d3_test = linspace(d3_lims(1),d3_lims(2),N_opt);


AA = B(valid_pitch_index).*[V_amp_L(valid_pitch_index).^2, V_amp_R(valid_pitch_index).^2];
BB = B(valid_pitch_index).*[V_amp_L(valid_pitch_index).^2, -V_amp_R(valid_pitch_index).^2];


CC = gamma_1*AA*[eta*gamma_2_1, 0 ; 0, 1*gamma_2_2];
CC_total = [(CC.*[V_off_L(valid_pitch_index), V_off_R(valid_pitch_index)])*[1;1], CC*[1;1]];

opt_pitch = (CC_total'*CC_total)^(-1)*(CC_total'*Tp(valid_pitch_index));

delta_3_opt = opt_pitch(1);
nu_opt = opt_pitch(2)/delta_3_opt;

% 
% for j = 1:N_opt
%     for k = 1:N_opt
%         D3 = d3_test(k);
%         NU = nu_test(j);
% %         Tp_temp = gamma_1.*gamma_2.*(delta_1.^2+delta_2.^2.*A2.^2) .* ...
% %                          	D3.*(eta*V_amp_L.^2.*(V_off_L+NU)+V_amp_R.^2.*(V_off_R+NU));
%         Tp_temp = gamma_1.*(delta_1.^2+delta_2.^2.*A2.^2) .* ...
%                          	D3.*(eta*V_amp_L.^2.*gamma_2_1.*(V_off_L+NU)+V_amp_R.^2.*gamma_2_2.*(V_off_R+NU));
% %         Tp_temp = gamma_1.*gamma_2.*(delta_1.^2+delta_2.^2.*A2.^2) .* ...
% %                          	D3.*(eta*V_amp_L.^2+V_amp_R.^2).*(V_off+NU);
%         TP_RMS = sqrt(sum((Tp-Tp_temp).^2)/N);
%         if TP_RMS < TP_RMS_min
%             TP_RMS_min = TP_RMS;
%             delta_3 = D3;
%             nu = NU;
%         end
%     end
% end



% nu = 0;
% delta_3 = 0.0038;

% 4: Yaw Torque


DD = gamma_1.*delta_1.*delta_2.*(eta*V_amp_L.^2*gamma_2_1+V_amp_R.^2*gamma_2_2);

iy = valid_yaw_index;                     % fit gamma_3/mu on the a2 (yaw) campaign only
DD_total = [DD(iy).*A2(iy), DD(iy)];
opt_yaw = (DD_total'*DD_total)^(-1)*(DD_total'*Ty(iy));

gamma_3_opt = opt_yaw(1);
mu_opt = opt_yaw(2)/gamma_3_opt;



BGUY = DD(iy).*A2(iy);
gamma_3 = BGUY \ Ty(iy);

mu_lims = [-0.2 0.2];
mu_test = linspace(mu_lims(1),mu_lims(2),N_opt);

g3a_lims = [-10 10]*8/(3*pi)*C_D./C_L;
g3a_test = linspace(g3a_lims(1),g3a_lims(2),N_opt);

% for j = 1:N_opt
%     for k = 1:N_opt
%         G3 = g3a_test(k)*cos(2*H_phase(omega)-H_phase(2*omega));
%         MU = mu_test(j);
%         Ty_temp = gamma_1.*G3.*delta_1.*delta_2.*(eta*V_amp_L.^2.*gamma_2_1+V_amp_R.^2.*gamma_2_2).*(A2+MU);
%         TY_RMS = sqrt(sum((Ty-Ty_temp).^2)/N);
%         if TY_RMS < TY_RMS_min
%             TY_RMS_min = TY_RMS;
%             gamma_3 = G3;
%             gamma_3a = g3a_test(k);
%             mu = MU;
%         end
%     end
% end

nu=nu_opt;
% nu = 1.01; 
delta_3 = delta_3_opt;
gamma_3= gamma_3_opt;
mu = mu_opt;

params_opt.gamma_1 = gamma_1;
params_opt.gamma_2_1 = gamma_2_1;
params_opt.gamma_2_2 = gamma_2_2;
params_opt.gamma_3 = gamma_3;
params_opt.delta_1 = delta_1(1);
params_opt.delta_2 = delta_2;
params_opt.delta_3 = delta_3;
params_opt.eta = eta;
params_opt.nu = nu;
params_opt.mu = mu;


%% Save file

optimal_fitting_parameter_save_file_name = strcat(folder_name, '/Optimal_fitting_parameter_proper_signs3.mat')
save(optimal_fitting_parameter_save_file_name, 'params_opt')


%% Estimate forces and torques

if all(delta_1==delta_1(1))
    delta_1 = delta_1(1);
end 

% Constant values (single frequency) for thrust, roll, pitch
g1 = gamma_1; g2 = gamma_2; g3 = gamma_3;
d1 = delta_1; d2 = delta_2; d3 = delta_3;



Ft_est = gamma_1*(delta_1.^2+delta_2.^2.*A2.^2).*(eta*V_amp_L.^2+V_amp_R.^2);
Tr_est = gamma_1.*(delta_1.^2+delta_2.^2.*A2.^2).*(eta*V_amp_L.^2.*gamma_2_1-V_amp_R.^2.*gamma_2_2);
Tp_est = gamma_1.*(delta_1.^2+delta_2.^2.*A2.^2).*delta_3.*(eta*V_amp_L.^2.*gamma_2_1.*(V_off_L+nu)+V_amp_R.^2*gamma_2_2.*(V_off_R+nu));
Ty_est = gamma_1.*gamma_3.*delta_1.*delta_2.*(eta*V_amp_L.^2.*gamma_2_1+V_amp_R.^2.*gamma_2_2).*(A2+mu);

%% Per-axis fit diagnostics (estimated vs measured) -----------------------
% Pearson correlation (no toolbox dependency) + RMS, each over its campaign.
pcorr = @(a,b) mean((a-mean(a)).*(b-mean(b))) / (std(a,1)*std(b,1) + eps);
rms_  = @(e) sqrt(mean(e.^2));
ip_all = find(campaign_lbl == 2);
fprintf('\n--- Fit quality (estimated vs measured) ---\n');
fprintf('  Thrust (all trials): corr=%+.3f  RMS=%.3g N\n', ...
    pcorr(Ft, Ft_est), rms_(Ft_est - Ft));
fprintf('  Roll   (roll camp) : corr=%+.3f  RMS=%.3g Nm\n', ...
    pcorr(Tr(valid_roll_index), Tr_est(valid_roll_index)), rms_(Tr_est(valid_roll_index) - Tr(valid_roll_index)));
fprintf('  Pitch  (fit band)  : corr=%+.3f  RMS=%.3g Nm\n', ...
    pcorr(Tp(valid_pitch_index), Tp_est(valid_pitch_index)), rms_(Tp_est(valid_pitch_index) - Tp(valid_pitch_index)));
fprintf('  Pitch  (full camp) : corr=%+.3f  <- low: plant pitch torque is a nonlinear hump vs drv_pitch\n', ...
    pcorr(Tp(ip_all), Tp_est(ip_all)));
fprintf('  Yaw    (yaw camp)  : corr=%+.3f  RMS=%.3g Nm\n', ...
    pcorr(Ty(valid_yaw_index), Ty_est(valid_yaw_index)), rms_(Ty_est(valid_yaw_index) - Ty(valid_yaw_index)));


%% Plot Results
N=length(Ft);

N_to_mN = 1e3;
Nm_to_mNmm = 1e6;   % 1 N*m = 10^6 mN*mm

figure()
subplot(2,4,1)
    hold on;
    plot(experiment_valid_index,N_to_mN*Ft, 'bo-');
    plot(experiment_valid_index,N_to_mN*Ft_est,'ro-');
    title('F_T');
    legend('Measured','Estimated');
    xlabel('Trials'); ylabel('Thrust [mN]');
    ax = gca;
%         ax.XLim = [0 N];
        ax.FontName = 'Times New Roman';
        ax.FontSize = 16;
subplot(2,4,2)
    hold on;
    plot(experiment_valid_index,Nm_to_mNmm*Tr, 'bo-');
    plot(experiment_valid_index,Nm_to_mNmm*Tr_est,'ro-');
    title('\tau_R');
    legend('Measured','Estimated');
    xlabel('Trials'); ylabel('Roll Torque [mNmm]');
    ax = gca;
%         ax.XLim = [0 N];
        ax.FontName = 'Times New Roman';
        ax.FontSize = 16;
subplot(2,4,3)
    hold on;
    plot(experiment_valid_index,Nm_to_mNmm*Tp, 'bo-');
    plot(experiment_valid_index,Nm_to_mNmm*Tp_est,'ro-');
    title('\tau_P');
    legend('Measured','Estimated');
    xlabel('Trials'); ylabel('Pitch Torque [mNmm]');
    ax = gca;
%         ax.XLim = [0 N];
        ax.FontName = 'Times New Roman';
        ax.FontSize = 16;
subplot(2,4,4)
    hold on;
    plot(experiment_valid_index,Nm_to_mNmm*Ty, 'bo-');
    plot(experiment_valid_index,Nm_to_mNmm*Ty_est,'ro-');
    title('\tau_Y');
    legend('Measured','Estimated');
    xlabel('Trials'); ylabel('Yaw Torque [mNmm]');
    ax = gca;
%         ax.XLim = [0 N];
        ax.FontName = 'Times New Roman';
        ax.FontSize = 16;
subplot(2,4,5)
    hold on;
    plot(experiment_valid_index,N_to_mN*(Ft_est - Ft), 'bo-');
    plot(experiment_valid_index,N_to_mN*sqrt(sum((Ft_est-Ft).^2)/N).*ones(N,1),'ro-');
    legend('Error','RMS');
    title('F_T');
    xlabel('Trials'); ylabel('Thrust Error [mN]');
    ax = gca;
%         ax.XLim = [0 N];
        ax.FontName = 'Times New Roman';
        ax.FontSize = 16;
subplot(2,4,6)
    hold on;
    plot(experiment_valid_index,Nm_to_mNmm*(Tr_est - Tr), 'bo-');
    plot(experiment_valid_index,Nm_to_mNmm*sqrt(sum((Tr_est-Tr).^2)/N).*ones(N,1),'ro-');
    legend('Error','RMS');
    title('\tau_R');
    xlabel('Trials'); ylabel('Roll Torque Error [mNmm]');
    ax = gca;
%         ax.XLim = [0 N];
        ax.FontName = 'Times New Roman';
        ax.FontSize = 16;
subplot(2,4,7)
    hold on;
    plot(experiment_valid_index,Nm_to_mNmm*(Tp_est - Tp), 'bo-');
    plot(experiment_valid_index,Nm_to_mNmm*sqrt(sum((Tp_est-Tp).^2)/N).*ones(N,1),'ro-');
    legend('Error','RMS');
    title('\tau_P');
    xlabel('Trials'); ylabel('Pitch Torque Error [mNmm]');
    ax = gca;
%         ax.XLim = [0 N];
        ax.FontName = 'Times New Roman';
        ax.FontSize = 16;
subplot(2,4,8)
    hold on;
    plot(experiment_valid_index,Nm_to_mNmm*(Ty_est - Ty), 'bo-');
    plot(experiment_valid_index,Nm_to_mNmm*sqrt(sum((Ty_est-Ty).^2)/N)*ones(N,1),'ro-');
    legend('Error','RMS');
    title('\tau_Y');
    xlabel('Trials'); ylabel('Yaw Torque Error [mNmm]');
    ax = gca;
%         ax.XLim = [0 N];
        ax.FontName = 'Times New Roman';
        ax.FontSize = 16;
