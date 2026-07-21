% io %% Geometric Adaptive control
%% Geometric Adaptive Control

clear all;
close all;
christian_params = load('RoboBee_optimal_fitting_parameter_155Hz_2022_BBee_v2.mat'); 

%% Timing and sampling rate
dt_s = 2.0e-4;
% sampling_f = 10000;			% controller sampling rate
sampling_f = 1/dt_s;
sampling_time = 1/sampling_f;

%% Network (target communication)
host = "127.0.0.1";
port = 4242;

reset_plant('updated_target_driver_2026_withVariants')
%% Control mode flags
landing_flag=0; % 0 : No landing, 1: landing on

control_flag=2; %2 % 1: Openloop, 2: closed loop.
adaptive_flag=1;					% 0: no-adaptive 1: adaptive
adaptive_lateral_flag=1;	% 0:  -adaptive 1: adaptive
autosim = 1; 
controller="MPC"; % geometric or MPC, for error saturation. 
replay_mode = 1; % 1 for control, 2 for replay. 
replay_log_file = fullfile('Robobee flight logs', ...
    '20221217_PBee_OL_1.mat');
%% Plant selection and start-up delays
% Plant = 1 for vicon, 2 for Drake
Plant = 2;
if Plant == 1
    start_delay = 2.0; % ensure start_delay exists if real experiment
    adaptive_delay = 0.1;
    ramp_delay = 0.1;

else
    start_delay = 0.0;
    adaptive_delay = 0.0;
    ramp_delay = 0.3;
end


start_delay_control = start_delay + ramp_delay;
start_delay_adaptive = start_delay + ramp_delay+adaptive_delay;


%% Driving signal setup and experiment setup
f = 155; %170; %165
f_int = f;					% initial frequency
f_fin = f;					% final frequency
T = 0.3;			% total time in seconds


%% Save settings
save_flag = 1; %1;
% save_file_name = '20210920_13_with_adaptive_trajectory_landing_1s_circle_costant_y_heading_h_gain_';
% save_file_name = '20220316_01_with_adaptive_trajectory_landing_new';
% save_file_name = '20220322_07_with_adaptive_trajectory_landing_15mm_w_ground';
% save_file_name = '20220328_openloop_test_150Hz_01';
% save_file_name = 'Freq_04192022_6';
% save_file_name = '20220427_02_with_adaptive_trajectory_landing';
% save_file_name = '20220506_BBee_OL40';
% save_file_name = 'Amp_05072022_1';
% save_file_name = '20220509_BBee_adaptive_landing_4_circle';
% save_file_name = '20221103_BBee_adaptive_landing_hovering_payload_29_38mg_updated_I_aligned';
% save_file_name = '20221207_DeadBee_falling_133mg_15_1_rigid_wide';30
% save_file_name = 'OL_20221213_60';
% save_file_name = '20221019_DeadBee_falling_121mg_21_rigid';
save_file_name = '20221217_PBee_OL_1';

%% Vehicle parameters
g=9.8; % gravity
m = 1.05e-4; %101e-6%90e-6;%86e-6; %86*1e-6%86*1e-6; % vehicle weight in kg (mg * 1e-6)
%
% Ixx = 1.42*1e-9; % Principal moment of inertia
% Iyy = 1.34*1e-9%1.34*1e-9;
% Izz = 0.45*1e-9;
%
% Ixx = 2.13e-10;
% Iyy = 2.33e-10;
% Izz = 3.27e-11;

% Ixx = 1.42*1e-9;		% Principal moment of inertia
% Iyy = 1.34*1e-9;		%1.34*1e-9;
% Izz = 0.45*1e-9;

Ixx = 2.03e-9;
Iyy = 2.30e-9;
Izz = 0.31e-9;
% Ixx = 3.8e-4;
% Iyy = 6.1e-4;
% Izz = 2.5e-4;

% Payload;
% payload = 23e-6;
payload = 0; 
l_payload = 0e-3;
Ixx_payload = payload*l_payload^2;
Iyy_payload = payload*l_payload^2;

Ixx = Ixx + Ixx_payload;
Iyy = Iyy + Iyy_payload;

m = m+payload;



I_moment_vec = [Ixx, Iyy, Izz];

I_moment =eye(3);
I_moment(1,1) = Ixx;
I_moment(2,2) = Iyy;
I_moment(3,3) = Izz;
e1=[1;0;0];
e2=[0;1;0];
e3=[0;0;1];

% Vicon compensation to the ground truth with calibration bar

% roll_offset_angle = deg2rad(5);
roll_offset_angle = 0; % New Vicon calibration

%% Voltage to Force Mapping
% params = load('RoboBee_optimal_fitting_parameter_150Hz.mat');
% params = load('RoboBee_optimal_fitting_parameter_150Hz_2022.mat');
% params = load('RoboBee_optimal_fitting_parameter_150Hz_2022_Apr.mat');

% BBee System ID without the leg
params = load('system id/Drake Model/Optimal_fitting_parameter_proper_signs2.mat');
% BBee System ID with the rigid leg
% params = load('RoboBee_optimal_fitting_parameter_155Hz_2022_BBee_rigid_leg_v1.mat');


% best so far, 180 Hz, gamma1 = 2.0e-9, gamma_2 = 0.008
params_opt = params.params_opt;
% [params_opt.gamma_1, params_opt.gamma_2] = deal(2.0e-9,0.008); % Good for 180 Hz 
% [params_opt.gamma_1, params_opt.gamma_2] = deal(4.0e-9,0.0119);
% params_opt.gamma_1 = 5.1653e-09;
% params_opt.gamma_1 = 2.0e-9; % good for f = 180 Hz flapping
% params_opt.gamma_1 = 2.1e-09; % rolls left
% params_opt.gamma_1 = 0.15e-09; 
% params_opt.gamma_2_1 = 0.008; % whoa, its really not close to r_cp!! 
% params_opt.gamma_2 = params_opt.gamma_2_1;
% params_opt.delta_1 = 4.6154;%4.6154 default
% params_opt.delta_2 = %1.8648 default 
% params_opt.delta_3 = -0.0038%-cont0.0038 default 
% params_opt.nu = -6.95; %manually found to be best 
% params_opt.eta = 1.0043; % manually foundto be best 
% params_opt.mu = 0.0; % nominal 
params_opt.gamma_2 = params_opt.gamma_2_1; 
% params_opt.delta_3 = -0.0038;
% params_opt.delta_3 = -0.0044; 
params_vec = [params_opt.delta_1, params_opt.delta_2, params_opt.delta_3, params_opt.gamma_1, params_opt.gamma_2, params_opt.gamma_3, params_opt.eta, params_opt.nu, params_opt.mu]';
% Becky 05072022
% params_vec = [4.615418611008412, 1.864800777306221, -0.003836409949443, 0.000000002617698, 0.002712888814185, 0.470047661467390, 1.252086811352254, -12.320534223706176, -0.106510851419032];

Thurst_limit = 1.2*1e-3;        % 1.2   mN
Torque_roll_limit = 0.35*1e-4;   % 0.2   mNmm;
Torque_pitch_limit = 0.2*1e-6;  % 0.2   mNmm;
Torque_yaw_limit = 0.2*1e-7;%0.2*1e-7;    % 0.02  mNmm;

% 2nd harmonic Coefficient bound for yaw
a2_ub = 0.25;
a2_lb = -0.25;


F_tau_limit = [Thurst_limit, Torque_roll_limit, Torque_pitch_limit, Torque_yaw_limit];

[u1_limit, u2_limit, u3_limit, u4_limit] = inverse_mapping_Force_to_voltage(F_tau_limit(1),F_tau_limit(2), F_tau_limit(3), F_tau_limit(4),params_opt);

V_L_p2p_limit = sqrt(u1_limit)*2;
V_R_p2p_limit = sqrt(u2_limit)*2;
V_offset_limit = u4_limit;

drv_amp_limit = abs((V_L_p2p_limit-V_R_p2p_limit))/2+min(V_L_p2p_limit,V_R_p2p_limit); % p2p
drv_roll_limit = (V_L_p2p_limit-V_R_p2p_limit)/4; % Amplitude
drv_pitch_left_limit = V_offset_limit;
drv_pitch_right_limit = V_offset_limit;

closedloop_max_drv_bias = max(V_L_p2p_limit,V_R_p2p_limit) + abs(V_offset_limit);

%% Open loop control set up (Wing Trajectory control)
% v = 140;
% v_roll_offset = 0; % Voltages
% v_pitch_offset = 0; % Voltages
Phi_p2p_nominal_openloop = 60
Phi_roll_offset_openloop = 8% degrees  % positive : high amp for right wing  (negative torque)
Phi_pitch_offset_openloop = -1% 0.5% degrees     % positive : (negative pitch torque)

Phi_p2p_nominal_left =Phi_p2p_nominal_openloop-Phi_roll_offset_openloop/2;
Phi_p2p_nominal_right =Phi_p2p_nominal_openloop+Phi_roll_offset_openloop/2;


% Identified Linear mapping (wing trajecotry to Voltage)
wing_to_voltage = load('Bee_Trajectory_Input_Linear_fitting.mat'); % Identified at 160Hz


% Correction based on the lift coefficient mismatch
% don't forget to install curve fitting tool box if you get loading
% variable error
C_R_over_C_L = 1.0;
wing_to_voltage.fit_R_total.p1 = C_R_over_C_L*wing_to_voltage.fit_R_total.p1;
wing_to_voltage.fit_R_total.p2 = C_R_over_C_L*wing_to_voltage.fit_R_total.p2;
wing_to_voltage.fit_pitch_R_total.p1 = C_R_over_C_L*wing_to_voltage.fit_pitch_R_total.p1;
wing_to_voltage.fit_pitch_R_total.p2 = C_R_over_C_L*wing_to_voltage.fit_pitch_R_total.p2;


left_mapping(1) = 1/wing_to_voltage.fit_L_total.p1;
left_mapping(2) = -wing_to_voltage.fit_L_total.p2/wing_to_voltage.fit_L_total.p1;
right_mapping(1) = 1/wing_to_voltage.fit_R_total.p1;
right_mapping(2) = -wing_to_voltage.fit_R_total.p2/wing_to_voltage.fit_R_total.p1;

left_mapping(3) = 1/wing_to_voltage.fit_pitch_L_total.p1;
left_mapping(4) = -wing_to_voltage.fit_pitch_L_total.p2/wing_to_voltage.fit_pitch_L_total.p1;
right_mapping(3) = 1/wing_to_voltage.fit_pitch_R_total.p1;
right_mapping(4) = -wing_to_voltage.fit_pitch_R_total.p2/wing_to_voltage.fit_pitch_R_total.p1;

% Open_loop controller setup

[drv_amp, drv_roll, drv_pitch_left, drv_pitch_right] = Wing_traj_to_Voltage(Phi_p2p_nominal_left,Phi_p2p_nominal_right,Phi_pitch_offset_openloop,left_mapping,right_mapping);
drv_pch = drv_pitch_left;


% CHRISTIAN OPENLOOP Control
% drv_amp = 200;
% drv_roll = 0;
% drv_pitch_left = 0;
% drv_pitch_right = 0;
% % max_drv_bias =300;
% drv_pch = drv_pitch_left;
%
% a2_openloop = -0; %0.2;2

% Franklin Open Loop
drv_amp = 180;
% drv_roll = -1e-3;
drv_roll = 20; 
% drv_pitch_left = 6.95;
% drv_pitch_right = 6.95;
drv_pitch_left = 6.95; 
drv_pitch_right = 6.95; 
a2_openloop = 0;


%% Feedback Control paramter (Force Control)
scale = 1e3;

% Default position (with the kevlar string), in meters
% default_x = 0.0594;%0.050;
% default_y = 0.0235;%0.137;
% default_z = 0.1278;
% default_z = 0.0855;%0.1535;

default_x = 0;
default_y = 0;
default_z = 0;
jump_height = 0.40;
% jump_height = 0.02;
soft_landing_height = 0.012; % 3mm

% Task transition and Landing control
transition_rate = 3 %8; % 5 : 1 second from 0.1 to 0.9
transition_time = 1.5;
prelanding_time =1.5;
landing_time = 3; %1.5;
desired_x_landing = 0.0806;
desired_y_landing = 0.1058;
desired_z_landing = 0.0456;

landing_z_decrease = -(jump_height-soft_landing_height);%-(default_z+jump_height-desired_z_landing-soft_landing_height);%-0.03;
landing_y_decrease = 0%-(default_y-desired_y_landing); %0; %-0.02;
landing_x_decrease = 0%-(default_x-desired_x_landing); %0; %-0.02;

rising_time = start_delay_control+transition_time;
falling_time =rising_time + T;

landing_rising_time = falling_time+prelanding_time;

total_T = transition_time+T+prelanding_time+landing_time;

% Task parameter

task_flag = 0; % Task 0 : Hovering, Task 1 : Circle, Task 2 : Ellipse
							 % Task 3 : Vertical up and down
               % Task 4 : Y-Z circle, Task 5 : Lissajous curve,
               % Task 6 :Ellipse with Z
							 % Task 7 : Twp way points test (Speed test)
							 % Task 8 : Landing on different set point

radius_ref = 0.05; % 4 cm
period_ref = 3.0; % Lissajous 1.5; % Circle : 3seconds
frequency_ref = 2*pi/period_ref;
radius_ratio = 0.5;
landing_parameter = [start_delay+T,transition_rate, rising_time, falling_time, landing_rising_time, landing_z_decrease, landing_y_decrease, landing_x_decrease];
lissajous_ratio = 0.5;
lissajous_delta = pi/2;
lissajous_amplitude_ratio = 2/3;

z_radius_ratio = 0.15;
task_parameter = [radius_ref, frequency_ref, radius_ratio, lissajous_ratio, lissajous_delta, lissajous_amplitude_ratio,z_radius_ratio];

% position control
z_desired = default_z+jump_height; %0.135; %m
vz_desired = 0; % m/s
% r_desired = [default_x-landing_x_decrease, default_y-landing_y_decrease,z_desired];%[-0.023, -0.035,z_desired]; %[-0.029, -0.032, z_desired];
r_desired = [default_x, default_y,z_desired];%[-0.023, -0.035,z_desired]; %[-0.029, -0.032, z_desired];
v_desired = [0, 0, vz_desired];
xd_ddot = [0, 0, 0];

% heading control
b_1_d_desired = [1,0,0];

% %Lateral gain for geometric
% k_x = 1.0/scale; %0.5
% k_v = 0.5/scale; %0.05
% 
% %Attitude gain
% k_R = 0.5/(scale^2);%0.5/(scale^2);		yaw pitch
% k_Rx = 13/(scale^2);%0.6/(scale^2);		roll
% k_Omega =  10.0/(scale^2);%0.25/(scale^2);
% 
% %Altitude gain
% k_z = 1.2/scale;%0.2/scale;
% k_vz = 0.25/scale;%0.25/scale;


%Lateral gain for MPC
% k_x = 5.0/scale; %0.5
% k_v = 0.5/scale; %0.05
% 
% %Attitude gain
% k_R = 0.5/(scale^2);%0.5/(scale^2);		yaw pitch
% k_Rx = 20/(scale^2);%0.6/(scale^2);		roll
% k_Omega =  7.0/(scale^2);%0.25/(scale^2);
% 
% %Altitude gain
% k_z = 20.0/scale;%0.2/scale;
% k_vz = 0.25/scale;%0.25/scale;

% for somewhat stable flight, set 
% max pitch to 1e-7 or so. 
% default x, y, z to 0, 0, 0.5 or so. 
% control_gain = [2500, 100, 100, 400, 100, 0.04, 0.04, 0.04, 2000,10]. 

%Try params for higher pitch bandwidth. 
k_x = 600; 
k_v = 100; 
k_R      = 200;   % pitch attitude 100
k_Rx     = 800;   % roll attitude 400 
k_R_yaw  = 100;   % yaw attitude 100 
k_Omega       = 0.015;   % roll  rate  (index 4, unchanged)
k_Omega_pitch = 0.04;   % pitch rate  (laggy weak axis -> more damping)
k_Omega_yaw   = 0.04;   % yaw   rate
k_z = 4000; 
k_vz = 10; 

% Layout consumed by mpc_fcn / Desired_Attitude (indices 1-7 preserved; 8-10 appended):
% [k_x k_v k_R k_Omega k_z k_vz k_Rx | k_Omega_pitch k_Omega_yaw k_R_yaw]
control_gain = [k_x, k_v, k_R, k_Omega, k_z, k_vz, k_Rx, ...
                k_Omega_pitch, k_Omega_yaw, k_R_yaw];

% Altitude feed back saturation
upp_bound_z = 0.1; % m
low_bound_z =-0.1; % m
upp_bound_vz = 0.5; % m/s
low_bound_vz =-0.5; % m/s

upp_bound_x = 0.6; % m
low_bound_x =-0.6; % m
upp_bound_vx = 0.5; % m/s
low_bound_vx =-0.5; % m/s

upp_bound_y = 0.6; % m
low_bound_y =-0.6; % m
upp_bound_vy = 0.5; % m/s
low_bound_vy =-0.5; % m/s


upp_bound =10; %4 % rad/s this helped a lot!! 
low_bound =-10; %-4 % rad/s
upp_bound_eR =1.2; %0.3   % attitude error
low_bound_eR =-1.2; %-0.3 % attitude error

%% Adaptive Control gain
gamma_adaptive = 5e-8*upp_bound;
adaptive_roll_limit = 0.3/(scale^2);%0.17/(scale^2);
adaptive_pitch_limit = 0.1/(scale^2);%0.09/(scale^2);
adaptive_yaw_limit = 0.035/(scale^2); %0.017/(scale^2);

adaptive_roll_limit_low = -adaptive_roll_limit;
adaptive_pitch_limit_low = -adaptive_pitch_limit;
adaptive_yaw_limit_low = -adaptive_yaw_limit;


c_2_upp_bound = sqrt(k_R*upp_bound_eR*I_moment(3,3))/I_moment(1,1);
c_2_adaptive=c_2_upp_bound*upp_bound_eR/upp_bound;


adaptive_roll_init = 0; %0.1/(scale^2);%-0.1/(scale^2); %0;
adaptive_pitch_init =  0;%-0.02/(scale^2);%-0.05/(scale^2);
adaptive_yaw_init = 0;%-0.029/(scale^2);

% Lateral addaptive control
gamma_lateral_adaptive = 9e-4*upp_bound_vx;
adaptive_x_limit = 0.15/(scale);
adaptive_y_limit = 0.15/(scale);
adaptive_z_limit = 0.20/(scale);% 0.18/(scale);%0.015/(scale);

adaptive_x_limit_low = -adaptive_x_limit;
adaptive_y_limit_low = -adaptive_y_limit;
adaptive_z_limit_low = -adaptive_z_limit/1;


alpha_xx = 0.3;

c1_upp_bound = min(4*(k_x/upp_bound_x*k_v/upp_bound_vx*(1-alpha_xx)^2)/((k_v/upp_bound_vx)^2*(1+alpha_xx)^2+4*m*k_x/upp_bound_x*(1-alpha_xx)), sqrt(k_x/upp_bound_x/m));
c_1_adaptive=c1_upp_bound*upp_bound_x/upp_bound_vx;

c_1_adaptive = c_1_adaptive*10;

adaptive_x_init = 0;%-0.0/(scale); %-0.045/(scale); %-0.12/(scale);
adaptive_y_init = 0;%-0.07/(scale); %-0.1/(scale); %0;
adaptive_z_init = 0/scale; %-0.015/(scale); %0.016/(scale);

adaptive_gain = [gamma_adaptive, adaptive_roll_limit, adaptive_pitch_limit, adaptive_yaw_limit, c_2_adaptive, gamma_lateral_adaptive, adaptive_x_limit, adaptive_y_limit, adaptive_z_limit, c_1_adaptive];


%% low pass filter
% butter() normalizes the cutoff to Nyquist (sampling_f/2), so a cutoff of
% lp_cutoff_hz must be divided by (sampling_f/2), NOT by sampling_f (which
% would give half the intended cutoff).
lp_cutoff_hz = 100;   % desired low-pass cutoff [Hz]
[lp_num, lp_den]  =butter(5, lp_cutoff_hz/(sampling_f/2));
[vlp_num, vlp_den]=butter(5, lp_cutoff_hz/(sampling_f/2));


%% attitude controller setup (Filtering to get angular velocity wrt body frame)

att_s = 500;
k_da = exp(-att_s*sampling_time);
att_s2 = 60;
k_da2 = exp(-att_s2*sampling_time);
att_k = 72e-9;  %72
att_Lambda = 12.5;		% or 12.5?
simple_s = 500;
simple_lp = tf([1],[1 simple_s]);
simple_lp = c2d(simple_lp, sampling_time);

%% Angular velocity estimation

fs_vicon = 500;
f_res = f;
vicon_sample_cycle = ceil(sampling_f/fs_vicon);
sample_cycle = ceil(sampling_f/f_res);
num_cycle =3; % Prepare for the delay
cnt=1;
for i=0:vicon_sample_cycle:num_cycle*sample_cycle
   delay(cnt) = vicon_sample_cycle*(cnt-1);
   cnt=cnt+1;
end
delay = delay-vicon_sample_cycle/2;

%% Drive bias (PZT)
max_drv_bias = 200;
drv_bias = max_drv_bias;
phase = 0; %pi; %degrees (added on left wing)

%% Run time setup (depends on start_delay, start_delay_control, total_T, T)
if control_flag==2  % CLosedloop control

    closedloop_flag = 1; % For simulink
	% Setup the Bias for PZT
    % drv_bias = max(closedloop_max_drv_bias,max_drv_bias);
	drv_bias = max_drv_bias;
    phase = 0; %pi; %degrees (added on left wing)
    running_time = start_delay+total_T;
    running_time_control = start_delay_control+total_T+0.0;   %3.0+T+0.5 %11
    % running_time = start_delay + T;
    % running_time_control = start_delay_control + T;
else % else, open loop
    closedloop_flag = 0;
    drv_bias = max_drv_bias;
    running_time = start_delay+T;
	running_time_control = start_delay_control+T+0.0;   %3.0+T+0.5 %11
end

%% Model configuration (needs dt_s and running_time defined above)
mdl = 'updated_target_driver_2026_withVariants';
set_param(mdl, 'SolverType', 'Fixed-step');
set_param(mdl, 'Solver', 'FixedStepDiscrete');
set_param(mdl, 'FixedStep', 'dt_s');
set_param(mdl, 'StopTime', 'running_time');

if autosim == 1
    sim('updated_target_driver_2026_withVariants'); 
end 

function reset_plant(mdl)
%RESET_PLANT Drop the persistent Drake TCP socket so the next sim() gets a
% fresh simulation (pose reset, wrench CSV truncated). Unloading the model
% sim-target MEX destroys the native client's static socket; the server then
% sees the disconnect, flushes/closes the current wrench CSV, and rebuilds a
% clean simulation on the next connect.
    sfun = [mdl '_sfun'];
    clear(sfun);
    clear mex; %#ok<CLMEX>  % ensure the sim-target library is fully unloaded
    pause(0.2);            % give the server time to notice the disconnect + flush
end
