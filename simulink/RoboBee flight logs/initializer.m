%% Geometric Adaptive Control

clc;
clear all;
close all;

sampling_f = 10000;							% controller sampling rate
sampling_time = 1/sampling_f;
start_delay = 2.0;%+2.0					% start controller after xx s
ramp_delay = 0.1;								% start voltage ramp after xx s

adaptive_delay = 0.1;
start_delay_control = start_delay + ramp_delay;
start_delay_adaptive = start_delay + ramp_delay + adaptive_delay;


%% Driving signal setup and experiment setup

f = 155;			%170; %155; %165;
f_int = f;		% initial frequency
f_fin = f;		% final frequency
T = 0.4; %0.4; %3.5;			% total time in seconds: 0.4s for open-loop

% ADJUST CONTROLLER FLAGS FOR SIMULINK
landing_flag = 0;						% 0 : No landing, 1: landing on
control_flag = 1;					  % 1: open loop, 2: closed loop
adaptive_flag = 0;					% Adaptive Attitude Control: 0: no-adaptive, 1: adaptive
adaptive_lateral_flag = 0;	% 0: no-adaptive, 1: adaptive

% % 0.4s for open-loop
% % 3.5s for open-loop
% if control_flag == 1
% 	T = 0.4;
% else
% 	T = 3.5;
% end

max_drv_bias = 300;					% Peak-to-peak voltage

save_flag = 1;

% RENAME SAVE FILE
% save_file_name = '20221218_BBee_rigid_leg_133mg_20_jump_12_hover';
% save_file_name = '20220111_BBee_rigid_leg_133mg_20_jump_12_hover';
% save_file_name = '20230111_BBee_rigid_leg_133mg_narrow_stance_20mm_jump_12mm_hover_300V_P2P_155Hz_control_flight_reconnected_signal';
% save_file_name = '20230112_BBee_rigid_leg_133mg_narrow_stance_20mm_jump_12mm_hover_300V_P2P_155Hz_control_flight_with_patrick';
% save_file_name = '20230301_BBee_rigid_leg_133mg_narrow_stance_20mm_jump_12mm_hover_300V_P2P_155Hz_control_flight_demo_test';
% save_file_name = '20230425_rigid_leg_133mg_narrow_stance_30mm_jump_12mm_hover_300V_P2P_155Hz_control_flight_demo_test';

% save_file_name = '20231208_BBee_compliant_leg_133mg_20mm_jump_12mm_hover_300V_P2P_155Hz_control_flight';
% save_file_name = '20231214_BBee_compliant_leg_133mg_20mm_jump_12mm_hover_300V_P2P_155Hz_control_flight_adaptive_gain_0';
% save_file_name = '20231214_BBee_compliant_leg_120mg_20mm_jump_12mm_hover_300V_P2P_155Hz_control_flight_adaptive_gain_0';
% save_file_name = '20231215_BBee_compliant_leg_120mg_20mm_jump_10mm_hover_300V_P2P_155Hz_control_flight_adaptive_gains_0';
% save_file_name = '20231215_BBee_compliant_leg_118mg_20mm_jump_5mm_hover_300V_P2P_155Hz_control_flight_adaptive_gains_0';
% save_file_name = '20231215_BBee_compliant_leg_117mg_20mm_jump_5mm_hover_300V_P2P_155Hz_control_flight_adaptive_gains_0';
% save_file_name = '20231215_BBee_compliant_leg_119mg_20mm_jump_10mm_hover_300V_P2P_155Hz_CL';
% save_file_name = '20231215_BBee_compliant_leg_landing_119mg_20mm_jump_10mm_hover_300V_P2P_155Hz';
% save_file_name = '20231219_BBee_compliant_leg_119mg_OL_300V_P2P_155Hz';
% save_file_name = '20231219_BBee_compliant_leg_119mg_20mm_jump_10mm_hover_300V_P2P_155Hz';
% save_file_name = '20231220_BBee_compliant_leg_119mg_20mm_jump_10mm_hover_300V_P2P_155Hz';


% Static-Flight Tests
% save_file_name = 'system_ID/Patrick_Bee_Open_Loop_Data_20231025/Static_PBee_20231025';


% Open-Loop Tests
% save_file_name = 'system_ID/Patrick_Bee_systemID_20231103/Open_Loop_Data/OL_PBee_20231103_1';


% Closed-Loop Tests
% save_file_name = 'system_ID/Patrick_Bee_Open_Loop_Data_20230927/CL_PBee_20231005_1';


% Landing Rate Tests
% save_file_name = '20231221_BBee_compliant_leg_119mg_30mm_jump_trans_rate_3_300V_P2P_155Hz';
% save_file_name = '20231221_BBee_compliant_leg_119mg_30mm_jump_trans_rate_1_300V_P2P_155Hz';
% save_file_name = '20231221_BBee_compliant_leg_119mg_30mm_jump_trans_rate_5_300V_P2P_155Hz';
% save_file_name = '20231221_BBee_compliant_leg_119mg_30mm_jump_trans_rate_7_300Vpp_155Hz';
% save_file_name = '20231221_BBee_compliant_leg_119mg_30mm_jump_trans_rate_9_300Vpp_155Hz';

% save_file_name = '20231224_BBee_compliant_leg_119mg_30mm_jump_landing_rate_1_300Vpp_155Hz';
% save_file_name = '20231224_BBee_compliant_leg_119mg_30mm_jump_landing_rate_3_300Vpp_155Hz';
% save_file_name = '20231224_BBee_compliant_leg_119mg_30mm_jump_landing_rate_5_300Vpp_155Hz';
% save_file_name = '20231224_BBee_compliant_leg_119mg_30mm_jump_landing_rate_5_300Vpp_155Hz_2';
% save_file_name = '20231224_BBee_compliant_leg_119mg_30mm_jump_landing_rate_7_300Vpp_155Hz';
% save_file_name = '20231224_BBee_compliant_leg_119mg_30mm_jump_landing_rate_9_300Vpp_155Hz';

% save_file_name = '20240318_BBee_compliant_leg_119mg_jump_40mm_soft_landing_10mm_alpha_1_300Vpp_155Hz';
	
% save_file_name = '20240326_BBee_compliant_leg_119mg_jump_40mm_soft_landing_10mm_alpha_3_300Vpp_155Hz';
% save_file_name = '20240326_BBee_compliant_leg_119mg_static_300Vpp_155Hz';
% save_file_name = '20240326_BBee_compliant_leg_119mg_jump_40mm_soft_landing_10mm_alpha_3_300Vpp_155Hz';
save_file_name = '20240327_BBee_compliant_leg_119mg_static_300Vpp_155Hz';


% Leaf-Hopping Demo
% save_file_name = '20231222_BBee_compliant_leg_119mg_300Vpp_155Hz_leaf_hop';
% save_file_name = '20231224_BBee_compliant_leg_119mg_300Vpp_155Hz_leaf_hop_untethered';
% save_file_name = '20231224_BBee_compliant_leg_119mg_300Vpp_155Hz_leaf_hop_untethered_2';



%% Vehicle parameters

% ADJUST VEHICLE WEIGHT
% WEIGH VEHICLE BEFORE TESTING
m = 119e-6; %120e-6; %133e-6;		%101e-6; %90e-6 ;%86e-6; %86*1e-6; %86*1e-6; % vehicle weight
g = 9.8;			% gravity

Ixx = 1.42*1e-9;		% Principal moment of inertia
Iyy = 1.34*1e-9;		%1.34*1e-9;
Izz = 0.45*1e-9;

% Payload
payload = 0; %23e-6;
l_payload = 0e-3;
Ixx_payload = payload*l_payload^2;
Iyy_payload = payload*l_payload^2;

Ixx = Ixx + Ixx_payload;
Iyy = Iyy + Iyy_payload;
m = m  +payload;

I_moment_vec = [Ixx, Iyy, Izz];

I_moment =eye(3);
I_moment(1,1) = Ixx;
I_moment(2,2) = Iyy;
I_moment(3,3) = Izz;
e1 = [1;0;0];
e2 = [0;1;0];
e3 = [0;0;1];

% Vicon compensation to the ground truth with calibration bar

% roll_offset_angle = deg2rad(5);
roll_offset_angle = 0; % New Vicon calibration

%% Voltage to Force Mapping

% BBee System ID without the leg
% params = load('RoboBee_optimal_fitting_parameter_155Hz_2022_BBee_v2.mat');

% BBee System ID with the rigid leg
params = load('RoboBee_optimal_fitting_parameter_155Hz_2022_BBee_v2.mat');
% params = load('Patrick Bee Open Loop Data/Optimal_fitting_parameter_155Hz_PBee_2023.mat');
% params = load('system_ID/Patrick_Bee_Open_Loop_Data_20230927/Optimal_fitting_parameter_PBee_20231005.mat');

params_opt = params.params_opt;
params_opt.gamma_2 = params_opt.gamma_2_1;

params_vec = [params_opt.delta_1, params_opt.delta_2, params_opt.delta_3, params_opt.gamma_1, params_opt.gamma_2, params_opt.gamma_3, params_opt.eta, params_opt.nu, params_opt.mu]';
% Becky 05072022
% params_vec = [4.615418611008412, 1.864800777306221, -0.003836409949443, 0.000000002617698, 0.002712888814185, 0.470047661467390, 1.252086811352254, -12.320534223706176, -0.106510851419032];

Thurst_limit = 1.2*1e-3;        % 1.2   mN 
Torque_roll_limit = -0.35*1e-6;   % 0.2   mNmm;
Torque_pitch_limit = 0.2*1e-6;  % 0.2   mNmm;
Torque_yaw_limit = 0.2*1e-7;%0.2*1e-7;    % 0.  02  mNmm;

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


% CHANGE THESE FOR OPEN-LOOP SYS-ID
max_drv_bias = 300;
drv_amp = 160; %180; %190; %180; %160;
drv_roll = 0;
drv_pitch_left = 0;
drv_pitch_right = 0;
drv_pch = drv_pitch_left;
a2_openloop = 0;


%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Feedback Control paramter (Force Control) %%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
scale = 1e3;

% UPDATE THIS from Vicon plots
% Default position (with the kevlar string)
default_x = 0.0192; %0.0563; %0.0586; %0.050;  % from Vicon plot
default_y = 0.0265; %0.0199; %0.0185; %0.137;  % from Vicon plot
default_z = 0.0091; %0.0868; %0.0838; %0.1535;  % from Vicon plot
jump_height = 0.040; %0.040; %0.02  % in meter
soft_landing_height = 0.010; %0; %0.010; %0.005; %0.010;  % 12mm in meter
  
% Task transition and Landing control
% transition_rate = 1; %3; %9; %7; %5; %1; %3; %%8; % 5 : 1 second from 0.1 to 0.9
% transition_time = 1;
% prelanding_time = 1;
% landing_time = 1; %3; %1.5;
transition_rate = 3; %[3 2 4 1 5]; %3; %8; % 5 : 1 second from 0.1 to 0.9 (alpha parameter)
transition_time = 1.5;
prelanding_time =1.5;
landing_time = 3; %1.5;

% desired_x_landing = 0; %0.0806;
% desired_y_landing = 0; %0.1058;
% desired_z_landing = 0; %0.0456;
landing_x_decrease = 0; %-(default_x-desired_x_landing); %0; %-0.02; 
landing_y_decrease = 0; %-(default_y-desired_y_landing); %0; %-0.02;
landing_z_decrease = -(jump_height-soft_landing_height); %-(default_z+jump_height-desired_z_landing-soft_landing_height) ;%-0.03;

% Leaf positions
% Leaf 1
% (-0.0077, -0.0307, 0.1818)
%
% Leaf 2
% (0.1518, 0.0468, 0.1958)
%
% Leaf 3
% (0.0080, 0.1545, 0.1734)
%
% desired_x_landing = default_x; %0.1031; %default_x; %0.1377; %0.0509;
% desired_y_landing = default_y; %0.1758; %default_y; %0.0271; %0.0268;
% desired_z_landing = default_z; %0.1921; %default_z; %0.1997; %0.1074;
% 
% landing_z_decrease = -(jump_height-soft_landing_height+default_z-desired_z_landing);%-(default_z+jump_height-desired_z_landing-soft_landing_height);%-0.03;
% landing_y_decrease = -(default_y-desired_y_landing); %0; %-0.02;
% landing_x_decrease = -(default_x-desired_x_landing); %0; %-0.02;


rising_time = start_delay_control+transition_time;
falling_time = rising_time + T;

landing_rising_time = falling_time+prelanding_time;

total_T = transition_time+T+prelanding_time+landing_time;

% Task parameter

task_flag = 0; % Task 0 : Hovering
							 % Task 1 : Circle
							 % Task 2 : Ellipse 
							 % Task 3 : Vertical up and down
               % Task 4 : Y-Z circle
							 % Task 5 : Lissajous curve
               % Task 6 :Ellipse with Z
							 % Task 7 : Two way points test (Speed test)
							 % Task 8 : Landing on different set point

radius_ref = 0.05; % 4 cm
period_ref = 3.0; % Lissajous 1.5; % Circle : 3seconds
frequency_ref = 2*pi/period_ref;
radius_ratio = 0.5;
landing_parameter = [start_delay+T, transition_rate, rising_time, falling_time, landing_rising_time, landing_z_decrease, landing_y_decrease, landing_x_decrease];
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

%Lateral gain
k_x = 0.55/scale;
k_v = 0.05/scale;

%Attitude gain
k_R = 0.5/(scale^2);%0.5/(scale^2);		yaw pitch
k_Rx = 0.5/(scale^2);%0.6/(scale^2);		roll
k_Omega =  0.25/(scale^2);%0.25/(scale^2);

%Altitude gain 
k_z = 0.1/scale; %0.2/scale;%0.2/scale;
k_vz = 0.5/scale; %0.35/scale;%0.25/scale; (Damping coefficients on z direction)

control_gain = [k_x,k_v, k_R, k_Omega, k_z, k_vz, k_Rx];

% Altitude feed back saturation
upp_bound_z = 0.04; % m
low_bound_z =-0.04; % m
upp_bound_vz = 0.5; % m/s
low_bound_vz =-0.5; % m/s

upp_bound_x = 0.06; % m
low_bound_x =-0.06; % m
upp_bound_vx = 0.5; % m/s
low_bound_vx =-0.5; % m/s

upp_bound_y = 0.06; % m
low_bound_y =-0.06; % m
upp_bound_vy = 0.5; % m/s
low_bound_vy =-0.5; % m/s


upp_bound =10; %4 % rad/s
low_bound =-10; %-4 % rad/s
upp_bound_eR =0.8; %0.3   % attitude error
low_bound_eR =-0.8; %-0.3 % attitude error

% Adaptive Control gain
gamma_adaptive = 5e-8*upp_bound;
adaptive_roll_limit = 0.3/(scale^2);%0.17/(scale^2);
adaptive_pitch_limit = 0.1/(scale^2);%0.09/(scale^2);
adaptive_yaw_limit = 0.035/(scale^2); %0.017/(scale^2);

adaptive_roll_limit_low = -adaptive_roll_limit;
adaptive_pitch_limit_low = -adaptive_pitch_limit;
adaptive_yaw_limit_low = -adaptive_yaw_limit;


c_2_upp_bound = sqrt(k_R*upp_bound_eR*I_moment(3,3))/I_moment(1,1);
c_2_adaptive=c_2_upp_bound*upp_bound_eR/upp_bound;


% Change these (set to 0 on first closed-loop trial)
% RPY initial conditions
adaptive_roll_init = 0; %0.045; %0.1/(scale^2);%-0.1/(scale^2); %0;
adaptive_pitch_init =  0; %-0.02;%-0.02/(scale^2);%-0.05/(scale^2);
adaptive_yaw_init = 0; %-0.05;%-0.029/(scale^2);

% Scale to milli-Newton * millimeter
adaptive_roll_init = adaptive_roll_init/(scale^2);
adaptive_pitch_init =  adaptive_pitch_init/(scale^2);
adaptive_yaw_init = adaptive_yaw_init/(scale^2);


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

% Change these (set to 0 on first closed-loop trial)
% Thrust Vector initial conditions
adaptive_x_init = 0; %-0.06;%-0.0/(scale); %-0.045/(scale); %-0.12/(scale);
adaptive_y_init = 0; %-0.03;%-0.07/(scale); %-0.1/(scale); %0;
adaptive_z_init = 0; %-0.09; %-0.015/(scale); %0.016/(scale);

% Scale to milli-Newtons
adaptive_x_init = adaptive_x_init/scale;
adaptive_y_init = adaptive_y_init/scale;
adaptive_z_init = adaptive_z_init/scale;


adaptive_gain = [gamma_adaptive, adaptive_roll_limit, adaptive_pitch_limit, adaptive_yaw_limit, c_2_adaptive, gamma_lateral_adaptive, adaptive_x_limit, adaptive_y_limit, adaptive_z_limit, c_1_adaptive];


%% setup code 
% Default openloop

closedloop_flag=0;

drv_bias = max_drv_bias;
phase = 0 %pi; %degrees (added on left wing)

if control_flag==2  % Open loop control

	closedloop_flag =1; % For simulink
	% Setup the Bias for PZT
%   drv_bias = max(closedloop_max_drv_bias,max_drv_bias);
	drv_bias = max_drv_bias;
  phase = 0 %pi; %degrees (added on left wing)
end

%% end of setup code
% running_time = start_delay+T+0.0;   %3.0+T+0.5 %11
% running_time_control = start_delay_control+T+0.0;   %3.0+T+0.5 %11

if control_flag==1
	running_time = start_delay+T;
	running_time_control = start_delay_control+T+0.0;   %3.0+T+0.5 %11

else
	running_time = start_delay+total_T;
	running_time_control = start_delay_control+total_T+0.0;   %3.0+T+0.5 %11	
% 	running_time = start_delay + T;
% 	running_time_control = start_delay_control + T;
end

%% low pass filter

[lp_num, lp_den]=butter(5, 80/sampling_f);  % 80
[vlp_num, vlp_den]=butter(5, 40/sampling_f);  % 80

%% Attitude controller setup (Filtering to get angular velocity wrt body frame)

att_s = 500;
k_da = exp(-att_s*sampling_time);
att_s2 = 60;
k_da2 = exp(-att_s*sampling_time);
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
