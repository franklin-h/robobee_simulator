clear all;

sampling_f = 10000;			% controller sampling rate
sampling_time = 1/sampling_f;
start_delay = 2.0;		%2.0	% start after xx s
ramp_delay =0.1;
start_delay_control = start_delay + ramp_delay;

%% Driving signal setup and experiment setup
f = 160; %170; %165
f_int = f;					% initial frequency
f_fin = f;					% final frequency
T = 0.35; %0.4;			% total time in seconds

control_flag=1; % 1: Openloop, 2: closed loop
max_drv_bias =250;

save_flag =0;
save_file_name = '20210329_1.mat';

%% Open loop control set up (Voltage control)
% v = 140;
% v_roll_offset = 0; % Voltages
% v_pitch_offset = 0; % Voltages
Phi_p2p_nominal_openloop = 48
Phi_roll_offset_openloop = -4% degrees  % positive : high amp for right wing  (negative torque)
Phi_pitch_offset_openloop = -3% 0.5% degrees     % positive : (negative pitch torque)

Phi_p2p_nominal_left =Phi_p2p_nominal_openloop-Phi_roll_offset_openloop/2;
Phi_p2p_nominal_right =Phi_p2p_nominal_openloop+Phi_roll_offset_openloop/2;


%% Closed loop control set up (Wing trajectory)

% Altitude control flag
altitude_flag=0;

Phi_p2p_nominal = 46; % Base line stroke p2p for the lift

% Pitch and roll offset angles
Phi_roll_offset = 0% degrees  % positive : high amp for right wing  (negative torque)
Phi_pitch_offset = 0.0% 0.5% degrees     % positive : (negative pitch torque)


%% Identified Linear mapping (wing trajecotry to Voltage)
wing_to_voltage = load('Bee_Trajectory_Input_Linear_fitting.mat');


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

% Open_loop phi
[drv_amp, drv_roll, drv_pitch_left, drv_pitch_right] = Wing_trajectory_to_Voltage(Phi_p2p_nominal_left,Phi_p2p_nominal_right,Phi_pitch_offset_openloop,left_mapping,right_mapping);
drv_pch = drv_pitch_left;
%% Feedback Control paramter


% Altitude control
z_desired = 0.15; %m
vz_desired = 0; % m/s

% Desired Attitude
R_desired_mat = eye(3);
R_desired = [R_desired_mat(1,1), R_desired_mat(1,2), R_desired_mat(1,3), R_desired_mat(2,1), R_desired_mat(2,2), R_desired_mat(2,3), R_desired_mat(3,1), R_desired_mat(3,2), R_desired_mat(3,3)];
omega_desired = [0;0;0]; 

% Controller bounds
kz_Phi_gain= 2.0; %degrees
kvz_Phi_gain = 3.0; %degrees
kpw_x_Phi_gain = 10.0; %degrees
kpw_y_Phi_gain = 1.5; % degrees
kpeR_x_Phi_gain = 20.0; %degrees
kpeR_y_Phi_gain= 2.5; %degrees


% Altitude feed back saturation
upp_bound_z = 0.04; % m
low_bound_z =-0.04; % m
upp_bound_vz = 0.5; % m/s
low_bound_vz =-0.5; % m/s

upp_bound =8; %4 % rad/s
low_bound =-8; %-4 % rad/s
upp_bound_eR =0.5; %0.3   % attitude error
low_bound_eR =-0.5; %-0.3 % attitude error

% Compute the maximum bounds of the control input
Phi_avg_bar = (Phi_p2p_nominal+( kz_Phi_gain + kvz_Phi_gain))/2;
Phi_diff_bar = -abs(Phi_roll_offset)/4;
Phi_eff_squared = Phi_avg_bar.^2 + Phi_diff_bar.^2;
Phi_diff_tilda = -(kpw_x_Phi_gain+kpeR_x_Phi_gain)/4;
Phi_avg_tilda = (Phi_eff_squared-(Phi_diff_bar+Phi_diff_tilda).^2).^(1/2)-Phi_avg_bar;

Phi_amp_p2p = (Phi_avg_bar+Phi_avg_tilda)*2;
Phi_roll_p2p = (Phi_diff_bar+Phi_diff_tilda)*2;

Phi_left_p2p_command = Phi_amp_p2p+Phi_roll_p2p; % Following the Drive_signal block
Phi_right_p2p_command = Phi_amp_p2p-Phi_roll_p2p;% Following the Drive_signal block
Phi_pitch_command = kpw_y_Phi_gain+kpeR_y_Phi_gain+abs(Phi_pitch_offset); %15;%21.5+30;%22; % DC gain
[drv_amp_upp_bound, drv_roll_upp_bound, drv_pitch_left_upp_bound, drv_pitch_right_upp_bound] = Wing_traj_to_Voltage(Phi_left_p2p_command,Phi_right_p2p_command,Phi_pitch_command,left_mapping,right_mapping);

closedloop_max_drv_bias = (drv_amp_upp_bound/2 + abs(drv_roll_upp_bound) + max(abs(drv_pitch_left_upp_bound),abs(drv_pitch_right_upp_bound)))*2 % Maximum closed loop signal

%% setup code 
% Default openloop

closedloop_flag=0;
% v_nominal =v;
% vleft = v-v_roll_offset/2; %205;%+13-5; %160 %205
% vright =v+v_roll_offset/2; %193;%-13+5;  %200
% drv_pch = v_pitch_offset; %15;%21.5+30;%22;
% drv_amp = abs((vleft-vright))/2+min(vleft,vright);
% drv_roll = (vleft-vright)/4; 
drv_bias = max_drv_bias;
phase = 0 %pi; %degrees (added on left wing)

if control_flag==2  % Open loop control

	closedloop_flag =1; % For simulink
	% Setup the Bias for PZT
  drv_bias = max(closedloop_max_drv_bias,max_drv_bias);
  phase = 0 %pi; %degrees (added on left wing)
end

%% end of setup code
running_time = start_delay+T+0.0;   %3.0+T+0.5 %11
running_time_control = start_delay_control+T+0.0;   %3.0+T+0.5 %11
%% low pass filter
[lp_num, lp_den]=butter(5, 80/sampling_f);  % 80


%% attitude controller setup (Filtering to get angular velocity wrt body frame)

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
