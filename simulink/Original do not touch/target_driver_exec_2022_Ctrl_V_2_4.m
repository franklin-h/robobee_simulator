+tg;
pause(running_time+0.5)
tg.stop 
pause(2)  
yout = tg.outputLog;

%% Save yout
% save('Bee1_Static_170Hz_180V_10KSF.mat', 'yout')

if save_flag==1
	save(save_file_name);
end

%% Read output data 

time_vicon = yout(:,1);
x_vicon = yout(:,6);
y_vicon = yout(:,7);
z_vicon = yout(:,8);
alpha_vicon = yout(:,9);
beta_vicon = yout(:,10);
gamma_vicon = yout(:,11);

omega_x = yout(:,12);
omega_y = yout(:,13);
omega_z = yout(:,14);

Geometric_omega_x_filtered = yout(:,15);
Geometric_omega_y_filtered = yout(:,16);
Geometric_omega_z_filtered = yout(:,17);

quat_0_avg = yout(:,18);
quat_1_avg = yout(:,19);
quat_2_avg = yout(:,20);
quat_3_avg = yout(:,21);

quat_0 = yout(:,22);
quat_1 = yout(:,23);
quat_2 = yout(:,24);
quat_3 = yout(:,25);

update_cnt = yout(:,26);

e_Omega_x = yout(:,27);
e_Omega_y = yout(:,28);
e_Omega_z = yout(:,29);

drv_amp_closedloop = yout(:,30);
drv_pch_left_closedloop = yout(:,31);
drv_pch_right_closedloop = yout(:,32);
drv_roll_closedloop = yout(:,33);
a2_closedloop = yout(:,34);

gain_voltage = 0.01;

bias_raw = yout(:,2)/gain_voltage;
v_l_raw = yout(:,3)/gain_voltage;
v_r_raw = yout(:,4)/gain_voltage;


bias_raw_closedloop = yout(:,35)/gain_voltage;
v_l_raw_closedloop = yout(:,36)/gain_voltage;
v_r_raw_closedloop = yout(:,37)/gain_voltage;

eR_x = yout(:,38);
eR_y = yout(:,39);
eR_z = yout(:,40);

vx_avg = yout(:,41);
vy_avg = yout(:,42);
vz_avg = yout(:,43);
	
x_avg = yout(:,44);
y_avg = yout(:,45);
z_avg = yout(:,46);
	
normalized_altitude_error_z = yout(:,47);
normalized_altitude_error_vz = yout(:,48);

normalized_ex_x = yout(:,47);
normalized_ex_y = yout(:,48);
normalized_ex_z = yout(:,49);

normalized_ev_x = yout(:,80);
normalized_ev_y = yout(:,81);
normalized_ev_z = yout(:,82);



omegax_d = yout(:,58);
omegay_d = yout(:,59);
omegaz_d = yout(:,60);

omegax_d_dot = yout(:,61);
omegay_d_dot = yout(:,62);
omegaz_d_dot = yout(:,63);
    
% 	Phi_p2p_nominal_closed = yout(:,47);

drv_amp_output = yout(:,50);
drv_pitch_left_output = yout(:,51);
drv_pitch_right_output = yout(:,52);
drv_roll_output = yout(:,53);
a2_output = yout(:,54);
	
% [Phi_left_p2p_command,Phi_right_p2p_command,Phi_pitch_left_command,Phi_pitch_right_command] = Voltage_to_Wing_trajectory(drv_amp_output, drv_roll_output, drv_pitch_left_output, drv_pitch_right_output,left_mapping,right_mapping);
% [Phi_left_p2p_closed,Phi_right_p2p_closed,Phi_pitch_left_closed,Phi_pitch_right_closed] = Voltage_to_Wing_trajectory(drv_amp_closedloop, drv_roll_output, drv_pch_left_closedloop, drv_pch_right_closedloop,left_mapping,right_mapping);

% drv_amp1 = abs((vleft_p2p-vright_p2p))/2+min(vleft_p2p,vright_p2p); % p2p
% drv_roll1 = (vleft_p2p-vright_p2p)/4; % Amplitude

% Force control output
roll_desired_output = yout(:,55);
pitch_desired_output = yout(:,56);
thrust_desired_output = yout(:,57);
yaw_desired_output = yout(:,64);

% Desired Attitude
Rd_desired_output = [yout(:,67), yout(:,70), yout(:,73)];
heading_output = [yout(:,65),yout(:,68),yout(:,71)];
adaptive_output = [yout(:,74), yout(:,75), yout(:,76)];
adaptive_lateral_output = [yout(:,77), yout(:,78), yout(:,79)];

I=find(time_vicon-start_delay<0);
I_end=find(time_vicon-running_time<0);
start_delay_time_index = I(end);
end_time_index = I_end(end);

% Reference trajecotry

r_reference_x = yout(:,83);
r_reference_y = yout(:,84);
r_reference_z = yout(:,85);

v_reference_x = yout(:,86);
v_reference_y = yout(:,87);
v_reference_z = yout(:,88);

b_1_d_reference_x = yout(:,89);
b_1_d_reference_y = yout(:,90);
b_1_d_reference_z = yout(:,91);

% Attitude error function

Rotation_avg = quat2rotm([quat_0_avg,quat_1_avg,quat_2_avg,quat_3_avg]);

Error_R = zeros(1,length(time_vicon));
e_Omega_I_e_Omega = zeros(1,length(time_vicon));

for i=1:length(time_vicon)
	Rd_temp = [yout(i,65),yout(i,66),yout(i,67);...
						 yout(i,68),yout(i,69),yout(i,70);...
						 yout(i,71),yout(i,72),yout(i,73)];
  Rot_m = squeeze(Rotation_avg(:,:,i));
	
	Error_R(i) = 1/2*trace(eye(3)-Rd_temp.'*Rot_m);
	e_Omega_I_e_Omega(i)=upp_bound^2*[e_Omega_x(i),e_Omega_y(i),e_Omega_z(i)]*I_moment*[e_Omega_x(i),e_Omega_y(i),e_Omega_z(i)].';
end

% Lyapunov function

c_1 = c_1_adaptive
c_2 = c_2_adaptive

V_Lyapunov = 1/2*(normalized_ex_x.^2*upp_bound_x*k_x+normalized_ex_y.^2*upp_bound_y*k_x+normalized_ex_z.^2*upp_bound_z*k_z)...
	          + 1/2*m*(vx_avg.^2+vy_avg.^2+vz_avg.^2)...
						+c_1*(normalized_ex_x.*vx_avg*upp_bound_x+normalized_ex_y.*vy_avg*upp_bound_y+normalized_ex_z.*vz_avg*upp_bound_z)...
						+k_R*Error_R'...
						+c_2*upp_bound*upp_bound_eR*(e_Omega_x.*eR_x+e_Omega_y.*eR_y+e_Omega_z.*eR_z)...
						+1/2*e_Omega_I_e_Omega';


% Plot Figures
figure(1);
subplot(1,3,1)
plot(time_vicon, x_vicon, 'r');
hold on;
plot(time_vicon, y_vicon, 'g');
plot(time_vicon, z_vicon, 'b');
plot(time_vicon, x_avg, 'r--');
plot(time_vicon, y_avg, 'g--');
plot(time_vicon, z_avg, 'b--');
xlabel('Time (s)')
ylabel('m')
legend('x', 'y','z')
xlim([start_delay-0.2, running_time+0.2])
plot(time_vicon(start_delay_time_index), x_vicon(start_delay_time_index),'ro','MarkerFaceColor','r', 'HandleVisibility','off')
plot(time_vicon(start_delay_time_index), y_vicon(start_delay_time_index),'go','MarkerFaceColor','g', 'HandleVisibility','off')
plot(time_vicon(start_delay_time_index), z_vicon(start_delay_time_index),'bo','MarkerFaceColor','b', 'HandleVisibility','off')
plot(time_vicon(end_time_index), x_vicon(end_time_index),'ro','MarkerFaceColor','r', 'HandleVisibility','off')
plot(time_vicon(end_time_index), y_vicon(end_time_index),'go','MarkerFaceColor','g', 'HandleVisibility','off')
plot(time_vicon(end_time_index), z_vicon(end_time_index),'bo','MarkerFaceColor','b', 'HandleVisibility','off')

title('CoM position')

subplot(1,3,2)
plot(time_vicon, alpha_vicon, 'r');
hold on;
plot(time_vicon, beta_vicon, 'g');
plot(time_vicon, gamma_vicon, 'b');
xlabel('Time (s)')
ylabel('rad')
legend('Rx', 'Ry','Rz')
xlim([start_delay-0.2, running_time+0.2])
plot(time_vicon(start_delay_time_index), alpha_vicon(start_delay_time_index),'ro','MarkerFaceColor','r', 'HandleVisibility','off')
plot(time_vicon(start_delay_time_index), beta_vicon(start_delay_time_index),'go','MarkerFaceColor','g', 'HandleVisibility','off')
plot(time_vicon(start_delay_time_index), gamma_vicon(start_delay_time_index),'bo','MarkerFaceColor','b', 'HandleVisibility','off')
plot(time_vicon(end_time_index), alpha_vicon(end_time_index),'ro','MarkerFaceColor','r', 'HandleVisibility','off')
plot(time_vicon(end_time_index), beta_vicon(end_time_index),'go','MarkerFaceColor','g', 'HandleVisibility','off')
plot(time_vicon(end_time_index), gamma_vicon(end_time_index),'bo','MarkerFaceColor','b', 'HandleVisibility','off')

title('Orientation')

subplot(1,3,3)
plot(time_vicon, update_cnt,'r');
xlabel('Time (s)')
ylabel('Count')
title('Vicon Update rate')

figure(2);
subplot(4,1,1)
hold on;
plot( time_vicon, quat_0, 'r');
plot( time_vicon, quat_0_avg, 'b');
xlabel('Time (s)')
ylabel('q0')

subplot(4,1,2)
hold on;
plot( time_vicon, quat_1, 'r');
plot( time_vicon, quat_1_avg, 'b');
xlabel('Time (s)')
ylabel('q1')
legend('raw','R(t) averaged')

hold off;
subplot(4,1,3)
hold on;
plot( time_vicon, quat_2, 'r');
plot( time_vicon, quat_2_avg, 'b');
xlabel('Time (s)')
ylabel('q2')
subplot(4,1,4)
hold on;
plot( time_vicon, quat_3, 'r');
plot( time_vicon, quat_3_avg, 'b');
xlabel('Time (s)')
ylabel('q3')
title('Orientation')

% figure(2);
% plot(time_vicon, alpha_vicon, 'r');
% hold on;
% plot(time_vicon, beta_vicon, 'g');
% plot(time_vicon, gamma_vicon, 'b');
% xlabel('Time (s)')
% ylabel('rad')
% legend('Rx', 'Ry','Rz')

figure(3);
subplot(1,2,1)
plot(time_vicon, omega_x, 'r');
hold on;
plot(time_vicon, omega_y, 'g');
plot(time_vicon, omega_z, 'b');

plot( time_vicon,Geometric_omega_x_filtered, 'r--');
plot( time_vicon,Geometric_omega_y_filtered, 'g--');
plot( time_vicon,Geometric_omega_z_filtered, 'b--');

plot(time_vicon(start_delay_time_index), omega_x(start_delay_time_index),'r*')
plot(time_vicon(start_delay_time_index), omega_y(start_delay_time_index),'g*')
plot(time_vicon(start_delay_time_index), omega_z(start_delay_time_index),'b*')
plot(time_vicon(end_time_index), omega_x(end_time_index),'r*')
plot(time_vicon(end_time_index), omega_y(end_time_index),'g*')
plot(time_vicon(end_time_index), omega_z(end_time_index),'b*')
plot(time_vicon(start_delay_time_index), Geometric_omega_x_filtered(start_delay_time_index),'ro','MarkerFaceColor','r', 'HandleVisibility','off')
plot(time_vicon(start_delay_time_index), Geometric_omega_x_filtered(start_delay_time_index),'go','MarkerFaceColor','g', 'HandleVisibility','off')
plot(time_vicon(start_delay_time_index), Geometric_omega_x_filtered(start_delay_time_index),'bo','MarkerFaceColor','b', 'HandleVisibility','off')
plot(time_vicon(end_time_index), Geometric_omega_x_filtered(end_time_index),'ro','MarkerFaceColor','r', 'HandleVisibility','off')
plot(time_vicon(end_time_index), Geometric_omega_x_filtered(end_time_index),'go','MarkerFaceColor','g', 'HandleVisibility','off')
plot(time_vicon(end_time_index), Geometric_omega_x_filtered(end_time_index),'bo','MarkerFaceColor','b', 'HandleVisibility','off')




xlabel('Time (s)')
ylabel('rad/s')
legend('wx (Euler)', 'wy (Euler)','wz (Euler)', 'wx (Geom)','wy (Geom)','wz (Geom)')
xlim([start_delay-0.2, running_time+0.2])
title('Angular velocity')

subplot(1,2,2)
plot(time_vicon, eR_x*upp_bound_eR, 'r');
hold on;
plot(time_vicon,  eR_y*upp_bound_eR, 'g');
plot(time_vicon,  eR_z*upp_bound_eR, 'b');
plot(time_vicon(start_delay_time_index), eR_x(start_delay_time_index)*upp_bound_eR, 'ro','MarkerFaceColor','r', 'HandleVisibility','off');
hold on;
plot(time_vicon(start_delay_time_index),  eR_y(start_delay_time_index)*upp_bound_eR, 'go','MarkerFaceColor','g', 'HandleVisibility','off');
plot(time_vicon(start_delay_time_index),  eR_z(start_delay_time_index)*upp_bound_eR, 'bo','MarkerFaceColor','b', 'HandleVisibility','off');
plot(time_vicon(end_time_index), eR_x(end_time_index)*upp_bound_eR, 'ro','MarkerFaceColor','r', 'HandleVisibility','off');
hold on;
plot(time_vicon(end_time_index),  eR_y(end_time_index)*upp_bound_eR, 'go','MarkerFaceColor','g', 'HandleVisibility','off');
plot(time_vicon(end_time_index),  eR_z(end_time_index)*upp_bound_eR, 'bo','MarkerFaceColor','b', 'HandleVisibility','off');


xlabel('Time (s)')
ylabel('Geometric error')
legend('eR_x (roll)','eR_y (pitch)', 'eR_z (yaw)')
title('Attitude error (from upright)')
ylim([-1, 1])

figure(4);
subplot(2,3,1)
plot(time_vicon, bias_raw, 'r');
hold on;
plot(time_vicon, v_l_raw, 'b');
plot(time_vicon, v_r_raw, 'g');
xlabel('Time (s)')
ylabel('V')
legend('Bias','Left', 'Right')
title('Commanded Voltage Output')
subplot(2,3,2)
plot(time_vicon, drv_amp_output, 'r');
hold on;
plot(time_vicon, drv_pitch_left_output, 'b-');
plot(time_vicon, drv_pitch_right_output, 'b--');

plot(time_vicon, drv_roll_output, 'g');
xlabel('Time (s)')
ylabel('V')
legend('drv_amp','drv_pch_left','drv_pch_right', 'drv_roll')
title('Commanded Control Params')


subplot(2,3,3)
plot(time_vicon, (drv_amp_output-2*(drv_roll_output)), 'r');
hold on;
plot(time_vicon,  (drv_amp_output+2*(drv_roll_output)), 'b');
xlabel('Time (s)')
ylabel('V')
legend('Right Vp-p','Left Vp-p')
title('Commanded Voltage P-P')

subplot(2,3,4)
plot(time_vicon, bias_raw_closedloop, 'r');
hold on;
plot(time_vicon, v_l_raw_closedloop, 'b');
plot(time_vicon, v_r_raw_closedloop, 'g');

xlabel('Time (s)')
ylabel('V closeloop')
legend('Bias','Left', 'Right')
title('Closed loop Signal')

subplot(2,3,5)
plot(time_vicon, drv_amp_closedloop, 'r');
hold on;
plot(time_vicon, drv_pch_left_closedloop, 'b-');
plot(time_vicon, drv_pch_right_closedloop, 'b--');

plot(time_vicon, drv_roll_closedloop, 'g');
xlabel('Time (s)')
ylabel('V')
legend('drv_amp','drv_pch_left','drv_pch_right', 'drv_roll')
title('Closed LoopControl Params')

subplot(2,3,6)
plot(time_vicon, (drv_amp_closedloop-2*(drv_roll_closedloop)), 'r');
hold on;
plot(time_vicon,  (drv_amp_closedloop+2*(drv_roll_closedloop)), 'b');
xlabel('Time (s)')
ylabel('V')
legend('Right Vp-p','Left Vp-p')
title('Closed loop Voltage P-P')

figure(55)
% subplot(1,3,1)
% hold on;
% plot(time_vicon, -normalized_altitude_error_z*kz_Phi_gain,'r');
% plot(time_vicon, -normalized_altitude_error_vz*kvz_Phi_gain,'b');
% plot(time_vicon, -normalized_altitude_error_z*kz_Phi_gain-normalized_altitude_e-0.1/(scale^2)rror_vz*kvz_Phi_gain,'g');
% plot(time_vicon(start_delay_time_index), -normalized_altitude_error_z(start_delay_time_index)*kz_Phi_gain,'ro','MarkerFaceColor','r', 'HandleVisibility','off')
% plot(time_vicon(start_delay_time_index), -normalized_altitude_error_vz(start_delay_time_index)*kvz_Phi_gain,'bo','MarkerFaceColor','b', 'HandleVisibility','off')
% plot(time_vicon(end_time_index), -normalized_altitude_error_z(end_time_index)*kz_Phi_gain,'ro','MarkerFaceColor','r', 'HandleVisibility','off')
% plot(time_vicon(end_time_index), -normalized_altitude_error_vz(end_time_index)*kvz_Phi_gain,'bo','MarkerFaceColor','b', 'HandleVisibility','off')
% 
% xlabel('Time (s)')
% ylabel('Degrees')
% legend('altitude (z) feedback','velocity (vz) feedback')
% title('Altitude control ')
subplot(1,2,1)
plot(time_vicon, vz_avg,'r');
xlabel('Time (s)')
ylabel('vz velocity (m/s)')
title('Altitude velocity')
subplot(1,2,2)
hold on;
plot(time_vicon, z_avg,'r');
plot(time_vicon, r_desired(3)*ones(1,length(time_vicon)),'b')
xlabel('Time (s)')
ylabel('Altitude (m)')
legend('Altitude','Desired')
title('Altitude')

% figure(5);
% subplot(1,2,1)
% plot(time_vicon, Phi_right_p2p_command, 'r');
% hold on;
% plot(time_vicon, Phi_left_p2p_command, 'b');
% plot(time_vicon, Phi_pitch_right_command, 'r--');
% plot(time_vicon, Phi_pitch_left_command, 'b--');
% xlabel('Time (s)')
% ylabel('degrees')
% legend('right (p2p)','left (p2p)', 'right (pitch)', 'left (pitch)')
% title('Commanded Wing trajectory Params')

% subplot(1,2,2)
% plot(time_vicon, Phi_right_p2p_closed, 'r');
% hold on;
% plot(time_vicon, Phi_left_p2p_closed, 'b');
% plot(time_vicon, Phi_pitch_right_closed, 'r--');
% plot(time_vicon, Phi_pitch_left_closed, 'b--');
% xlabel('Time (s)')
% ylabel('degrees')
% legend('right (p2p)','left (p2p)', 'right (pitch)', 'left (pitch)')
% title('Closed loop Wing trajectory Params')


%% Post processing : Estimation of the force and torque

% % Post processing
% scale =1e3;
thrust_bound = 3; % 3 mN
torque_bound = 3; % 3 mNmm
% g=9.8;
% m = 86*1e-6;
% Ixx = 1.42*1e-9;
% Iyy = 1.34*1e-9;
% Izz = 0.45*1e-9;

I_moment =eye(3);
I_moment(1,1) = Ixx;
I_moment(2,2) = Iyy;
I_moment(3,3) = Izz;
e1=[1;0;0];
e2=[0;1;0];
e3=[0;0;1];

% get avg rotation matrix
Rotation_avg = quat2rotm([quat_0_avg,quat_1_avg,quat_2_avg,quat_3_avg]);
R_avg_e3 = zeros(3,length(time_vicon));
heading_avg_e1 = zeros(length(time_vicon),3);
for i=1:length(time_vicon)
	R_avg_e3(:,i) = squeeze(Rotation_avg(:,:,i))*e3;
	heading_avg_e1(i,:) = (squeeze(Rotation_avg(:,:,i))*e1)';
end
% Acceleration data
acceleration_vx = filter([1 -1],[1],vx_avg)*sampling_f;
acceleration_vy = filter([1 -1],[1],vy_avg)*sampling_f;
acceleration_vz = filter([1 -1],[1],vz_avg)*sampling_f;
acceleration_v = [acceleration_vx,acceleration_vy,acceleration_vz];

acceleration_wx = filter([1 -1],[1],Geometric_omega_x_filtered)*sampling_f;
acceleration_wy = filter([1 -1],[1],Geometric_omega_y_filtered)*sampling_f;
acceleration_wz = filter([1 -1],[1],Geometric_omega_z_filtered)*sampling_f;
acceleration_w = [acceleration_wx,acceleration_wy,acceleration_wz];

% compute w (x) Iw (cross product)
Geometric_omega_filtered=[Geometric_omega_x_filtered,Geometric_omega_y_filtered,Geometric_omega_z_filtered];
w_hat_I_moment_w=zeros(3,length(time_vicon));
y_thrust = zeros(1,length(time_vicon));
y_torque_x = zeros(1,length(time_vicon));
y_torque_y = zeros(1,length(time_vicon));
y_torque_z = zeros(1,length(time_vicon));
y_torque_x_coriolis = zeros(1,length(time_vicon));
y_torque_y_coriolis = zeros(1,length(time_vicon));
y_torque_z_coriolis = zeros(1,length(time_vicon));
for i=1:length(time_vicon)
	w_hat_I_moment_w(:,i) = hat_operation(Geometric_omega_filtered(i,:))*I_moment*Geometric_omega_filtered(i,:)';
% Estimate the thrust and torque	
	y_thrust(i) = R_avg_e3(:,i)'*(m*acceleration_v(i,:)'+m*g*e3);
	y_torque_x(i) = e1'*(I_moment*acceleration_w(i,:)'+w_hat_I_moment_w(:,i));
	y_torque_y(i) = e2'*(I_moment*acceleration_w(i,:)'+w_hat_I_moment_w(:,i));
	y_torque_z(i) = e3'*(I_moment*acceleration_w(i,:)'+w_hat_I_moment_w(:,i));
	y_torque_x_coriolis(i) = e1'*(w_hat_I_moment_w(:,i));
	y_torque_y_coriolis(i) = e2'*(w_hat_I_moment_w(:,i));
	y_torque_z_coriolis(i) = e3'*(w_hat_I_moment_w(:,i));

end
% 
% 
% for i=1:length(time_vicon)
% 	w_hat_I_moment_w(:,i) = hat_operation(Geometric_omega_filtered(i,:))*I_moment*Geometric_omega_filtered(i,:)';
% end
% % Estimate the thrust and torque
% y_thrust = diag(R_avg_e3'*(m*acceleration_v'+m*g*e3*ones(1,length(time_vicon))));
% y_torque_x = diag((e1*ones(1,length(time_vicon)))'*(I_moment*acceleration_w'+w_hat_I_moment_w));
% y_torque_y = diag((e2*ones(1,length(time_vicon)))'*(I_moment*acceleration_w'+w_hat_I_moment_w));
% y_torque_z = diag((e3*ones(1,length(time_vicon)))'*(I_moment*acceleration_w'+w_hat_I_moment_w));
% y_torque_x_coriolis = diag((e1*ones(1,length(time_vicon)))'*(w_hat_I_moment_w));
% y_torque_y_coriolis = diag((e2*ones(1,length(time_vicon)))'*(w_hat_I_moment_w));
% y_torque_z_coriolis = diag((e3*ones(1,length(time_vicon)))'*(w_hat_I_moment_w));

% Plot the estimated thrust and torque

figure(6);
subplot(4,1,1)
plot(time_vicon, y_thrust*scale, 'r');
hold on;
plot(time_vicon, m*g*ones(1,length(time_vicon))*scale, 'k--');
plot(time_vicon(start_delay_time_index), y_thrust(start_delay_time_index)*scale, 'ro','MarkerFaceColor','r', 'HandleVisibility','off');
plot(time_vicon(end_time_index), y_thrust(end_time_index)*scale, 'ro','MarkerFaceColor','r', 'HandleVisibility','off');
xlabel('Time (s)')
ylabel('Thrust (mN)')
legend('Estimated','Body weight')
title('Thrust estimation')
xlim([time_vicon(start_delay_time_index), time_vicon(end_time_index)])
ylim([0, thrust_bound])
-0.1/(scale^2)
subplot(4,1,2)
plot(time_vicon, y_torque_x*scale^2, 'r');
hold on;
plot(time_vicon, y_torque_x_coriolis*scale^2, 'g');
plot(time_vicon(start_delay_time_index), y_torque_x(start_delay_time_index)*scale^2, 'ro','MarkerFaceColor','r', 'HandleVisibility','off');
plot(time_vicon(end_time_index), y_torque_x(end_time_index)*scale^2, 'ro','MarkerFaceColor','r', 'HandleVisibility','off');
xlabel('Time (s)')
ylabel('Roll Torque x (mNmm)')
legend('Estimated torque', 'Coriolis term')
title('Roll Torque estimation')
xlim([time_vicon(start_delay_time_index), time_vicon(end_time_index)])
ylim([-torque_bound, torque_bound])

subplot(4,1,3)
plot(time_vicon, y_torque_y*scale^2, 'r');
hold on;
plot(time_vicon, y_torque_y_coriolis*scale^2, 'g');
plot(time_vicon(start_delay_time_index), y_torque_y(start_delay_time_index)*scale^2, 'ro','MarkerFaceColor','r', 'HandleVisibility','off');
plot(time_vicon(end_time_index), y_torque_y(end_time_index)*scale^2, 'ro','MarkerFaceColor','r', 'HandleVisibility','off');
xlabel('Time (s)')
ylabel('Pitch Torque x (mNmm)')
legend('Estimated torque', 'Coriolis term')
title('Pitch Torque estimation')
xlim([time_vicon(start_delay_time_index), time_vicon(end_time_index)])
ylim([-torque_bound, torque_bound])

subplot(4,1,4)
plot(time_vicon, y_torque_z*scale^2, 'r');
hold on;
plot(time_vicon, y_torque_z_coriolis*scale^2, 'g');
plot(time_vicon(start_delay_time_index), y_torque_z(start_delay_time_index)*scale^2, 'ro','MarkerFaceColor','r', 'HandleVisibility','off');
plot(time_vicon(end_time_index), y_torque_z(end_time_index)*scale^2, 'ro','MarkerFaceColor','r', 'HandleVisibility','off');
xlabel('Time (s)')
ylabel('Yaw Torque x (mNmm)')
legend('Estimated torque', 'Coriolis term')
title('Yaw Torque estimation')
xlim([time_vicon(start_delay_time_index), time_vicon(end_time_index)])
ylim([-torque_bound, torque_bound])


%% Closed loop control plots

figure(7)
subplot(1,2,1)
plot(time_vicon, thrust_desired_output*scale, 'r');
hold on;
plot(time_vicon, m*g*ones(1,length(time_vicon))*scale, 'k--');
xlabel('Time (s)')
ylabel('Thrust (mN)')
legend('thrust', 'body weight')
title('Desired thrust')

subplot(1,2,2)
plot(time_vicon, roll_desired_output*scale^2, 'r');
hold on;
plot(time_vicon, pitch_desired_output*scale^2, 'g');
plot(time_vicon, yaw_desired_output*scale^2, 'b');
xlabel('Time (s)')
ylabel('Torque (mNmm)')
legend('roll torque', 'pitch torque', 'yaw torque')
title('Desired torque')

figure(8)
subplot(1,2,1)
plot(time_vicon, omegax_d, 'r');
hold on;
plot(time_vicon, omegay_d, 'g');
% 
plot(time_vicon, omegaz_d, 'b');
xlabel('Time (s)')
ylabel('rad/s')
title('Desired omega')
subplot(1,2,2)
plot(time_vicon, omegax_d_dot, 'r');
hold on;
plot(time_vicon, omegay_d_dot, 'g');
% 
plot(time_vicon, omegaz_d_dot, 'b');
xlabel('Time (s)')
ylabel('rad/s^2')
title('Desired omega derivative')

figure(9)
subplot(2,4,1)
plot(time_vicon, R_avg_e3(1,:), 'r');
hold on;
plot(time_vicon, R_avg_e3(2,:), 'g');
plot(time_vicon, R_avg_e3(3,:), 'b');

plot(time_vicon, Rd_desired_output(:,1), 'r--');
plot(time_vicon, Rd_desired_output(:,2), 'g--');
plot(time_vicon, Rd_desired_output(:,3), 'b--');

xlabel('Time (s)')
ylabel('Unit vector Re3')
ylim([-1.1,1.1])

subplot(2,4,2)
plot(time_vicon, normalized_ex_x*upp_bound_x, 'r');
hold on;
plot(time_vicon, normalized_ex_y*upp_bound_y, 'g');
plot(time_vicon, normalized_ex_z*upp_bound_z, 'b');
xlabel('Time (s)')
ylabel('ex')

subplot(2,4,3)
plot(time_vicon, normalized_ev_x*upp_bound_vx, 'r');
hold on;
plot(time_vicon, normalized_ev_y*upp_bound_vy, 'g');
plot(time_vicon, normalized_ev_z*upp_bound_vz, 'b');
xlabel('Time (s)')
ylabel('ev')

subplot(2,4,4)
plot(time_vicon, eR_x*upp_bound_eR, 'r');
hold on;
plot(time_vicon,  eR_y*upp_bound_eR, 'g');
plot(time_vicon,  eR_z*upp_bound_eR, 'b');
plot(time_vicon(start_delay_time_index), eR_x(start_delay_time_index)*upp_bound_eR, 'ro','MarkerFaceColor','r', 'HandleVisibility','off');
hold on;
plot(time_vicon(start_delay_time_index),  eR_y(start_delay_time_index)*upp_bound_eR, 'go','MarkerFaceColor','g', 'HandleVisibility','off');
plot(time_vicon(start_delay_time_index),  eR_z(start_delay_time_index)*upp_bound_eR, 'bo','MarkerFaceColor','b', 'HandleVisibility','off');
plot(time_vicon(end_time_index), eR_x(end_time_index)*upp_bound_eR, 'ro','MarkerFaceColor','r', 'HandleVisibility','off');
hold on;
plot(time_vicon(end_time_index),  eR_y(end_time_index)*upp_bound_eR, 'go','MarkerFaceColor','g', 'HandleVisibility','off');
plot(time_vicon(end_time_index),  eR_z(end_time_index)*upp_bound_eR, 'bo','MarkerFaceColor','b', 'HandleVisibility','off');


xlabel('Time (s)')
ylabel('e_R')
legend('eR_x (roll)','eR_y (pitch)', 'eR_z (yaw)')
title('e_R')

subplot(2,4,5)
plot(time_vicon, e_Omega_x*upp_bound, 'r');
hold on;
plot(time_vicon,  e_Omega_y*upp_bound, 'g');
plot(time_vicon,  e_Omega_z*upp_bound, 'b');
plot(time_vicon(start_delay_time_index), e_Omega_x(start_delay_time_index)*upp_bound, 'ro','MarkerFaceColor','r', 'HandleVisibility','off');
hold on;
plot(time_vicon(start_delay_time_index),  e_Omega_y(start_delay_time_index)*upp_bound, 'go','MarkerFaceColor','g', 'HandleVisibility','off');
plot(time_vicon(start_delay_time_index),  e_Omega_z(start_delay_time_index)*upp_bound, 'bo','MarkerFaceColor','b', 'HandleVisibility','off');
plot(time_vicon(end_time_index), e_Omega_x(end_time_index)*upp_bound, 'ro','MarkerFaceColor','r', 'HandleVisibility','off');
hold on;
plot(time_vicon(end_time_index),  e_Omega_y(end_time_index)*upp_bound, 'go','MarkerFaceColor','g', 'HandleVisibility','off');
plot(time_vicon(end_time_index),  e_Omega_z(end_time_index)*upp_bound, 'bo','MarkerFaceColor','b', 'HandleVisibility','off');


xlabel('Time (s)')
ylabel('e_\Omega')
legend('e_\Omega_x (roll)','e_\Omega_y (pitch)', 'e_\Omega_z (yaw)')
title('e_\Omega')

subplot(2,4,6)
plot(time_vicon, Error_R, 'r');
hold on;
plot(time_vicon(start_delay_time_index),  Error_R(start_delay_time_index), 'ro','MarkerFaceColor','r', 'HandleVisibility','off');
plot(time_vicon(end_time_index), Error_R(end_time_index), 'ro','MarkerFaceColor','r', 'HandleVisibility','off');
xlabel('Time (s)')
ylabel('Error function R')
title('\Psi_R')

subplot(2,4,7)
plot(time_vicon, V_Lyapunov, 'r');
hold on;
plot(time_vicon(start_delay_time_index),  V_Lyapunov(start_delay_time_index), 'ro','MarkerFaceColor','r', 'HandleVisibility','off');
plot(time_vicon(end_time_index), V_Lyapunov(end_time_index), 'ro','MarkerFaceColor','r', 'HandleVisibility','off');
xlabel('Time (s)')
ylabel('V Lyapunov')
title('Lyapunov Function')


figure(10)
subplot(1,2,1)
plot(time_vicon,adaptive_output(:,1)*(scale)^2, 'r');
hold on;
plot(time_vicon,adaptive_output(:,2)*(scale)^2, 'g');
plot(time_vicon,adaptive_output(:,3)*(scale)^2, 'b');
xlabel('Time (s)')
ylabel('Torques (mNmm)')
legend('roll torque', 'pitch torque', 'yaw torque')

title('Adaptive control')
subplot(1,2,2)
plot(time_vicon,adaptive_lateral_output(:,1)*(scale), 'r');
hold on;
plot(time_vicon,adaptive_lateral_output(:,2)*(scale), 'g');
plot(time_vicon,adaptive_lateral_output(:,3)*(scale), 'b');
xlabel('Time (s)')
ylabel('Forces (mN)')
legend('x', 'y', 'z')

title('Adaptive lateral control')


figure(11)
plot(time_vicon,a2_output, 'r');
xlabel('Time (s)')
ylabel('a2')

figure(12)
plot(time_vicon,normalized_ex_x*upp_bound_x*c_1_adaptive*50,'r');
hold on;
plot(time_vicon,normalized_ev_x*upp_bound_vx,'r--');
plot(time_vicon,normalized_ev_x*upp_bound_vx+normalized_ex_x*upp_bound_x*c_1_adaptive*50,'b');
xlabel('Time (s)')
ylabel('comparison ex vs ev')

figure(13)
subplot(1,2,1)
plot(time_vicon, x_avg, 'r');
hold on;
plot(time_vicon, y_avg, 'g');
plot(time_vicon, z_avg, 'b');
plot(time_vicon, r_reference_x, 'r--');
plot(time_vicon, r_reference_y, 'g--');
plot(time_vicon, r_reference_z, 'b--');

xlabel('Time (s)')
ylabel('position (m)')
ylim([-1.1,1.1])

subplot(1,2,2)
plot(time_vicon, vx_avg, 'r');
hold on;
plot(time_vicon, vy_avg, 'g');
plot(time_vicon, vz_avg, 'b');
plot(time_vicon, v_reference_x, 'r--');
plot(time_vicon, v_reference_y, 'g--');
plot(time_vicon, v_reference_z, 'b--');

xlabel('Time (s)')
ylabel('velocity (m/s)')
ylim([-1.1,1.1])

figure(14)
plot3(x_avg(start_delay_time_index:end_time_index),y_avg(start_delay_time_index:end_time_index),z_avg(start_delay_time_index:end_time_index),'b');
hold on;
plot3(r_reference_x,r_reference_y,r_reference_z,'r--');

list_time = linspace(start_delay, min(start_delay+period_ref*2, start_delay+total_T),300);
list_time_index = floor(list_time*sampling_f);

plot3(x_avg(list_time_index),y_avg(list_time_index),z_avg(list_time_index),'bo', 'MarkerFaceColor', 'b');
plot3(r_reference_x(list_time_index),r_reference_y(list_time_index),r_reference_z(list_time_index),'ro', 'MarkerFaceColor', 'r');
quiver3(r_reference_x(list_time_index),r_reference_y(list_time_index),r_reference_z(list_time_index),b_1_d_reference_x(list_time_index),b_1_d_reference_y(list_time_index),b_1_d_reference_z(list_time_index),0.5,'r')
% quiver3(x_avg(list_time_index),y_avg(list_time_index),z_avg(list_time_index), heading_output(list_time_index,1), heading_output(list_time_index,2),heading_output(list_time_index,3),0.5,'b');
quiver3(x_avg(list_time_index),y_avg(list_time_index),z_avg(list_time_index), heading_avg_e1(list_time_index,1), heading_avg_e1(list_time_index,2),heading_avg_e1(list_time_index,3),0.5,'b');

grid on;
axis equal;
% zlim([0.08,0.16])
% xlim([-0.10, 0.08])
% ylim([-0.10, 0.08])
xlabel('x')
ylabel('y')
zlabel('z')
% -0.1/(scale^2)

figure(15)
hold on;
plot(time_vicon,b_1_d_reference_x,'r--');
plot(time_vicon,heading_avg_e1(:,1),'r-');
plot(time_vicon,b_1_d_reference_y,'g--');
plot(time_vicon,heading_avg_e1(:,2),'g-');
plot(time_vicon,b_1_d_reference_z,'b--');
plot(time_vicon,heading_avg_e1(:,3),'b-');

xlabel('Time (s)')
ylabel('Heading b_1_d')