clear all;
close all;
clc;

% RNG = [1 1 42 4];
% exp_data_table=csvread('./OpenLoop_Survivor_March2022_CSV_new.csv',1,1,RNG);
% exp_data_table=csvread('./OpenLoop_Survivor_March2022_CSV_new.csv',1,1,RNG);

% TOGGLE TO VISUALIZE DATA
figure_state_flag = 0;

% TOGGLE TO SAVE DATA
save_flag = 0;

%%

start_time = 2.22;
end_time = 2.245;
sampling_freq = 10000;
start_index = start_time*sampling_freq + 1;
end_index = end_time*sampling_freq + 1;

% Select non-corrupted data files
% experiment_valid_index = [7, 13:19, 21:23, 26:27, 29:31, 39:43, 48:53, 56:62, 64:68];
% experiment_valid_index = 1:42;
experiment_valid_index = 1:42;

num_exp_index = length(experiment_valid_index);

%%

for index_exp = 1:num_exp_index

		% Read output data 
% 		file_name=strcat('Open_Loop_data/OL_20220307_',num2str(index_exp),'.mat');
% 		file_name=strcat('Open_Loop_data_new/OL_20220408_',num2str(experiment_valid_index(index_exp)),'.mat');
% 		file_name=strcat('20220506_BBee_OL',num2str(experiment_valid_index(index_exp)),'.mat');

		folder_name = 'Patrick_Bee_Open_Loop_Data_20230927';
		file_name = strcat(folder_name, '/OL_PBee_20230927_', num2str(experiment_valid_index(index_exp)),'.mat')
		load(file_name)
		message = strcat('Exp number:',num2str(experiment_valid_index(index_exp)),'\n');

		sprintf(message)

		% Extract Vicon output data
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

		if isempty(find(isnan(Geometric_omega_x_filtered)==1, 1))
		else
				Geometric_omega_x_filtered = omega_x;
				Geometric_omega_y_filtered = omega_y;
				Geometric_omega_z_filtered = omega_z;
		end


		quat_0_avg = yout(:,18);
		quat_1_avg = yout(:,19);
		quat_2_avg = yout(:,20);
		quat_3_avg = yout(:,21);

		quat_0 = yout(:,22);
		quat_1 = yout(:,23);
		quat_2 = yout(:,24);
		quat_3 = yout(:,25);

		update_cnt = yout(:,26);

		saturated_wx = yout(:,27);
		saturated_wy = yout(:,28);
		saturated_wz = yout(:,29);

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


		% 	Phi_p2p_nominal_closed = yout(:,47);


		drv_amp_output = yout(:,50);
		drv_pitch_left_output = yout(:,51);
		drv_pitch_right_output = yout(:,52);
		drv_roll_output = yout(:,53);
		a2_output = yout(:,54);

		[Phi_left_p2p_command,Phi_right_p2p_command,Phi_pitch_left_command,Phi_pitch_right_command] = Voltage_to_Wing_trajectory(drv_amp_output, drv_roll_output, drv_pitch_left_output, drv_pitch_right_output,left_mapping,right_mapping);
		[Phi_left_p2p_closed,Phi_right_p2p_closed,Phi_pitch_left_closed,Phi_pitch_right_closed] = Voltage_to_Wing_trajectory(drv_amp_closedloop, drv_roll_output, drv_pch_left_closedloop, drv_pch_right_closedloop,left_mapping,right_mapping);

		% drv_amp1 = abs((vleft_p2p-vright_p2p))/2+min(vleft_p2p,vright_p2p); % p2p
		% drv_roll1 = (vleft_p2p-vright_p2p)/4; % Amplitude

		% Set up for the input command for the optimization

		u1(index_exp) = Phi_left_p2p_command(floor(start_index))^2;
		u2(index_exp) = Phi_right_p2p_command(floor(start_index))^2;
		u3(index_exp) = Phi_pitch_left_command(floor(start_index));
		u4(index_exp) = Phi_pitch_right_command(floor(start_index));

		I = find(time_vicon-start_delay<0);
		I_end = find(time_vicon-running_time<0);
		start_delay_time_index = I(end);
		end_time_index = I_end(end);

		% Plot Figures
		if figure_state_flag==1
				figure(1);
				subplot(1,3,1)
				plot(time_vicon, x_vicon, 'r');
				hold on;
				plot(time_vicon, y_vicon, 'g');
				plot(time_vicon, z_vicon, 'b');
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
				hold on;
				% plot(time_vicon, omega_x, 'r');
				% hold on;
				% plot(time_vicon, omega_y, 'g');
				% plot(time_vicon, omega_z, 'b');

				plot( time_vicon,Geometric_omega_x_filtered, 'r-');
				plot( time_vicon,Geometric_omega_y_filtered, 'g-');
				plot( time_vicon,Geometric_omega_z_filtered, 'b-');

				% plot(time_vicon(start_delay_time_index), omega_x(start_delay_time_index),'r*')
				% plot(time_vicon(start_delay_time_index), omega_y(start_delay_time_index),'g*')
				% plot(time_vicon(start_delay_time_index), omega_z(start_delay_time_index),'b*')
				% plot(time_vicon(end_time_index), omega_x(end_time_index),'r*')
				% plot(time_vicon(end_time_index), omega_y(end_time_index),'g*')
				% plot(time_vicon(end_time_index), omega_z(end_time_index),'b*')
				plot(time_vicon(start_delay_time_index), Geometric_omega_x_filtered(start_delay_time_index),'ro','MarkerFaceColor','r', 'HandleVisibility','off')
				plot(time_vicon(start_delay_time_index), Geometric_omega_x_filtered(start_delay_time_index),'go','MarkerFaceColor','g', 'HandleVisibility','off')
				plot(time_vicon(start_delay_time_index), Geometric_omega_x_filtered(start_delay_time_index),'bo','MarkerFaceColor','b', 'HandleVisibility','off')
				plot(time_vicon(end_time_index), Geometric_omega_x_filtered(end_time_index),'ro','MarkerFaceColor','r', 'HandleVisibility','off')
				plot(time_vicon(end_time_index), Geometric_omega_x_filtered(end_time_index),'go','MarkerFaceColor','g', 'HandleVisibility','off')
				plot(time_vicon(end_time_index), Geometric_omega_x_filtered(end_time_index),'bo','MarkerFaceColor','b', 'HandleVisibility','off')


				xlabel('Time (s)')
				ylabel('rad/s')
				legend('wx (Geom)','wy (Geom)','wz (Geom)')
				xlim([time_vicon(start_delay_time_index)+0., time_vicon(start_delay_time_index)+0.3])
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
				% 
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
		end

		%% Post processing : Estimation of the force and torque

		% Post processing
		scale = 1e3;
		thrust_bound = 3; % 3 mN
		torque_bound = 3; % 3 mNmm
		g = 9.8;
		m = 90*1e-6;
		Ixx = 1.42*1e-9;
		Iyy = 1.34*1e-9;
		Izz = 0.45*1e-9;

		I_moment = eye(3);
		I_moment(1,1) = Ixx;
		I_moment(2,2) = Iyy;
		I_moment(3,3) = Izz;
		e1 = [1;0;0];
		e2 = [0;1;0];
		e3 = [0;0;1];

		% get avg rotation matrix
		Rotation_avg = quat2rotm([quat_0_avg,quat_1_avg,quat_2_avg,quat_3_avg]);
		R_avg_e3 = zeros(3,length(time_vicon));

		for i=1:length(time_vicon)
				R_avg_e3(:,i) = squeeze(Rotation_avg(:,:,i))*e3;
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
		Geometric_omega_filtered = [Geometric_omega_x_filtered,Geometric_omega_y_filtered,Geometric_omega_z_filtered];
		w_hat_I_moment_w = zeros(3,length(time_vicon));

		for i = 1:length(time_vicon)
				w_hat_I_moment_w(:,i) = hat_operation(Geometric_omega_filtered(i,:))*I_moment*Geometric_omega_filtered(i,:)';
		end

		% Estimate the thrust and torque
		y_thrust = diag(R_avg_e3'*(m*acceleration_v'+m*g*e3*ones(1,length(time_vicon))));
		y_torque_x = diag((e1*ones(1,length(time_vicon)))'*(I_moment*acceleration_w'+w_hat_I_moment_w));
		y_torque_y = diag((e2*ones(1,length(time_vicon)))'*(I_moment*acceleration_w'+w_hat_I_moment_w));
		y_torque_z = diag((e3*ones(1,length(time_vicon)))'*(I_moment*acceleration_w'+w_hat_I_moment_w));
		y_torque_x_coriolis = diag((e1*ones(1,length(time_vicon)))'*(w_hat_I_moment_w));
		y_torque_y_coriolis = diag((e2*ones(1,length(time_vicon)))'*(w_hat_I_moment_w));
		y_torque_z_coriolis = diag((e3*ones(1,length(time_vicon)))'*(w_hat_I_moment_w));


		% average the torque
		y_thrust_average(index_exp) = mean(y_thrust(start_index:end_index));
		y_torque_x_average(index_exp) = mean(y_torque_x(start_index:end_index));
		y_torque_y_average(index_exp) = mean(y_torque_y(start_index:end_index));
		y_torque_z_average(index_exp) = mean(y_torque_z(start_index:end_index));

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

		% Pause to visualize data
		if figure_state_flag==1
			pause;
			close all
		end

		left_voltage_p2p(index_exp) = drv_amp_output(end_index)+2*drv_roll_output(end_index);
		right_voltage_p2p(index_exp) = drv_amp_output(end_index)-2*drv_roll_output(end_index);
		left_voltage_offset(index_exp) = drv_pitch_left_output(end_index);
		% right_voltage_offset(index_exp) = drv_pitch_left_output(end_index);
		right_voltage_offset(index_exp) = drv_pitch_right_output(end_index);
		a2_yaw(index_exp) = a2_output(end_index);

end


% %% Optimization based system identification
% rho = 1.2041 % 1.2041 kg/m^3
% beta_geometry = 1.1462e-9
% 
% A = 1/4*rho*beta_geometry;
% 
% % Stage 1: find the Lift gain for left and right wing
% y_1_all = y_thrust_average(thrust_roll_exp_index_list);
% U_1n_all = [u1(thrust_roll_exp_index_list)', u2(thrust_roll_exp_index_list)'];
% delta_opt = 1/A*inv(U_1n_all'*U_1n_all)*U_1n_all'*y_1_all';
% 
% delta_opt_L = delta_opt(1);
% delta_opt_R = delta_opt(2);
% 
% estimated_y_1_all = A*delta_opt'*U_1n_all'*scale;
% % Stage 2: Find the roll gain (related to r_cp) for the left and right wing;
% 
% y_2_all = y_torque_x_average(thrust_roll_exp_index_list);
% U_delta_all = [u1(thrust_roll_exp_index_list)'*delta_opt_L, -u2(thrust_roll_exp_index_list)'*delta_opt_R];
% gamma_opt = 1/A*inv(U_delta_all'*U_delta_all)*U_delta_all'*y_2_all';
% 
% gamma_opt_L = gamma_opt(1);
% gamma_opt_R = gamma_opt(2);
% 
% estimated_roll_all = A*(delta_opt'.*gamma_opt'.*[1, -1])*U_1n_all'*scale^2;
% 
% % Stage 3 : Find the pitch offset
% % I_pitch = find((roll_offset_list==-4 & Phi_ammp_p2p_list==46) | (roll_offset_list==-5 & Phi_ammp_p2p_list==48) | (roll_offset_list==-6 & Phi_ammp_p2p_list==50) );
% % % start_pitch_exp_index =1;
% % % end_pitch_exp_index
% % pitch_exp_index_list = I_pitch;
% 
% % length_pitch_exp = length(y_torque_y_average)-start_pitch_exp_index+1;
% y_3_all = y_torque_y_average(pitch_exp_index_list);
% % U_34n_all = [u3',u4'];
% U_delta_gamma_all = [(u1(pitch_exp_index_list)'.*u3(pitch_exp_index_list)')*delta_opt_L*gamma_opt_L+(u2(pitch_exp_index_list)'.*u4(pitch_exp_index_list)')*delta_opt_R*gamma_opt_R];
% U_delta_gamma_offset = [u1(pitch_exp_index_list)'*delta_opt_L*gamma_opt_L+u2(pitch_exp_index_list)'*delta_opt_R*gamma_opt_R];
% U_delta_gamma_total = [U_delta_gamma_all, U_delta_gamma_offset];
% % for jj =1:32-21+1
% %     y_delta_gamma_all(jj) = A*U_delta_gamma_all(jj,:)*[u3(21+jj-1);u4(21+jj-1)];
% % end
% phi_opt = 1/A*inv(U_delta_gamma_total'*U_delta_gamma_total)*U_delta_gamma_total'*y_3_all';
% 
% phi_opt_L = phi_opt(1);
% phi_opt_R = phi_opt(1);
% phi_opt_offset_L = phi_opt(2);
% phi_opt_offset_R = phi_opt(2);
% 
% estimated_pitch_all_in = A*U_delta_gamma_all*[phi_opt_L]*scale^2;
% estimated_pitch_offset_all = A*U_delta_gamma_offset*[phi_opt_offset_L]*scale^2;
% estimated_pitch_all = estimated_pitch_all_in+estimated_pitch_offset_all
% % estimated_pitch_all = A*(delta_opt'.*gamma_opt'.*[phi_opt_L, phi_opt_R]*(U_1n_all(start_pitch_exp_index:end,:)'.*U_34n_all(start_pitch_exp_index:end,:)')...
% %                          +delta_opt'.*gamma_opt'.*[phi_opt_offset_L, phi_opt_offset_R]*(U_1n_all(start_pitch_exp_index:end,:)'))*scale^2;
% % estimated_pitch_offset_all = A*(delta_opt'.*gamma_opt'.*[phi_opt_offset_L, phi_opt_offset_R]*(U_1n_all(start_pitch_exp_index:end,:)'))*scale^2;
% 
% 
% 
% % Inverse mapping ( From Thrust/ torque to Stroke amplitude/ offset angle
% 
% T_thrust_roll = A*[ delta_opt_L              ,               delta_opt_R;...
%                     delta_opt_L*gamma_opt_L  , - delta_opt_R*gamma_opt_R];
% T_pitch_thrust = A*[delta_opt_L*gamma_opt_L  , delta_opt_R*gamma_opt_R].*[phi_opt_L, phi_opt_R];
% T_pitch_offset = A*[delta_opt_L*gamma_opt_L  , delta_opt_R*gamma_opt_R].*[phi_opt_offset_L, phi_opt_offset_R];
% 
% params_force_wing.delta_opt_L = delta_opt_L;
% params_force_wing.delta_opt_R = delta_opt_R;
% params_force_wing.gamma_opt_L = gamma_opt_L;
% params_force_wing.gamma_opt_R = gamma_opt_R;
% params_force_wing.phi_opt_L = phi_opt_L;
% params_force_wing.phi_opt_R = phi_opt_R;
% params_force_wing.phi_opt_offset_L = phi_opt_offset_L;
% params_force_wing.phi_opt_offset_R = phi_opt_offset_R;
% params_force_wing.A =A;
% 
% desired_thrust = 1.2/scale;
% desired_roll_torque = -0.4/(scale^2);
% desired_pitch_torque = -0.2/(scale^2);
% 
% [u1_desired, u2_desired, u3_desired]=Wing_to_Force(desired_thrust, desired_roll_torque, desired_pitch_torque, params_force_wing);
% 
% desired_p2p_left = sqrt(u1_desired)
% desired_p2p_right = sqrt(u2_desired)
% 
% 
% [drv_amp_desired, drv_roll_desired, drv_pitch_left_desired, drv_pitch_right_desired]=Wing_traj_to_Voltage(desired_p2p_left,desired_p2p_right,u3_desired,left_mapping,right_mapping);
% 
% left_voltage_p2p_desired = drv_amp_desired+2*drv_roll_desired
% right_voltage_p2p_desired = drv_amp_desired-2*drv_roll_desired
% left_voltage_pitch_desired = drv_pitch_left_desired
% right_voltage_pitch_desired = drv_pitch_right_desired


% Plot
% figure(7); 
% subplot(1,3,1)
% plot(roll_offset_list(1:end), y_thrust_average*scale,'ro','MarkerFaceColor','r', 'HandleVisibility','off');
% xlabel('roll offset (deg)')
% ylabel('Average Thrust (mN)')
% subplot(1,3,2)
% plot(roll_offset_list(1:end), y_torque_x_average*scale^2,'ro','MarkerFaceColor','r', 'HandleVisibility','off');
% xlabel('roll offset (deg)')
% ylabel('Average Roll Torque (mNmm)')
% subplot(1,3,3)
% plot(pitch_offset_list(pitch_exp_index_list), y_torque_y_average(pitch_exp_index_list)*scale^2,'ro','MarkerFaceColor','r', 'HandleVisibility','off');
% xlabel('pitch offset (deg)')
% ylabel('Average Pitch Torque (mNmm)')

if save_flag==1
%     save('Force_to_Wing_mapping_no_air_with_yaw_2022.mat', 'params_force_wing');
    save('Patrick Bee Open Loop Data/Open_loop_test_PBee_2023.mat','y_thrust_average','y_torque_x_average', 'y_torque_y_average', 'y_torque_z_average','left_voltage_p2p','right_voltage_p2p','left_voltage_offset','right_voltage_offset','a2_yaw')

end

% 
% % System Identifiation
% figure(8) 
% set(gcf,'color','white')
% 
% subplot(3,3,1)
% scatter(sqrt(u1(thrust_roll_exp_index_list)),sqrt(u2(thrust_roll_exp_index_list)),[],y_1_all*scale,'fill','o')
% c=colorbar;
% c.Label.String= 'Thrust (mN)';
% c.Label.FontSize = 12;
% caxis([0.8, 1.3]);
% % colormap(jet)
% xlabel('Left Stroke p-p (deg)')
% ylabel('right Stroke p-p (deg)')
% xlim([40, 60])
% ylim([40, 56])
% axis equal
% title('Vicon Measured (mN)')
% subplot(3,3,2)
% scatter(sqrt(u1(thrust_roll_exp_index_list)),sqrt(u2(thrust_roll_exp_index_list)),[],estimated_y_1_all,'fill','o')
% c=colorbar;
% c.Label.String= 'Thrust (mN)';
% c.Label.FontSize = 12;
% caxis([0.8, 1.3]);
% % colormap(jet)
% xlabel('Left Stroke p-p (deg)')
% ylabel('right Stroke p-p (deg)')
% xlim([40, 60])
% ylim([40, 56])
% title('Estimation (mN)')
% axis equal
% subplot(3,3,3)
% scatter(sqrt(u1(thrust_roll_exp_index_list)),sqrt(u2(thrust_roll_exp_index_list)),[],y_1_all*scale-estimated_y_1_all,'d')
% c=colorbar;
% c.Label.String= 'Thrust (mN)';
% c.Label.FontSize = 12;
% caxis([-0.15, 0.15]);
% % colormap(jet)
% xlabel('Left Stroke p-p (deg)')
% ylabel('right Stroke p-p (deg)')
% xlim([40, 60])
% ylim([40, 56])
% title('Difference (mN)')
% axis equal
% 
% subplot(3,3,4)
% scatter(sqrt(u1(thrust_roll_exp_index_list)),sqrt(u2(thrust_roll_exp_index_list)),[],y_2_all*scale^2,'fill','o')
% c=colorbar;
% c.Label.String= 'Roll torque (mNmm)';
% c.Label.FontSize = 12;
% caxis([-0.4, 0.4]);
% xlabel('Left Stroke p-p (deg)')
% ylabel('right Stroke p-p (deg)')
% xlim([40, 60])
% ylim([40, 56])
% axis equal
% 
% subplot(3,3,5)
% scatter(sqrt(u1(thrust_roll_exp_index_list)),sqrt(u2(thrust_roll_exp_index_list)),[],estimated_roll_all,'fill','o')
% c=colorbar;
% c.Label.String= 'Roll torque (mNmm)';
% c.Label.FontSize = 12;
% caxis([-0.4, 0.4]);
% xlabel('Left Stroke p-p (deg)')
% ylabel('right Stroke p-p (deg)')
% xlim([40, 60])
% ylim([40, 56])
% axis equal;
% 
% subplot(3,3,6)
% scatter(sqrt(u1(thrust_roll_exp_index_list)),sqrt(u2(thrust_roll_exp_index_list)),[],y_2_all*scale^2-estimated_roll_all,'d')
% c=colorbar;
% c.Label.String= 'Roll torque (mNmm)';
% c.Label.FontSize = 12;
% caxis([-0.4, 0.4]);
% xlabel('Left Stroke p-p (deg)')
% ylabel('right Stroke p-p (deg)')
% xlim([40, 60])
% ylim([40, 56])
% axis equal;
% 
% % Plot pitch
% subplot(3,3,7)
% plot(u3(pitch_exp_index_list), y_3_all*scale^2, 'ro');
% xlabel('Offset angle (deg)')
% ylabel('Pitch torque (mNmm)')
% % xlim([40, 58])
% ylim([-0.4, 0.4])
% % axis equal
% 
% subplot(3,3,8)
% plot(u3(pitch_exp_index_list), estimated_pitch_all, 'ro');
% xlabel('Offset angle (deg)')
% ylabel('Pitch torque (mNmm)')
% ylim([-0.4, 0.4])
% % xlim([40, 58])
% % ylim([40, 58])
% % axis equal
% subplot(3,3,9)
% plot(u3(pitch_exp_index_list), y_3_all*scale^2-estimated_pitch_all', 'ro');
% xlabel('Offset angle (deg)')
% ylabel('Pitch torque (mNmm)')
% ylim([-0.4, 0.4])
% % xlim([40, 58])
% % ylim([40, 58])
% % axis equal
% 
% 
% % axis equal
