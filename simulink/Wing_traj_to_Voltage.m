function [drv_amp, drv_roll, drv_pitch_left, drv_pitch_right] = Wing_traj_to_Voltage(Phi_left_p2p_command,Phi_right_p2p_command,Phi_pitch_command,left_mapping,right_mapping)

% Mapping for Voltage peak to peak from desired Stroke peak to peak
p1_left = left_mapping(1); % V_left_p2p = p1_left*Phi_p2p + p2_left
p2_left = left_mapping(2);
p1_right = right_mapping(1); % V_right_p2p = p1_right*Phi_p2p + p2_right
p2_right = right_mapping(2);

% Mapping for Voltage offset to peak from desired Stroke peak to peak
p1_pitch_left = left_mapping(3); % vleft_pitch = p1_pitch_left*Phi_pitch_command + p2_pitch_left
p2_pitch_left = left_mapping(4);
p1_pitch_right = right_mapping(3); % vright_pitch = p1_pitch_right*Phi_pitch_command + p2_pitch_right
p2_pitch_right = right_mapping(4);

vleft_p2p = p1_left*Phi_left_p2p_command + p2_left;
vright_p2p = p1_right*Phi_right_p2p_command + p2_right;
vleft_pitch = p1_pitch_left*Phi_pitch_command + p2_pitch_left;
vright_pitch = p1_pitch_right*Phi_pitch_command + p2_pitch_right;


drv_amp = abs((vleft_p2p-vright_p2p))/2+min(vleft_p2p,vright_p2p); % p2p
drv_roll = (vleft_p2p-vright_p2p)/4; % Amplitude
drv_pitch_left = vleft_pitch;
drv_pitch_right = vright_pitch;

end
