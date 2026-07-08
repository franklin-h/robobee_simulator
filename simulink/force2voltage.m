function [drv_amp, drv_roll, drv_pitch_left, drv_pitch_right, a2_coeff] = ...
    force2voltage(Thrust_desired, Roll_torque_desired, Pitch_torque_desired, ...
    Yaw_torque_desired, params_vec)

% global params_opt

delta_1 = params_vec(1);
delta_2 = params_vec(2);
delta_3 = params_vec(3);
gamma_1 = params_vec(4);
gamma_2 = params_vec(5);
gamma_3 = params_vec(6);
eta = params_vec(7);
nu = params_vec(8);
mu = params_vec(9);


% Initializing for averaging filter at the end to prevent InF
if Thrust_desired==0    
    u4 = 0;
else
    u4 = Pitch_torque_desired/(Thrust_desired*delta_3*gamma_2)-nu;
end


% u3 calculation when tau approaches to zero
if abs(Yaw_torque_desired)<1e-12
    
   u3 = -mu;
else
    % Bounds for the mapping to produce real solution
    lb =(Thrust_desired/2)*(gamma_2*gamma_3*delta_2*mu/delta_1-sqrt((gamma_2*gamma_3*delta_2*Thrust_desired*mu/delta_1)^2+gamma_2*gamma_3));
    ub =(Thrust_desired/2)*(gamma_2*gamma_3*delta_2*mu/delta_1+sqrt((gamma_2*gamma_3*delta_2*Thrust_desired*mu/delta_1)^2+gamma_2*gamma_3));
    
    if Yaw_torque_desired>=ub
       Yaw_torque_desired = ub;
    elseif Yaw_torque_desired<=lb
       Yaw_torque_desired = lb; 
    end

    % u3 calculation when tau approaches zero
    if abs(Yaw_torque_desired) < 1e-12
        u3 = -mu;
    else
        A = gamma_2*gamma_3*Thrust_desired;
        B = gamma_2*gamma_3*delta_2*Thrust_desired*mu/delta_1;
    
        disc = A^2 - 4*Yaw_torque_desired*(Yaw_torque_desired - B);
    
    % Numerical / feasibility guard
    if disc < 0
        disc = 0;
    end
    
    u3 = delta_1*(A - sqrt(disc))/(2*delta_2*Yaw_torque_desired);
    end
end

D = delta_1^2+delta_2^2*u3^2;
% gamma_2*Thrust_desired+Roll_torque_desired 
u1 = (gamma_2*Thrust_desired+Roll_torque_desired)/(2*gamma_1*gamma_2*D*eta);
u2 = (gamma_2*Thrust_desired-Roll_torque_desired)/(2*gamma_1*gamma_2*D);

u1 = max(u1, 0); 
u2 = max(u2, 0); % Guard against negative voltage
vleft_p2p = sqrt(u1)*2; 
vright_p2p = sqrt(u2)*2;
vleft_pitch = u4;
vright_pitch = u4;


drv_amp = abs((vleft_p2p-vright_p2p))/2+min(vleft_p2p,vright_p2p); % p2p
drv_roll = (vleft_p2p-vright_p2p)/4; % Amplitude
drv_pitch_left = vleft_pitch;
drv_pitch_right = vright_pitch;
a2_coeff = u3;

end
