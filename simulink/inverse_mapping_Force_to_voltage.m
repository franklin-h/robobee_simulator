function [u1, u2, u3, u4] = inverse_mapping_Force_to_voltage(Thrust_desired, Roll_torque_desired, Pitch_torque_desired, Yaw_torque_desired, params_opt)

delta_1 = params_opt.delta_1;
delta_2 = params_opt.delta_2;
delta_3 = params_opt.delta_3;
gamma_1 = params_opt.gamma_1;
gamma_2 = params_opt.gamma_2;
gamma_3 = params_opt.gamma_3;
eta = params_opt.eta;
nu = params_opt.nu;
mu = params_opt.mu;


u4 = Pitch_torque_desired/(Thrust_desired*delta_3*gamma_2)-nu;

if abs(Yaw_torque_desired)<1e-12
    
   u3 = -mu;
else
    lb =(Thrust_desired/2)*(gamma_2*gamma_3*delta_2*mu/delta_1-sqrt((gamma_2*gamma_3*delta_2*Thrust_desired*mu/delta_1)^2+gamma_2*gamma_3));
    ub =(Thrust_desired/2)*(gamma_2*gamma_3*delta_2*mu/delta_1+sqrt((gamma_2*gamma_3*delta_2*Thrust_desired*mu/delta_1)^2+gamma_2*gamma_3));
   
   if Yaw_torque_desired>=ub
       Yaw_torque_desired = ub;
   elseif Yaw_torque_desired<=lb
       Yaw_torque_desired = lb; 
   end
       
   u3 = delta_1*(gamma_2*gamma_3*Thrust_desired-sqrt((gamma_2*gamma_3*Thrust_desired)^2-4*Yaw_torque_desired*(Yaw_torque_desired-gamma_2*gamma_3*delta_2*Thrust_desired*mu/delta_1)))/(2*delta_2*Yaw_torque_desired);

end

D = delta_1^2+delta_2^2*u3^2;
u1 = (gamma_2*Thrust_desired+Roll_torque_desired)/(2*gamma_1*gamma_2*D*eta);
u2 = (gamma_2*Thrust_desired-Roll_torque_desired)/(2*gamma_1*gamma_2*D);

end