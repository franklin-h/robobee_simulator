function [roll_c,pitch_c,yaw_c] = drake2controller(alpha,beta,gamma) 
% This function converts the drake axes to controller axes convention 
    S = [ 0  1  0;
         -1  0  0;
          0  0  1];

    % MATLAB ZYX convention uses [yaw pitch roll]
    R_p = eul2rotm([gamma, beta, alpha], 'ZYX');

    % Same physical attitude, expressed in controller axes
    R_c = S * R_p * S';

    eul_c = rotm2eul(R_c, 'ZYX');

    yaw_c   = eul_c(1);
    pitch_c = eul_c(2);
    roll_c  = eul_c(3);
end 