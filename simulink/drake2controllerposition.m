function [x_c,y_c,z_c] = drake2controllerposition(x,y,z)
% Convert a position from Drake root axes to controller axes.
% Controller +x = Drake +y, controller +y = Drake -x, controller +z = Drake +z.
    S = [ 0  1  0;
         -1  0  0;
          0  0  1];
    p_c = S * [x; y; z];
    x_c = p_c(1);
    y_c = p_c(2);
    z_c = p_c(3);
end
