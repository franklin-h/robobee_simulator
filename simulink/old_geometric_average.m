function Rout = fcn(R_prev,R_prev2,R_prev3,R_prev4,R_prev5,R_prev6,R_cur,update,Rout_prev)
%#codegen
% Rot_cur = [R_cur(1), R_cur(2), R_cur(3); R_cur(4), R_cur(5), R_cur(6); R_cur(7), R_cur(8), R_cur(9)];
% Rot_prev = [R_prev(1), R_prev(2), R_prev(3); R_prev(4), R_prev(5), R_prev(6); R_prev(7), R_prev(8), R_prev(9)];
% Rot_prev2 = [R_prev2(1), R_prev2(2), R_prev2(3); R_prev2(4), R_prev2(5), R_prev2(6); R_prev2(7), R_prev2(8), R_prev2(9)];
% Rot_prev3 = [R_prev3(1), R_prev3(2), R_prev3(3); R_prev3(4), R_prev3(5), R_prev3(6); R_prev3(7), R_prev3(8), R_prev3(9)];
% Rot_prev4 = [R_prev4(1), R_prev4(2), R_prev4(3); R_prev4(4), R_prev4(5), R_prev4(6); R_prev4(7), R_prev4(8), R_prev4(9)];
% Rot_prev5 = [R_prev5(1), R_prev5(2), R_prev5(3); R_prev5(4), R_prev5(5), R_prev5(6); R_prev5(7), R_prev5(8), R_prev5(9)];
% Rot_prev6 = [R_prev6(1), R_prev6(2), R_prev6(3); R_prev6(4), R_prev6(5), R_prev6(6); R_prev6(7), R_prev6(8), R_prev6(9)];
% 
% Rout = zeros(9,1);

Rout = zeros(9,1);

R_cur   = reshape(R_cur,9,1);
R_prev  = reshape(R_prev,9,1);
R_prev2 = reshape(R_prev2,9,1);
R_prev3 = reshape(R_prev3,9,1);
R_prev4 = reshape(R_prev4,9,1);
R_prev5 = reshape(R_prev5,9,1);
R_prev6 = reshape(R_prev6,9,1);
Rout_prev = reshape(Rout_prev,9,1);

Rot_cur = reshape(R_cur,3,3).';
Rot_prev = reshape(R_prev,3,3).';
Rot_prev2 = reshape(R_prev2,3,3).';
Rot_prev3 = reshape(R_prev3,3,3).';
Rot_prev4 = reshape(R_prev4,3,3).';
Rot_prev5 = reshape(R_prev5,3,3).';
Rot_prev6 = reshape(R_prev6,3,3).';

if update>0
   Rout=Rout_prev; 
else
% % 1 cycle average
%     n= 3;
%     
%     w_R0 = exp_map(Rot_cur);
%     w_R1 = exp_map(Rot_prev);
%     w_R2 = exp_map(Rot_prev2);
%     
%     
%     if sign(w_R2'*w_R0)<0
%         w_R0=-w_R0;
%     end
%     if sign(w_R2'*w_R1)<0
%         w_R1=-w_R1;
%     end
%    
%     
%     meanR0=frac_rot(w_R0,n);
%     meanR1=frac_rot(w_R1,n);
%     meanR2=frac_rot(w_R2,n);
%    
%     mean_R = meanR2*meanR1*meanR0;

% 1 cycle average
    n= 4;
    
    w_R0 = exp_map(Rot_cur);
    w_R1 = exp_map(Rot_prev);
    w_R2 = exp_map(Rot_prev2);
    w_R3 = exp_map(Rot_prev3);
    
    
    if sign(w_R3'*w_R0)<0
        w_R0=-w_R0;
    end
    if sign(w_R3'*w_R1)<0
        w_R1=-w_R1;
    end
    if sign(w_R3'*w_R2)<0
        w_R2=-w_R2;
    end
    
    meanR0=frac_rot(w_R0,n);
    meanR1=frac_rot(w_R1,n);
    meanR2=frac_rot(w_R2,n);
    meanR3=frac_rot(w_R3,n);

    mean_R = meanR3*meanR2*meanR1*meanR0;

% % 2 cycle average
%     n=7;
%     w_R0 = exp_map(Rot_cur);
%     w_R1 = exp_map(Rot_prev);
%     w_R2 = exp_map(Rot_prev2);
%     w_R3 = exp_map(Rot_prev3);
%     w_R4 = exp_map(Rot_prev4);
%     w_R5 = exp_map(Rot_prev5);
%     w_R6 = exp_map(Rot_prev6);
%     
%     if sign(w_R6'*w_R0)<0
%         w_R0=-w_R0;
%     end
%     if sign(w_R6'*w_R1)<0
%         w_R1=-w_R1;
%     end
%     if sign(w_R6'*w_R2)<0
%         w_R2=-w_R2;
%     end
%     if sign(w_R6'*w_R3)<0
%         w_R3=-w_R3;
%     end
%     if sign(w_R6'*w_R4)<0
%         w_R4=-w_R4;
%     end
%     if sign(w_R5'*w_R5)<0
%         w_R5=-w_R5;
%     end
%     meanR0=frac_rot(w_R0,n); 
%     meanR1=frac_rot(w_R1,n);
%     meanR2=frac_rot(w_R2,n);
%     meanR3=frac_rot(w_R3,n);
%     meanR4=frac_rot(w_R4,n);
%     meanR5=frac_rot(w_R5,n);
%     meanR6=frac_rot(w_R6,n);
% 
% %     mean_R = meanR3*meanR2*meanR1*meanR0;
%     mean_R = meanR6*meanR5*meanR4*meanR3*meanR2*meanR1*meanR0;
    
    Rout = [mean_R(1,1); mean_R(1,2); mean_R(1,3); mean_R(2,1); mean_R(2,2); mean_R(2,3); mean_R(3,1); mean_R(3,2); mean_R(3,3)]; 
end

end

function w = exp_map(R)

tr_R_diff = trace(R);
% isreal(tr_R_diff)
if abs((tr_R_diff-1)/2)<=1
    theta = acos((tr_R_diff-1)/2);
else
    theta=acos((tr_R_diff-1)/2/abs((tr_R_diff-1)/2));
end
if theta==0
    w= [0;0;0];
else
    w = theta/(2*sin(theta))*[R(3,2)-R(2,3); R(1,3) - R(3,1); R(2,1)-R(1,2)];
end
end

function fractional_rot = frac_rot(w,n)

w_hat = zeros(3,3);
w_hat = [0,-w(3),w(2);...
          w(3),0,-w(1);...
          -w(2), w(1), 0]/n;
fractional_rot = expm(w_hat);
end
% mean_R = R_prev_prev^(1/3)*R_prev^(1/3)*R_cur^(1/3);

% R is 1x9

      