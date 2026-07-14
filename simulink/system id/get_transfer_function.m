function [H_mag,H_phase] = ...
                        get_transfer_function(m_a,J_phi,T,r_cp,b,k_a,k_t)

m_eq = m_a+T^2*J_phi;
b_eq = T^2*r_cp*b;
k_eq = k_a+T^2*k_t;

H_mag = @(omega) 1./sqrt((k_eq-omega.^2.*m_eq).^2+(b_eq.*omega).^2);
H_phase = @(omega) -atan2((b_eq.*omega),(k_eq-omega.^2.*m_eq));

end