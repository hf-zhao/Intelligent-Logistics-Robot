function T = standard_transfer(alpha_L, a_L, d_N, theta_N)
    T = [cos(theta_N) -sin(theta_N) 0 a_L;
         sin(theta_N)*cos(alpha_L) cos(theta_N)*cos(alpha_L) -sin(alpha_L) -sin(alpha_L)*d_N;
         sin(theta_N)*sin(alpha_L) cos(theta_N)*sin(alpha_L) cos(alpha_L) cos(alpha_L)*d_N;
         0 0 0 1];
end
