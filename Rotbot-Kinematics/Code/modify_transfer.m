function T = modify_transfer(alpha_L, a_L, d_N, theta_N)
    T = [cosd(theta_N) -sind(theta_N) 0 a_L;
         sind(theta_N)*cosd(alpha_L) cosd(theta_N)*cosd(alpha_L) -sind(alpha_L) -sind(alpha_L)*d_N;
         sind(theta_N)*sind(alpha_L) cosd(theta_N)*sind(alpha_L) cosd(alpha_L) cosd(alpha_L)*d_N;
         0 0 0 1];
end
