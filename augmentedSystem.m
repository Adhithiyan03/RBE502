function [dz, vel] = augmentedSystem(t, z, uav_dyn, u, K, p, r, n, uav_traj, tspan_I)

% Decouple r and b states from the augmented state z
z_t = z(1:12, 1);
y_t = z(13:15, 1);

disp(uav_traj(t) - y_t)
% zd = [y_t; zeros(3,1); uav_dyn(t); zeros(3,1)];
vel = y_t - uav_dyn(t-1/2000);
zd = [y_t ; zeros(3,1); (uav_traj(t) - uav_traj(t-0.001))/0.001; zeros(3,1)];

ud = [1 1 1 1]'*p(1)*p(3)/4;
%need to change from uav dyn to prediction values
dz = [quadrotor(t, z_t, u(z_t, zd, ud, K), p, r, n) ; uav_dyn(t)];

end