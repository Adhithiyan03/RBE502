function dz = augmentedSystem(t, z, dz, uav_dyn, u, K, p, r, n)

% Decouple r and b states from the augmented state z
z_t = z(1:12, 1);
y_t = z(13:15, 1);

zd = [y_t; zeros(3,1); uav_dyn(t, y_t), zeros(3,1)];

ud = [1 1 1 1]'*m*g/4;
%change from uav dyn to prediction values
dz = [quadrotor(t, z_t, u(z, zd, ud, K), p, r, n) ; uav_dyn(t, y_t)];

end