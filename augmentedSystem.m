function [dz, vel] = augmentedSystem(t, z, uav_dyn, u, K, p, r, n, uav_traj)

% Decouple r and b states from the augmented state z
z_t = z(1:12, 1);
y_t = z(13:15, 1);

for i = 1:length(uav_traj)
    if uav_traj(i,:) == y_t
        temp = uav_traj(i-1,:);
        disp("me")
    end
end

zd = [y_t; zeros(3,1); uav_dyn(t); zeros(3,1)];
vel = y_t - uav_dyn(t-1/2000);
% zd = [y_t ; zeros(3,1); zeros(3,1); zeros(3,1)];

ud = [1 1 1 1]'*p(1)*p(3)/4;
%need to change from uav dyn to prediction values
dz = [quadrotor(t, z_t, u(z_t, zd, ud, K), p, r, n) ; uav_dyn(t)];

end