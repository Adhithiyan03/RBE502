function [value,isterminal,direction] = interceptdrone(t, z, epsilon)

value = norm( z(13:15,1) - z(1:3,1) ) > epsilon;
isterminal = 1; % = 1 -> the integration is to terminate.
direction = 0;

end