function [value,isterminal,direction] = interceptdrone(t, z, epsilon, threshold)

value = norm( z(13:15,1) - z(1:3,1) ) > epsilon & all((z(13:15,1) - threshold) < 0);
isterminal = 1; % = 1 -> the integration is to terminate.
direction = 0;

end