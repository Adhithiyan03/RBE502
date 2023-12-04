function [value,isterminal,direction] = interceptdrone(t, z, epsilon)

value = norm( z([5, 6]) - z([1, 2]) ) > epsilon;
isterminal = 1; % = 1 -> the integration is to terminate.
direction = 0;

end