
function [value,isterminal,direction] = interceptdrone(t, z, epsilon, threshold)

l_limit = threshold(:, 1);
r_limit = threshold(:, 2);

value = (norm( z(13:15,1) - z(1:3,1) ) > epsilon) & all(z(13:15,1) > l_limit) & all(r_limit > z(13:15,1));

isterminal = 1; % = 1 -> the integration is to terminate.
direction = 0;

end