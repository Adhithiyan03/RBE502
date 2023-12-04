function [tk, c] = dist(tspan, tau, gamma)

% tspan: The domain of the piecewise constant function 
% tau = [tau_min, tau_max]: minimum and maximum time intervals
% gamma: Maximum norm of the c at every t

dim = 2;    % The dimension of c at every t
p = 2;      % p-norm of c at every t. p = 2 -> l2 norm or Euclidean norm

delta_t = diff(tau);

tk = tspan(1);
while(tk(end) < tspan(2))
    tk(end+1,1) = tk(end) + delta_t*rand + tau(1);
end

N = length(tk);
c = 2*rand(N-1, dim) - 1;
c = gamma*rand(N-1, 1).*(c./vecnorm(c,p,2));
end