% State vector definition
%
%      x1, x2, x3, phi, theta, psi, dx1, dx2, dx3, omega1, omega2, omega3
% z = [z1, z2, z3,  z4,    z5,  z6,  z7,  z8,  z9,    z10,    z11,    z12]
%
% Parameter vector definition
%
%       g,  l,  m, I11, I22, I33, mu, sigma
% p = [p1, p2, p3,  p4,  p5,  p6, p7,    p8]

clc; clear all; close all;


syms z1 z2 z3 z4 z5 z6 z7 z8 z9 z10 z11 z12 'real'
syms g l m I11 I22 I33 mu sigma 'positive'
syms u1 u2 u3 u4 'positive' % Assuming 'positive', instead of 'real' to
                            % simplify the saturation by 0 in the 
                            % quadrotor.m file.
syms x1 x2 x3 'real'


z = [z1; z2; z3; z4; z5; z6; z7; z8; z9; z10; z11; z12];
u = [u1; u2; u3; u4];

p = [g l m I11 I22 I33 inf sigma]; % mu = inf eliminates saturation of u

dz = quadrotor(0, z, u, p, [0;0;0], [0;0;0]);


z0 = [x1; x2; x3; zeros(9,1)];
u0 = [1 1 1 1]'*m*g/4;


%% Check if (z0, u0) is an equilibrium point

equlibriumCheck = simplify(subs(dz,[z; u],[z0; u0]));

fprintf('f(z0, u0) = \n');
pretty(equlibriumCheck);

if(norm(equlibriumCheck,inf) == 0)
    fprintf('=> point (z0, u0) is an equilibrium point.\n');
else
    fprintf('=> point (z0, u0) is NOT an equilibrium point.\n');
end
fprintf('\n\n');

%% Find A and B matrices

A = simplify(subs(jacobian(dz,z), [z; u], [z0; u0]));
B = simplify(subs(jacobian(dz,u), [z; u], [z0; u0]));

%% Check controllability

Ctrl = sym(zeros(length(z), length(z)*length(u))); 

for k=0:(size(A,1)-1)
    Ctrl( : ,  size(B,2)*k + 1 : size(B,2)*(k+1)) = A^k * B;
end

r = rank(Ctrl);
fprintf('rank(Ctrl) = %d => ', r);

if(r == length(z))
    fprintf('System is controllable.\n');
else
    fprintf('System is NOT controllable.\n');
end

