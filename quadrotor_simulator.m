%% Initializations
clc; clear; close all;

g = 9.81;   % The gravitational acceleration [m/s^2]
l =  0.2;   % Distance from the center of mass to each rotor [m]
m =  0.5;   % Total mass of the quadrotor [kg]
I = [1.24, 1.24, 2.48]; % Mass moment of inertia [kg m^2]. DIAGONAL of 3x3
mu = 3.0;   % Maximum thrust of each rotor [N]
sigma = 0.01; % The proportionality constant relating thrust to torque [m]


p = [g l m I mu sigma];


%% Initial conditions
r = [0; 0; 0];
n = [0; 0; 0];

% State Vectors - start and final:
z0 = [0; 0; 0; zeros(9,1)];       % starting pose

u0 = [1 1 1 1]'*m*g/4; % Initial input - drone hovering

%UAV position
y0 = [4 2 5]';

%UAV Kinematics
uav_dyn = @(t,y) [4; 2*sin(t); 5];

%Initial augmented state vector
z0_I = [z0; y0];

%% LQR Test - using MATLAB LQR:

% Matrices:
A = [0, 0, 0,  0, 0, 0, 1, 0, 0, 0, 0, 0;
     0, 0, 0,  0, 0, 0, 0, 1, 0, 0, 0, 0;
     0, 0, 0,  0, 0, 0, 0, 0, 1, 0, 0, 0;
     0, 0, 0,  0, 0, 0, 0, 0, 0, 1, 0, 0;
     0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 1, 0;
     0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 1;
     0, 0, 0,  0, g, 0, 0, 0, 0, 0, 0, 0;
     0, 0, 0, -g, 0, 0, 0, 0, 0, 0, 0, 0;
     0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0;
     0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0;
     0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0;
     0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0];
B = [   0,      0,      0,      0;
        0,      0,      0,      0;
        0,      0,      0,      0;
        0,      0,      0,      0;
        0,      0,      0,      0;
        0,      0,      0,      0;
        0,      0,      0,      0;
        0,      0,      0,      0;
      1/m,    1/m,    1/m,    1/m;
        0, l/I(1),      0,-l/I(1);
  -l/I(2),      0, l/I(2),      0;
  sigma/I(3),-sigma/I(3),sigma/I(3),-sigma/I(3)];


% dz = quadrotor(0, z0, u0, p, r, n);
%B = simplify(subs(jacobian(dz,u), [z; u], [z0; u0]));

Q = 0.9*eye(12);
R = 1/(mu)*eye(4);
C = eye(12,12);
D = zeros(12,4);
%A =
% syms z1 z2 z3 z4 z5 z6 z7 z8 z9 z10 z11 z12 r1 r2 r3 n1 n2 n3 'real'
% syms u1 u2 u3 u4 'positive'
% zp = [z1; z2; z3; z4; z5; z6; z7; z8; z9; z10; z11; z12];
% up = [u1; u2; u3; u4];
% dz = quadrotor(0, zp, up, p, [r1;r2;r3], [n1;n2;n3]);
% Au = @(t,z) (subs(jacobian(dz,zp), [zp; up], [z0; u0]));
% Bu = @(t,z) (subs(jacobian(dz,up), [zp; up], [z0; u0]));
% G = Au(100)
% H = Bu(100)


%poles = eig(A - inv(R)*B'*S);
%N = zeros(12,4)
syscl = ss(A,B,C,D);
[K,S,pole] = lqr(syscl, Q, R)
%poles = [-2 -0.5+1i -0.5+1i -0.2+0.1i -0.2-0.1i -0.4-0.2i -0.08 -0.3-0.23i -0.8 + 0.4i -1.2 -1.1 -1.5];
%Pcl = pole(syscl)
%K = place(A,B,pole)

%zd = @(t) [2*cos(t); 2*sin(t); min(t,7); 0; zeros(8,1)];    % desired pose for test
% zd = [4; 0; 5; 0; zeros(8,1)];

% Inputs:
% ud = [1 1 1 1]'*m*g/4;
%u  = @(t, z) ud + K*(zd(t) - z);
u  = @(z, zd, ud, K) ud + K*(zd - z);
% Problem parameters
epsilon = 0.5*p(2);

%% Phase I: Pursue
tspan_I = [0 10];
% t = linspace(0, 20, 300);
options = odeset('Event', @(t,z) interceptdrone(t, z, epsilon),...
    'RelTol', 1e-6);

[t_I, z_I, te, ze] = ode45(@(t, z) augmentedSystem(t, z, quadrotor(t, z, u(z, [z(13:15, 1); uav_dyn(t, y)], ud, K), p, r, n), uav_dyn, u, K),...
    tspan_I, z0_I, options);

% %% Solving the initial-value problem
% 
% 
% 
% [t,z] = ode45(@(t,z) quadrotor(t, z, u(t,z), p, r, n), t, z0);                                                                                                                                                              zr = @(t,z) zd(t) - z

%% Phase II: Return

% if(isempty(te)) % if the robot failed to catch the bug
%     % Decoupling the augmented state vector z from only phase I
%     t = t_I;
%     z = z_I(:,1:4);
%     y = z_I(:,[5, 6]);
% else
%     tspan_II = [te, tspan_I(end)];
%     r0_II = z_I(end,1:4)';
% 
%     [tk, c] = dist(tspan_II, [0.1 0.5], 2);
%     g = @(t)[0; 0; c(find(tk <= t, 1, 'last'),:)'];
% 
%     rd = zeros(4,1); ud = zeros(2,1);
% 
%     [t_II, r_II] = ode45(@(t,r) dr(t, r, u(r, rd, ud, K), g(t) ),...
%         tspan_II, r0_II);
% 
% 
%     % Decoupling the augmented state vector z from phase I and phase II
%     t = [t_I; t_II];
%     r = [z_I(:,1:4); r_II];
%     b = [z_I(:,[5, 6]); r_II(:,[1,2])]; % Since the bug is captured by the robot,
%                                   % bug's states are equal to the robot's.
% end

%% Plotting the results

for i=1:4
    ax(i) = subplot(2,2,i,'NextPlot','Add','Box','on','XGrid','on','YGrid','on',...
                'Xlim',[t(1), t(end)],...
                'TickLabelInterpreter','LaTeX','FontSize',14);
    xlabel('t','Interpreter','LaTeX','FontSize',14);        
end


plot(ax(1), t,z(:,1:3), 'LineWidth', 1.5);
legend(ax(1), {'$x_1$', '$x_2$', '$x_3$'},... 
    'Interpreter', 'LaTeX', 'FontSize', 14);
title(ax(1), '${\bf x}$','Interpreter','LaTeX','FontSize',14);
xlabel(ax(1), 't','Interpreter','LaTeX','FontSize',14);

plot(ax(3), t, z(:,4:6), 'LineWidth', 1.5);
legend(ax(3), {'$\phi$', '$\theta$', '$\psi$'},...
    'Interpreter', 'LaTeX', 'FontSize', 14);
title(ax(3), '\boldmath$\alpha$','Interpreter','LaTeX','FontSize',14);

plot(ax(2), t, z(:,7:9), 'LineWidth', 1.5);
legend(ax(2), {'$\dot{x}_1$', '$\dot{x}_2$', '$\dot{x}_3$'},...
    'Interpreter', 'LaTeX', 'FontSize', 14);
title(ax(2), '$\dot{\bf x}$','Interpreter','LaTeX','FontSize',14);

plot(ax(4), t, z(:,10:12), 'LineWidth', 1.5);
legend(ax(4), {'$\omega_1$', '$\omega_2$', '$\omega_3$'},...
    'Interpreter', 'LaTeX', 'FontSize', 14);
title(ax(4), '\boldmath$\omega$','Interpreter','LaTeX','FontSize',14);



%% Animation
animation_fig = figure;

airspace_box_length = 10;

animation_axes = axes('Parent', animation_fig,...
    'NextPlot','add','DataAspectRatio',[1 1 1],...
    'Xlim',airspace_box_length*[-1 1],...
    'Ylim',airspace_box_length*[-1 1],...
    'Zlim',airspace_box_length*[0 1],...
    'box','on','Xgrid','on','Ygrid','on','Zgrid','on',...
    'TickLabelInterpreter','LaTeX','FontSize',14);

view(animation_axes, 3);

% Editing the Sim-Drone shape:
circleRadius = 2;
bodySize = 1.2;
droneLineWidth = 1;

N = 10;
Q = linspace(0,2*pi,N)';
circle = circleRadius*l*[cos(Q) sin(Q) zeros(N,1)];
loc = bodySize*[1 0 0; 0 1 0; -1 0 0; 0 -1 0];


silhouette = plot3(0,0,0, '--', 'Color', 0.5*[1 1 1], 'LineWidth', 1 ,...
    'Parent', animation_axes);
body = plot3(0,0,0, 'Color',lines(1), 'LineWidth', droneLineWidth,...
        'Parent', animation_axes);
uav = plot3(0, 0, 0, 'Color', lines(1), 'LineWidth', droneLineWidth,...
        'Parent', animation_axes);
for i=1:4
    rotor(i) = plot3(0,0,0, 'Color', lines(1), 'LineWidth', droneLineWidth,...
        'Parent', animation_axes);
end
tic;
for k=1:length(t)
    
    %f(k,1:12) = zd(k)' - z(k,:);
    %f(k,1:12) = zd' - z(k,:);
    
    R = [ cos(z(k,5))*cos(z(k,6)), sin(z(k,4))*sin(z(k,5))*cos(z(k,6)) - cos(z(k,4))*sin(z(k,6)), sin(z(k,4))*sin(z(k,6)) + cos(z(k,4))*sin(z(k,5))*cos(z(k,6));
          cos(z(k,5))*sin(z(k,6)), cos(z(k,4))*cos(z(k,6)) + sin(z(k,4))*sin(z(k,5))*sin(z(k,6)), cos(z(k,4))*sin(z(k,5))*sin(z(k,6)) - sin(z(k,4))*cos(z(k,6));
                     -sin(z(k,5)),                                 sin(z(k,4))*cos(z(k,5)),                                 cos(z(k,4))*cos(z(k,5))];
    for i=1:4
        ctr(i,:) = z(k,1:3) + loc(i,:)*R';
        pose = ones(N,1)*z(k,1:3) + (ones(N,1)*loc(i,:) + circle)*R';
        set(rotor(i), 'XData', pose(:,1), 'YData', pose(:,2),  'ZData', pose(:,3) );
         
    end
    set(silhouette,'XData', [0, z(k,1), z(k,1), z(k,1)],...
        'YData', [0, 0, z(k,2), z(k,2)],...
        'ZData', [0, 0, 0, z(k,3)]);
    set(body, 'XData', [ctr([1 3],1); NaN; ctr([2 4],1)], ...
        'YData', [ctr([1 3],2); NaN; ctr([2 4],2)],...
        'ZData', [ctr([1 3],3); NaN; ctr([2 4],3)] );
    pause(t(k)-toc);
    pause(0.01);

end