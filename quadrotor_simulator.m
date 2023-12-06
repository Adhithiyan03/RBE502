%% Initializations
clc; clear; close all;

g = 9.81;   % The gravitational acceleration [m/s^2]
l =  0.2;   % Distance from the center of mass to each rotor [m]
m =  0.5;   % Total mass of the quadrotor [kg]
I = [1.24, 1.24, 2.48]; % Mass moment of inertia [kg m^2]. DIAGONAL of 3x3
mu = 3.0;   % Maximum thrust of each rotor [N]
sigma = 0.01; % The proportionality constant relating thrust to torque [m]

%Problem parameters
p = [g l m I mu sigma];

%UAV Kinematics
uav_dyn = @(t,y) [sin(t);0; 0];

%% Initial conditions
r = [0; 0; 0];
n = [0; 0; 0];

% State Vectors - start and final:
z0 = [0; 0; 0; zeros(9,1)];       % starting pose

%UAV initial position
y0 = [0.5 5 5]';

%Initial augmented state vector
z0_I = [z0; y0];

%% LQR Test - using MATLAB LQR:

% syms z1 z2 z3 z4 z5 z6 z7 z8 z9 z10 z11 z12 r1 r2 r3 n1 n2 n3 'real'
% syms u1 u2 u3 u4 'positive'
% zp = [z1; z2; z3; z4; z5; z6; z7; z8; z9; z10; z11; z12];
% up = [u1; u2; u3; u4];
% dz = quadrotor(0, zp, up, p, [r1;r2;r3], [n1;n2;n3]);
% Au = @(t,z) (subs(jacobian(dz,zp), [zp; up], [z0; u0]));
% Bu = @(t,z) (subs(jacobian(dz,up), [zp; up], [z0; u0]));
% G = Au(100)
% H = Bu(100)

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


Q = 0.12*eye(12);

R = 2/(mu)*eye(4);

[K,S,pole] = lqr(A, B, Q, R)
%poles = [-2 -0.5+1i -0.5+1i -0.2+0.1i -0.2-0.1i -0.4-0.2i -0.08 -0.3-0.23i -0.8 + 0.4i -1.2 -1.1 -1.5];
%Pcl = pole(syscl)
%K = place(A,B,pole)

%zd = @(t) [2*cos(t); 2*sin(t); min(t,7); 0; zeros(8,1)];    % desired pose for test
% zd = [4; 0; 5; 0; zeros(8,1)];

% Inputs:
ud = [1 1 1 1]'*m*g/4;

u  = @(z, zd, ud, K) ud + K*(zd - z);

% Problem parameters
epsilon = 0.5*p(2);

%Drone returns back home after UAV leaves airfield
threshold = [10; 10; 10];
%% Phase I: Pursue
tspan_I = [0 40];
% t = linspace(0, 20, 300);
options = odeset('Events', @(t,z) interceptdrone(t, z, epsilon, threshold),...
      'RelTol', 1e-6);
%options = odeset('Events', @(t,z) interceptdrone(t, z, epsilon),...
  % 'Events', @(t,z) boundary_condition(t,z,threshold), 'RelTol', 1e-6);

% option2 = odeset('Events', @(t,z) boundary_condition(t,z,threshold));
% options = odeset(option1, option2)
[t_I, z_I, te, ze] = ode45(@(t, z) augmentedSystem(t, z, uav_dyn, u, K, p, r, n),...
    tspan_I, z0_I, options);                                                                                                                                                            zr = @(t,z) zd(t) - z

%% Phase II: Return

if(isempty(te)) % if the robot failed to catch the bug
    % Decoupling the augmented state vector z from only phase I
    t = t_I;
    z = z_I(:,1:12);
    y = z_I(:,13:15);

    disp('Drone incapable of capturing the UAV...Returning back to base')

else
    if any(ze(13:15)' - threshold > 0)
        disp('UAV left the airfield...Returning back to base')
    else
        %add disturbance and if any change in dynamics to solve
        disp(['Ayo UAV, L + Ratio + Go touch grass + Keep crying'])

        %disp('UAV capture successful...It is resisting the capture...Bringing it back to base')
        r = [0; 0; 0];
        n = [0; 0; 0];

    end
    tspan_II = [te, tspan_I(end)];
    z0_II = z_I(end,1:12)';
    zd = [0; 0; 1; zeros(9,1)]; 
    ud = [1 1 1 1]'*m*g/4;

    [t_II, z_II] = ode45(@(t,z) quadrotor(t, z, u(z, zd, ud, K), p, r, n),...
        tspan_II, z0_II);


    % Decoupling the augmented state vector z from phase I and phase II
    t = [t_I; t_II];
    % t = t_I;
    % z = z_I;
    z = [z_I(:,1:12); z_II];
    y = [z_I(:,13:15); z_II(:,1:3)];
    % b = [z_I(:,[5, 6]); r_II(:,[1,2])]; % Since the bug is captured by the robot,
                                  % bug's states are equal to the robot's.
end

%using this to track distance until we have point-mass in simulation
for i = 1:length(z)
    dist_y_z(i,1) = norm(z(i,1:3) - y(i,1:3));
end

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