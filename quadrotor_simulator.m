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

%% Initial conditions
r = [0; 0; 0];
n = [0; 0; 0];

% State Vectors - start and final:
z0 = [0; 0; 0; zeros(9,1)];       % starting pose

%UAV Kinematics
%UAV initial position
% y0 = [2 2 5]';
% freq = 2*pi*0.3;
% uav_dyn = @(t) [-sin(freq*t); cos(freq*t); 0]*2;

% uav_traj = @(t) y0 + [cos(freq*t); sin(freq*t); 0]*2/freq;

% tailing
% z0 = [0; 10; 0; zeros(9,1)];
% y0 = [2 2 5]';
% uav_dyn = @(t) y0*0.2;
% uav_traj = @(t) y0 + t*uav_dyn(t);

%head on
% z0 = [0; 10; 0; zeros(9,1)];
% y0 = [2 2 5]';
% uav_dyn = @(t) -y0/10;
% uav_traj = @(t) y0 + t*uav_dyn(t);

%steep climb and descent
% z0 = [0; 10; 0; zeros(9,1)];
% y0 = [2 2 5]';
% uav_dyn = @(t) [0; 0; square(t)];
% uav_traj = @(t) y0 + t*uav_dyn(t);

%sinusoidal climb and descent
% z0 = [0; 10; 0; zeros(9,1)];
% y0 = [1 2 5]';
% uav_dyn = @(t) [0.5; 0; cos(t)]*2.3; %for multiplier of 2 it goes out of control while returning
% uav_traj = @(t) y0 + [0.5*t; 0; sin(t)]*2.3;

%spiral ascent
% z0 = [8; 4; 0; zeros(9,1)];
% y0 = [1 2 5]';
% uav_dyn = @(t) [0.5; 0; cos(t)]*2;
% uav_traj = @(t) y0 + [0.5*t; 0; sin(t)]*2;

% %test trajectory1
% z0 = [2; 2; 0; zeros(9,1)];
% y0 = [6; 6; 9]
% uav_dyn = @(t) [0.5; 0.5; -0.5*t + 0.1]*1.2 %failure to return home due to bad angles
% uav_traj = @(t) y0 + t*uav_dyn(t);

%test trajectory2
% z0 = [2; 2; 0; zeros(9,1)];       % starting pose
% y0 = [6; 0.1; 0.1]
% uav_dyn = @(t) [0.5; 0.5; 0.5*t + 0.1]
% uav_traj = @(t) y0 + t*uav_dyn(t);

% %test trajectory3
z0 = [0; 10; 0; zeros(9,1)];       % starting pose
y0 = [6; 9; 9]
uav_dyn = @(t) [-0.1; -0.5; (-0.2*t - 0.5)]*0.8;
uav_traj = @(t) y0 + t*uav_dyn(t);

% %test trajectory4
% z0 = [0; 0; 0; zeros(9,1)];       % starting pose
% y0 = [5; 0.1; 8];
% uav_dyn = @(t) [0.5; 0.09*exp(0.09*t); 0];
% uav_traj = @(t) y0 + [0.5*t; exp(0.09*t); 0];

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


Q = [0.2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
     0, 0.2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
     0, 0, 0.2, 0, 0, 0, 0, 0, 0, 0, 0, 0;
     0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 0.0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 0, 0.0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 0, 0, 0.0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0;
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0;
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1];

R = 1/(mu)*eye(4);

[K,S,pole] = lqr(A, B, Q, R);
%poles = [-2 -0.5+1i -0.5+1i -0.2+0.1i -0.2-0.1i -0.4-0.2i -0.08 -0.3-0.23i -0.8 + 0.4i -1.2 -1.1 -1.5];
%Pcl = pole(syscl)
%K = place(A,B,pole)

%Alternate way of defining our system
syscl = ss(A, B, eye(12), 0);


%zd = @(t) [2*cos(t); 2*sin(t); min(t,7); 0; zeros(8,1)];    % desired pose for test
% zd = [4; 0; 5; 0; zeros(8,1)];

% Inputs:
ud = [1 1 1 1]'*m*g/4;

u  = @(z, zd, ud, K) ud + K*(zd - z);

% Problem parameters
epsilon = 0.2*2;
% epsilon = 1;
%Drone returns back home after UAV leaves airfield
threshold = [0 10;0 10;0 10];


%% Phase I: Pursue
sample = 2000;
tspan_I = linspace(0, 40, sample);

options = odeset('Events', @(t,z) interceptdrone(t, z, epsilon, threshold),...
      'RelTol', 1e-2);
%options = odeset('Events', @(t,z) interceptdrone(t, z, epsilon),...
  % 'Events', @(t,z) boundary_condition(t,z,threshold), 'RelTol', 1e-6);

vel = y0;
count = 0;
% [t_K, uav_traj] = ode45(@(t,uav_traj) uav_dyn(t), tspan_I, y0);
% option2 = odeset('Events', @(t,z) boundary_condition(t,z,threshold));
% options = odeset(option1, option2)
[t_I, z_I, te, ze] = ode45(@(t, z) augmentedSystem(t, z, uav_dyn, u, K, p, zeros(3,1), zeros(3,1), uav_traj),...
    tspan_I, z0_I, options);                                                                                                                                                           

%% Phase II: Return
if(isempty(te)) % if the robot failed to catch the bug
    % Decoupling the augmented state vector z from only phase I
    t = t_I;
    z = z_I(:,1:12);
    y = z_I(:,13:15);

    disp('Drone incapable of capturing the UAV...Returning to base')

else

    tspan_II = [te, tspan_I(end)];
    z0_II = z_I(end,1:12)';
    zd = [0; 0; 1; zeros(9,1)]; 
    ud = [1 1 1 1]'*m*g/4;

    if (any(threshold (:,1) > ze(13:15)') || any(ze(13:15)' > threshold(:,2)))
        disp('UAV left the airfield...Returning empty handed')

        [t_II, z_II] = ode45(@(t,z) quadrotor(t, z, u(z, zd, ud, K), p, r, n),...
        tspan_II, z0_II);

        
    else
        %add disturbance and if any change in dynamics to solve
        disp('Target acquired. Its coming home!')
        
        %disp('UAV capture successful...It is resisting the capture...Bringing it back to base')
        r = @(t) [0.1*sin(t); 0; 0.1*square(t)]*0
        n = @(t) [0.1; 0.2; 0.3]*0;

        % q = [0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
        %      0, 0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
        %      0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0;
        %      0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0;
        %      0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0;
        %      0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0;
        %      0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0;
        %      0, 0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0;
        %      0, 0, 0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0;
        %      0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0;
        %      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0;
        %      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1];
        % 
        % Q = q;
        % 
        % R = 1/(mu)*eye(4);
        % 
        % [K,S,pole] = lqr(A, B, Q, R);
    [t_II, z_II] = ode45(@(t,z) quadrotor(t, z, u(z, zd, ud, K), p, r(t), n(t)),...
        tspan_II, z0_II);
        

    end
    % Decoupling the augmented state vector z from phase I and phase II
    t = [t_I; t_II];
    % t = t_I;
    % z = z_I;
    z = [z_I(:,1:12); z_II];
    y = [z_I(:,13:15); z_II(:,1:3)];

end

%using this to track distance until we have point-mass in simulation
cost = zeros(length(z_I),1);
for i = 1:length(z_I)
    dist_y_z(i,1) = norm(y(i,1:3) - z(i,1:3));

end


%% Plotting the results

for i=1:4
    ax(i) = subplot(2,2,i,'NextPlot','Add','Box','on','XGrid','on','YGrid','on',...
                'Xlim',[t(1), t(end)],...
                'TickLabelInterpreter','LaTeX','FontSize',14);
    xlabel('t','Interpreter','LaTeX','FontSize',14);        
end


plot(ax(1), t,y(:,1:3), 'LineWidth', 1.5);
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

figure(2)
plot3(z(1:length(t_I),1),z(1:length(t_I),2),z(1:length(t_I),3), 'g', 'LineWidth', 3)
hold on
plot3(y(1:length(t_I),1),y(1:length(t_I),2),y(1:length(t_I),3), 'r', 'LineWidth', 2)
xlabel('X')
ylabel('Y')
zlabel('Z')
grid on
legend(["Drone", "UAV"])
title("Trajectories of Drone and UAV in space")

%% Animation
animation_fig = figure;

airspace_box_length = 10;

animation_axes = axes('Parent', animation_fig,...
    'NextPlot','add','DataAspectRatio',[1 1 1],...
    'Xlim',airspace_box_length*[-0.5 1.5],...
    'Ylim',airspace_box_length*[-0.5 1.5],...
    'Zlim',airspace_box_length*[0 1],...
    'box','on','Xgrid','on','Ygrid','on','Zgrid','on',...
    'TickLabelInterpreter','LaTeX','FontSize',14);

view(animation_axes, 3);

% Editing the Sim-Drone shape:
circleRadius = 0.8;
bodySize = 0.4;
droneLineWidth = 0.2;

N = 10;
Q = linspace(0,2*pi,N)';
circle = circleRadius*l*[cos(Q) sin(Q) zeros(N,1)];
loc = bodySize*[1 0 0; 0 1 0; -1 0 0; 0 -1 0];


silhouette = plot3(0,0,0, '--', 'Color', 0.5*[1 1 1], 'LineWidth', 1 ,...
    'Parent', animation_axes);
uav = plot3(0, 0, 0, 'Color', 'r', 'LineWidth', droneLineWidth,...
        'Parent', animation_axes);
body = plot3(0,0,0, 'Color', 'k', 'LineWidth', droneLineWidth,...
        'Parent', animation_axes);
for i=1:4
    rotor2(i) = plot3(0,0,0, 'Color', 'r', 'LineWidth', droneLineWidth,...
        'Parent', animation_axes);
    rotor(i) = plot3(0,0,0, 'Color', 'g', 'LineWidth', droneLineWidth,...
        'Parent', animation_axes);

end
tic;
for k=1:length(t)
    
    R = [ cos(z(k,5))*cos(z(k,6)), sin(z(k,4))*sin(z(k,5))*cos(z(k,6)) - cos(z(k,4))*sin(z(k,6)), sin(z(k,4))*sin(z(k,6)) + cos(z(k,4))*sin(z(k,5))*cos(z(k,6));
          cos(z(k,5))*sin(z(k,6)), cos(z(k,4))*cos(z(k,6)) + sin(z(k,4))*sin(z(k,5))*sin(z(k,6)), cos(z(k,4))*sin(z(k,5))*sin(z(k,6)) - sin(z(k,4))*cos(z(k,6));
                     -sin(z(k,5)),                                 sin(z(k,4))*cos(z(k,5)),                                 cos(z(k,4))*cos(z(k,5))];
    for i=1:4
        ctr(i,:) = z(k,1:3) + loc(i,:)*R';
        ctr2(i,:) = y(k,1:3) + loc(i,:)*R';
        pose = ones(N,1)*z(k,1:3) + (ones(N,1)*loc(i,:) + circle)*R';
        set(rotor(i), 'XData', pose(:,1), 'YData', pose(:,2),  'ZData', pose(:,3) );
        pose2 = ones(N,1)*y(k,1:3) + (ones(N,1)*loc(i,:) + circle)*R';
        set(rotor2(i), 'XData', pose2(:,1), 'YData', pose2(:,2),  'ZData', pose2(:,3) );         
    end

    set(silhouette,'XData', [0, z(k,1), z(k,1), z(k,1)],...
        'YData', [0, 0, z(k,2), z(k,2)],...
        'ZData', [0, 0, 0, z(k,3)]);

    set(uav, 'XData', [ctr2([1 3],1); NaN; ctr2([2 4],1)], ...
        'YData', [ctr2([1 3],2); NaN; ctr2([2 4],2)],...
        'ZData', [ctr2([1 3],3); NaN; ctr2([2 4],3)] );

    set(body, 'XData', [ctr([1 3],1); NaN; ctr([2 4],1)], ...
        'YData', [ctr([1 3],2); NaN; ctr([2 4],2)],...
        'ZData', [ctr([1 3],3); NaN; ctr([2 4],3)] );
    
    pause(t(k)-toc);
    pause(0.01);

end