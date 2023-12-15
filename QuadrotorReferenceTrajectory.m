function [ xdesired ] = QuadrotorReferenceTrajectory( t )
% This function generates reference signal for nonlinear MPC controller
% used in the quadrotor path following example.

% Copyright 2019 The MathWorks, Inc.

    y =2*sin(t);
    x =2*cos(t);
    z =2*(t);
    phi = zeros(1,length(t));
    theta = zeros(1,length(t));
    psi = zeros(1,length(t));
    xdot = zeros(1,length(t));
    ydot = zeros(1,length(t));
    zdot = zeros(1,length(t));
    phidot = zeros(1,length(t));
    thetadot = zeros(1,length(t));
    psidot = zeros(1,length(t));
    xdesired = [x;y;z;phi;theta;psi;xdot;ydot;zdot;phidot;thetadot;psidot];

