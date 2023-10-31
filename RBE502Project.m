clear; close all; clc;
syms x1 x2 x3 v1 v2 v3 real

x_desired = [0 0 1];
v_desired = [0 0 0];
state = [x1 x2 x3 0 0 0 v1 v2 v3 0 0 0];

g = -9.81;
m = 1;

u = [u1 u2 u3 u4];

Tf = 10;                                                                   % simulation end time
tSpan = [0, Tf];                                                           % define simulation time span                                                                     % benchmarking
[T, X] = ode45(@(t,x) thrustODE(t, x), tSpan, zeros(1, 12));               % solve robot dynamical model dq=F(q,dq), robot state space is defined as X=[q, dq]

%ODE
function dx = thrustODE(~, ~)
    state(1)
    thrust = PID(x_desired, v_desired, state);
    u1 = thrust(3)/4;
    u2 = thrust(3)/4;
    u3 = thrust(3)/4;
    u4 = thrust(3)/4;
    dx = g+(thrust(3)/m);
end 


%Simple PD controller for position for now
function output = PID(x_desired, v_desired, state)
    Kp = 0;
    Ki = 0;
    Kd = 0;

    x1 = (x_desired(1)-state(1))*Kp + (v_desired(1)-state(7))*Kd;
    x2 = (x_desired(2)-state(2))*Kp + (v_desired(2)-state(8))*Kd;
    x3 = (x_desired(3)-state(3))*Kp + (v_desired(3)-state(9))*Kd;

    output = [x1 x2 x3];
end