clear; close all; clc;

%function stub for simulation example
interval = 0.1;
for i = 1:interval:tf
    new_position = PID([1 1 1], [1 1 1], z);
    plot3(new_position)
end

%Simple PD controller for position for now
function output = PID(x_desired, v_desired, state)
    Kp = 0;
    Ki = 0;
    Kd = 0;

    x1 = (x_desired(1)-state(1))*Kp + (v_desired(1)-state(7))*Kd
    x2 = (x_desired(2)-state(2))*Kp + (v_desired(2)-state(8))*Kd
    x3 = (x_desired(3)-state(3))*Kp + (v_desired(3)-state(9))*Kd
end

