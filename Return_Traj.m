function [ xdesired ] = Return_Traj(xk,t)
    x =zeros(1,length(t));
    y =zeros(1,length(t));
    z = zeros(1,length(t));
    Z_des = [0;0;0;0;0;0;0;0;0;0;0;0]
    x_des(1,1:length(t))=Z_des(1,1);
    y_des(1,1:length(t)) = Z_des(2,1);
    z_des(1,1:length(t)) = Z_des(3,1);
    x(1,1:length(t)) = xk(1,1);
    y(1,1:length(t)) = xk(1,2);
    z(1,1:length(t)) = xk(1,3);
    x = ((x -x_des)/18).*t
    y = ((y-y_des)/18).*t ;
    z = ((z -z_des)/18).*t ;

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