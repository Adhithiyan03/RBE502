r = [0;0;0]
n = [0;0;0]
getQuadrotorDynamicsAndJacobian;
nx = 12;
ny = 12;
nu = 4;
nlmpcobj = nlmpc(nx,ny,nu);
nlmpcobj.Model.StateFcn = "QuadrotorStateFcn";
nlmpcobj.Jacobian.StateFcn = @QuadrotorStateJacobianFcn;
rng(0)
validateFcns(nlmpcobj,rand(nx,1),rand(nu,1));
Ts = 0.1;
p = 20;
m = 10;
nlmpcobj.Ts = Ts;
nlmpcobj.PredictionHorizon = p;
nlmpcobj.ControlHorizon = m;
nlmpcobj.MV = struct( ...
   Min={0;0;0;0}, ...
   Max={10;10;10;10}, ...
   RateMin={-2;-2;-2;-2}, ...
   RateMax={2;2;2;2} ...
   );
% nlmpcobj.MV = struct( ...
%     Min={0}, ...
%     Max={10}, ...
%     RateMin={-2}, ...
%     RateMax={2} ...
%     );
nlmpcobj.Weights.OutputVariables = [1 1 1 1 1 1 0 0 0 0 0 0];
%nlmpcobj.Weights.ManipulatedVariables = 0.1;
nlmpcobj.Weights.ManipulatedVariables = [0.1 0.1 0.1 0.1];
nlmpcobj.Weights.ManipulatedVariablesRate = [0.1 0.1 0.1 0.1];
x = [5;0;0;0;0;0;0;0;0;0;0;0];

% Nominal control target (average to keep quadrotor floating)
nloptions = nlmpcmoveopt;
nloptions.MVTarget = [2.45 2.45 2.45 2.45]; 
mv = nloptions.MVTarget;

% Simulation duration in seconds
Duration = 7.5;

% Display waitbar to show simulation progress
hbar = waitbar(0,"Simulation Progress");

% MV last value is part of the controller state
lastMV = mv;

% Store states for plotting purposes
xHistory = x';
y_current = QuadrotorReferenceTrajectory(0);
y_final = y_current';
uHistory = lastMV;

% Simulation loop
for k = 1:(Duration/Ts)
    t = linspace(k*Ts, (k+p-1)*Ts,p);
    xk = xHistory(k,:); 
    y_current = QuadrotorReferenceTrajectory(k*Ts);
    % Set references for previewing
    if norm(xk(1,1:3)-y_current(1:3,1)')< 0.5
        break
    end
    yref = QuadrotorReferenceTrajectory(t);
            %gets the trajectory of reference 18
    [uk,nloptions,info] = nlmpcmove(nlmpcobj,xk,lastMV,yref',[],nloptions);
    % Store control move
    uHistory(k+1,:) = uk';
    lastMV = uk;
    % Simulate quadrotor for the next control interval (MVs = uk) 
    ODEFUN = @(t,xk) QuadrotorStateFcn(xk,uk);
    [TOUT,XOUT] = ode45(ODEFUN,[0 Ts], xHistory(k,:)');
    y_final(k+1,:)=  y_current'
    % Update quadrotor state
    xHistory(k+1,:) = XOUT(end,:);

    % Update waitbar
end
r = [1; 1; 1];
n = [0.1; 0.1; 0.1];
getQuadrotorDynamicsAndJacobian;
for ts= k:(Duration/Ts)
    t = linspace(ts*Ts, (ts+p-1)*Ts,p);
    xk = xHistory(ts,:); 
    y_current = QuadrotorReferenceTrajectory(ts*Ts);
    % Set references for previewing
    yref = Return_Traj(xk,t);
    %gets the trajectory of reference 18
    [uk,nloptions,info] = nlmpcmove(nlmpcobj,xk,lastMV,yref',[],nloptions);
    % Store control move
    uHistory(ts+1,:) = uk';
    lastMV = uk;
    % Simulate quadrotor for the next control interval (MVs = uk)
    %g = @(t)[0; 0; c(find(tk <= t, 1, 'last'),:)];
    %[A,B] = QuadrotorStateJacobianFcn(xk,uk);
    %dr = @(t, xk, u, g) A*xk + B*u + g;
    ODEFUN = @(t,xk) QuadrotorStateFcn(xk,uk);
    [TOUT,XOUT] = ode45(ODEFUN,[0 Ts], xHistory(ts,:)');
    % Update quadrotor state
    xHistory(ts+1,:) = XOUT(end,:);
    y_final(ts+1,:) = xk;
    % Update waitbar
    waitbar(ts*Ts/Duration,hbar);
end
   

% Close waitbar 
close(hbar)
plotQuadrotorTrajectory;
%figure('Name','QuadrotorAnimation');
%hold on
%for i=1:size(xHistory,1)
%    clf;
%    animateQuadrotorTrajectory(time(i), xHistory(i,:));
%    pause(Ts);    
%end
animateQuadrotorTrajectory;