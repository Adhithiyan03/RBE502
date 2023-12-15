%hold on
%for i=1:size(xHistory,1)
%    clf;
%    animateQuadrotor(time(i), xHistory(i,:));
%    pause(Ts);    
%end
animation_fig = figure;

airspace_box_length = 10;

animation_axes = axes('Parent', animation_fig,...
    'NextPlot','add','DataAspectRatio',[1 1 1],...
    'Xlim',airspace_box_length*[-0.5 1],...
    'Ylim',airspace_box_length*[-0.5 1],...
    'Zlim',airspace_box_length*[-0.5 1],...
    'box','on','Xgrid','on','Ygrid','on','Zgrid','on',...
    'TickLabelInterpreter','LaTeX','FontSize',14);

view(animation_axes, 3);

% Editing the Sim-Drone shape:
circleRadius = 2;
bodySize = 1.2;
droneLineWidth = 1;

N = 10;
Q = linspace(0,2*pi,N)';
circle = 2*0.2*[cos(Q) sin(Q) zeros(N,1)];
loc = bodySize*[1 0 0; 0 1 0; -1 0 0; 0 -1 0];


silhouette = plot3(0,0,0, '--', 'Color', 0.5*[1 1 1], 'LineWidth', 1 ,...
    'Parent', animation_axes);
body = plot3(0,0,0, 'Color', 'b', 'LineWidth', droneLineWidth,...
        'Parent', animation_axes);
uav = plot3(0, 0, 0, 'Color', 'r', 'LineWidth', droneLineWidth,...
        'Parent', animation_axes);
for i=1:4
    rotor(i) = plot3(0,0,0, 'Color', 'b', 'LineWidth', droneLineWidth,...
        'Parent', animation_axes);
    rotor2(i) = plot3(0,0,0, 'Color', 'r', 'LineWidth', droneLineWidth,...
        'Parent', animation_axes);
end
tic;
for k=1:ts
    
    R = [ cos(xHistory(k,5))*cos(xHistory(k,6)), sin(xHistory(k,4))*sin(xHistory(k,5))*cos(xHistory(k,6)) - cos(xHistory(k,4))*sin(xHistory(k,6)), sin(xHistory(k,4))*sin(xHistory(k,6)) + cos(xHistory(k,4))*sin(xHistory(k,5))*cos(xHistory(k,6));
          cos(xHistory(k,5))*sin(xHistory(k,6)), cos(xHistory(k,4))*cos(xHistory(k,6)) + sin(xHistory(k,4))*sin(xHistory(k,5))*sin(xHistory(k,6)), cos(xHistory(k,4))*sin(xHistory(k,5))*sin(xHistory(k,6)) - sin(xHistory(k,4))*cos(xHistory(k,6));
                     -sin(xHistory(k,5)),                                 sin(xHistory(k,4))*cos(xHistory(k,5)),                                 cos(xHistory(k,4))*cos(xHistory(k,5))];
    for i=1:4
        ctr(i,:) = xHistory(k,1:3) + loc(i,:)*R';
        ctr2(i,:) = y_final(k,1:3) + loc(i,:)*R';
        pose = ones(N,1)*xHistory(k,1:3) + (ones(N,1)*loc(i,:) + circle)*R';
        pose2 = ones(N,1)*y_final(k,1:3) + (ones(N,1)*loc(i,:) + circle)*R';
        set(rotor(i), 'XData', pose(:,1), 'YData', pose(:,2),  'ZData', pose(:,3) );
        set(rotor2(i), 'XData', pose2(:,1), 'YData', pose2(:,2),  'ZData', pose2(:,3) );
         
    end
    set(silhouette,'XData', [0, xHistory(k,1), xHistory(k,1), xHistory(k,1)],...
        'YData', [0, 0, xHistory(k,2), xHistory(k,2)],...
        'ZData', [0, 0, 0, xHistory(k,3)]);

    set(uav, 'XData', [ctr2([1 3],1); NaN; ctr2([2 4],1)], ...
        'YData', [ctr2([1 3],2); NaN; ctr2([2 4],2)],...
        'ZData', [ctr2([1 3],3); NaN; ctr2([2 4],3)] );
    set(body, 'XData', [ctr([1 3],1); NaN; ctr([2 4],1)], ...
        'YData', [ctr([1 3],2); NaN; ctr([2 4],2)],...
        'ZData', [ctr([1 3],3); NaN; ctr([2 4],3)] );
    pause(k-toc);
    pause(0.01);
    M(k) = getframe;
end
movie(M,5)