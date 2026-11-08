dt = 0.05;          % time step (s)
T = 8;              % simulation duration (s)
L = 0.20;           % distance between wheels (m)

% wheel velocities
vL = 0.25;          % left wheel (m/s)
vR = 0.25;          % right wheel (m/s)

% starting position
x = 0;
y = 0;
theta = 0;

% calculate robot velocities
v = (vR + vL)/2;
omega = (vR - vL)/L;

%% create figure

figure('Color','w','Position',[100 100 1000 550]);
axes('Position',[0.08 0.14 0.58 0.78]);

axis equal;
axis([-0.5 3 -1.5 1.5]);
grid on;
hold on;

xlabel('x position (m)');
ylabel('y position (m)');
title('differential drive robot - straight line');

% draw robot
robot = plot(x,y,'bo','MarkerSize',12,...
    'MarkerFaceColor','b');

% draw direction arrow
heading = quiver(x,y,0.3*cos(theta),...
    0.3*sin(theta),0,'r','LineWidth',2);

% draw path
trail = animatedline('Color','b','LineWidth',1.5);
addpoints(trail,x,y);

% results panel
resultsBox = annotation('textbox',[0.70 0.40 0.28 0.35],...
    'String','status: moving',...
    'FontSize',11,...
    'BackgroundColor',[0.96 0.96 0.96]);

%% simulation loop

for step = 1:round(T/dt)

    % update position
    x = x + v*cos(theta)*dt;
    y = y + v*sin(theta)*dt;
    theta = theta + omega*dt;

    % update robot
    set(robot,'XData',x,'YData',y);

    % update direction
    set(heading,'XData',x,'YData',y,...
        'UData',0.3*cos(theta),...
        'VData',0.3*sin(theta));

    % update path
    addpoints(trail,x,y);

    drawnow;

end

%% final results

elapsedTime = step*dt;

set(resultsBox,'String',{
    'status: simulation complete'
    ' '
    sprintf('left wheel: %.2f m/s',vL)
    sprintf('right wheel: %.2f m/s',vR)
    sprintf('final position: (%.2f, %.2f) m',x,y)
    sprintf('simulation time: %.2f s',elapsedTime)
});

drawnow;
