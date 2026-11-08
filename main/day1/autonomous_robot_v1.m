
clear;
clc;
close all;

dt = 0.05;          % time step (s)
T = 20;             % maximum simulation time (s)
L = 0.20;           % distance between wheels (m)

%% robot starting position
x = 0;              % x position (m)
y = 0;              % y position (m)
theta = 0;          % robot heading (rad)

%% target position
xTarget = 2;
yTarget = 2;
tolerance = 0.10;  

%% controller settings
Kv = 0.8;           % forward speed gain
Ktheta = 5.0;       % heading gain
vMax = 0.35;        % maximum forward speed (m/s)
wheelMax = 0.40;    % maximum wheel speed (m/s)

%% creating figure
figure('Color','w','Position',[100 100 1000 550]);
axes('Position',[0.08 0.14 0.58 0.78]);
axis equal;
axis([-0.5 3 -0.5 3]);
grid on;
hold on;
xlabel('x position (m)');
ylabel('y position (m)');
title('autonomous differential drive robot');

plot(xTarget,yTarget,'rx','MarkerSize',14,'LineWidth',2);

% target tolerance circle
a = linspace(0,2*pi,100);

plot(xTarget+tolerance*cos(a), ...
     yTarget+tolerance*sin(a),'r--');

% plot robot
robot = plot(x,y,'bo','MarkerSize',12, ...
    'MarkerFaceColor','b');

% robot heading arrow
heading = quiver(x,y,0.3*cos(theta), ...
    0.3*sin(theta),0,'r','LineWidth',2);

% robot path
trail = animatedline('Color','b','LineWidth',1.5);

%% results 
resultsBox = annotation('textbox',[0.70 0.40 0.28 0.35], ...
    'String','status: navigating...', ...
    'FontSize',11, ...
    'BackgroundColor',[0.96 0.96 0.96], ...
    'EdgeColor',[0.7 0.7 0.7], ...
    'VerticalAlignment','top');

%% simulation loop
elapsedTime = 0;

for step = 1:round(T/dt)

    % calculate distance to target
    dx = xTarget - x;
    dy = yTarget - y;
    distance = sqrt(dx^2 + dy^2);
    desiredHeading = atan2(dy,dx);
    headingError = desiredHeading - theta;

    % keep heading error between -pi and pi
    headingError = atan2(sin(headingError), cos(headingError));
    % proportional controllers
    omegaCmd = Ktheta * headingError;

    vCmd = min(vMax,Kv*distance);
    vCmd = vCmd * max(0,cos(headingError));

    % calculate wheel velocities
    vR = vCmd + omegaCmd*L/2;
    vL = vCmd - omegaCmd*L/2;

    % limit wheel speeds
    maxSpeed = max(abs(vL),abs(vR));

    if maxSpeed > wheelMax
        vL = vL * wheelMax/maxSpeed;
        vR = vR * wheelMax/maxSpeed;
    end

    % calculate robot velocities
    v = (vR + vL)/2;
    omega = (vR - vL)/L;

    % update robot position
    x = x + v*cos(theta)*dt;
    y = y + v*sin(theta)*dt;
    theta = theta + omega*dt;

    % update simulation time
    elapsedTime = step*dt;

    % update robot position on figure
    set(robot,'XData',x,'YData',y);

    % update heading arrow
    set(heading,'XData',x,'YData',y, ...
        'UData',0.3*cos(theta), ...
        'VData',0.3*sin(theta));

    % update path
    addpoints(trail,x,y);

    drawnow;

    % check if robot has reached target
    distance = sqrt((xTarget-x)^2 + (yTarget-y)^2);

    if distance <= tolerance
        break;
    end

end

%% final results
finalError = sqrt((xTarget-x)^2 + (yTarget-y)^2);

if finalError <= tolerance
    statusText = 'target reached';
else
    statusText = 'target not reached';
end

% show results in figure
set(resultsBox,'String',{
    ['status: ' statusText]
    ' '
    sprintf('heading gain: %.1f',Ktheta)
    sprintf('final position: (%.2f, %.2f) m',x,y)
    sprintf('final distance error: %.3f m',finalError)
    sprintf('simulation time: %.2f s',elapsedTime)
});

drawnow;
