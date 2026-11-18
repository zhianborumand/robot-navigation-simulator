clear;
clc;
close all;

srcFolder = fullfile(fileparts(mfilename('fullpath')), 'src');
if isfolder(srcFolder)
    addpath(srcFolder);
end

%% time settings
dt = 0.02;                   % time step (s)
T = 80;                      % max simulation time (s)
time = 0:dt:T;

% navigation settings
navParams.Kv = 0.8;          % forward speed gain
navParams.Ktheta = 2.0;      % heading gain
navParams.vMax = 0.35;       % max forward speed (m/s)
navParams.wheelMax = 0.40;   % max wheel speed (m/s)
navParams.wheelBase = 0.20;  % distance between wheels (m)
navParams.tolerance = 0.10;  % waypoint reached distance (m)

waypoints = [2 0;
             2 2;
             0 2;
             0 0];
targetIndex = 1;

%% robot starting pose
x = 0;
y = 0;
theta = 0;

%% storage
xHistory = zeros(size(time));
yHistory = zeros(size(time));
elapsedTime = 0;
lastStep = 1;

%% simulation loop
for k = 2:length(time)

    % 1. navigation: where do the wheels go? call function
    [vLeftCmd, vRightCmd, targetIndex, finished] = navController(x, y, theta, waypoints, targetIndex, navParams);
    if finished
        break;
    end

    % 4. the real robot moves (ideal wheels)
    v = (vRightCmd + vLeftCmd) / 2;                         % forward speed from the two wheel speeds
    omega = (vRightCmd - vLeftCmd) / navParams.wheelBase;   % turn rate from the two wheel speeds

    x = x + v * cos(theta) * dt;
    y = y + v * sin(theta) * dt;
    theta = theta + omega * dt;

    xHistory(k) = x;
    yHistory(k) = y;
    elapsedTime = time(k);
    lastStep = k;
end

%% results
distanceToFinish = sqrt((waypoints(end, 1) - x)^2 + (waypoints(end, 2) - y)^2);
fprintf('waypoints reached: %d of %d\n', min(targetIndex - 1, size(waypoints,1)), size(waypoints,1));
fprintf('simulation time: %.2f s\n', elapsedTime);
fprintf('final position: (%.3f, %.3f) m\n', x, y);
fprintf('distance to finish: %.3f m\n', distanceToFinish);

%% plot
figure('Color','w');
plot(xHistory(1:lastStep), yHistory(1:lastStep), 'b', 'LineWidth', 2);
hold on;
plot(waypoints(:,1), waypoints(:,2), 'rx', 'MarkerSize', 12, 'LineWidth', 2);
axis equal;  grid on;
xlabel('x position (m)');
ylabel('y position (m)');
title('stage 1: waypoint navigation with ideal wheels');
