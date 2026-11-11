
clear;
clc;
close all;

dt = 0.02;
T = 80;
time = 0:dt:T;

%% robot and encoder settings
wheelRadius = 0.033;        % wheel radius (m)
wheelBase = 0.20;           % distance between the wheels (m)
ticksPerRev = 360;          % encoder ticks per wheel revolution
distancePerTick = 2*pi*wheelRadius / ticksPerRev;

%% true wheel speeds
trueSpeedLeft = 0.30;       % m/s
trueSpeedRight = 0.29;      % m/s

%% storage
trueDistanceLeft = zeros(size(time));
trueDistanceRight = zeros(size(time));
tickCountLeft = zeros(size(time));
tickCountRight = zeros(size(time));

%% encoder loop
for k = 2:length(time)
    trueDistanceLeft(k) = trueDistanceLeft(k-1) + trueSpeedLeft*dt;
    trueDistanceRight(k) = trueDistanceRight(k-1) + trueSpeedRight*dt;

    tickCountLeft(k) = floor(trueDistanceLeft(k) / distancePerTick);
    tickCountRight(k) = floor(trueDistanceRight(k) / distancePerTick);
end

%% starting positions
headingEstimate = 0;    % rad
xEstimate = 0;          % m
yEstimate = 0;          % m

headingTrue = 0;        
xTrue = 0;
yTrue = 0;

%% storage
xHistory = zeros(size(time));
yHistory = zeros(size(time));

xTrueHistory = zeros(size(time));
yTrueHistory = zeros(size(time));

%% odometry loop
for k = 2:length(time)

    % ticks that arrived this step
    ticksLeft = tickCountLeft(k) - tickCountLeft(k-1);
    ticksRight = tickCountRight(k) - tickCountRight(k-1);

    % convert to distances
    distanceLeft = ticksLeft * distancePerTick;
    distanceRight = ticksRight * distancePerTick;

    % motion of the robot in this step
    distanceCentre = (distanceRight + distanceLeft) / 2;
    headingChange = (distanceRight - distanceLeft) / wheelBase; % wheelbase is distance between wheels

    % update the believed positions
    headingEstimate = headingEstimate + headingChange;
    xEstimate = xEstimate + distanceCentre * cos(headingEstimate);
    yEstimate = yEstimate + distanceCentre * sin(headingEstimate);

    xHistory(k) = xEstimate;
    yHistory(k) = yEstimate;

    % true positions to compare for drift
    trueStepLeft = trueDistanceLeft(k) - trueDistanceLeft(k-1);
    trueStepRight = trueDistanceRight(k) - trueDistanceRight(k-1);
    trueStepCentre = (trueStepLeft + trueStepRight) / 2;
    trueHeadingChange = (trueStepRight - trueStepLeft) / wheelBase;

    headingTrue = headingTrue + trueHeadingChange;
    xTrue = xTrue + trueStepCentre * cos(headingTrue);
    yTrue = yTrue + trueStepCentre * sin(headingTrue);

    xTrueHistory(k) = xTrue;
    yTrueHistory(k) = yTrue;

end

%% plot estimated path against true path for drift
plot(xHistory, yHistory, 'LineWidth', 2);
hold on;
plot(xTrueHistory, yTrueHistory, '--', 'LineWidth', 2);
axis equal;
grid on;
xlabel('x position (m)');
ylabel('y position (m)');
title('path estimated from encoders');
legend('estimated path', 'true path', 'Location', 'best');



