
clear;
clc;
close all;

dt = 0.02;
T = 80;
time = 0:dt:T;

%% robot and encoder settings
wheelRadiusAssumed = 0.033;        % wheel radius the robot software believes (m)
wheelRadiusLeftActual = 0.033;     % real left wheel radius (m)
wheelRadiusRightActual = 0.0325;   % real right wheel radius, slightly worn (m)
wheelBase = 0.20;           % distance between the wheels (m)
ticksPerRev = 360;          % encoder ticks per wheel revolution
distancePerTickAssumed = 2*pi*wheelRadiusAssumed / ticksPerRev;
distancePerTickLeftActual = 2*pi*wheelRadiusLeftActual / ticksPerRev;
distanceperTickRightActual = 2*pi*wheelRadiusRightActual / ticksPerRev;


%% true wheel speeds
trueSpeedLeft = 0.30;       % m/s
trueSpeedRight = 0.29;      % m/s

%% storage
trueDistanceLeft = zeros(size(time));
trueDistanceRight = zeros(size(time));
tickCountLeft = zeros(size(time));
tickCountRight = zeros(size(time));

%% noise settings
noiseStd = 0;               % typical tick error per step (set to 0 for no noise)
rng(1);                     % fix the random sequence so results can be repeated (use rng('shuffle'))
%% encoder loop
for k = 2:length(time)
    trueDistanceLeft(k) = trueDistanceLeft(k-1) + trueSpeedLeft*dt;
    trueDistanceRight(k) = trueDistanceRight(k-1) + trueSpeedRight*dt;

    % ticks a perfect encoder would count this step
    idealTicksLeft = floor(trueDistanceLeft(k) / distancePerTickLeftActual) - floor(trueDistanceLeft(k-1) / distancePerTickLeftActual);
    idealTicksRight = floor(trueDistanceRight(k) / distanceperTickRightActual) - floor(trueDistanceRight(k-1) / distanceperTickRightActual);

    % add random error, keep whole numbers, can be negative
    noisyTicksLeft = idealTicksLeft + round(noiseStd * randn);
    noisyTicksRight = idealTicksRight + round(noiseStd * randn);

    % running tick counts
    tickCountLeft(k) = tickCountLeft(k-1) + noisyTicksLeft;
    tickCountRight(k) = tickCountRight(k-1) + noisyTicksRight;
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
    distanceLeft = ticksLeft * distancePerTickAssumed;
    distanceRight = ticksRight * distancePerTickAssumed;

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



