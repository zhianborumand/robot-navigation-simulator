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

%% motor settings (left and right motors can differ)
tauLeft = 0.30;                         % left motor time constant (s)
tauRight = 0.80;                        % right motor time constant (s)
efficiencyLeft = 1.0;                   % 1.0 = normal motor, 0.85 = 15% weaker
efficiencyRight = 1.0;
maxSpeedNominal = navParams.wheelMax;   % full-duty speed the controller ASSUMES (m/s)
maxSpeedLeft = efficiencyLeft * maxSpeedNominal;     % what the real motors actually reach (m/s)
maxSpeedRight = efficiencyRight * maxSpeedNominal;

%% speed controller settings
usePI = false;          % false = open loop, true = PI feedback
Kp = 3;                 % proportional gain
Ki = 6;                 % integral gain
integralLeft = 0;       % accumulated error, left wheel
integralRight = 0;      % accumulated error, right wheel


%% storage
xHistory = zeros(size(time));
yHistory = zeros(size(time));
elapsedTime = 0;
lastStep = 1;

speedLeft = zeros(size(time));
speedRight = zeros(size(time));
dutyLeftHistory = zeros(size(time));
dutyRightHistory = zeros(size(time));
vLeftCmdHistory = zeros(size(time));
vRightCmdHistory = zeros(size(time));

%% simulation loop
for k = 2:length(time)

    % 1. navigation: where do the wheels go? call function
    [vLeftCmd, vRightCmd, targetIndex, finished] = navController(x, y, theta, waypoints, targetIndex, navParams);
    if finished
        break;
    end

    % 2. speed controller: choose a duty for each wheel
    if usePI
        [dutyLeft, integralLeft] = piController(vLeftCmd, speedLeft(k-1), integralLeft, Kp, Ki, dt);
        [dutyRight, integralRight] = piController(vRightCmd, speedRight(k-1), integralRight, Kp, Ki, dt);
    else
        % open loop: assume full duty gives the nominal maximum speed
        dutyLeft = vLeftCmd / maxSpeedNominal;
        dutyRight = vRightCmd / maxSpeedNominal;
    end

    % 3. motors respond to the duty
    speedLeft(k) = motorModel(speedLeft(k-1), dutyLeft, maxSpeedLeft, tauLeft, dt);
    speedRight(k) = motorModel(speedRight(k-1), dutyRight, maxSpeedRight, tauRight, dt);


    % 4. the real robot moves (ideal wheels)
    v = (speedRight(k) + speedLeft(k)) / 2;                         % forward speed from the two wheel speeds
    omega = (speedRight(k) - speedLeft(k)) / navParams.wheelBase;   % turn rate from the two wheel speeds

    x = x + v * cos(theta) * dt;
    y = y + v * sin(theta) * dt;
    theta = theta + omega * dt;

    xHistory(k) = x;
    yHistory(k) = y;
    dutyLeftHistory(k) = dutyLeft;
    dutyRightHistory(k) = dutyRight;
    vLeftCmdHistory(k) = vLeftCmd;
    vRightCmdHistory(k) = vRightCmd;
    elapsedTime = time(k);
    lastStep = k;
end

%% results
distanceToFinish = sqrt((waypoints(end, 1) - x)^2 + (waypoints(end, 2) - y)^2);
fprintf('waypoints reached: %d of %d\n', min(targetIndex - 1, size(waypoints,1)), size(waypoints,1));
fprintf('simulation time: %.2f s\n', elapsedTime);
fprintf('final position: (%.3f, %.3f) m\n', x, y);
fprintf('distance to finish: %.3f m\n', distanceToFinish);

speedTrackingError = sqrt(mean([(vLeftCmdHistory(2:lastStep) - speedLeft(2:lastStep)).^2, ...
    (vRightCmdHistory(2:lastStep) - speedRight(2:lastStep)).^2]));
fprintf('rms speed tracking error: %.4f m/s\n', speedTrackingError);

%% plot
figure('Color','w');
plot(xHistory(1:lastStep), yHistory(1:lastStep), 'b', 'LineWidth', 2);
hold on;
plot(waypoints(:,1), waypoints(:,2), 'rx', 'MarkerSize', 12, 'LineWidth', 2);
axis equal;  grid on;
xlabel('x position (m)');
ylabel('y position (m)');
title('stage 2: waypoint navigation with motors');

figure('Color','w','Position',[100 100 900 650]);
n = lastStep;

subplot(2,1,1);
plot(time(1:n), speedLeft(1:n), 'b', 'LineWidth', 1.5);
hold on;
plot(time(1:n), speedRight(1:n), 'r', 'LineWidth', 1.5);
plot(time(1:n), vLeftCmdHistory(1:n), 'b:', 'LineWidth', 1);
plot(time(1:n), vRightCmdHistory(1:n), 'r:', 'LineWidth', 1);
grid on;
xlabel('time (s)');
ylabel('wheel speed (m/s)');
title('wheel speeds: actual (solid) and commanded (dotted)');
legend('left actual', 'right actual', 'left command', 'right command', 'Location', 'best');

subplot(2,1,2);
plot(time(1:n), dutyLeftHistory(1:n) * 100, 'b', 'LineWidth', 1.5);
hold on;
plot(time(1:n), dutyRightHistory(1:n) * 100, 'r', 'LineWidth', 1.5);
grid on;
xlabel('time (s)');
ylabel('duty cycle (%)');
ylim([-110 110]);
title('PWM duty cycles');
legend('left', 'right', 'Location', 'best');
