
clear;
clc;
close all;

dt = 0.01;                      % time step (s)
T = 3;                          % simulation duration (s)
time = 0:dt:T;                  % every simulated moment

%% motor settings
tau = 0.30;                     % motor time constant (s)
maxSpeed = 0.40;                % wheel speed at 100% duty (m/s)
referenceSpeed = 0.30;          % speed we want (m/s)

wheelSpeed = 0;                 % actual speed
speedError = 0;                 % gap between needed and actual speed

%% tuning
Kp = 2;                         % proportional gain
Ki = 0;                         % integral gain (start at 0)

%% storage
errorIntegral = 0;
wheelSpeed = zeros(size(time)); % array of zeros for actual speed at each instance
duty = zeros(size(time));       % array for duty

%% loop
for k = 2: length(time)
    speedError = referenceSpeed - wheelSpeed(k-1);
    errorIntegral = errorIntegral + speedError * dt;
    dutyRaw = Kp * speedError + Ki * errorIntegral;         
    duty(k) = min(max(dutyRaw, -1), 1);                     % limits duty to allowed range
    targetSpeed = duty(k) * maxSpeed;
    wheelSpeed(k) = wheelSpeed(k-1) + (dt/tau) * (targetSpeed - wheelSpeed(k-1));
end

%% plot
figure('Color','w','Position',[100 100 900 650]);

subplot(2,1,1);
plot(time, wheelSpeed, 'b', 'LineWidth', 2);
hold on;
yline(referenceSpeed, 'r--', 'LineWidth', 1.5);
grid on;
xlabel('time (s)');
ylabel('wheel speed (m/s)');
legend('actual wheel speed', 'reference speed', 'Location', 'southeast');
title('PI speed controller response');

subplot(2,1,2);
plot(time, duty, 'k', 'LineWidth', 2);
grid on;
xlabel('time (s)');
ylabel('duty cycle (fraction');
title('controller output');

sgtitle(sprintf('Kp = %.1f | Ki = %.1f', Kp, Ki));
