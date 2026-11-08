
clear;
clc;
close all;

dt = 0.01;                  % time step (s)
T = 3;                      % simulation duration (s)
time = 0:dt:T;

%% motor settings
tauL = 0.30;                % left motor time constant (s)
tauR = 0.30;                % right motor time constant (s)
maxSpeed = 0.40;            % maximum wheel speed (m/s)
duty = 0.75;                % pwm duty cycle (75%)
targetSpeed = duty*maxSpeed;

%% create arrays
speedL = zeros(size(time));
speedR = zeros(size(time));
speedDifference = zeros(size(time));

%% simulation loop

for k = 2:length(time)

    % calculate left motor speed
    errorL = targetSpeed - speedL(k-1);
    speedL(k) = speedL(k-1) + (dt/tauL)*errorL;

    % calculate right motor speed
    errorR = targetSpeed - speedR(k-1);
    speedR(k) = speedR(k-1) + (dt/tauR)*errorR;

    % calculate difference between motors
    speedDifference(k) = speedL(k) - speedR(k);

end

%% plot results

figure('Color','w','Position',[100 100 1000 650]);

% motor speed comparison
subplot(2,1,1);
plot(time,speedL,'b','LineWidth',2);
hold on;
plot(time,speedR,'r','LineWidth',2);
yline(targetSpeed,'k--','LineWidth',1.5);
grid on;
xlabel('time (s)');
ylabel('wheel speed (m/s)');
title('left and right motor response');
legend('left motor','right motor','requested speed','Location','southeast');

% motor speed difference
subplot(2,1,2);
plot(time,speedDifference,'m','LineWidth',2);
grid on;
xlabel('time (s)');
ylabel('speed difference (m/s)');
title('difference between motor speeds');

sgtitle(sprintf('motor comparison | tauL = %.2f s | tauR = %.2f s',tauL,tauR));
