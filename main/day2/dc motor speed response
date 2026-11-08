
clear;
clc;
close all;

dt = 0.01;             % time step (s)
T = 3;                 % simulation duration (s)

time = 0:dt:T;

%% motor settings
targetSpeed = 0.30;    % requested wheel speed (m/s)
tau = 0.3;            % motor time constant (s)
wheelSpeed = zeros(size(time));

%% simulation loop

for k = 2:length(time)
    % calculate speed difference
    speedError = targetSpeed - wheelSpeed(k-1);

    % calculate change in speed
    speedChange = (dt/tau)*speedError;
    
    % update wheel speed
    wheelSpeed(k) = wheelSpeed(k-1) + speedChange;

end

%% plot results

figure('Color','w','Position',[100 100 900 500]);

plot(time,wheelSpeed,'b','LineWidth',2);
hold on;

yline(targetSpeed,'r--','LineWidth',1.5);

grid on;

xlabel('time (s)');
ylabel('wheel speed (m/s)');
title(sprintf('dc motor speed response (tau = %.2f s)',tau));

legend('actual wheel speed','requested wheel speed','Location','southeast');

xlim([0 T]);
ylim([0 0.35]);
