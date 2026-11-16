clear;
clc;
close all;

dt = 0.02;
T = 80;
time = 0:dt:T;
L = 0.20;

%% motor settings
tauL = 0.30;
tauR = 0.80;
wheelMax = 0.40;

%% pi controller settings
usePI = true;
useEncoderSpeed = true;
Kp = 3;
Ki = 6;
integralLimit = 0.15;
integralL = 0;
integralR = 0;

%% encoder settings
wheelRadiusAssumed = 0.033;
wheelRadiusLeftActual = 0.033;
wheelRadiusRightActual = 0.033;
ticksPerRev = 360;
distancePerTickAssumed = 2*pi*wheelRadiusAssumed/ticksPerRev;
distancePerTickLeftActual = 2*pi*wheelRadiusLeftActual/ticksPerRev;
distancePerTickRightActual = 2*pi*wheelRadiusRightActual/ticksPerRev;
noiseStd = 0;
rng(1);

%% robot starting position
x = 0;
y = 0;
theta = 0;
xEstimate = 0;
yEstimate = 0;
headingEstimate = 0;

%% waypoints
waypoints = [
    2 0;
    2 2;
    0 2;
    0 0
];

currentTarget = 1;
tolerance = 0.10;

%% navigation controller
Kv = 0.8;
Ktheta = 2.0;
vMax = 0.35;

%% create arrays
speedL = zeros(size(time));
speedR = zeros(size(time));
dutyL = zeros(size(time));
dutyR = zeros(size(time));
trueDistanceLeft = zeros(size(time));
trueDistanceRight = zeros(size(time));
tickCountLeft = zeros(size(time));
tickCountRight = zeros(size(time));
xTrueHistory = zeros(size(time));
yTrueHistory = zeros(size(time));
xHistory = zeros(size(time));
yHistory = zeros(size(time));
positionError = zeros(size(time));
measuredSpeedLeft = zeros(size(time));
measuredSpeedRight = zeros(size(time));
vLcmdHistory = zeros(size(time));
vRcmdHistory = zeros(size(time));

%% create figure
figure('Color','w','Position',[100 100 1150 650]);
axes('Position',[0.07 0.13 0.55 0.78]);
hold on;
grid on;
axis equal;
axis([-0.5 2.5 -0.5 2.5]);
xlabel('x position (m)');
ylabel('y position (m)');
if useEncoderSpeed
    feedbackText = 'encoder speed';
else
    feedbackText = 'true speed';
end
title(sprintf('waypoint navigation, pi feedback: %s (%d ticks/rev)',feedbackText,ticksPerRev));

plot([0;waypoints(:,1)],[0;waypoints(:,2)],'k--','LineWidth',1);
plot(waypoints(:,1),waypoints(:,2),'rx','MarkerSize',12,'LineWidth',2);

for j = 1:size(waypoints,1)
    text(waypoints(j,1)+0.1,waypoints(j,2)+0.1,num2str(j));
end

robot = plot(x,y,'bo','MarkerSize',12,'MarkerFaceColor','b');
heading = quiver(x,y,0.2*cos(theta),0.2*sin(theta),0,'r','LineWidth',2);
trail = animatedline('Color','b','LineWidth',2);
estimateTrail = animatedline('Color',[0.85 0.33 0.10],'LineWidth',1.5,'LineStyle','--');
addpoints(trail,x,y);
addpoints(estimateTrail,xEstimate,yEstimate);
legend([trail estimateTrail],{'true path','encoder estimate'},'Location','northeast');

%% simulation loop
elapsedTime = 0;

for k = 2:length(time)

    dx = waypoints(currentTarget,1)-x;
    dy = waypoints(currentTarget,2)-y;
    distance = sqrt(dx^2+dy^2);

    if distance <= tolerance
        currentTarget = currentTarget+1;

        if currentTarget > size(waypoints,1)
            break;
        end

        dx = waypoints(currentTarget,1)-x;
        dy = waypoints(currentTarget,2)-y;
        distance = sqrt(dx^2+dy^2);
    end

    desiredHeading = atan2(dy,dx);
    headingError = desiredHeading-theta;
    headingError = atan2(sin(headingError),cos(headingError));

    omegaCmd = Ktheta*headingError;
    vCmd = min(vMax,Kv*distance);
    vCmd = vCmd*max(0,cos(headingError));

    vRcmd = vCmd+omegaCmd*L/2;
    vLcmd = vCmd-omegaCmd*L/2;

    maxSpeed = max(abs(vLcmd),abs(vRcmd));

    if maxSpeed > wheelMax
        vLcmd = vLcmd*wheelMax/maxSpeed;
        vRcmd = vRcmd*wheelMax/maxSpeed;
    end

    vLcmdHistory(k) = vLcmd;
    vRcmdHistory(k) = vRcmd;

    if useEncoderSpeed
        errorL = vLcmd-measuredSpeedLeft(k-1);
        errorR = vRcmd-measuredSpeedRight(k-1);
    else
        errorL = vLcmd-speedL(k-1);
        errorR = vRcmd-speedR(k-1);
    end

    if usePI
        integralL = integralL+errorL*dt;
        integralR = integralR+errorR*dt;
        integralL = max(-integralLimit,min(integralLimit,integralL));
        integralR = max(-integralLimit,min(integralLimit,integralR));
        pwmL = Kp*errorL+Ki*integralL;
        pwmR = Kp*errorR+Ki*integralR;
    else
        pwmL = vLcmd/wheelMax;
        pwmR = vRcmd/wheelMax;
    end

    pwmL = max(-1,min(1,pwmL));
    pwmR = max(-1,min(1,pwmR));
    dutyL(k) = pwmL;
    dutyR(k) = pwmR;

    targetL = pwmL*wheelMax;
    targetR = pwmR*wheelMax;

    speedL(k) = speedL(k-1)+(dt/tauL)*(targetL-speedL(k-1));
    speedR(k) = speedR(k-1)+(dt/tauR)*(targetR-speedR(k-1));

    %% true robot motion
    v = (speedR(k)+speedL(k))/2;
    omega = (speedR(k)-speedL(k))/L;

    x = x+v*cos(theta)*dt;
    y = y+v*sin(theta)*dt;
    theta = theta+omega*dt;

    %% wheel encoders
    trueDistanceLeft(k) = trueDistanceLeft(k-1)+speedL(k)*dt;
    trueDistanceRight(k) = trueDistanceRight(k-1)+speedR(k)*dt;

    idealTicksLeft = floor(trueDistanceLeft(k)/distancePerTickLeftActual)-floor(trueDistanceLeft(k-1)/distancePerTickLeftActual);
    idealTicksRight = floor(trueDistanceRight(k)/distancePerTickRightActual)-floor(trueDistanceRight(k-1)/distancePerTickRightActual);

    noisyTicksLeft = idealTicksLeft+round(noiseStd*randn);
    noisyTicksRight = idealTicksRight+round(noiseStd*randn);

    tickCountLeft(k) = tickCountLeft(k-1)+noisyTicksLeft;
    tickCountRight(k) = tickCountRight(k-1)+noisyTicksRight;

    %% odometry estimate
    ticksLeft = tickCountLeft(k)-tickCountLeft(k-1);
    ticksRight = tickCountRight(k)-tickCountRight(k-1);
    distanceLeft = ticksLeft*distancePerTickAssumed;
    distanceRight = ticksRight*distancePerTickAssumed;
    measuredSpeedLeft(k) = distanceLeft/dt;
    measuredSpeedRight(k) = distanceRight/dt;
    distanceCentre = (distanceRight+distanceLeft)/2;
    headingChange = (distanceRight-distanceLeft)/L;

    xEstimate = xEstimate+distanceCentre*cos(headingEstimate);
    yEstimate = yEstimate+distanceCentre*sin(headingEstimate);
    headingEstimate = headingEstimate+headingChange;

    %% record results
    xTrueHistory(k) = x;
    yTrueHistory(k) = y;
    xHistory(k) = xEstimate;
    yHistory(k) = yEstimate;
    positionError(k) = sqrt((xEstimate-x)^2+(yEstimate-y)^2);

    elapsedTime = time(k);

    set(robot,'XData',x,'YData',y);
    set(heading,'XData',x,'YData',y,'UData',0.2*cos(theta),'VData',0.2*sin(theta));
    addpoints(trail,x,y);
    addpoints(estimateTrail,xEstimate,yEstimate);
    drawnow;
end

%% motor speed graph
n = round(elapsedTime/dt)+1;

axes('Position',[0.70 0.64 0.27 0.23]);
plot(time(1:n),speedL(1:n),'b','LineWidth',1.5);
hold on;
plot(time(1:n),speedR(1:n),'r','LineWidth',1.5);
grid on;
xlabel('time (s)');
ylabel('speed (m/s)');
title('motor speeds');
legend('left','right','Location','best');

%% pwm graph
axes('Position',[0.70 0.34 0.27 0.22]);
plot(time(1:n),dutyL(1:n)*100,'b','LineWidth',1.5);
hold on;
plot(time(1:n),dutyR(1:n)*100,'r','LineWidth',1.5);
grid on;
xlabel('time (s)');
ylabel('pwm (%)');
title('motor pwm commands');
ylim([-110 110]);

%% final results
completed = min(currentTarget-1,size(waypoints,1));
finalError = sqrt((waypoints(end,1)-x)^2+(waypoints(end,2)-y)^2);
finalPositionError = positionError(n);
maxPositionError = max(positionError(1:n));
rmsPositionError = sqrt(mean(positionError(1:n).^2));
finalHeadingError = atan2(sin(headingEstimate-theta),cos(headingEstimate-theta));
speedTrackingError = sqrt(mean([(vLcmdHistory(2:n)-speedL(2:n)).^2 (vRcmdHistory(2:n)-speedR(2:n)).^2]));
speedMeasurementError = sqrt(mean([(measuredSpeedLeft(2:n)-speedL(2:n)).^2 (measuredSpeedRight(2:n)-speedR(2:n)).^2]));
pwmChange = sqrt(mean([diff(dutyL(1:n)).^2 diff(dutyR(1:n)).^2]));

if completed == size(waypoints,1)
    statusText = 'course completed';
else
    statusText = 'course incomplete';
end

if usePI
    controllerText = 'pi enabled';
else
    controllerText = 'pi disabled';
end

resultsBox = annotation('textbox',[0.70 0.02 0.27 0.25],'FontSize',10,'BackgroundColor',[0.96 0.96 0.96]);

set(resultsBox,'String',{
    ['status: ' statusText]
    ['controller: ' controllerText ', ' feedbackText]
    sprintf('waypoints reached: %d / %d',completed,size(waypoints,1))
    sprintf('simulation time: %.2f s',elapsedTime)
    sprintf('rms position error: %.1f mm',rmsPositionError*1000)
    sprintf('rms speed tracking error: %.4f m/s',speedTrackingError)
    sprintf('rms speed measurement error: %.4f m/s',speedMeasurementError)
    sprintf('rms pwm change per step: %.2f %%',pwmChange*100)
});

drawnow;

%% measured speed against true speed
figure('Color','w','Position',[150 150 900 700]);

subplot(3,1,1);
plot(time(1:n),measuredSpeedLeft(1:n),'Color',[0.6 0.6 1],'LineWidth',1);
hold on;
plot(time(1:n),speedL(1:n),'b','LineWidth',2);
grid on;
ylabel('left speed (m/s)');
title('left wheel');
legend('measured speed','true speed','Location','best');

subplot(3,1,2);
plot(time(1:n),measuredSpeedRight(1:n),'Color',[1 0.6 0.6],'LineWidth',1);
hold on;
plot(time(1:n),speedR(1:n),'r','LineWidth',2);
grid on;
ylabel('right speed (m/s)');
title('right wheel');
legend('measured speed','true speed','Location','best');

subplot(3,1,3);
plot(time(1:n),positionError(1:n)*1000,'k','LineWidth',1.5);
grid on;
xlabel('time (s)');
ylabel('error (mm)');
title('encoder position error');

sgtitle(sprintf('measured wheel speed from encoder ticks vs true speed (%d ticks/rev)',ticksPerRev));

fprintf('pi feedback: %s, ticks per rev: %d\n',feedbackText,ticksPerRev);
fprintf('simulation time: %.2f s\n',elapsedTime);
fprintf('final position error: %.2f mm\n',finalPositionError*1000);
fprintf('max position error: %.2f mm\n',maxPositionError*1000);
fprintf('rms position error: %.2f mm\n',rmsPositionError*1000);
fprintf('final heading error: %.3f deg\n',finalHeadingError*180/pi);
fprintf('rms speed tracking error: %.4f m/s\n',speedTrackingError);
fprintf('rms speed measurement error: %.4f m/s\n',speedMeasurementError);
fprintf('rms pwm change per step: %.2f %%\n',pwmChange*100);
