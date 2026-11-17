clear; clc;
srcFolder = fullfile(fileparts(mfilename('fullpath')), '..', 'src');
if isfolder(srcFolder)
    addpath(srcFolder);
end

dt = 0.01;  tau = 0.3;  maxSpeed = 0.4;

% test 1: 75% duty for 3 s, checking the speed at 0.3 s, 1 s and 3 s
speed = 0;
for k = 1:300
    speed = motorModel(speed, 0.75, maxSpeed, tau, dt);
    if k == 30,  speedAt30 = speed;  end
    if k == 100, speedAt100 = speed; end
end
assert(abs(speedAt30 - 0.1915) < 1e-4, 'speed at 0.3 s is wrong');
assert(abs(speedAt100 - 0.2899) < 1e-4, 'speed at 1 s is wrong');
assert(abs(speed - 0.3) < 1e-3, 'final speed is wrong');

% test 2: duty above 100% must be limited (target 1.0 * 0.4 = 0.4 m/s)
speed = 0;
for k = 1:300
    speed = motorModel(speed, 1.5, maxSpeed, tau, dt);
end
assert(abs(speed - 0.4) < 1e-3, 'duty above 1 was not limited');

% test 3: reverse duty below -100% must be limited too
speed = 0;
for k = 1:300
    speed = motorModel(speed, -2, maxSpeed, tau, dt);
end
assert(abs(speed + 0.4) < 1e-3, 'duty below -1 was not limited');

disp('all motorModel tests passed');
