clear; clc;
srcFolder = fullfile(fileparts(mfilename('fullpath')), '..', 'src');
if isfolder(srcFolder)
    addpath(srcFolder);
end

dt = 0.01;  tau = 0.3;  maxSpeed = 0.4;

% test 1: P only (Ki = 0) cannot reach 0.30 m/s, it settles at about 0.1333
speed = 0;  integral = 0;
for k = 1:300
    [duty, integral] = piController(0.30, speed, integral, 2, 0, dt);
    speed = motorModel(speed, duty, maxSpeed, tau, dt);
end
assert(abs(speed - 0.1333) < 1e-3, 'P-only final speed is wrong');
assert(abs(duty - 0.3333) < 1e-3, 'P-only final duty is wrong');

% test 2: PI reaches 0.30 m/s and the duty settles at 0.30/0.40 = 0.75
speed = 0;  integral = 0;
for k = 1:300
    [duty, integral] = piController(0.30, speed, integral, 2, 10, dt);
    speed = motorModel(speed, duty, maxSpeed, tau, dt);
end
assert(abs(speed - 0.30) < 1e-3, 'PI final speed is wrong');
assert(abs(duty - 0.75) < 1e-3, 'PI final duty is wrong');

% test 3: anti-windup. Unreachable reference of 1.0 m/s for 2 s, then 0.2 m/s
speed = 0;  integral = 0;
for k = 1:500
    if k <= 200
        reference = 1.0;
    else
        reference = 0.2;
    end
    [duty, integral] = piController(reference, speed, integral, 2, 10, dt);
    speed = motorModel(speed, duty, maxSpeed, tau, dt);
    if k == 200
        integralAt2s = integral;
    end
end
assert(integralAt2s < 0.01, 'the integral wound up');
assert(abs(speed - 0.2) < 0.01, 'did not recover to 0.2 m/s');

disp('all piController tests passed');
