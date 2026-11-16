clear; clc;
srcFolder = fullfile(fileparts(mfilename('fullpath')), '..', 'src');
if isfolder(srcFolder)
    addpath(srcFolder);
end

wheelBase = 0.20;                              % distance between the wheels (m)
distancePerTickAssumed = 2*pi*0.033/360;       % about 0.000576 m per tick
tol = 1e-6;                                    % how close counts as "equal"

% test 1: equal ticks, facing along x -> straight line, no turn
[x, y, h] = odometryUpdate(0, 0, 0, 10, 10, distancePerTickAssumed, wheelBase);
assert(abs(x - 0.005760) < tol && abs(y - 0) < tol && abs(h - 0) < tol, 'straight test failed');

% test 2: opposite ticks -> spins on the spot
[x, y, h] = odometryUpdate(0, 0, 0, -10, 10, distancePerTickAssumed, wheelBase);
assert(abs(x - 0) < tol && abs(y - 0) < tol && abs(h - 0.057596) < tol, 'spin test failed');

% test 3: equal ticks while facing along y -> moves in y only
[x, y, h] = odometryUpdate(0, 0, pi/2, 10, 10, distancePerTickAssumed, wheelBase);
assert(abs(x - 0) < tol && abs(y - 0.005760) < tol && abs(h - 1.570796) < tol, 'facing-y test failed');

% test 4: unequal ticks (40 left, 50 right) -> gentle curve
[x, y, h] = odometryUpdate(0, 0, 0, 40, 50, distancePerTickAssumed, wheelBase);
assert(abs(x - 0.025918) < tol && abs(y - 0) < tol && abs(h - 0.028798) < tol, 'curve test failed');

disp('all odometryUpdate tests passed');
