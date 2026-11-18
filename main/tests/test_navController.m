clear; clc;
srcFolder = fullfile(fileparts(mfilename('fullpath')), '..', 'src');
if isfolder(srcFolder)
    addpath(srcFolder);
end

navParams.Kv = 0.8;  navParams.Ktheta = 2.0;  navParams.vMax = 0.35;
navParams.wheelMax = 0.40;  navParams.wheelBase = 0.20;  navParams.tolerance = 0.10;
tol = 1e-4;

% test 1: target straight ahead -> both wheels at vMax
[vL, vR, idx, fin] = navController(0, 0, 0, [2 0], 1, navParams);
assert(abs(vL - 0.35) < tol && abs(vR - 0.35) < tol && idx == 1 && ~fin, 'ahead test failed');

% test 2: target behind -> spin on the spot, wheel speeds limited to wheelMax
[vL, vR, idx, fin] = navController(0, 0, 0, [-2 0], 1, navParams);
assert(abs(vL + 0.40) < tol && abs(vR - 0.40) < tol && idx == 1 && ~fin, 'behind test failed');

% test 3: target to the left -> turn left on the spot
[vL, vR, idx, fin] = navController(0, 0, 0, [0 2], 1, navParams);
assert(abs(vL + 0.31416) < tol && abs(vR - 0.31416) < tol && idx == 1 && ~fin, 'left test failed');

% test 4: target to the right -> turn right on the spot
[vL, vR, idx, fin] = navController(0, 0, 0, [0 -2], 1, navParams);
assert(abs(vL - 0.31416) < tol && abs(vR + 0.31416) < tol && idx == 1 && ~fin, 'right test failed');

% test 5: close to the target -> slows in proportion to distance (0.8 * 0.2 = 0.16)
[vL, vR, idx, fin] = navController(0, 0, 0, [0.2 0], 1, navParams);
assert(abs(vL - 0.16) < tol && abs(vR - 0.16) < tol && idx == 1 && ~fin, 'slow approach test failed');

% test 6: reached waypoint 1 -> switches to waypoint 2 at (2,2) and turns left
[vL, vR, idx, fin] = navController(2, 0, 0, [2 0; 2 2], 1, navParams);
assert(abs(vL + 0.31416) < tol && abs(vR - 0.31416) < tol && idx == 2 && ~fin, 'waypoint switch test failed');

% test 7: reached the last waypoint -> finished, wheels stopped
[vL, vR, idx, fin] = navController(2, 0, 0, [2 0], 1, navParams);
assert(vL == 0 && vR == 0 && idx == 2 && fin, 'finished test failed');

disp('all navController tests passed');
