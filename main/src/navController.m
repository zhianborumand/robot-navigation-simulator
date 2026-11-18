function [vLeftCmd, vRightCmd, targetIndex, finished] = navController(x, y, heading, waypoints, targetIndex, navParams)
% NAVCONTROLLER  Steers towards the current waypoint and moves on when it is reached.
%   Inputs:  believed pose (x, y, heading), waypoint list, current target number,
%            parameter struct (Kv, Ktheta, vMax, wheelMax, wheelBase, tolerance).
%   Outputs: left and right wheel speed commands (m/s), updated target number,
%            and a flag that is true once the final waypoint has been reached.

    finished = false;

    % distance to the current waypoint
    dx = waypoints(targetIndex, 1) - x;
    dy = waypoints(targetIndex, 2) - y;
    distance = sqrt(dx^2 + dy^2);

    % loop to check if reached target and move on
    if distance <= navParams.tolerance      % if robot is within tolerance, count as reached and move to next one (targetIndex + 1)
        targetIndex = targetIndex + 1;
        if targetIndex > size(waypoints, 1) % if target is past last row, course finished
            finished = true;
            vLeftCmd = 0;
            vRightCmd = 0;
            return;
        end
        dx = waypoints(targetIndex, 1) - x; % otherwise recalculate and head to new target
        dy = waypoints(targetIndex, 2) - y;
        distance = sqrt(dx^2 + dy^2);
    end

    % heading error, kept between -pi and pi
    desiredHeading = atan2(dy,dx);              % angle of the arrow from robot to target from x axis
    headingError = desiredHeading - heading;    % how far robot needs to turn
    headingError = atan2(sin(headingError), cos(headingError));     % forces angle into range -pi to pi

    % proportional controllers
    omegaCmd = navParams.Ktheta * headingError;             % commanded turn rate rad/s
    vCmd = min(navParams.vMax, navParams.Kv * distance);    % distance rule, robot slows down closer to target
    vCmd = vCmd * max(0, cos(headingError));                % alignment rule,

    % wheel speeds
    vRightCmd = vCmd + omegaCmd * navParams.wheelBase / 2;  % depending on heading, omega is positive or negative.
    vLeftCmd = vCmd - omegaCmd * navParams.wheelBase / 2;   % if negative, left wheel faster and turns.

    % limit wheel speeds, keeping the turn ratio
    maxSpeed = max(abs(vLeftCmd), abs(vRightCmd));
    if maxSpeed > navParams.wheelMax
        vLeftCmd = vLeftCmd * navParams.wheelMax / maxSpeed;    % keep both wheels to same scale
        vRightCmd = vRightCmd * navParams.wheelMax / maxSpeed;
    end
end
