function [xNew, yNew, headingNew] = odometryUpdate(x, y, heading, ticksLeft, ticksRight, distancePerTickAssumed, wheelBase)
% ODOMETRYUPDATE Updates believed robot pose from one step of encoder ticks
%   Inputs:  previous pose (x, y, heading), ticks this step for each wheel,
%            assumed distance per tick (m), wheel spacing (m).
%   Outputs: new believed pose (xNew, yNew, headingNew).

    % convert ticks to distances
    distanceLeft = ticksLeft * distancePerTickAssumed;
    distanceRight = ticksRight * distancePerTickAssumed;

    % motion of the robot in this step
    distanceCentre = (distanceRight + distanceLeft) / 2;
    headingChange = (distanceRight - distanceLeft) / wheelBase; % wheelbase is distance between wheels

    % update the believed pose (position uses the old heading, heading is
    % updated last)
    xNew = x + distanceCentre * cos(heading);
    yNew = y + distanceCentre * sin(heading);
    headingNew = heading + headingChange;
end
