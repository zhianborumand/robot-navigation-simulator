function tickCount = encoderModel(trueDistance, trueDistancePrevious, tickCountPrevious, distancePerTickActual, noiseStd)
% ENCODERMODEL Simulates one wheel's encoder for one time step.
%   Returns the new running tick count, with quantisation and random noise.

    % ticks a perfect encoder would count this step
    idealTicks = floor(trueDistance / distancePerTickActual) - floor(trueDistancePrevious / distancePerTickActual);

    % add random error and keep whole number
    noisyTicks = idealTicks + round(noiseStd * randn);

    % runnin count: the previous count plus this step's ticks
    tickCount = tickCountPrevious + noisyTicks;
end
