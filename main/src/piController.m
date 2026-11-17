function [duty, integralNew] = piController(referenceSpeed, measuredSpeed, integral, Kp, Ki, dt)
% PICONTROLLER  PI speed controller for one wheel and one time step, with anti-windup.
%   Inputs:  reference and measured speed (m/s), accumulated error so far,
%            gains Kp and Ki, time step (s).
%   Outputs: duty (-1 to 1) and the updated accumulated error.

speedError = referenceSpeed - measuredSpeed;                       
integralNew = integral + speedError * dt;                      
dutyRaw = Kp * speedError + Ki * integralNew;                          

% anti-windup: if this would saturate the duty, stop accumulating
if (dutyRaw > 1 && speedError > 0) || (dutyRaw < -1 && speedError < 0)                                  
    integralNew = integral;                           % freeze the integral
    dutyRaw = Kp * speedError + Ki * integralNew;     % recompute with the frozen value
end

% limit the duty to what the hardware can do
duty = min(max(dutyRaw, -1), 1);
end
