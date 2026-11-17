function speedNew = motorModel(speed, duty, maxSpeed, tau, dt)
% MOTORMODEL  First-order DC motor model for one wheel and one time step.
%  Inputs: current speed (m/s), PWM duty (-1to1), speed at full duty (m/s),
%          time constant (s), time step (s).
%  Output: new wheel speed (m/s).

    % a real motor cannot be driven beyond 100% duty either way
    duty = min(max(duty, -1), 1);

    % the speed the motor is heading towards
    targetSpeed = duty * maxSpeed;

    % move a fraction (dt/tau) of the way towards it
    speedNew = speed + (dt / tau) * (targetSpeed - speed);
end
