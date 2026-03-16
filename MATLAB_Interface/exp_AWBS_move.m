function [] = exp_AWBS_move(s, x, theta, options)
%EXP_AWBS_MOVE Move the automatic wake bar to linear position x and angle theta.
%   "s" is the open serialport object,
%   "x" is the linear distance from the centre in mm,
%   "theta" is the roll angle of the bar in degrees from 0 (parallel to probe stem).
%   This function blocks until the move is completed.
%   Rotation is done first, then linear motion.
%   This function uses the default speed of the system, but custom speed/acceleration can be used by sending the full
%   command "move <axis> <pos> <speed> <accel>".

arguments
    s
    x
    theta
    options.debug = false;
end


% Check limits of motion:
% Allow 0-90deg rotation, +/- 30mm linear travel.
% The system can go slightly further than this for setting up home positions etc, but this should be done using the
% serial terminal directly, as it gives warnings on the absolute limits.

if (theta >= 0 && theta <= 90)
    exp_AWBS_sendCommand(s, sprintf("move rot %.2f", theta), debug=options.debug);
else
    error("Wake Bar Error: Rotation angle theta = %.2f deg is outside of limits.", theta);
end
if (x >= -30 && x <= 30)
    exp_AWBS_sendCommand(s, sprintf("move lin %.2f", x), debug=options.debug);
else
    error("Wake Bar Error: Linear position x = %.2f mm is outside of limits.", x);
end

end

