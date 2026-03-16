function [] = exp_AWBS_home(s, options)
%EXP_AWBS_HOME Home both axes of the automatic wake bar system.
%   "s" is the open serialport object.
%   This function blocks until the homing process completed.

arguments
    s
    options.debug = false;
end


exp_AWBS_sendCommand(s, "home all", waitTime=-1, debug=options.debug);

end

