function [] = exp_AWBS_sendCommand(s, command, options)
%EXP_AWBS_SENDCOMMAND Send specified command string to automatic wake bar system.
%   "s" is the open serialport objects
%   "command" is a string
%   "waitTime" in seconds, if greater than zero, will hold program until completed message 
%   is returned, or throw and error if it times out. If -1, will wait for max timeout in setup. Default 0.
%   "returnLines" is maximum number of lines to read when looking for completion signal. Default 10.
%   "debug" is a boolean, if true prints all messages sent and received by wake bar

arguments
    s
    command
    options.waitTime = 0
    options.returnLines = 10;
    options.debug = false;
end
waitTime = options.waitTime;
returnLines = options.returnLines;

if waitTime > 0
    oldTimeout = s.Timeout;
    s.Timeout = waitTime;
end


try

s.flush()
if options.debug
    fprintf("TX: %s\n",command)
end
s.writeline(command)
if waitTime ~= 0
    for i=1:returnLines
        returnmsg = s.readline();
        if size(returnmsg, 1) > 0
            if options.debug
                fprintf("RX: %s\n",returnmsg)
            end
            if contains(returnmsg, "COMPLETED")
                return;
            elseif contains(returnmsg, "WARNING") | contains(returnmsg, "ERROR")
                error("Wake Bar Error: %s\nFor command: %s", returnmsg, command);
            end 
        end
    end
    error("Wake Bar Timeout: completion signal was not received for command %s", command);
end

catch ME
    % if there is an error, make sure to close serial
    s.delete()
    rethrow(ME)
end

if waitTime > 0
    s.Timeout = oldTimeout;
end

end

