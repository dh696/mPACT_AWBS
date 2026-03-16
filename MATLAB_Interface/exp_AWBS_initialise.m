function [s] = exp_AWBS_initialise(port, options)
%EXP_AWBS_INITIALISE Start serial coms with automatic wake bar system.
%   Specify port as string, e.g. "COM1"
%   Returns a serialport object

arguments
    port
    options.home = false;
    options.debug = false;
end

if options.debug
    fprintf("Opening serial port: %s\n", port)
end
s = serialport(port, 115200);

try

s.Timeout = 20;  % set to max time a move is expected to take.
% When serialPort opens, the DTR pin toggles, causing the ESP32 to reboot. 
% In case it does not reboot, can send reboot command to check it is running sucessfully:
% exp_AWBS_sendCommand(s, "reboot", debug=options.debug);

for i=1:20
    returnmsg = s.readline();
    if options.debug
        fprintf("RX: %s\n", returnmsg);
    end
    if contains(returnmsg, "STARTED")
        if options.debug
            exp_AWBS_home(s, debug=options.debug);
        end
        return;
    elseif contains(returnmsg, "WARNING") | contains(returnmsg, "ERROR")
        error("Wake Bar Error: %s\nOn startup: %s", returnmsg, command);
    end
end

error("Wake Bar Error: No Connection on %s", port)

catch ME
    s.delete();
    rethrow(ME);

end

end

