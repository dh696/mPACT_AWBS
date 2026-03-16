clear
close all

% The new serialport API causes the DTR pin to toggle, rebooting the esp32. This isn't ideal.
s = serialport("COM5", 115200);

% So using the old API, can disable DTR before opening the port, requires a few more lines:
%{
s = serial("COM5");
s.BaudRate = 115200;
s.DataterminalReady = 'off';
fopen(s);
%}

s.Timeout = 20;  % 20 second timeout
try


s.flush()
s.writeline("home rot");
for i=1:10
    returnmsg = s.readline();
    fprintf(returnmsg + "\n");
    if contains(returnmsg, "COMPLETED")
        fprintf("Completed home\n")
        break
    end
end






catch ME
s.delete()
fprintf("ERROR! Serial port closed.\n")
rethrow(ME)
end


s.delete()
