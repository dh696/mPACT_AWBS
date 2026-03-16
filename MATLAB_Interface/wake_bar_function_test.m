clear
close all



% Initialise:
s = exp_AWBS_initialise("COM5", home=true, debug=true);

% pause(5) % reboots when serial starts, give time for it to setup
% Initialise function now waits for "STARTED" to be sent.

% Home:
% exp_AWBS_home(s);
% Can now be done through initialise function.


% Some test positions
x = [0, -30, 30, 0];
theta = [90, 45, 0, 0];

for i=1:4
    exp_AWBS_move(s, x(i), theta(i));
    % move function blocks until motion is complete, but space the moves out a bit:
    pause(5)
end

% Make sure to close serial at end of session otherwise it cannot be opened again.
s.delete()









