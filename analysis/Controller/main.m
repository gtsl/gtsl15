delx = 5;
dely = 5;
table = generate_table(delx, dely);
full_table = imresize(table, delx);
full_table(full_table > 90) = 90;
full_table(full_table < 0) = 0;

% % Plot
% surfc(full_table, 'EdgeColor', 'none')
% xlabel('Vertical Velocity (ft/s)')
% ylabel('Altitude (ft)')
% zlabel('Pin Angle (deg)')

% Convert to control_matrix
% for hdot: 

% Write to file
addpath(genpath('../../systems/rocket/FlightSoftware/testing/'));
write_text(table(101:end,:), delx)