% Generate lookup table for (h, hdot) --> (Servo angle command)

% Max altitude and vertical velocity expected
hmax = 3000; % (ft)
hdotmax = 500; % (ft/s)
% Allocate table to store all possible h, hdot combinations
table = zeros(hmax + 100, hdotmax + 100);
% Rocket parameters
Cd_r = .5; % --- changeme ----
A_r = .1; % (m2)

% Populate table
for i = 1:size(table, 1)
    for j = 1:size(table, 2)
        % Use spacing of 1 so values = own index
        % i = h
        % j = hdot
        area = find_area(i, j, Cd_r, A_r);
        table(i, j) = find_angle(area);
    end
end