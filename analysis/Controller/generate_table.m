function table = generate_table(delx,dely)
% Delx, Dely are arguments that determine the coarseness of the table and hence accuracy of interpolation
% Generate lookup table for (h, hdot) --> (Servo angle command)

% Max altitude and vertical velocity expected
hmax = 3000; % (ft)
hdotmax = 500; % (ft/s)
% Allocate table to store all possible h, hdot combinations
table = zeros(round((hmax + 100)/delx), round((hdotmax + 100)/dely));
% Rocket parameters
Cd_r = .5; % --- changeme ----
A_r = .1; % (m2)

yaah = 0;

% Populate table
for i = delx*(1:size(table, 1))
    for j = dely*(1:size(table, 2))
        % Use spacing of 1 so values = own index
        % i = h
        % j = hdot
        yaah = yaah + 1
        area = find_area(i, j);
        table(round(i/delx), round(j/dely)) = find_angle(area);
    end
end
