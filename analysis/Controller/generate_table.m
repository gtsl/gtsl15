function table = generate_table(delx,dely)
% Delx, Dely are arguments that determine the coarseness of the table and
% hence accuracy of interpolation
% Generate lookup table for (h, hdot) --> (Servo angle command)

% Max altitude and vertical velocity expected
hmax = 3000; % (ft)
hdotmax = 500; % (ft/s)
% Allocate table to store all possible h, hdot combinations
table = 90 .* ones(round((hmax + 100)/delx), round((hdotmax + 100)/dely));
% Rocket parameters
Cd_r = .5; % --- changeme ----
A_r = .1; % (m2)

yaah = 0;
ii = size(table, 1);

% i = h
% j = hdot

wb = waitbar(0, 'Populating table...');
% Populate table
for j = dely*(1:size(table, 2))
    % For each vertical velocity, go through altitude options
    skip = 0;
    waitbar(j / (dely * size(table, 2)), wb, 'Populating table...');
    for i = delx*(ii:-1:1)
%         fprintf('i: %.0f, j: %.0f\n', j / dely, i / delx)
        % If the altitude is less than the highest 0 angle point
%         if i > ii
%             % Used to skip up to point of interest
%             table(round(i/delx), round(j/dely)) = 90;
%             continue
%         end
        % If 90 deg has already been reached it wont go back
        if skip == 1
            table(round(i/delx), round(j/dely)) = 0;
            continue
        end
        yaah = yaah + 1;
        area = find_area(i, j);
        angle = find_angle(area);
        table(round(i/delx), round(j/dely)) = angle;
        if angle == 90
            % Retain flag of last velocity index where angle == 0
            ii = i / delx;
        elseif angle == 0
            % Skip rest of velocity, set flag
            skip = 1;
        end
    end
end
delete(wb);
end