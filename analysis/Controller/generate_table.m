dt = 0.01;
tvec = 0:dt:20;
nSteps = length(tvec);
hmax = 3000;
hdotmax = 500;
table = zeros(hmax + 100, hdotmax + 100)
% Nominal path is size (2, nSteps). Table should include +- 100 for h and
% hdot. Flight software will default to switch controller when outside of 
% range.
% 
%
%

for i = 1:size(table, 1)
    for j = 1:size(table, 2)
        % Use spacing of 1 so values = own index
        % i = h
        % j = hdot
        area = find_area(i, j);
        table(i, j) = find_angle(area);
    end
end