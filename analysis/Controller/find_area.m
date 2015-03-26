function A = find_area(h, hdot)
% Return area so that integrating state [h, hdot] with additional drag
% from area S such that [h = 3000, hdot = 0] is reached
%
% D = Q * A * Cd
% Use standard atmosphere
Cd = 1.7; % raphael needs to make this better
m = 1; % mass in kg

% TODO
% Newton iterate on S starting at A = 0 ST
A = 0
end