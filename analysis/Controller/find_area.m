function A_ats = find_area(h, hdot, Cd_r, A_r)
% Return area so that integrating state [h, hdot] with additional drag
% from area S such that [h = 3000, hdot = 0] is reached
%
% D = Q * A * Cd
% Use standard atmosphere
Cd = 1.7; % raphael needs to make this better
m = 1; % mass in kg

% TODO
% Newton iterate on S starting at A = 0 ST

% State being iterated
x_i = [h; hdot];
x_i_0 = x_i;
A_ats = 0;
end

function x_f = integrate(x, Cd_r, A_r, A_ats)

dt = 0.01;

% Loop until the altitude >= 3000 feet
while x(1) < 3000
    xdot = accel(x, Cd_r, A_r, A_ats);
    x = x + xdot .* dt;
end

x_f = x;

end

function xdot = accel(x, Cd_r, A_r, A_ats)

% Given parameters
Cd_ats = 1.7;
m = 1;
ft2m = 0.3048; m2ft = 1 / ft2m;
xdot = zeros(2,1);

% hdot = hdot
xdot(1) = x(2);
% Sum forces in metric:
% hddot = F / m, F = D_r + D_ats
[T, A, P, rho] = atmosisa(x(1) * ft2m);
Q = .5 * rho * xdot(1) ^ 2;
D_r = Q * A_r * Cd_r;
D_ats = Q * A_ats * Cd_ats;
hddot = (D_r + D_ats) / m;
% Back to ft/s2 for return
xdot(2) = hddot * m2ft;

end