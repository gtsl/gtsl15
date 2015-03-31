function A_ats = find_area(h, hdot)
% Return area so that integrating state [h, hdot] with additional drag
% from area S such that [h = 3000, hdot = 0] is reached
%
% D = Q * A * Cd
% System parameters (Cd, m, ect) all defined in accel() to avoid passing
% variables through

global x
x = [h, hdot]';

A_ats = fsolve(@apogee, 1, optimoptions('fsolve','Display','off'));

end

function h_f = apogee(A_ats)

global x

% ode45

% Include A_ats in state
xvec = [x; A_ats];
warning('off', 'MATLAB:ode45:IntegrationTolNotMet');
options = odeset('Events', @events, 'RelTol', 1e-3, 'AbsTol', 1e-3);
[~, YOUT] = ode45(@accel, [0 10], xvec, options);

% Return errror in apogee
h_f = YOUT(end, 1) - 3000;

end

function xvecdot = accel(~, xvec)

% Given parameters
Cd_r = 0.42;
A_r = 0.008;
Cd_ats = 2; % guess
m = 7.6370 - 0.5760; % mass after burnout
g = 9.81;

ft2m = 0.3048; m2ft = 1 / ft2m;
in2m = 0.0254;

xvecdot = zeros(3,1);
A_ats = xvec(3) * in2m ^ 2;

% hdot = hdot
xvecdot(1) = xvec(2);

% Sum forces in metric:
[~, ~, ~, rho] = atmosisa(xvec(1) * ft2m);
Q = .5 * rho * (xvec(2) * ft2m) ^ 2;

% Drag from rocket and ats
D_r = Q * A_r * Cd_r;
D_ats = Q * A_ats * Cd_ats;

% Newtons 2nd law : hddot = F / m
hddot = - (D_r + D_ats) / m - g;

% Back to ft/s2 for return
xvecdot(2) = hddot * m2ft;
% A_ats is constant
xvecdot(3) = 0;

end

function [value, isterminal, direction] = events(~, W)
% Used by odeset to determine end condition
% End condition is Velocity = 0
% Input: Time, [h, hdot, A_ats]

value = W(2);
isterminal = 1;
direction = -1;
end
