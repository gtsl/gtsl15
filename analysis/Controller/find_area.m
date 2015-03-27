function A_ats = find_area(h, hdot)
% Return area so that integrating state [h, hdot] with additional drag
% from area S such that [h = 3000, hdot = 0] is reached
%
% D = Q * A * Cd
% System parameters (Cd, m, ect) all defined in accel() to avoid passing
% variables through

% State being iterated
x_i = [h; hdot];
% Initalize error variable and initial guess of area
error = 10; A_ats = 1; A_ats_max = 1.6536;
% Iteration counter and iteration limit
maxIter = 100; i = 1;
% Holder for last values
A_ats_l = A_ats; x_f_l = x_i;
% Search for area S.T error is minimized
errorVec = zeros(1,maxIter);
while and(abs(error) > 1, i < maxIter)
    
    x_f = integrate(x_i, A_ats);
    % If A_ats < 0 or A_ats > A_ats_max, break
    if and(x_f(1) > 3000, A_ats > A_ats_max)
        A_ats = A_ats_max;
        return
    end
    if and(x_f(1) < 3000, A_ats < 0)
        A_ats = 0;
        return
    end
    % If hdot at 3000 ft > 0, increase A_ats
    % If hdot at 3000 ft < 0, decrease A_ats
    
    % NOT WORKING AT ALL
    
%     if x_f(1) > 3000
%         A_ats = A_ats + 0.05;
%     else
%         A_ats = A_ats - 0.05;
%     end
    A_ats = A_ats + 0.25 * (x_f(1) - 3000);
    A_ats_l = A_ats; x_f_l = x_f;
    
    error = x_f(1) - 3000;
    errorVec(i) = error;
    i = i + 1;
end

plot(errorVec)

end

function x_f = integrate(x, A_ats)

dt = 0.01;

% Loop until hdot <= 0
while x(2) > 0
    xdot = accel(x, A_ats);
    x = x + xdot .* dt;
end

x_f = x;

end

function xdot = accel(x, A_ats)

% Given parameters
Cd_r = 0.42;
A_r = 0.008;
Cd_ats = 1.75; % guess
m = 7.6370 - 0.5760; % mass after burnout
g = 9.81;

ft2m = 0.3048; m2ft = 1 / ft2m;
in2m = 0.0254;

xdot = zeros(2,1);
A_ats = A_ats * in2m ^ 2;

% hdot = hdot
xdot(1) = x(2);
% Sum forces in metric:
% hddot = F / m, F = D_r + D_ats
[~, ~, ~, rho] = atmosisa(x(1) * ft2m);
Q = .5 * rho * xdot(1) ^ 2;
D_r = Q * A_r * Cd_r;
D_ats = Q * A_ats * Cd_ats;
hddot = - (D_r + D_ats) / m - g;
% Back to ft/s2 for return
xdot(2) = hddot * m2ft;

end