function [h_est v_est] = kalman_filter1

load('h_act');
load('t_act');
load('v_act');
load('a_sim');
u = a_sim;
global P_Kalman
R = [80^2 0; 0 140^2];  % Assume std. deviation of altimeter is 80 ft
                        % Assume std. deviation of speed (finite
                        % difference) is 140 ft/s
Q = [0 0; 0 30^2];  % Assume std. deviation of accelerometer is 30 ft/s^2

for i = (1:length(h_act)-1)
    count = i
    if t_act(i) == 0
        P_Kalman = eye(2);
        x_est(:,i) = [h_act(i); v_act(i)];
    end
    dt = t_act(i+1) - t_act(i); % or 0.05 seconds, whichever is accurate
    A = [1 dt; 0 1];
    B = [0; dt];
    x_minus = A*x_est(:,i) + B*u(i+1);
    P_minus = A*P_Kalman*A' + Q;
    K = P_minus*inv(P_minus + R);
    x_est(:,i+1) = x_minus + K*([h_act(i+1);v_act(i+1)] - x_minus);
    P_Kalman = (eye(2) - K)*P_minus;
end
h_est = x_est(1,:);
v_est = x_est(2,:);
plot(t_act,h_act,t_act,h_est)
figure
plot(t_act,v_act,t_act,v_est)