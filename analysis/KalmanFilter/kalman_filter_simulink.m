function [h_est v_est t] = kalman_filter_simulink(sensed_h,sensed_v,sensed_a,current_t,prev_t)

h_act = sensed_h;
t_act = sense_t;
v_act = sensed_v;
a_sim = sensed_a;
u = a_sim;
global P_Kalman h_est1 v_est1
R = [80^2 0; 0 140^2];  % Assume std. deviation of altimeter is 80 ft
                        % Assume std. deviation of speed (finite
                        % difference) is 140 ft/s
Q = [0 0; 0 30^2];  % Assume std. deviation of accelerometer is 30 ft/s^2

if t_act == 0
    P_Kalman = eye(2);
    x_est = [h_act; v_act];
else
    x_est = [h_est; v_est];
end
dt = current_t - prev_t; % or 0.05 seconds, whichever is accurate
A = [1 dt; 0 1];
B = [0; dt];
x_minus = A*x_est + B*u;
P_minus = A*P_Kalman*A' + Q;
K = P_minus*inv(P_minus + R);
x_est = x_minus + K*([h_act;v_act] - x_minus);
P_Kalman = (eye(2) - K)*P_minus;
h_est1 = x_est(1);
v_est1 = x_est(2);
h_est = h_est1;
v_est = v_est1;