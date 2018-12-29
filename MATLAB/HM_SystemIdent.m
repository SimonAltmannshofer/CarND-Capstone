%% load data from csv file
file = 'driving_log_acc_coast_brake.csv';
m = 1080;   % total vehicle mass
r = 0.335;  % wheel radius

data = readtable(file, 'Delimiter', ',');
data = data(50:700,:);

%% extract necessary data
% calculate acceleration
dt = diff(data.time);
dv = diff(data.v_raw);
acc = dv./dt;

% extract necessary data for H
vel = data.v_raw(1:end-1);
brk = data.brake(1:end-1);
thr = data.throttle(1:end-1);

% in our recorded data the brake-torque is ramped up/down
% the recorded velocities suggest otherwise, remove ramps
brk(brk>0) = max(brk);

% integrate states and actors
dist = sumseq(vel.*dt);
thr_int = sumseq(thr.*dt);
brk_int = sumseq(brk.*dt);

%% system identification

% we might want to reduce the data
use = vel >= 0.0;

% data-matrix before integration 
% only used for prediction after identification)
H = [vel, thr, brk];

% integrated data-matrix
H_int = [dist, thr_int, brk_int];

% least squares to find our constants [alpha, beta, gamma]
params = H_int(use, :) \ vel(use);

% make a prediction with our new model-parameters
predict_acc = H*params;
predict_vel = sumseq(predict_acc .* dt);

% show results
alpha = params(1);
beta = params(2);
gamma = params(3);

d = - alpha*m;
p = beta*r*m;
b = -gamma*r*m;

fprintf('alpha = %12.7f --> d = %12.7f\n', alpha, d)
fprintf('beta  = %12.7f --> p = %12.7f\n', beta, p)
fprintf('gamma = %12.7f --> b = %12.7f ... this should be 1.0\n', gamma, b)

%% visualize results
figure; 

subplot(2,1,1); hold on; grid on;
title(['acceleration: ', file], 'Interpreter', 'none')
plot(acc, 'b', 'LineWidth', 3)
plot(predict_acc, 'm', 'LineWidth', 2)
ylabel('m/s^2')
legend('recorded', 'predicted')

subplot(2,1,2); hold on; grid on;
title(['velocity: ', file], 'Interpreter', 'none')
plot(vel, 'b', 'LineWidth', 3)
plot(predict_vel, 'm', 'LineWidth', 2)
ylabel('m/s')
xlabel('sample')
legend('recorded', 'predicted')

%% script output
% alpha =   -0.1054171 --> d =  113.8504152
% beta  =    5.4999052 --> p = 1989.8657130
% gamma =   -0.0003163 --> b =    0.1144455 ... this should be 1.0
