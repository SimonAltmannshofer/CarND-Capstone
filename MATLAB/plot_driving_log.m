close all


v_sg = sgolayfilt(v, 4, 25);

N = length(v_sg);
v_sg_d = zeros(1,N);
dt = mean(diff(time)); %time(i+1)-time(i);
for i=2:N-1
    v_sg_d(i) = (v_sg(i+1) - v_sg(i-1))/(2*dt);
end


figure

ax(1) = subplot(4,1,1);
plot(throttle)
ylabel('throttle')

ax(2) = subplot(4,1,2);
plot(steer)
ylabel('steer')

ax(3) = subplot(4,1,3);
plot(v_sg_d)
hold on
plot(v_d)
ylabel('v_d')
grid on
legend('v\_sg\_d', 'v\_d')

ax(4) = subplot(4,1,4);
plot(v)
hold on
plot(v_sg)
plot(v_des)
legend('v', 'v\_sg', 'v\_des')
ylabel('v')

linkaxes(ax, 'x')

