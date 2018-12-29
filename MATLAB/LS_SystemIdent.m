clc

v_sg = sgolayfilt(v, 4, 25);
N = length(v_sg);
r = 0.2413;
m = 1736.35;
Phi = [];
Y = [];

v_sg_d = zeros(1,N);
dt = mean(diff(time)); %time(i+1)-time(i);
for i=2:N-1
    v_sg_d(i) = (v_sg(i+1) - v_sg(i-1))/(2*dt);
end

for i=1:N
    if v(i) > 2 && v_d(i) > 0
%         phi = [throttle(i)/r, -1, -v(i)*v(i)];
        phi = [throttle(i)/r, -v(i)];
        Phi = [Phi; phi];
        
        y = m*v_d(i);
%         y = m*v_sg_d(i);
        Y = [Y; y];
    end
end

p = (Phi'*Phi)\Phi'*Y

p(2)/m
p(1)/r/m