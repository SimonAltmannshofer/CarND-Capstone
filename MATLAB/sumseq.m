function y = sumseq(x,x0)

[n, m] = size(x);

if n == 1; 
    tr = true;
    x  = x';
    n  = m;
    m  = 1;
else
    tr = false;
end

if nargin < 2; x0 = zeros(1,m); end

y = [x0;x];

for i = 2:(n+1)
    y(i,:) = y(i-1,:) + y(i,:);
end

if tr; y = y'; end