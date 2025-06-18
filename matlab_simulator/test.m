function y = test(u)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
persistent buf;
if isempty(buf)
    buf = zeros(10,1);
else
    buf = [u; buf(1:end-1)];
end
y = buf;
end