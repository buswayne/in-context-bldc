function value = trapezoidal(angle)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

angle = mod(angle, pi*2);
angle = angle / pi * 180;

if angle <= 30
    value = angle/30;
elseif angle > 30 && angle <= 150
    value = 1;
elseif angle > 150 && angle <= 210
    value = 1-(angle-150)/60;
elseif angle > 210 && angle <= 330
    value = -1;
else
    value = -1 + (angle-330)/30;
end

end