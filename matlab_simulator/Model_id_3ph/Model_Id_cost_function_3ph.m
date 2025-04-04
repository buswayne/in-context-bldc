function [error] = Model_Id_cost_function_3ph(var, input_list, output_list)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
p(1) = var.p1;
p(2) = var.p2;
p(3) = var.p3;
p(4) = var.p4;
% p(5) = var.p5;
% p(5) = 0;
% p(6) = var.p6;

x_prev = [0,0,0,0,0]; 

y_pred = zeros(size(output_list));

for i = 1:length(input_list)
    [x_new, y] =  bldcBOModel_3ph([],x_prev,input_list(i,:),p, []);
    x_prev = x_new;
    y_pred(i,:) = y;
end

% flag = abs(output_list(:,2)) >= 1;
% error = mse(y_pred(flag,1), output_list(flag,1)) + mse(y_pred(flag,2), output_list(flag,2));
error = mse(y_pred(:,1), output_list(:,1)) + mse(y_pred(:,2), output_list(:,2)) + mse(y_pred(:,3), output_list(:,3));
% error = mse(y_pred(:,2), output_list(:,2));
end