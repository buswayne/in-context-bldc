function [error] = Model_Id_cost_function_fine_tuning(var, input_list, theta_list, const_par, init_angle)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
p(1) = const_par(1);
p(2) = const_par(2);
p(3) = const_par(3);
p(4) = const_par(4);
p(5) = var.p5;
% p(6) = var.p6;

x_prev = [0,0,0,init_angle]; 

theta_pred = zeros(size(theta_list));

for i = 1:length(input_list)
    [x_new, y] =  bldcBOModel_dq_fine_tuning([],x_prev,input_list(i,:),p, []);
    x_prev = x_new;
    theta_pred(i) = x_new(4);
end

% flag = abs(output_list(:,2)) >= 1;
% error = mse(y_pred(flag,1), output_list(flag,1)) + mse(y_pred(flag,2), output_list(flag,2));
% error = mse(theta_pred(:,1), output_list(:,1)) + mse(theta_pred(:,2), output_list(:,2));
error = mse(sin(theta_pred), sin(theta_list)) + mse(cos(theta_pred), cos(theta_list));
% error = mse(y_pred(:,2), output_list(:,2));
end