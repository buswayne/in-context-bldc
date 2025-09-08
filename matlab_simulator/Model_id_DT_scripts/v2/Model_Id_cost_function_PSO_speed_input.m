function [error] = Model_Id_cost_function_PSO_speed_input(var, input_list, output_list)
%BO cost function for model identification. Minimizes current error
%   
p(1) = var(1);
p(2) = var(2);
p(3) = var(3);
% p(4) = var(4);
% p(5) = var(5);

x_prev = [0,0]; 

y_pred = zeros(size(output_list));

for i = 1:length(input_list)
    [x_new, y] =  bldcBOModel_dq_speed_input([],x_prev,input_list(i,:),p, []);
    x_prev = x_new;
    y_pred(i,:) = y;
end

error = mse(y_pred(:,1), output_list(:,1)) + mse(y_pred(:,2), output_list(:,2));
end