function [error] = Model_Id_cost_function_PSO_speed_input(variables, input_list, output_list)
%BO cost function for model identification. Minimizes current error
%   
p(1) = variables(1);
p(2) = variables(2);
p(3) = variables(3);
p(4) = variables(4);
p(5) = variables(5);

x_prev = [0,0]; 

y_pred = zeros(size(output_list));

for i = 1:length(input_list)
    [x_new, y] =  bldcBOModel_dq_speed_input([],x_prev,input_list(i,:),p, []);
    x_prev = x_new;
    y_pred(i,:) = y;
end

error = mse(y_pred(:,1), output_list(:,1))/var(output_list(:,1)) + ...
        mse(y_pred(:,2), output_list(:,2))/var(output_list(:,2))*10;
end