function [error] = Model_Id_cost_function(var, input_list, output_list)
%BO cost function for model identification. Minimizes current error
%   
p(1) = var.p1;
p(2) = var.p2;
p(3) = var.p3;
p(4) = var.p4;
p(5) = 0;

x_prev = [0,0,0,0]; 

y_pred = zeros(size(output_list));

for i = 1:length(input_list)
    [x_new, y] =  bldcBOModel_dq([],x_prev,input_list(i,:),p, []);
    x_prev = x_new;
    y_pred(i,:) = y;
end

error = mse(y_pred(:,1), output_list(:,1)) + mse(y_pred(:,2), output_list(:,2));
end