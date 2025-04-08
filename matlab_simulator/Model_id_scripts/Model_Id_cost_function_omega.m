function [error] = Model_Id_cost_function_omega(var, input_list, omega_list)
%BO cost function for model identification. Minimizes speed error
%   
p(1) = var.p1;
p(2) = var.p2;
p(3) = var.p3;
p(4) = var.p4;
p(5) = 0;
x_prev = [0,0,0,0]; 

omega_pred = zeros(size(omega_list));

for i = 1:length(input_list)
    [x_new, y] =  bldcBOModel_dq([],x_prev,input_list(i,:),p, []);
    x_prev = x_new;
    omega_pred(i) = x_prev(3);
end

error = mse(omega_pred/pi*30, omega_list);
end