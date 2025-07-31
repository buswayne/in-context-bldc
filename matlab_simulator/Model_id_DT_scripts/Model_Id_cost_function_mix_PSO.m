function [error] = Model_Id_cost_function_mix_PSO(p, input_list, output_list)
%BO cost function for model identification. Minimizes speed error
%   
% var
% p(1) = parameters(1);
% p(2) = parameters(2);
% p(3) = parameters(3);
% p(4) = parameters(4);
% p(5) = parameters(5);

x_prev = [0,0,0,0]; 

omega_pred = zeros(size(output_list(:,3)));
y_pred = zeros(size(output_list(:,1:2)));

for i = 1:length(input_list)
    [x_new, y] =  bldcBOModel_dq([],x_prev,input_list(i,:),p, []);
    x_prev = x_new;
    omega_pred(i) = x_prev(3);
    y_pred(i,:) = y;
end
% var(output_list(:,1))
% mse(y_pred(:,1), output_list(:,1))/var(output_list(:,1))
% mse(y_pred(:,2), output_list(:,2))/var(output_list(:,2))*10
% mse(omega_pred/pi*30, output_list(:,3))/var(output_list(:,3))*5


error = mse(y_pred(:,1), output_list(:,1))/var(output_list(:,1))/10 + ...
        mse(y_pred(:,2), output_list(:,2))/var(output_list(:,2))*20 + ...
        mse(omega_pred/pi*30, output_list(:,3))/var(output_list(:,3));

end