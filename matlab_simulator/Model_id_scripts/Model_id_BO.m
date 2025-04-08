clear
clc
close all
tic

%%% performes grey-box model identification via BO, by minimizing the motor
%%% speed error
%%% the direct/quadrature model is used, then relevant parameters are
%%% transferred in the alpha/beta one
%%% B was set to 0 as it was considered negligible in the data from the
%%% real motor


inertia_number = "11";

temp_name = strsplit(pwd,'in-context-bldc');
now_string = string(datetime('now'),"yyyy-MM-dd_HH-mm-ss");
name = now_string + "_BO_result.mat";
bayes_save_file_name = fullfile(temp_name{1}, "in-context-bldc", "matlab_simulator","BO_results_final", name);

real_data_path = fullfile(temp_name{1}, "in-context-bldc","data","CL_experiments_double_sensor_low_speed\final\inertia" + inertia_number + "_ki-0.0029-kp-3.0000\train");

file_list_tmp = dir(real_data_path);
file_list_tmp = {file_list_tmp(3:end).name};

real_data_path = fullfile(real_data_path, file_list_tmp{1});

data = readtable(real_data_path);

V_d = data.vd;
V_q = data.vq;
I_d = data.id;
I_q = data.iq;
omega = data.omega;
theta_e = data.theta_e;


input_list = [V_d, V_q];
output_list = [I_d, I_q];

%Rs/Ls, 1/Ls, lambda_m/Ls, (3/2 * P * lambda_m) / J

p1 = optimizableVariable("p1",[100,400],"Type","real");
p2 = optimizableVariable("p2",[10,200],"Type","real");
p3 = optimizableVariable("p3",[2,50],"Type","real");
p4 = optimizableVariable("p4",[40,1000],"Type","real");

% fun = @(x)Model_Id_cost_function(x, input_list, output_list);
fun = @(x)Model_Id_cost_function_omega(x, input_list, omega);

result = bayesopt(fun, [p1,p2,p3,p4],"Verbose",2, ...
    "AcquisitionFunctionName","expected-improvement-plus", ...
    "UseParallel",true, ...
    NumSeedPoints=100, MaxObjectiveEvaluations=300, ExplorationRatio=0.5);
save(bayes_save_file_name, "result")

p(1) = result.XAtMinObjective.p1;
p(2) = result.XAtMinObjective.p2;
p(3) = result.XAtMinObjective.p3;
p(4) = result.XAtMinObjective.p4;

result.XAtMinObjective
Rs = p(1)/p(2)
Ls = 1/p(2)
kt = p(3)/p(2)
J = 3/2*7*kt/p(4)

p(5) = 0.0;


x_prev = [0,0,0,0]; 

y_pred = zeros(size(output_list));
omega_pred = zeros(size(output_list(:,1)));

for i = 1:length(input_list)
    [x_new, y] =  bldcBOModel_dq([],x_prev,input_list(i,:),p, []);
    x_prev = x_new;
    omega_pred(i)=x_new(3);
    y_pred(i,:) = y;
end

figure
subplot(221)
plot(y_pred(:,1))
hold on
plot(output_list(:,1))
legend(["Id_{est}","Id_{real}"])


subplot(222)
plot(y_pred(:,2))
hold on
plot(output_list(:,2))
legend(["Iq_{est}","Iq_{real}"])

subplot(223)
plot(input_list(:,1))
hold on
plot(input_list(:,2))
legend(["Vd","Vq"])

subplot(224)
plot(omega_pred/pi*30)
hold on 
plot(omega)
legend(["Omega_{est}","Omega_{real}"])

toc