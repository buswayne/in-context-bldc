clear
clc
close all
tic

%%% performes grey-box model identification via pso, by minimizing the motor
%%% speed error
%%% the direct/quadrature model is used, then relevant parameters are
%%% transferred in the alpha/beta one
%%% B was set to 0 as it was considered negligible in the data from the
%%% real motor


inertia_number = "13";

temp_name = strsplit(pwd,'in-context-bldc');
now_string = string(datetime('now'),"yyyy-MM-dd_HH-mm-ss");
name = now_string + "_PSO_result.mat";
PSO_save_file_name = fullfile(temp_name{1}, "in-context-bldc", "matlab_simulator","PSO_results_DT", name);

% real_data_path = fullfile(temp_name{1}, "in-context-bldc","data","CL_experiments_double_sensor_low_speed\final\inertia" + inertia_number + "_ki-0.0029-kp-3.0000\train");
real_data_path = fullfile(temp_name{1}, "in-context-bldc","data","mixed_data\DT\inertia07_ki-0.0030-kp-5.0000");

file_list_tmp = dir(real_data_path);
file_list_tmp = {file_list_tmp(3:end).name};

real_data_path = fullfile(real_data_path, file_list_tmp{1});

data = readtable(real_data_path);

%%%%% try filtering the data first tipo filtfilt o simili
%%%%% separa ld e lq(?)


V_d = data.vd(1:16200);
V_q = data.vq(1:16200);
I_d = data.id(1:16200);
I_q = data.iq(1:16200);
omega = data.omega(1:16200);
theta_e = data.theta_e(1:16200);


% V_d = data.vd;
% V_q = data.vq;
% I_d = data.id;
% I_q = data.iq;
% omega = data.omega;
% theta_e = data.theta_e;



input_list = [V_d, V_q];
output_list = [I_d, I_q, omega];

%Rs/Ls, 1/Ls, lambda_m/Ls, (3/2 * P * lambda_m) / J
% 
% p1 = optimizableVariable("p1",[10,400],"Type","real");
% p2 = optimizableVariable("p2",[10,400],"Type","real");
% p3 = optimizableVariable("p3",[2,50],"Type","real");
% p4 = optimizableVariable("p4",[1,100],"Type","real");
% p5 = optimizableVariable("p5",[0.00001,0.01],"Type","real");



lb(1) = 0.1; ub(1) = 800;
lb(2) = 0.1; ub(2) = 400;
lb(3) = 0.1; ub(3) = 200;
lb(4) = 0.1; ub(4) = 200;
lb(5) = 1e-6; ub(5) = 1;

fun = @(x)Model_Id_cost_function_mix_PSO(x, input_list, output_list);
% fun = @(x)Model_Id_cost_function_omega(x, input_list, omega);
% 
% result = bayesopt(fun, [p1,p2,p3,p4,p5],"Verbose",2, ...
%     "AcquisitionFunctionName","expected-improvement-plus", ...
%     "UseParallel",true, ...
%     NumSeedPoints=300, MaxObjectiveEvaluations=1200, ExplorationRatio=0.5);
% save(bayes_save_file_name, "result")

hybridopts = optimoptions('fmincon', 'Display','iter-detailed');
options = optimoptions('particleswarm', 'UseParallel',true, 'Display', 'iter', 'HybridFcn', {@fmincon,hybridopts});

[result,fval,exitflag,output,points] = particleswarm(fun, 5, lb, ub, options);
% 
% p(1) = result.XAtMinObjective.p1;
% p(2) = result.XAtMinObjective.p2;
% p(3) = result.XAtMinObjective.p3;
% p(4) = result.XAtMinObjective.p4;
save(PSO_save_file_name, "result")
p=result;

% result.XAtMinObjective
result
Rs = p(1)/p(2)
Ls = 1/p(2)
kt = p(3)/p(2)
J = 3/2*7*kt/p(4)
B = J * p(5)

% p(5) = 0.0;


x_prev = [0,0,0,0]; 

y_pred = zeros(size(output_list(:,1:2)));
omega_pred = zeros(size(output_list(:,3)));

for i = 1:length(input_list)
    [x_new, y] =  bldcBOModel_dq([],x_prev,input_list(i,:),p, []);
    x_prev = x_new;
    omega_pred(i)=x_prev(3);
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

mse(y_pred(:,1), output_list(:,1))/var(output_list(:,1))
mse(y_pred(:,2), output_list(:,2))/var(output_list(:,2))
mse(omega_pred/pi*30, output_list(:,3))/var(output_list(:,3))

toc