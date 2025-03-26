clear
clc
close all
tic


temp_name = strsplit(pwd,'in-context-bldc');
savepath = fullfile(temp_name{1}, "in-context-bldc","data","simulated\CL_speed_matlab\");
now_string = string(datetime('now'),"yyyy-MM-dd_HH-mm-ss");
name = now_string + "_BO_result.mat";
bayes_save_file_name = fullfile(temp_name{1}, "in-context-bldc", "matlab_simulator","BO_results_final", name);
name_tmp = now_string + "_BO_result_tmp.mat";
bayes_save_file_name_tmp = fullfile(temp_name{1}, "in-context-bldc", "matlab_simulator","BO_results", name_tmp);
real_data_path = fullfile(temp_name{1}, "in-context-bldc","data","CL_experiments_double_sensor_low_speed\final\inertia13_ki-0.0029-kp-3.0000\2025-03-25--17-26-19_exp  10.csv");
% real_data_path = "C:\Users\aless\OneDrive - Politecnico di Milano\in-context-bldc-data\simulated_current_with_alfa_beta_new\Experiment__2025-03-17_09-17-14_i_omega_2_4970.csv";
% real_data_path = "C:\Users\aless\OneDrive - Politecnico di Milano\in-context-bldc-data\low_speed_alpha_beta\Experiment_2025-03-24_09-33-56_i_omega_2_4970.csv";
data = readtable(real_data_path);

% Extract relevant columns
% V_alpha = data.va;
% V_beta = data.vb;
% I_alpha = data.ia;
% I_beta = data.ib;
V_d = data.vd;
V_q = data.vq;
I_d = data.id;
I_q = data.iq;
omega = data.omega;
theta_e = data.theta_e;


% input_list = [V_alpha, V_beta];
% output_list = [I_alpha, I_beta];
input_list = [V_d, V_q];
output_list = [I_d, I_q];

%Rs/Ls, 1/Ls, lambda_m/Ls, (3/2 * P * lambda_m) / J, B/J, theta0
% 250, 700, 10, 200, 0.00001, ---

p1 = optimizableVariable("p1",[100,400],"Type","real");
p2 = optimizableVariable("p2",[10,200],"Type","real");
p3 = optimizableVariable("p3",[2,50],"Type","real");
p4 = optimizableVariable("p4",[50,1000],"Type","real");
% p5 = optimizableVariable("p5",[0.000001,0.001],"Type","real");
% p6 = optimizableVariable("p6",[-pi,+pi],"Type","real");

% fun = @(x)Model_Id_cost_function(x, input_list, output_list);
fun = @(x)Model_Id_cost_function_omega(x, input_list, omega);


% result = bayesopt(fun, [p1,p2,p3,p4,p5,p6],"Verbose",2, ...
% result = bayesopt(fun, [p1,p2,p3,p4,p5],"Verbose",2, ...
result = bayesopt(fun, [p1,p2,p3,p4],"Verbose",2, ...
    "AcquisitionFunctionName","expected-improvement-plus", ...
    "UseParallel",true, ...
    NumSeedPoints=100, MaxObjectiveEvaluations=300, ExplorationRatio=0.5);
save(bayes_save_file_name, "result")

p(1) = result.XAtMinObjective.p1;
p(2) = result.XAtMinObjective.p2;
p(3) = result.XAtMinObjective.p3;
p(4) = result.XAtMinObjective.p4;
% p(5) = result.XAtMinObjective.p5;
% p(6) = result.XAtMinObjective.p6;

result.XAtMinObjective
Rs = p(1)/p(2)
Ls = 1/p(2)
kt = p(3)/p(2)
J = 3/2*7*kt/p(4)
% B = p(5) * J


% % 250, 700, 10, 200, 0.001, ---

% p(1) = 250;
% p(2) = 700;
% p(3) = 10;
% p(4) = 200;
p(5) = 0.0;
% p(6) = 0;


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
plot(omega)
hold on 
plot(omega_pred/pi*30)
legend(["Omega_{est}","Omega_{real}"])

toc