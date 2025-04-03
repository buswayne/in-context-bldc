clear
clc
close all
tic


temp_name = strsplit(pwd,'in-context-bldc');
savepath = fullfile(temp_name{1}, "in-context-bldc","data","simulated\CL_speed_matlab\");
now_string = string(datetime('now'),"yyyy-MM-dd_HH-mm-ss");
name = now_string + "_BO_result.mat";
bayes_save_file_name = fullfile(temp_name{1}, "in-context-bldc", "matlab_simulator","BO_results_ft", name);
name_tmp = now_string + "_BO_result_tmp.mat";
bayes_save_file_name_tmp = fullfile(temp_name{1}, "in-context-bldc", "matlab_simulator","BO_results", name_tmp);
real_data_path = fullfile(temp_name{1}, "in-context-bldc","data","CL_experiments_double_sensor\train\inertia13_ki-0.0061-kp-11.8427\2025-03-03--14-51-19_exp   1.csv");
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

results_file = "2025-03-24_12-59-14_BO_result_good_on_real_13.mat";
load(fullfile(temp_name{1}, "in-context-bldc", "matlab_simulator","BO_results", results_file));


c_p(1) = result.XAtMinObjective.p1;
c_p(2) = result.XAtMinObjective.p2;
c_p(3) = result.XAtMinObjective.p3;
c_p(4) = result.XAtMinObjective.p4;
init_theta_e = theta_e(1);

%Rs/Ls, 1/Ls, lambda_m/Ls, (3/2 * P * lambda_m) / J, B/J, theta0
% 250, 700, 10, 200, 0.00001, ---
% 
% p1 = optimizableVariable("p1",[150,700],"Type","real");
% p2 = optimizableVariable("p2",[50,550],"Type","real");
% p3 = optimizableVariable("p3",[4,50],"Type","real");
% p4 = optimizableVariable("p4",[400,800],"Type","real");
p5 = optimizableVariable("p5",[0.5, 2],"Type","real");
% p6 = optimizableVariable("p6",[-1,+1],"Type","real");

fun = @(x)Model_Id_cost_function_fine_tuning(x, input_list, theta_e, c_p, init_theta_e);


result_ft = bayesopt(fun, [p5],"Verbose",2, ...
    "AcquisitionFunctionName","expected-improvement-plus", ...
    "UseParallel",true, ...
    NumSeedPoints=100, MaxObjectiveEvaluations=600, ExplorationRatio=0.5);
save(bayes_save_file_name, "result")

p(1) = c_p(1);
p(2) = c_p(2);
p(3) = c_p(3);
p(4) = c_p(4);
p(5) = result_ft.XAtMinObjective.p5;
% p(6) = result_ft.XAtMinObjective.p6;

% result.XAtMinObjective
% Rs = p(1)/p(2)
% Ls = 1/p(2)
% kt = p(3)/p(2)
% J = 3/2*7*kt/p(4)


% % 250, 700, 10, 200, 0.001, ---

% p(1) = 250;
% p(2) = 700;
% p(3) = 10;
% p(4) = 200;
% p(5) = 0.0;
% p(6) = 0;


x_prev = [0,0,0,0]; 

y_pred = zeros(size(output_list));
omega_pred = zeros(size(output_list(:,1)));
theta_pred = zeros(size(output_list(:,1)));

for i = 1:length(input_list)
    [x_new, y] =  bldcBOModel_dq_fine_tuning([],x_prev,input_list(i,:),p, []);
    x_prev = x_new;
    omega_pred(i)=x_new(3);
    theta_pred(i)=x_new(4);
    y_pred(i,:) = y;
end

figure
subplot(411)
plot(y_pred(:,1))
hold on
plot(output_list(:,1))
legend(["Id_{est}","Id_{real}"])


subplot(412)
plot(y_pred(:,2))
hold on
plot(output_list(:,2))
legend(["Iq_{est}","Iq_{real}"])

subplot(413)
plot(input_list(:,1))
hold on
plot(input_list(:,2))
legend(["Vd","Vq"])

subplot(414)
plot(omega)
hold on 
plot(omega_pred/pi*30)
legend(["Omega_{est}","Omega_{real}"])

figure
plot(mod(theta_pred+pi,2*pi)-pi)
hold on
plot(theta_e)
legend(["theta_{est}","theta_{real}"])


figure
plot(sin(mod(theta_pred+pi,2*pi)-pi))
hold on
plot(sin(theta_e))
legend(["sin(theta_{est})","sin(theta_{real})"])


toc