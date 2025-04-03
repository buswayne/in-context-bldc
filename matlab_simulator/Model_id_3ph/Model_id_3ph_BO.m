clear
clc
close all
tic


temp_name = strsplit(pwd,'in-context-bldc');
savepath = fullfile(temp_name{1}, "in-context-bldc","data","simulated\CL_speed_matlab\");
now_string = string(datetime('now'),"yyyy-MM-dd_HH-mm-ss");
name = now_string + "_BO_result.mat";
bayes_save_file_name = fullfile(temp_name{1}, "in-context-bldc", "matlab_simulator","BO_results", name);
name_tmp = now_string + "_BO_result_tmp.mat";
bayes_save_file_name_tmp = fullfile(temp_name{1}, "in-context-bldc", "matlab_simulator","BO_results", name_tmp);
real_data_path = fullfile(temp_name{1}, "in-context-bldc","data","CL_experiments_double_sensor\train\inertia13_ki-0.0061-kp-11.8427\2025-03-03--14-51-19_exp   1.csv");
% real_data_path = "C:\Users\aless\OneDrive - Politecnico di Milano\in-context-bldc-data\simulated_current_with_alfa_beta_new\Experiment__2025-03-17_09-17-14_i_omega_2_4970.csv";
data = readtable(real_data_path);

% Extract relevant columns
V_alpha = data.va;
V_beta = data.vb;
I_alpha = data.ia;
I_beta = data.ib;
V_d = data.vd;
V_q = data.vq;
I_d = data.id;
I_q = data.iq;
omega = data.omega;
theta_e = data.theta_e;
% 
% 
% inv_clarke_mat = [1, 0;
%                  -1/2, sqrt(3)/2;
%                  -1/2, -sqrt(3)/2];

dq2abc = @(theta) sqrt(2/3)*[cos(theta), -sin(theta);
                             cos(theta - 2/3*pi), -sin(theta - 2/3*pi);
                             cos(theta) + 2/3*pi, -sin(theta + 2/3*pi)];


V_d = V_d * 0;
V_q = V_q * 0+10;

v_dq = [V_d';V_q'];
i_dq = [I_d';I_q'];

v_abc = zeros(3, length(V_d));
i_abc = zeros(3, length(V_d));


for i = 1:length(V_d)

    i_abc(:,i) = dq2abc(theta_e(i)) * i_dq(:,i);
    v_abc(:,i) = dq2abc(theta_e(i)) * v_dq(:,i);

end

% 
% v_ab = [V_alpha, V_beta];
% i_ab = [I_alpha, I_beta];
% 
% V_abc = inv_clarke_mat * v_ab';
% I_abc = inv_clarke_mat * i_ab';


% input_list = [V_alpha, V_beta];
% output_list = [I_alpha, I_beta];
input_list = v_abc';
output_list = i_abc';

%Rs/Ls, 1/Ls, lambda_m/Ls, (3/2 * P * lambda_m) / J, B/J, theta0
% 250, 700, 10, 200, 0.00001, ---

p1 = optimizableVariable("p1",[150,700],"Type","real");
p2 = optimizableVariable("p2",[150,550],"Type","real");
p3 = optimizableVariable("p3",[8,50],"Type","real");
p4 = optimizableVariable("p4",[400,800],"Type","real");
% p5 = optimizableVariable("p5",[0.000001,0.00001],"Type","real");
% p6 = optimizableVariable("p6",[-pi,+pi],"Type","real");

fun = @(x)Model_Id_cost_function_3ph(x, input_list, output_list);


% result = bayesopt(fun, [p1,p2,p3,p4,p5,p6],"Verbose",2, ...
% result = bayesopt(fun, [p1,p2,p3,p4,p5],"Verbose",2, ...
result = bayesopt(fun, [p1,p2,p3,p4],"Verbose",2, ...
    "AcquisitionFunctionName","expected-improvement-plus", ...
    "UseParallel",true, ...
    NumSeedPoints=800, MaxObjectiveEvaluations=1500, ExplorationRatio=0.7);
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


% % 250, 700, 10, 200, 0.001, ---

% p(1) = 250;
% p(2) = 700;
% p(3) = 10;
% p(4) = 200;
p(5) = 0.0;
% p(6) = 0;


x_prev = [0,0,0,0]; 

y_pred = zeros(size(output_list));

for i = 1:length(input_list)
    [x_new, y] =  bldcBOModel_dq([],x_prev,input_list(i,:),p, []);
    x_prev = x_new;
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
legend(["Omega"])

toc