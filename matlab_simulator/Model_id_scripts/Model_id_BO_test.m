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
% real_data_path = fullfile(temp_name{1}, "in-context-bldc","data","CL_experiments_double_sensor\train\inertia13_ki-0.0061-kp-11.8427\2025-03-03--14-51-19_exp   1.csv");
real_data_path = "C:\Users\aless\OneDrive - Politecnico di Milano\in-context-bldc-data\simulated_current_with_alfa_beta_new\Experiment__2025-03-17_09-17-14_i_omega_2_4970.csv";
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


% input_list = [V_alpha, V_beta];
% output_list = [I_alpha, I_beta];
input_list = [V_d, V_q];
output_list = [I_d, I_q];
results_file = "2025-03-18_20-21-36_BO_result_good_one.mat";
load(fullfile(temp_name{1}, "in-context-bldc", "matlab_simulator","BO_results", results_file));

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
y_pred_ab = zeros(size(output_list));
omega_pred = zeros(length(output_list(:,1)),1);
theta_pred = zeros(length(output_list(:,1)),1);

for i = 1:length(input_list)
    [x_new, y] =  bldcBOModel_dq([],x_prev,input_list(i,:),p, []);
    x_prev = x_new;
    omega_pred(i) = x_prev(3);
    y_pred(i,:) = y;
    theta = x_prev(4);
    theta_pred(i) = x_prev(4);
    y_pred_ab(i,:) = [cos(theta), -sin(theta), 0, 0;
          sin(theta),  cos(theta), 0, 0] *x_prev;
end

figure
subplot(511)
plot(y_pred(:,1))
hold on
plot(output_list(:,1))
legend(["Id_{est}","Id_{real}"])


subplot(512)
plot(y_pred(:,2))
hold on
plot(output_list(:,2))
legend(["Iq_{est}","Iq_{real}"])

subplot(513)
plot(input_list(:,1))
hold on
plot(input_list(:,2))
legend(["Vd","Vq"])

subplot(514)
plot(omega_pred/pi*30)
hold on
plot(omega)
legend(["Omega_{est}", "Omega_{real}"])

subplot(515)
plot(mod(theta_pred+pi, pi*2)-pi)
hold on
plot(theta_e)
legend(["Theta_{est}", "Theta_{real}"])

figure
subplot(211)
plot(y_pred_ab(:,1))
hold on
plot(I_alpha)
legend(["Ia_{est}","Ia_{real}"])

subplot(212)
plot(y_pred_ab(:,2))
hold on
plot(I_beta)
legend(["Ib_{est}","Ib_{real}"])

toc