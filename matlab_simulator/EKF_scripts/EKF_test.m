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
% real_data_path = "C:\Users\aless\OneDrive - Politecnico di Milano\in-context-bldc-data\10_percent_with_alfa_beta_speed_corrected\Experiment__2025-03-01_15-42-36_i_omega_2_6094.csv";
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

% result.XAtMinObjective
Rs = p(1)/p(2);
Ls = 1/p(2);
Kt = p(3)/p(2);
J = 3/2*7*Kt/p(4);
Ts = 0.01;

initial_state = [0,0,0,0]';
R =  [1,0;0,1];

esp = [0.55243    7.746    1.7234    0.26586];
Q = diag([10^esp(1), ...
          10^esp(2), ...
          10^esp(3), ...
          10^esp(4)]);

EKF = extendedKalmanFilter(@(x,u)bldcEKFModel_F_dq(x,u, Rs,Ls,Kt,J,Ts), ...
                           @(x)bldcEKFModel_H_dq(x, Rs,Ls,Kt,J,Ts), ...
                           initial_state,...
                           "HasAdditiveProcessNoise", true, ...
                           "ProcessNoise", Q,...
                           "HasAdditiveMeasurementNoise", true,...
                           "MeasurementNoise", R);

y_pred = zeros(size(output_list));
% y_pred_ab = zeros(size(output_list));
omega_pred = zeros(length(output_list(:,1)),1);

[PredictedState,PredictedStateCovariance] = predict(EKF, [0,0]);

for i = 1:length(input_list)
    [Residual,ResidualCovariance] = residual(EKF,output_list(i,:));
    [CorrectedState,CorrectedStateCovariance] = correct(EKF,output_list(i,:));
    [PredictedState,PredictedStateCovariance] = predict(EKF, input_list(i,:));

    y_pred(i,:) = CorrectedState(1:2);
    omega_pred(i,:) = CorrectedState(3);

    residBuf(i,:) = Residual;
    xcorBuf(i,:) = CorrectedState';
    xpredBuf(i,:) = PredictedState';
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
plot(omega_pred/pi*30)
hold on
plot(omega)
legend(["Omega_{est}", "Omega_{real}"])

rmse(omega_pred/pi*30, omega)