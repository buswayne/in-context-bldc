clear
clc
close all
tic


temp_name = strsplit(pwd,'in-context-bldc');
savepath = fullfile(temp_name{1}, "in-context-bldc","data","simulated\CL_speed_matlab\");
now_string = string(datetime('now'),"yyyy-MM-dd_HH-mm-ss");
name = now_string + "_BO_result.mat";
bayes_save_file_name = fullfile(temp_name{1}, "in-context-bldc", "matlab_simulator","EKF_scripts/", name);

% real_data_path = fullfile(temp_name{1}, "in-context-bldc","data","CL_experiments_double_sensor\train\inertia13_ki-0.0061-kp-11.8427\2025-03-03--14-51-19_exp   1.csv");
real_data_path = "C:\Users\aless\OneDrive - Politecnico di Milano\in-context-bldc-data\simulated_current_with_alfa_beta_new2\Experiment__2025-03-17_09-17-14_i_omega_2_4970.csv";
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


input_list = [V_alpha, V_beta];
output_list = [I_alpha, I_beta];
% input_list = [V_d, V_q];
% output_list = [I_d, I_q];
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

fun = @(var) EKF_tuning_ab_cost_function2(var, input_list, output_list, omega, [Rs,Ls,Kt,J,Ts]);

% these are the order of magnitude of the elements on the diagonal in Q
p1 = optimizableVariable("p1",[-4,4],"Type","real");
p2 = optimizableVariable("p2",[-4,4],"Type","real");
p3 = optimizableVariable("p3",[-4,4],"Type","real");
p4 = optimizableVariable("p4",[-4,4],"Type","real");
p5 = optimizableVariable("p5",[-4,4],"Type","real");

result_kf = bayesopt(fun, [p1,p2,p3,p4,p5],"Verbose",2, ...
    "AcquisitionFunctionName","expected-improvement-plus", ...
    "UseParallel",true, ...
    NumSeedPoints=300, MaxObjectiveEvaluations=800, ExplorationRatio=0.7);

save(bayes_save_file_name, "result_kf")





initial_state = [0,0,0,0]';
R =  [1,0;0,1];

esp = [result_kf.XAtMinObjective.p1, ...
       result_kf.XAtMinObjective.p2, ...
       result_kf.XAtMinObjective.p3, ...
       result_kf.XAtMinObjective.p4];



Q = diag([10^esp(1), ...
          10^esp(2), ...
          10^esp(3), ...
          10^esp(4)]);

esp_P0 = result_kf.XAtMinObjective.p5;
P0 = diag([1,1,1,10^esp_P0]);

EKF = extendedKalmanFilter(@(x,u)bldcEKFModel_F_dq(x,u, Rs,Ls,Kt,J,Ts), ...
                           @(x)bldcEKFModel_H_dq(x, Rs,Ls,Kt,J,Ts), ...
                           initial_state,...
                           "HasAdditiveProcessNoise", true, ...
                           "ProcessNoise", Q,...
                           "HasAdditiveMeasurementNoise", true,...
                           "MeasurementNoise", R, ...
                           "StateCovariance", P0);

y_pred = zeros(size(output_list));
% y_pred_ab = zeros(size(output_list));
omega_pred = zeros(length(output_list(:,1)),1);

% [PredictedState,PredictedStateCovariance] = predict(EKF, [0,0]);


theta_e_last = initial_state(4);

conversion_mat_dq_ab = @(x) [cos(x) -sin(x); sin(x) cos(x)];
conversion_mat_ab_dq = @(x) [cos(x) sin(x); -sin(x) cos(x)];

converted_input_list = zeros(size(input_list));
converted_output_list = zeros(size(output_list));

for i = 1:length(input_list)

    output_now = conversion_mat_ab_dq(theta_e_last) * output_list(i,:)';
    input_now = conversion_mat_ab_dq(theta_e_last) * input_list(i,:)';

    converted_output_list(i,:) = output_now';
    converted_input_list(i,:) = input_now';

    [Residual,ResidualCovariance] = residual(EKF,output_now);
    [CorrectedState,CorrectedStateCovariance] = correct(EKF,output_now);
    [PredictedState,PredictedStateCovariance] = predict(EKF, input_now);

    % [Residual,ResidualCovariance] = residual(EKF,output_list(i,:));
    % [CorrectedState,CorrectedStateCovariance] = correct(EKF,output_list(i,:));
    % [PredictedState,PredictedStateCovariance] = predict(EKF, input_list(i,:));

    y_pred(i,:) = conversion_mat_dq_ab(CorrectedState(4)) * CorrectedState(1:2);
    omega_pred(i,:) = CorrectedState(3);
    theta_e_last = CorrectedState(4);

    % residBuf(i,:) = Residual;
    % xcorBuf(i,:) = CorrectedState';
    % xpredBuf(i,:) = PredictedState';
end


figure
subplot(511)
plot(y_pred(:,1))
hold on
plot(converted_output_list(:,1))
plot(I_d)
legend(["Id_{est}","Id_{converted}","Id_{real}"])


subplot(512)
plot(y_pred(:,2))
hold on
plot(converted_output_list(:,2))
plot(I_q)
legend(["Iq_{est}","Iq_{converted}","Iq_{real}"])

subplot(513)
plot(converted_input_list(:,1))
hold on
plot(V_d)
legend(["Vd_{converted}","Vd_{real}"])

subplot(514)
plot(converted_input_list(:,2))
hold on
plot(V_q)
legend(["Vq_{converted}","Vq_{real}"])

subplot(515)
plot(omega_pred/pi*30)
hold on
plot(omega)
legend(["Omega_{est}", "Omega_{real}"])

rmse(omega_pred/pi*30, omega)

toc
