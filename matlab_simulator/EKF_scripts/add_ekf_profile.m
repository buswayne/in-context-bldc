clear
clc
close all


temp_name = strsplit(pwd,'in-context-bldc');

folder_name = "inertia13_ki-0.0029-kp-3.0000\test";
model_name = "inertia_13.mat";
ekf_name = "inertia_13.mat";



folder_data_path1 = fullfile(temp_name{1}, "in-context-bldc","data","CL_experiments_double_sensor_low_speed\final", folder_name);
folder_data_path2 = fullfile(temp_name{1}, "in-context-bldc","data","CL_experiments_double_sensor_low_speed\final_fixed", folder_name);

folder_save_path1 = fullfile(temp_name{1}, "in-context-bldc","data","CL_experiments_double_sensor_low_speed_ekf\final", folder_name);
folder_save_path2 = fullfile(temp_name{1}, "in-context-bldc","data","CL_experiments_double_sensor_low_speed_ekf\final_fixed", folder_name);

[a,b] = mkdir(folder_save_path1);
[a,b] = mkdir(folder_save_path2);

load(fullfile(temp_name{1}, "in-context-bldc", "matlab_simulator","BO_results_final", model_name));
load(fullfile(temp_name{1}, "in-context-bldc", "matlab_simulator","EKF_BO_results", ekf_name));

file_list1 = dir(folder_data_path1);
file_list1 = {file_list1(3:end).name};
file_list2 = dir(folder_data_path2);
file_list2 = {file_list2(3:end).name};


p(1) = result.XAtMinObjective.p1;
p(2) = result.XAtMinObjective.p2;
p(3) = result.XAtMinObjective.p3;
p(4) = result.XAtMinObjective.p4;

Rs = p(1)/p(2);
Ls = 1/p(2);
Kt = p(3)/p(2);
J = 3/2*7*Kt/p(4);
Ts = 0.01;


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
P0 = diag([10^-6, ...
          10^-6, ...
          10^-6, ...
          10^esp_P0]);
j = 0;

for file = file_list1
    j = j+1;

    file_path = fullfile(folder_data_path1, file);
    data = readtable(file_path);
    


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

    % start_idx = find(omega>0,1);
    % 
    % V_alpha = data.va(start_idx:end);
    % V_beta = data.vb(start_idx:end);
    % I_alpha = data.ia(start_idx:end);
    % I_beta = data.ib(start_idx:end);
    % V_d = data.vd(start_idx:end);
    % V_q = data.vq(start_idx:end);
    % I_d = data.id(start_idx:end);
    % I_q = data.iq(start_idx:end);
    % omega = data.omega(start_idx:end);
    % theta_e = data.theta_e(start_idx:end);

    input_list = [V_alpha, V_beta];
    output_list = [I_alpha, I_beta];

    EKF = extendedKalmanFilter(@(x,u)bldcEKFModel_F_ab(x,u, Rs,Ls,Kt,J,Ts), ...
                               @(x)bldcEKFModel_H_ab(x, Rs,Ls,Kt,J,Ts), ...
                               initial_state,...
                               "HasAdditiveProcessNoise", true, ...
                               "ProcessNoise", Q,...
                               "HasAdditiveMeasurementNoise", true,...
                               "MeasurementNoise", R, ...
                               "StateCovariance", P0);

    omega_pred = zeros(length(output_list(:,1)),1);
    theta_pred = zeros(length(output_list(:,1)),1);
    current_pred = zeros(size(output_list));

    for i = (1:(length(input_list)-1))+1

        output_now =  output_list(i,:)'; 
        input_now =  input_list(i-1,:)';
    
        [PredictedState,PredictedStateCovariance] = predict(EKF, input_now);
        [Residual,ResidualCovariance] = residual(EKF,output_now);
        [CorrectedState,CorrectedStateCovariance] = correct(EKF,output_now);
        current_pred(i,:) = CorrectedState(1:2);
        omega_pred(i) = CorrectedState(3);
        theta_pred(i) = CorrectedState(4);
    end

    % figure
    % subplot(411)
    % plot(omega_pred/pi*30)
    % hold on
    % plot(omega)
    % legend(["Omega_{ekf}", "Omega_{real}"])
    % 
    % 
    % subplot(412)
    % plot(mod(theta_pred+pi, 2*pi)-pi)
    % hold on
    % plot(theta_e)
    % legend(["Theta_{ekf}", "Theta_{real}"])
    % 
    % subplot(413)
    % plot(current_pred(:,1))
    % hold on
    % plot(I_alpha)
    % legend(["Ia_{ekf}", "Ia_{real}"])
    % 
    % subplot(414)
    % plot(current_pred(:,2))
    % hold on
    % plot(I_beta)
    % legend(["Ib_{ekf}", "Ib_{real}"])

    figure
    plot(omega_pred/pi*30)
    hold on
    plot(omega)
    legend(["Omega_{ekf}", "Omega_{real}"])

    title(file{1})

    error(j) = rmse(omega_pred/pi*30, omega);


end

error_avg = mean(error)