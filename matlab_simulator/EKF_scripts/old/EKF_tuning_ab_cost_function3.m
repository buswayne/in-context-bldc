function error = EKF_tuning_ab_cost_function3(var, input_list, output_list, omega_list, P_BLDC_list)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

Rs = P_BLDC_list(1);
Ls = P_BLDC_list(2);
Kt = P_BLDC_list(3);
J  = P_BLDC_list(4);
Ts = P_BLDC_list(5);

p(1) = var.p1;
p(2) = var.p2;
p(3) = var.p3;


initial_state = [0,0,0,0]';
R =  [1,0;0,1];
Q = diag([10^p(1), ...
          10^p(1), ...
          10^p(2), ...
          10^p(3)]);


P0 = 1e-6*diag([1,1,1,1]);

EKF = extendedKalmanFilter(@(x,u)bldcEKFModel_F_dq(x,u, Rs,Ls,Kt,J,Ts), ...
                           @(x)bldcEKFModel_H_dq(x, Rs,Ls,Kt,J,Ts), ...
                           initial_state,...
                           "HasAdditiveProcessNoise", true, ...
                           "ProcessNoise", Q,...
                           "HasAdditiveMeasurementNoise", true,...
                           "MeasurementNoise", R, ...
                           "StateCovariance", P0);

omega_pred = zeros(length(output_list(:,1)),1);
theta_e_last = initial_state(4);

conversion_mat_dq_ab = @(x) [cos(x) -sin(x); sin(x) cos(x)];
conversion_mat_ab_dq = @(x) [cos(x) sin(x); -sin(x) cos(x)];

converted_input_list = zeros(size(input_list));
converted_output_list = zeros(size(output_list));

for i = 1:length(input_list)-1


    output_now = conversion_mat_ab_dq(theta_e_last) * output_list(i+1,:)';
    input_now = conversion_mat_ab_dq(theta_e_last) * input_list(i,:)';

    converted_output_list(i,:) = output_now';
    converted_input_list(i,:) = input_now';

    [PredictedState,PredictedStateCovariance] = predict(EKF, input_now);

    [Residual,ResidualCovariance] = residual(EKF,output_now);
    [CorrectedState,CorrectedStateCovariance] = correct(EKF,output_now);
    % [Residual,ResidualCovariance] = residual(EKF,output_list(i,:));
    % [CorrectedState,CorrectedStateCovariance] = correct(EKF,output_list(i,:));
    % [PredictedState,PredictedStateCovariance] = predict(EKF, input_list(i,:))

    y_pred(i,:) = conversion_mat_dq_ab(CorrectedState(4)) * CorrectedState(1:2);
    omega_pred(i,:) = CorrectedState(3);
    theta_e_last = CorrectedState(4);

    % residBuf(i,:) = Residual;
    % xcorBuf(i,:) = CorrectedState';
    % xpredBuf(i,:) = PredictedState';
end
omega_pred = omega_pred/pi*30;

error = mse(omega_list, omega_pred);

end