function error = EKF_tuning_cost_function(var, input_list, output_list, omega_list, P_BLDC_list)
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
p(4) = var.p4;


initial_state = [0,0,0,0]';
R =  [1,0;0,1];
Q = diag([10^p(1), ...
          10^p(2), ...
          10^p(3), ...
          10^p(4)]);

EKF = extendedKalmanFilter(@(x,u)bldcEKFModel_F_dq(x,u, Rs,Ls,Kt,J,Ts), ...
                           @(x)bldcEKFModel_H_dq(x, Rs,Ls,Kt,J,Ts), ...
                           initial_state,...
                           "HasAdditiveProcessNoise", true, ...
                           "ProcessNoise", Q,...
                           "HasAdditiveMeasurementNoise", true,...
                           "MeasurementNoise", R);

omega_pred = zeros(length(output_list(:,1)),1);


for i = 1:length(input_list)
    [Residual,ResidualCovariance] = residual(EKF,output_list(i,:));
    [CorrectedState,CorrectedStateCovariance] = correct(EKF,output_list(i,:));
    [PredictedState,PredictedStateCovariance] = predict(EKF, input_list(i,:));

    omega_pred(i,:) = CorrectedState(3);

    % residBuf(i,:) = Residual;
    % xcorBuf(i,:) = CorrectedState';
    % xpredBuf(i,:) = PredictedState';
end
omega_pred = omega_pred/pi*30;

error = mse(omega_list, omega_pred);

end