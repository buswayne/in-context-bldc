clear
close all
clc
% 
% path = '../../speed_controller/normal_and_disturbed_h10_40k_weights.pkl';
% net = import_transformer_model(path);


load('C:\Users\39340\Documents\GitHub\in-context-bldc\matlab_simulator\networks\normal_and_disturbed_and_perturbed_h10_40kH10H.mat')
exp_len = 100;
time_steps = zeros(exp_len,1);
for i = 1:exp_len
    tic
    input_tmp = dlarray(rand(1,10,8));
    output_tmp = extractdata(predict(net,input_tmp));
    time_steps(i) = toc;
end

figure
% plot(sort(time_steps))
plot(time_steps)

exp_len = 100;
time_steps = zeros(exp_len,1);
for i = 1:exp_len
    tic
    input_tmp = dlarray(rand(1,10,8));
    output_tmp = extractdata(predict(net,input_tmp));
    time_steps(i) = toc;
end

figure
% plot(sort(time_steps))
plot(time_steps)