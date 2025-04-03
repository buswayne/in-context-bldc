clear
clc
close all
tic
temp_name = strsplit(pwd,'in-context-bldc');
% data_path = fullfile(temp_name{1}, "in-context-bldc","data","CL_experiments\train\inertia13_ki-0.0061-kp-11.8427");
% data_path = fullfile(temp_name{1}, "in-context-bldc","data","CL_experiments\test\inertia04_ki-0.0061-kp-11.8427");
data_path = fullfile(temp_name{1}, "in-context-bldc","data","CL_experiments\test\inertia07_ki-0.0061-kp-11.8427");
data_path_sim = "C:\Users\39340\OneDrive - Politecnico di Milano\in-context-bldc-data\simulated2";
folder = "10_percent";

data_path_sim = fullfile(data_path_sim,folder);

list_struct = dir(data_path);
list = {list_struct(3:end).name};

% 
% list_struct_sim = dir(data_path_sim);
% list_sim = {list_struct_sim(3:end).name};

% single file
% exp_num = 1;
iq_mse = zeros(1,length(list));
id_mse = zeros(1,length(list));
vq_mse = zeros(1,length(list));
vd_mse = zeros(1,length(list));

for exp_num = 1:length(list)
    % sim = false;
    % if sim
    %     filename = fullfile(data_path_sim, list_sim{exp_num});
    %     exp = readmatrix(filename);
    %     t = exp(:,1);
    %     iq = exp(:,6);
    %     id = exp(:,5);
    %     vq = exp(:,9);
    %     vd = exp(:,8);
    %     omega = exp(:,3);
    %     ref = exp(:,4);



    % else
        filename = fullfile(data_path, list{exp_num});
        exp = readmatrix(filename);
        t = exp(:,1);
        iq = exp(:,2);
        id = exp(:,3);
        vq = exp(:,4);
        vd = exp(:,5);
        omega = exp(:,6);
        ref = exp(:,7);

    % end
 
    %%%%% obtained "by hand"
    %%% vq 0.1
    %%% vd 0.1
    %%% iq 0.01
    %%% id 0.008

    %%%%% obtained here
    %%% vq 0.7986, 0.4997, 0.3995
    %%% vd 0.2658, 0.2224, 0.2112 
    %%% iq 0.2454, 0.2057, 0.1597 
    %%% id 0.0050, 0.0047, 0.0054 

    %%%%% what I'll use (iq and vq move a lot, maybe the transitories stain
    %%%%% the calculations)
    %%% vq 0.2
    %%% vd 0.2
    %%% iq 0.1
    %%% id 0.005

    
    
    iq_filt = movavg(iq,"linear",10,"shrink");
    id_filt = movavg(id,"linear",10,"shrink");
    vq_filt = movavg(vq,"linear",10,"shrink");
    vd_filt = movavg(vd,"linear",10,"shrink");
    
    iq_noise = iq-iq_filt;
    id_noise = id-id_filt;
    vd_noise = vd-vd_filt;
    vq_noise = vq-vq_filt;

    iq_mse(exp_num) = rms(iq_noise-mean(iq_noise))^2;
    id_mse(exp_num) = rms(id_noise-mean(id_noise))^2;
    vq_mse(exp_num) = rms(vq_noise-mean(vq_noise))^2;
    vd_mse(exp_num) = rms(vd_noise-mean(vd_noise))^2;


end


fprintf("vq noise power: %.4f \n", mean(vq_mse))
fprintf("vd noise power: %.4f \n", mean(vd_mse))
fprintf("iq noise power: %.4f \n", mean(iq_mse))
fprintf("id noise power: %.4f \n", mean(id_mse))


