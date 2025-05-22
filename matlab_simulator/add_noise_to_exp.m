clear
clc
close all
tic
temp_name = strsplit(pwd,'in-context-bldc');
% savepath = fullfile(temp_name{1}, "in-context-bldc","data","simulated\CL_speed_matlab\");
data_path = "C:\Users\39340\OneDrive - Politecnico di Milano\in-context-bldc-data\simulated";
save_path = "C:\Users\39340\OneDrive - Politecnico di Milano\in-context-bldc-data\simulated_with_noise";

P_vq = 0.3^2;
P_vd = 0.45^2;
P_iq = 0.06^2;
P_id = 0.09^2;



folder_list = dir(data_path);
folder_list = {folder_list(3:end).name};
for folder = folder_list
    data_filepath = fullfile(data_path, folder);

    save_filepath = fullfile(save_path, folder + "_with_noise");
    [~, ~] = mkdir(save_filepath);

    file_list = dir(data_filepath);
    file_list = {file_list(3:end).name};

    counter = 0;

    for file = file_list
        counter = counter + 1;
        fprintf("from folder %s, converting file # %d, called ''%s'' \n", folder{1}, counter, file{1})

        exp = readmatrix(fullfile(data_filepath, file));
        t = exp(:,1);
        theta = exp(:,2);
        omega = exp(:,3);
        r = exp(:,4);
        i_d = exp(:,5);
        i_q = exp(:,6);
        i_q_ref = exp(:,7);
        v_d = exp(:,8);
        v_q = exp(:,9);

        iq_noise = sqrt(P_iq)*randn(length(t),1);
        id_noise = sqrt(P_id)*randn(length(t),1);
        vq_noise = sqrt(P_vq)*randn(length(t),1);
        vd_noise = sqrt(P_vd)*randn(length(t),1);

        struct_tmp.t = t;
        struct_tmp.theta = theta;
        struct_tmp.omega = omega;
        struct_tmp.r = r;
        struct_tmp.i_d = i_d + id_noise;
        struct_tmp.i_q = i_q + iq_noise;
        struct_tmp.i_q_ref = i_q_ref;
        struct_tmp.v_d = v_d + vd_noise;
        struct_tmp.v_q = v_q + vq_noise;

        tab = struct2table(struct_tmp);
        
        exp_name = file{1}(1:end-4) + "with_noise.csv";
        writetable(tab,fullfile(save_filepath,exp_name));
    end
end
