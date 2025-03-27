clear
clc
close all
tic
temp_name = strsplit(pwd,'in-context-bldc');
% savepath = fullfile(temp_name{1}, "in-context-bldc","data","simulated\CL_speed_matlab\");
data_path = "C:\Users\39340\OneDrive - Politecnico di Milano\in-context-bldc-data\simulated3";
save_path = "C:\Users\39340\OneDrive - Politecnico di Milano\in-context-bldc-data\simulated_with_alfa_beta_new";
% data_path = "C:\Users\aless\OneDrive - Politecnico di Milano\in-context-bldc-data\low_speed_800";
% save_path = "C:\Users\aless\OneDrive - Politecnico di Milano\in-context-bldc-data\low_speed_alpha_beta_800";

folder = "50_percent_low_speed";

data_filepath = fullfile(data_path, folder);
% data_filepath = data_path; 
save_filepath = fullfile(save_path, folder + "_with_alfa_beta_speed_corrected");
% save_filepath = save_path;
[~, ~] = mkdir(save_filepath);

file_list = dir(data_filepath);
file_list = {file_list(3:end).name};
conversion_mat = @(x) [cos(x) -sin(x); sin(x) cos(x)];
idx = 0;
for file = file_list
    idx
    idx = idx+1;

    tmp1 = split(file, "_i_omega_");
    tmp2 = split(tmp1{end}, ".");
    tmp3 = strrep(tmp2{1},"_",".");
    i_omega = str2double(tmp3);

    exp = readmatrix(fullfile(data_filepath, file));
    %%% t,theta,omega,r,i_d,i_q,i_q_ref,v_d,v_q
    t = exp(:,1);
    theta = exp(:,2);
    omega = exp(:,3);
    r = exp(:,4);
    id = exp(:,5);
    iq = exp(:,6);
    i_q_ref = exp(:,7);
    vd = exp(:,8);
    vq = exp(:,9);

    theta_e_grad = theta * 180 / pi * 7 * 1.41 * i_omega;
    % theta_e_grad = theta * 180 / pi * 7 * 1.41;
    % theta_e_grad = theta * 180 / pi * 7;
    theta_e = wrapTo180(theta_e_grad) / 180 * pi;

    
    i_dq = [id,iq]';
    v_dq = [vd,vq]';
    i_ab = zeros(size(i_dq));
    v_ab = zeros(size(v_dq));
    for j = 1:length(theta_e)
        i_ab(:,j) = conversion_mat(theta_e(j)) * i_dq(:,j);
        v_ab(:,j) = conversion_mat(theta_e(j)) * v_dq(:,j);
    end


    ia = i_ab(1,:)';
    ib = i_ab(2,:)';
    va = v_ab(1,:)';
    vb = v_ab(2,:)';

    % 
    % 
    % 
    % struct_tmp.t = t;
    % struct_tmp.theta = theta;
    % struct_tmp.omega = omega;
    % struct_tmp.r = r;
    % struct_tmp.i_d = i_d + id_noise;
    % struct_tmp.i_q = i_q + iq_noise;
    % struct_tmp.i_q_ref = i_q_ref;
    % struct_tmp.v_d = v_d + vd_noise;
    % struct_tmp.v_q = v_q + vq_noise;
    % 
    % tab = struct2table(struct_tmp);


    tab = table(t,iq,id,vq,vd,ia,ib,va,vb,theta_e,omega*1.41,r*1.41,'variableNames', ...
            {'t','iq','id','vq','vd','ia','ib','va','vb','theta_e','omega','r'});
    
    writetable(tab, fullfile(save_filepath, file),"WriteVariableNames",true);
    
    % exp_name = file;
    % writetable(tab,fullfile(save_filepath,exp_name));
end
