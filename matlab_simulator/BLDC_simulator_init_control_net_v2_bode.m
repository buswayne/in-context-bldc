clear
clc
close all

%%% starts the BLDC simulator and save control parameters

tic
temp_name = strsplit(pwd,'in-context-bldc');

perturbation_percent = 50;

user_tmp = strsplit(pwd,'Users\');
user_tmp2 = strsplit(user_tmp{2},'\');
user = user_tmp2{1};
if user == 'aless'
    usr_str = "__";
else
    usr_str = "_";
end
% savepath_tmp = "C:\Users\" + user + "\OneDrive - Politecnico di Milano\in-context-bldc-data\simulated";

perturbation = perturbation_percent / 100;



% model_name = 'noise_h10_40k_H10H.mat';
% model_name = 'noise_h20_40k_H20H.mat';
model_name = 'new_dataset_short_noise_h10_30k_H10H.mat';
% models_to_test = ["new_dataset_long_noise_h10_40k_H10H.mat", ...
%                   "new_dataset_long_noise_scaled_h10_30k_H10H.mat", ...
%                   "new_dataset_short_noise_h10_30k_H10H.mat", ...
%                   "new_dataset_short_noise_scaled_h10_40k_H10H.mat"];
set_parameters

nominal_inertia = disc.Inertia;

inertia_mult_list = [0.5, 1, 2, 5];
% inertia_list = nominal_inertia * [1];

setpoint_list = [500, 1000, 1500, 2000];
% setpoint_list = [1000];

chirpsize = 100;
delay = 5;
f_init = 0.05;
f_end = 100;

T = 100;
Ts = 1e-4;
time = 0:Ts:T-Ts;
speed_loop = 1;
current_loop = 1;

tmp_H = strsplit(model_name, 'H');
H = str2double(tmp_H{2});

model_path = fullfile(temp_name{1},'in-context-bldc', 'matlab_simulator/networks2', model_name);

tmp_model_name = char(model_name);

folder_name = sprintf('bode_analysis_%s_v3', tmp_model_name(1:end-4));
save_path = fullfile(pwd, folder_name);
[~, ~] = mkdir(save_path);


save_data = true;
show_figures = true;


conversion_mat = @(x) [cos(x) -sin(x); sin(x) cos(x)];

for inertia_mult_curr = inertia_mult_list
    disc.Inertia = inertia_mult_curr * nominal_inertia;
    inertia_string = string(inertia_mult_curr);
    inertia_string = replace(inertia_string, ".","");
    for stepsize = setpoint_list
        stepsize_string = string(stepsize);

        mdl_test = 'BLDC_simulator_controller_v2_ms';
        

        speed_input.time = time;

        time_tmp = time(time<T-delay);
        
        chirp_signal = chirpsize * chirp(time_tmp, f_init, time_tmp(end),f_end);
        chirp_signal = [zeros(1,(length(time)- length(time_tmp))), chirp_signal];
        

        reference = chirp_signal + stepsize;
        % plot(time, reference)

        speed_input.signals.values = reference'/30*pi;
        load_input.time = time;
        load_input.signals.values = zeros(length(time),1);
        current_input.time = time;
        current_input.signals.values = zeros(length(time),1);
        voltage_d_input.time = time;
        voltage_d_input.signals.values = zeros(length(time),1);
        voltage_q_input.time = time;
        voltage_q_input.signals.values = zeros(length(time),1);
        
        load_system(mdl_test)
        set_param(mdl_test+"/Predict",'NetworkFilePath',model_path);
        output = sim(mdl_test);

        t = output.output.time;
        theta = output.output.signals.values(:,1);
        omega = output.output.signals.values(:,2);
        r = output.output.signals.values(:,3);
        id = output.output.signals.values(:,4);
        iq = output.output.signals.values(:,5);
        iq_ref = output.output.signals.values(:,6);
        vd = output.output.signals.values(:,7);
        vq = output.output.signals.values(:,8);
        
        theta_e_grad = theta * 180/pi * BLDC.PolePairs * i_omega;
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
        


        if save_data
            out_tab = table(t,iq,iq_ref,id,vq,vd,ia,ib,va,vb,theta_e,omega,r,'variableNames', ...
                {'t','iq','iq_ref','id','vq','vd','ia','ib','va','vb','theta_e','omega','r'});
    
            exp_code = sprintf("setpoint_%s_in_%s", stepsize_string, inertia_string);
            exp_name = exp_code + "_data.csv";
            writetable(out_tab,fullfile(save_path,exp_name));
    
            param_names = exp_code + "_params.mat";
            save(fullfile(save_path,param_names), "BLDC", "disc", "i_omega");
    
            % result_name = exp_code + "_results.mat";
            % save(fullfile(save_path,result_name), "success", "final_speed","min_set_time", "T_ass", "S_pct");
    
    
        end        




        if show_figures
            figure
            ax1 = subplot(3,1,1);
            hold on
            grid on
            plot(output.output.time, output.output.signals.values(:,3), "DisplayName","Omega ref")
            plot(output.output.time, output.output.signals.values(:,2), "DisplayName","Omega")
            legend(["Omega ref", "Omega"])
    
    
            ax2 = subplot(3,1,2);
            hold on
            grid on
            plot(output.output.time, output.output.signals.values(:,6), "DisplayName","iq ref")
            plot(output.output.time, output.output.signals.values(:,5), "DisplayName","iq")
            plot(output.output.time, output.output.signals.values(:,4), "DisplayName","id")
            legend()
    
            ax3 = subplot(3,1,3);
            hold on
            grid on
            plot(output.output.time, output.output.signals.values(:,7), "DisplayName","vd")
            plot(output.output.time, output.output.signals.values(:,8), "DisplayName","vq")
            legend()
            linkaxes([ax1, ax2, ax3], 'x')
        end
        

    end
    
end



