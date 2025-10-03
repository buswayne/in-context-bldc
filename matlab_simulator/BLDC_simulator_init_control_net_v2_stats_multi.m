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
savepath_tmp = "C:\Users\" + user + "\OneDrive - Politecnico di Milano\in-context-bldc-data\simulated";

perturbation = perturbation_percent / 100;



% model_name = 'noise_h10_40k_H10H.mat';
% model_name = 'noise_h20_40k_H20H.mat';
% model_name = 'noise_h50_40k_H50H.mat';
models_to_test = ["new_dataset_long_noise_h10_40k_H10H.mat", ...
                  "new_dataset_long_noise_scaled_h10_30k_H10H.mat", ...
                  "new_dataset_short_noise_h10_30k_H10H.mat", ...
                  "new_dataset_short_noise_scaled_h10_40k_H10H.mat"];

N_models = length(models_to_test);
H_list = zeros(size(models_to_test));
model_path_list = strings(1,N_models);
save_path_list = strings(1,N_models);

for model_idx = 1:N_models
    model_name = models_to_test(model_idx);
    tmp_H = strsplit(model_name, 'H');
    H_list(model_idx) = str2double(tmp_H{2});

    model_path_list(model_idx) = fullfile(temp_name{1},'in-context-bldc', 'matlab_simulator/networks2', model_name);

    tmp_model_name = char(model_name);
    
    folder_name = sprintf('statistical_analysis_model_%s', tmp_model_name(1:end-4));
    save_path = fullfile(savepath_tmp, folder_name);
    [~, ~] = mkdir(save_path);
    save_path_list(model_idx) = save_path;

end


% model_path = fullfile(temp_name{1},'in-context-bldc', 'matlab_simulator/networks2', model_name);
% [net, H] = import_transformer_model(model_path);


% folder_name = sprintf('statistical_analysis_model_%s', model_name(1:end-4));
% save_path = fullfile(savepath_tmp, folder_name);
% [tmp, tmp2] = mkdir(save_path);


stepsize = 2000;
eps = 0.05 * stepsize;

T_s_th = 1.5;
OS_th = 20;

save_data = true;
show_figures = false;


N_exp = 1000; %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

success_counter = zeros(N_models,1);

mdl_check =  'BLDC_simulator'; 
mdl_test = 'BLDC_simulator_controller_v2';
conversion_mat = @(x) [cos(x) -sin(x); sin(x) cos(x)];

sim_duration_list = [];

for idx_exp = 1:N_exp
    fprintf("> simulating experiment %d out of %d \n", idx_exp, N_exp)
    now_string = string(datetime('now'),"yyyy-MM-dd_HH-mm-ss");


    fprintf("  sanity check... \n")
    
    start_sim = tic;

    T = T_s_th;
    Ts = 1e-4;
    time = 0:Ts:T-Ts;

    flag_sanity_check = true;

    while flag_sanity_check

        set_parameters_perturbed

        speed_loop = 0;
        current_loop = 1;


        speed_input.time = time;
        speed_input.signals.values = zeros(length(time),1);
        load_input.time = time;
        load_input.signals.values = zeros(length(time),1);
        current_input.time = time;
        current_input.signals.values = ones(length(time),1)*5;
        voltage_d_input.time = time;
        voltage_d_input.signals.values = zeros(length(time),1);
        voltage_q_input.time = time;
        voltage_q_input.signals.values = zeros(length(time),1);

        output = sim(mdl_check);
        final_speed = output.output.signals.values(end,2);
        test_speed = output.output.signals.values(:,2);
        test_time = output.output.time;


        if show_figures
            figure
            grid on
            hold on
            plot(test_time, test_speed)
            plot(test_time, stepsize*ones(size(test_speed)))
            xlabel('Time [s]')
            ylabel('\omega [rpm]')
        end

        fprintf("   detected final speed: %d RPM \n", final_speed)

        if final_speed >= stepsize

            flag_sanity_check = false;



            idx_cross_over = find(test_speed>=stepsize,1,"first");
            min_set_time = test_time(idx_cross_over);
            fprintf("   minimum settling time: %g s \n", min_set_time)
            
        else
            fprintf("   discarding parameter set, resampling... \n")
        end

    end



    for model_idx = 1:N_models

        model_path = model_path_list(model_idx);
        model_name = models_to_test(model_idx);
        save_path = save_path_list(model_idx);
        H = H_list(model_idx);
        fprintf("\n  testing model ''%s''\n", model_name)

        speed_loop = 1;
        current_loop = 1;

        T = 5;
        Ts = 1e-4;
        time = 0:Ts:T-Ts;

        speed_input.time = time;
        speed_input.signals.values = ones(length(time),1)*stepsize/30*pi;
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
        test_speed = output.output.signals.values(:,2);
        test_time = output.output.time;
        

    
        T_ass_idx = find(abs(stepsize-test_speed)>=eps, 1, "last");
        if isempty(T_ass_idx)
            fprintf("what\n\n")
            error("what")
        else
            T_ass = test_time(T_ass_idx);
            S_pct = max(test_speed-stepsize)/stepsize*100;
            if T_ass > 4
                fprintf("does not converge\n")
                meta_string = sprintf("T_ass:%.2f,S_pct:%.2f",T_ass, S_pct);
            else
                fprintf("  T_{ass}: %.2f s, S_{%%}: %.2f %%\n",T_ass, S_pct)
                meta_string = sprintf("T_ass:%.2f,S_pct:%.2f",T_ass, S_pct);
    
            end
        end

        t = output.output.time;
        theta = output.output.signals.values(:,1);
        omega = output.output.signals.values(:,2);
        r = output.output.signals.values(:,3);
        id = output.output.signals.values(:,4);
        iq = output.output.signals.values(:,5);
        iq_ref = output.output.signals.values(:,6);
        vd = output.output.signals.values(:,7);
        vq = output.output.signals.values(:,8);

        flag_ts = T_ass <= T_s_th;
        flag_OS = S_pct <= OS_th;
    
        if flag_ts && flag_OS
            success = 1;
            success_counter(model_idx) = success_counter(model_idx) +1;
        else
            success = 0;
        end
    
        
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
            out_tab = table(t,iq,iq_ref,id,vq,vd,ia,ib,va,vb,theta_e,omega,r,zeros(size(r)),'variableNames', ...
                {'t','iq','iq_ref','id','vq','vd','ia','ib','va','vb','theta_e','omega','r', char(meta_string)});
    
            exp_name = now_string + "_data.csv";
            writetable(out_tab,fullfile(save_path,exp_name));
    
            param_names = now_string + "_params.mat";
            save(fullfile(save_path,param_names), "BLDC", "disc", "i_omega");
    
            result_name = now_string + "_results.mat";
            save(fullfile(save_path,result_name), "success", "final_speed","min_set_time", "T_ass", "S_pct");
    
    
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
    sim_duration = toc(start_sim);
    sim_duration_list = [sim_duration_list, sim_duration];
    avg_duration = mean(sim_duration_list);
    s = duration(0,0,avg_duration);
    s_remaining = s *  (N_exp - idx_exp);
    
    str_tmp = string(s_remaining);
    full_line = "estimated remaining time: " + str_tmp + "\n\n";
    fprintf(full_line);

end
fprintf("\n")
for model_idx = 1:N_models
    model_name = models_to_test(model_idx);
    success_ratio = success_counter(model_idx) / N_exp* 100;
    fprintf("Model %s was successful in %.2f%% of the cases\n", model_name, success_ratio)


end

fprintf("\n")
toc