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
savepath_tmp = "C:\Users\" + user + "\OneDrive - Politecnico di Milano\in-context-bldc-data\simulated_OL";

perturbation = perturbation_percent / 100;





% model_path = fullfile(temp_name{1},'in-context-bldc', 'matlab_simulator/networks2', model_name);
% [net, H] = import_transformer_model(model_path);


% folder_name = sprintf('statistical_analysis_model_%s', model_name(1:end-4));
% save_path = fullfile(savepath_tmp, folder_name);
% [tmp, tmp2] = mkdir(save_path);


stepsize = 2000;
eps = 0.05 * stepsize;

T_s_th = 1.5;
OS_th = 20;

save_data = false;
show_figures = false;


N_exp = 100; %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


mdl =  'BLDC_simulator'; 
conversion_mat = @(x) [cos(x) -sin(x); sin(x) cos(x)];


sim_duration_list = [];


T_exp = 1.5;
Ts_exp = 1e-4;
time_exp = 0:Ts_exp:T_exp-Ts_exp;
time_out = 0:0.01:T_exp;

curr_ref = time_exp'*0;
curr_ref(time_exp>=0) = 2;
curr_ref(time_exp>=0.2) = 0;

curr_ref_out = time_out'*0;
curr_ref_out(time_out>=0) = 2;
curr_ref_out(time_out>=0.2) = 0;


omegas = zeros(N_exp, length(time_out));
iqs = zeros(N_exp, length(time_out));


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

        output = sim(mdl);
        final_speed = output.output.signals.values(end,2);
        test_speed = output.output.signals.values(:,2);
        test_time = output.output.time;


        % if show_figures
        %     figure
        %     grid on
        %     hold on
        %     plot(test_time, test_speed)
        %     plot(test_time, stepsize*ones(size(test_speed)))
        %     xlabel('Time [s]')
        %     ylabel('\omega [rpm]')
        % end

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



    T = T_exp;
    Ts = Ts_exp;
    time = time_exp;

    % curr_ref(time>=0.5) = -2;
    % curr_ref(time>=0.75) = 0;

    speed_input.time = time;
    speed_input.signals.values = zeros(length(time),1);
    load_input.time = time;
    load_input.signals.values = zeros(length(time),1);
    current_input.time = time;
    current_input.signals.values = curr_ref;
    voltage_d_input.time = time;
    voltage_d_input.signals.values = zeros(length(time),1);
    voltage_q_input.time = time;
    voltage_q_input.signals.values = zeros(length(time),1);

    output = sim(mdl);
    test_speed = output.output.signals.values(:,2);
    test_time = output.output.time;
        
    t = output.output.time;
    theta = output.output.signals.values(:,1);
    omega = output.output.signals.values(:,2);
    r = output.output.signals.values(:,3);
    id = output.output.signals.values(:,4);
    iq = output.output.signals.values(:,5);
    iq_ref = output.output.signals.values(:,6);
    vd = output.output.signals.values(:,7);
    vq = output.output.signals.values(:,8);

    omegas(idx_exp,:) = omega;
    iqs(idx_exp,:) = iq;

    
        
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


    sim_duration = toc(start_sim);
    sim_duration_list = [sim_duration_list, sim_duration];
    avg_duration = mean(sim_duration_list);
    s = duration(0,0,avg_duration);
    s_remaining = s *  (N_exp - idx_exp);
    
    str_tmp = string(s_remaining);
    full_line = "estimated remaining time: " + str_tmp + "\n\n";
    fprintf(full_line);

end

figure
ax1 = subplot(2,1,1);
hold on
grid on
for idx_exp = 1:N_exp
    plot(time_out, omegas(idx_exp,:))

end
ax2 = subplot(2,1,2);
hold on
grid on
for idx_exp = 1:N_exp
    plot(time_out, iqs(idx_exp,:))
end
plot(time_out, curr_ref_out, ':', 'LineWidth',3, 'Color','k')

group.omegas = omegas;
group.iqs = iqs;
group.iq_ref = curr_ref_out;
group.time = time_out;

save("OL_resp/" + string(datetime, "yyyy-MM-dd--HH-mm-ss") + ".mat", "group")

toc