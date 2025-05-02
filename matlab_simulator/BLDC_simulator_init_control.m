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

perturbation = perturbation_percent / 100;
savepath_tmp = "C:\Users\" + user + "\OneDrive - Politecnico di Milano\in-context-bldc-data\simulated";
folder_name = sprintf('%02.0f_percent_control', perturbation_percent);
savepath = fullfile(savepath_tmp, folder_name);
[tmp, tmp2] = mkdir(savepath);

speed_loop = 1;
current_loop = 1;

stepsize = 2000;
eps = 0.05 * stepsize;

save_data = true;
show_figures = false;

P_min = 0.01;
P_max = 1;
I_min = 0.0001;
I_max = 10;

P_min_exp = log10(P_min);
P_max_exp = log10(P_max);
I_min_exp = log10(I_min);
I_max_exp = log10(I_max);


N_exp = 400;

mdl = 'BLDC_simulator';
conversion_mat = @(x) [cos(x) -sin(x); sin(x) cos(x)];

for idx_exp = 1:N_exp
    fprintf("> simulating experiment %d out of %d \n", idx_exp, N_exp)
    now_string = string(datetime('now'),"yyyy-MM-dd_HH-mm-ss");


    flag_control_check = true;
    T = 5;
    Ts = 1e-4;
    time = 0:Ts:T-Ts;


    % some combination of perturbed parameters may lead to motor instances
    % in which the maximum speed is low. Hence we briefly check
    % how fast can the motor go and we discard configurations that cannot
    % get at least 2000 rpm
    while flag_control_check
    
        set_parameters_perturbed
        Kp = 10^( P_min_exp + rand()*(P_max_exp-P_min_exp));
        Ki = 10^( I_min_exp + rand()*(I_max_exp-I_min_exp));

        PID_speed.p = Kp;
        PID_speed.i = Ki;
        fprintf("trying coefficients Kp: %.4f,  Ki: %.4f\n", Kp, Ki)


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

        output = sim(mdl);
        test_speed = output.output.signals.values(:,2);
        test_time = output.output.time;
        if show_figures
            figure
            grid on
            hold on
            plot(test_time, test_speed)
            plot(test_time, stepsize*ones(size(test_speed)))
        end


        T_ass_idx = find(abs(stepsize-test_speed)>=eps, 1, "last");
        if isempty(T_ass_idx)
            fprintf("what\n\n")
        else
            T_ass = test_time(T_ass_idx);
            if T_ass > 4
                fprintf("does not converge\n\n")
            else
                S_pct = max(test_speed-stepsize)/stepsize*100;
                fprintf("T_{ass}: %.2f s, S_{%%}: %.2f %%\n",T_ass, S_pct)
                meta_string = sprintf("T_ass:%.2f,S_pct:%.2f,kp:%.4f,ki:%.4f",T_ass, S_pct,Kp,Ki);
                flag_control_check = false;
            end
        end

    end
    
    max_speed = 2500;
    BLDC.RotorVelocityInit = 0;

    
    T = 5.5;
    Ts = 1e-4;
    time = 0:Ts:T-Ts;
    reference_speed = time * 0;
    reference_speed(time <= 4.5) = rand() * max_speed;
    reference_speed(time <= 2.5) = rand() * max_speed;
    reference_speed(time <= 0.5) = 0;

    reference_speed = reference_speed / 30 * pi; %in rad/s
    

    speed_input.time = time;
    speed_input.signals.values = reference_speed';
    load_input.time = time;
    load_input.signals.values = zeros(length(time),1);
    current_input.time = time;
    current_input.signals.values = zeros(length(time),1);
    voltage_d_input.time = time;
    voltage_d_input.signals.values = zeros(length(time),1);
    voltage_q_input.time = time;
    voltage_q_input.signals.values = zeros(length(time),1);

    output = sim(mdl);
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

    
    % out_tab = struct2table(output_clean);
    % out_tab.(meta_string) = zeros(size(output.output.signals.values(:,1)));

    str_speed = sprintf("%.4f",i_omega);
    str_speed = strrep(str_speed, ".","_");
    exp_name = "Experiment_" + now_string + "_i_omega_" + str_speed + ".csv";

    if save_data
        out_tab = table(t,iq,iq_ref,id,vq,vd,ia,ib,va,vb,theta_e,omega,r,zeros(size(r)),'variableNames', ...
            {'t','iq','iq_ref','id','vq','vd','ia','ib','va','vb','theta_e','omega','r', char(meta_string)});
        writetable(out_tab,fullfile(savepath,exp_name));
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

toc