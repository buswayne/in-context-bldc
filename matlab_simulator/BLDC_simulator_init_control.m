clear
clc
close all

%%% starts the BLDC simulator and save control parameters

tic
temp_name = strsplit(pwd,'in-context-bldc');

perturbation_percent = 50;

save_data = false;
show_figures = true;
perturbed_reference = false;
current_disturbance = false;
if perturbed_reference && current_disturbance
    error("choose one or fix your code")
end

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

if perturbed_reference
    folder_name = sprintf('%02.0f_percent_control_v2_perturbed', perturbation_percent);
elseif current_disturbance
    folder_name = sprintf('%02.0f_percent_control_v2_disturbed', perturbation_percent);
else
    folder_name = sprintf('%02.0f_percent_control_v2', perturbation_percent);
end
savepath = fullfile(savepath_tmp, folder_name);
savepath_metadata = fullfile(savepath_tmp, folder_name, "metadata");
[~, ~] = mkdir(savepath);
[~, ~] = mkdir(savepath_metadata);

speed_loop = 1;
current_loop = 1;

stepsize = 2000;
eps = 0.05 * stepsize;

backoff_max = 20;
T_set_max = 1.5;
OS_max = 20;

P_min = 0.01;
P_max = 1;
I_min = 0.0001;
I_max = 10;

P_min_exp = log10(P_min);
P_max_exp = log10(P_max);
I_min_exp = log10(I_min);
I_max_exp = log10(I_max);


N_exp = 1;

backoff_log = zeros(N_exp,1);
multi_backoff_counter = 0;

mdl_step = 'BLDC_simulator';
if current_disturbance
    mdl_exp = 'BLDC_simulator_alt';
else
    mdl_exp = 'BLDC_simulator';
end

conversion_mat = @(x) [cos(x) -sin(x); sin(x) cos(x)];
now_string = string(datetime('now'),"yyyy-MM-dd_HH-mm-ss");

for idx_exp = 1:N_exp
    fprintf("> simulating experiment %d out of %d \n", idx_exp, N_exp)


    flag_control_check = true;


    % some combination of perturbed parameters may lead to motor instances
    % in which the maximum speed is low. Hence we briefly check
    % how fast can the motor go and we discard configurations that cannot
    % get at least 2000 rpm
    while flag_control_check
    
        set_parameters_perturbed

        % check max speed

        T = 1;
        Ts = 1e-4;
        time = 0:Ts:T-Ts;


        BLDC.RotorVelocityInit = 2000 /30 *pi /i_omega;

        speed_input.time = time;
        speed_input.signals.values = ones(length(time),1)*1e6;
        load_input.time = time;
        load_input.signals.values = zeros(length(time),1);
        current_input.time = time;
        current_input.signals.values = zeros(length(time),1);
        voltage_d_input.time = time;
        voltage_d_input.signals.values = zeros(length(time),1);
        voltage_q_input.time = time;
        voltage_q_input.signals.values = zeros(length(time),1);
        output = sim(mdl_step);
        final_speed = output.output.signals.values(end,2);

        fprintf("   detected final speed: %d RPM \n", final_speed)

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

        if final_speed >= stepsize
            % speed is good, look for a controller

            BLDC.RotorVelocityInit = 0;
            T = 5;
            Ts = 1e-4;
            time = 0:Ts:T-Ts;

            flag_find_controller = true;
            backoff_counter = 0;

            while flag_find_controller && backoff_counter < backoff_max
   
        
                Kp = 10^( P_min_exp + rand()*(P_max_exp-P_min_exp));
                Ki = 10^( I_min_exp + rand()*(I_max_exp-I_min_exp));
        
                PID_speed.p = Kp;
                PID_speed.i = Ki;
                fprintf("trying coefficients Kp: %.4f,  Ki: %.4f (test# %d)\n", Kp, Ki, backoff_counter+1)
        
        
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
        
                output = sim(mdl_step);
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
        
        
                T_ass_idx = find(abs(stepsize-test_speed)>=eps, 1, "last");
                if isempty(T_ass_idx)
                    fprintf("what\n\n")
                else
                    T_ass = test_time(T_ass_idx);
                    S_pct = max(test_speed-stepsize)/stepsize*100;

                    if T_ass <= T_set_max && S_pct <= OS_max
                        fprintf("T_{ass}: %.2f s, S_{%%}: %.2f %%\n",T_ass, S_pct)
                        meta_string = sprintf("T_ass:%.2f,S_pct:%.2f,kp:%.4f,ki:%.4f",T_ass, S_pct,Kp,Ki);
                        flag_control_check = false;
                        flag_find_controller = false;
                        backoff_log(idx_exp) = backoff_counter;
                        
                    else
                        if T_ass > 4
                            fprintf("does not converge\n")
                        elseif T_ass > T_set_max
                            fprintf("too slow (T_{ass}: %.2f s)\n", T_ass)
                        end
                        if S_pct > OS_max
                            fprintf("too much overshoot\n")
                        end
                        fprintf("\n")
                    end
                end
                backoff_counter = backoff_counter+1;
                if backoff_counter == backoff_max
                    multi_backoff_counter = multi_backoff_counter +1;
                end
            end
        end
        %else -> speed was bad

    end
    
    max_speed = 2500;
    BLDC.RotorVelocityInit = 0;

    
    T = 6.5;
    Ts = 1e-4;
    time = 0:Ts:T-Ts;

    if perturbed_reference

        max_freq = 10;
        min_freq = 0.5;
        max_amp = 200;
        min_amp = 0;
        max_phase = pi;
        min_phase = -pi;

        f_num = 3;

        freq = rand(f_num*2,1)*(max_freq-min_freq) + min_freq;
        amp = rand(f_num*2,1)*(max_amp-min_amp) + min_amp;
        phase = rand(f_num*2,1)*(max_phase-min_phase) + min_phase;

        %%% perturbation 1:
        perturbation_signal_1 = time * 0;
        for i=1:f_num
            perturbation_signal_1 = perturbation_signal_1 + amp(i) * sin(2 * pi * freq(i) * time + phase(i));
        end

        perturbation_signal_2 = time * 0;
        for i=(f_num+1):(f_num*2)
            perturbation_signal_2 = perturbation_signal_2 + amp(i) * sin(2 * pi * freq(i) * time + phase(i));
        end




        reference_speed = time * 0;
        reference_speed(time <= 5.5) = rand() * max_speed + perturbation_signal_2(time <= 5.5);
        reference_speed(time <= 3) = rand() * max_speed + perturbation_signal_1(time <= 3);
        reference_speed(time <= 0.5) = 0;
    else
        reference_speed = time * 0;
        reference_speed(time <= 5.5) = rand() * max_speed;
        reference_speed(time <= 3) = rand() * max_speed;
        reference_speed(time <= 0.5) = 0;
    end

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

    output = sim(mdl_exp);
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
    exp_name = now_string + "_B" + sprintf("%04d", idx_exp) + "_i_omega_" + str_speed + ".csv";

    if save_data
        out_tab = table(t,iq,iq_ref,id,vq,vd,ia,ib,va,vb,theta_e,omega,r,zeros(size(r)),'variableNames', ...
            {'t','iq','iq_ref','id','vq','vd','ia','ib','va','vb','theta_e','omega','r', char(meta_string)});
        writetable(out_tab,fullfile(savepath,exp_name));
        
        param_names = exp_name(1:end-4) + "_params.mat";
        save(fullfile(savepath_metadata,param_names), "BLDC", "disc", "i_omega", "PID_current");
    end

    if show_figures
        figure
        ax1 = subplot(3,1,1);
        hold on
        grid on
        plot(output.output.time, output.output.signals.values(:,3), "DisplayName","Omega ref")
        plot(output.output.time, output.output.signals.values(:,2), "DisplayName","Omega")
        xlabel('Time [s]')
        ylabel('\omega [rpm]')
        legend()
    
    
        ax2 = subplot(3,1,2);
        hold on
        grid on
        plot(output.output.time, output.output.signals.values(:,6), "DisplayName","iq ref")
        plot(output.output.time, output.output.signals.values(:,5), "DisplayName","iq")
        plot(output.output.time, output.output.signals.values(:,4), "DisplayName","id")
        xlabel('Time [s]')
        ylabel('Current [A]')
        legend()
    
        ax3 = subplot(3,1,3);
        hold on
        grid on
        plot(output.output.time, output.output.signals.values(:,7), "DisplayName","vd")
        plot(output.output.time, output.output.signals.values(:,8), "DisplayName","vq")
        xlabel('Time [s]')
        ylabel('Voltage [V]')
        legend()
        linkaxes([ax1, ax2, ax3], 'x')
    end
end

toc