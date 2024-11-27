clear
clc
close all
tic
temp_name = strsplit(pwd,'in-context-bldc');
savepath = fullfile(temp_name{1}, "in-context-bldc","data","simulated\CL_speed_matlab\");


speed_loop = 1;
current_loop = 1;
T = 10;
Ts = 1e-4;
time = 0:Ts:T-Ts;
Min_value = 0;
Max_value = 1500; %in rpm
Min_duration = 1.5;
Max_duration = 3;


N_exp = 150;

for idx_exp = 1:N_exp
    sprintf("simulating experiment %d out of %d", idx_exp, N_exp)

    now_string = string(datetime('now'),"yyyy-MM-dd_HH-mm-ss");
    set_parameters_real_perturbed
    % set_parameters
    
    reference_speed = step_sequence(T, Ts, Min_value, Max_value, Min_duration, Max_duration);
    reference_speed = reference_speed / 30 * pi; %in rad/s
    
    speed_input.time = time;
    speed_input.signals.values = reference_speed;
    load_input.time = time;
    load_input.signals.values = zeros(length(time),1);
    current_input.time = time;
    current_input.signals.values = zeros(length(time),1);
    
    mdl = 'BLDC_simulator';
    
    sim(mdl)
    
    
    output_clean.t = output.time;
    output_clean.theta = output.signals.values(:,1);
    output_clean.omega = output.signals.values(:,2);
    output_clean.r = output.signals.values(:,3);
    output_clean.i_d = output.signals.values(:,4);
    output_clean.i_q = output.signals.values(:,5);
    output_clean.i_q_ref = output.signals.values(:,6);
    output_clean.v_d = output.signals.values(:,7);
    output_clean.v_q = output.signals.values(:,8);
    
    out_tab = struct2table(output_clean);
    
    exp_name = "Experiment_" + now_string + ".csv";
    writetable(out_tab,fullfile(savepath,exp_name));
    
    
    % figure
    % ax1 = subplot(3,1,1);
    % hold on
    % grid on
    % plot(output.time, output.signals.values(:,3), "DisplayName","Omega ref")
    % plot(output.time, output.signals.values(:,2), "DisplayName","Omega")
    % legend(["Omega ref", "Omega"])
    % 
    % 
    % ax2 = subplot(3,1,2);
    % hold on
    % grid on
    % plot(output.time, output.signals.values(:,6), "DisplayName","iq ref")
    % plot(output.time, output.signals.values(:,5), "DisplayName","iq")
    % plot(output.time, output.signals.values(:,4), "DisplayName","id")
    % legend()
    % 
    % ax3 = subplot(3,1,3);
    % hold on
    % grid on
    % plot(output.time, output.signals.values(:,7), "DisplayName","vd")
    % plot(output.time, output.signals.values(:,8), "DisplayName","vq")
    % legend()
    % linkaxes([ax1, ax2, ax3], 'x')
end

toc