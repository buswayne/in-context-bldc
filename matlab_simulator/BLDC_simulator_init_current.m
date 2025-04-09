clear
clc
close all

%%% starts the BLDC simulator for current loop experiments

tic
temp_name = strsplit(pwd,'in-context-bldc');

perturbation_percent = 50;

perturbation = perturbation_percent / 100;
savepath_tmp = "C:\Users\39340\OneDrive - Politecnico di Milano\in-context-bldc-data\simulated_current";
folder_name = sprintf('%02.0f_percent', perturbation_percent);
savepath = fullfile(savepath_tmp, folder_name);
[tmp, tmp2] = mkdir(savepath);

speed_loop = 0;
current_loop = 1;
T = 3;
Ts = 1e-4;
time = 0:Ts:T-Ts;
set_parameters
Max_abs_value = BLDC.CurrentMax;
Min_duration = 0.1;
Max_duration = 0.3;


N_exp = 1;

mdl = 'BLDC_simulator';

for idx_exp = 1:N_exp
    fprintf("> simulating experiment %d out of %d \n", idx_exp, N_exp)

    now_string = string(datetime('now'),"yyyy-MM-dd_HH-mm-ss");
    set_parameters_perturbed


    reference_current = step_sequence_current(T, Ts,Max_abs_value, Min_duration, Max_duration);
    
    speed_input.time = time;
    speed_input.signals.values = zeros(length(time),1);
    load_input.time = time;
    load_input.signals.values = zeros(length(time),1);
    current_input.time = time;
    current_input.signals.values = reference_current;
    voltage_d_input.time = time;
    voltage_d_input.signals.values = zeros(length(time),1);
    voltage_q_input.time = time;
    voltage_q_input.signals.values = zeros(length(time),1);

    output = sim(mdl);
    
    
    output_clean.t = output.output.time;
    output_clean.theta = output.output.signals.values(:,1);
    output_clean.omega = output.output.signals.values(:,2);
    output_clean.r = output.output.signals.values(:,3);
    output_clean.i_d = output.output.signals.values(:,4);
    output_clean.i_q = output.output.signals.values(:,5);
    output_clean.i_q_ref = output.output.signals.values(:,6);
    output_clean.v_d = output.output.signals.values(:,7);
    output_clean.v_q = output.output.signals.values(:,8);

    out_tab = struct2table(output_clean);
    str_speed = sprintf("%.4f",i_omega);
    str_speed = strrep(str_speed, ".","_");
    
    exp_name = "Experiment__" + now_string + "_i_omega_" + str_speed + ".csv";
    % writetable(out_tab,fullfile(savepath,exp_name));
    
    
    figure
    ax1 = subplot(3,1,1);
    hold on
    grid on
    plot(output.output.time, output_clean.omega, "DisplayName","Omega")
    legend()


    ax2 = subplot(3,1,2);
    hold on
    grid on
    plot(output.output.time, output_clean.i_q_ref, "DisplayName","iq ref")
    plot(output.output.time, output_clean.i_q, "DisplayName","iq")
    plot(output.output.time, output_clean.i_d, "DisplayName","id")
    legend()

    ax3 = subplot(3,1,3);
    hold on
    grid on
    plot(output.output.time, output_clean.v_d, "DisplayName","vd")
    plot(output.output.time, output_clean.v_q, "DisplayName","vq")
    legend()
    linkaxes([ax1, ax2, ax3], 'x')
end

toc