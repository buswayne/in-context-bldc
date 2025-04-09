clear
clc
close all

%%% starts the BLDC simulator for high speed experiments

tic
temp_name = strsplit(pwd,'in-context-bldc');

perturbation_percent = 50;

perturbation = perturbation_percent / 100;
savepath_tmp = "C:\Users\39340\OneDrive - Politecnico di Milano\in-context-bldc-data\simulated";
folder_name = sprintf('%02.0f_percent_high_speed', perturbation_percent);
savepath = fullfile(savepath_tmp, folder_name);
[tmp, tmp2] = mkdir(savepath);

speed_loop = 1;
current_loop = 1;

T = 4.5;
Ts = 1e-4;
time = 0:Ts:T-Ts;


N_exp = 1;

mdl = 'BLDC_simulator';

for idx_exp = 1:N_exp
    fprintf("> simulating experiment %d out of %d \n", idx_exp, N_exp)
    now_string = string(datetime('now'),"yyyy-MM-dd_HH-mm-ss");
    flag_speed_check = true;

    T = 1;
    Ts = 1e-4;
    time = 0:Ts:T-Ts;


    % some combination of perturbed parameters may lead to motor instancese
    % in which the maximum speed is low. Hence we briefly check
    % how fast can the motor go and we discard configurations that cannot
    % get at least 2000 rpm
    while flag_speed_check
    
        set_parameters_perturbed
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
        output = sim(mdl);
        final_speed = output.output.signals.values(end,2);

        fprintf("   detected final speed: %d RPM \n", final_speed)
        if (final_speed > 2000)
            flag_speed_check = false;
        else
            fprintf("   discarding parameters set \n")
        end
    end
    
    max_speed = min(final_speed, 4000);
    BLDC.RotorVelocityInit = 0;

    
    T = 5;
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
    
    exp_name = "Experiment_" + now_string + "_i_omega_" + str_speed + ".csv";
    writetable(out_tab,fullfile(savepath,exp_name));
 
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

toc