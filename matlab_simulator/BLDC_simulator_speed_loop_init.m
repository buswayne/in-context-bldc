clear
clc
close all
tic
temp_name = strsplit(pwd,'in-context-bldc');
savepath = fullfile(temp_name{1}, "in-context-bldc","data","simulated\CL_speed_matlab\");
now_string = string(datetime('now'),"yyyy-MM-dd_HH-mm-ss");

speed_loop = 1;
current_loop = 1;

% set_parameters_perturbed
% set_parameters_real
% set_parameters_corrected
% perturbation = 0.5;
% set_parameters_corrected_perturbed

% BLDC.ViscousFrictionCoefficient = BLDC.ViscousFrictionCoefficient*1e-6;


T = 5;
Ts = 1e-4;
time = 0:Ts:T-Ts;

Min_value = 0;
Max_value = 4000; %in rpm
Min_duration = 1;
Max_duration = 2;

reference_speed = step_sequence(T, Ts, Min_value, Max_value, Min_duration, Max_duration);
reference_speed = reference_speed / 30 * pi; %in rad/s

% 
% PID_current.p = 40;
% PID_current.i = 1;


P_list = [0.1];
I_list = [0.1];
% P_list = [0.5];
% I_list = [10];

for P = P_list
    for I = I_list
        PID_speed.p = P;
        PID_speed.i = I;

        perturbation = 0.5;
        set_parameters_corrected %_perturbed
        % BLDC.RatedSpeed / pi * 30

        
        speed_input.time = time;
        speed_input.signals.values = reference_speed;
        load_input.time = time;
        load_input.signals.values = zeros(length(time),1);
        current_input.time = time;
        current_input.signals.values = zeros(length(time),1);% + BLDC.CurrentMax/4;
        voltage_d_input.time = time;
        voltage_d_input.signals.values = zeros(length(time),1);
        voltage_q_input.time = time;
        voltage_q_input.signals.values = zeros(length(time),1);

        mdl = 'BLDC_simulator2';
        
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
        
        exp_name = "Experiment_" + now_string + ".csv";
        % writetable(out_tab,fullfile(savepath,exp_name));
        toc
        
        figure
        ax1 = subplot(3,1,1);
        hold on
        grid on
        plot(output.output.time, output.output.signals.values(:,3), "DisplayName","Omega ref")
        plot(output.output.time, output.output.signals.values(:,2), "DisplayName","Omega")
        legend(["Omega ref", "Omega"])
        tit = "P: " + P + ", I: " + I;
        title(tit)

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

% logsout_autotuned = logsout;
% save('AutotunedSpeed','logsout_autotuned')

