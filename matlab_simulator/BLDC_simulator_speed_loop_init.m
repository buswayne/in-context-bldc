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
set_parameters_real
T = 20;
Ts = 1e-4;
time = 0:Ts:T-Ts;

Min_value = 0;
Max_value = 1000; %in rpm
Min_duration = 1.5;
Max_duration = 3;

reference_speed = step_sequence(T, Ts, Min_value, Max_value, Min_duration, Max_duration);
reference_speed = reference_speed / 30 * pi; %in rad/s


PID_current.p = 0.01;
PID_current.i = 100;


P_list = [11.84];
I_list = [0.0061];
% P_list = [0.5];
% I_list = [10];

for P = P_list
    for I = I_list
        PID_speed.p = P;
        PID_speed.i = I;
        
        
        
        speed_input.time = time;
        speed_input.signals.values = reference_speed;
        load_input.time = time;
        load_input.signals.values = zeros(length(time),1);
        current_input.time = time;
        current_input.signals.values = zeros(length(time),1);% + BLDC.CurrentMax/4;
        
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
        % writetable(out_tab,fullfile(savepath,exp_name));
        toc
        
        figure
        subplot(3,1,1)
        hold on
        grid on
        plot(output.time, output.signals.values(:,3))
        plot(output.time, output.signals.values(:,2))
        tit = "P: " + P + ", I: " + I;
        title(tit)

        subplot(3,1,2)
        hold on
        grid on
        plot(output.time, output.signals.values(:,6))
        plot(output.time, output.signals.values(:,5))
        plot(output.time, output.signals.values(:,4))
        
        subplot(3,1,3)
        hold on
        grid on
        plot(output.time, output.signals.values(:,7))
        plot(output.time, output.signals.values(:,8))
        legend(["vd", "vq"])
    end
end

% logsout_autotuned = logsout;
% save('AutotunedSpeed','logsout_autotuned')

