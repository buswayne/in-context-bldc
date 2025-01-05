clear
clc
close all
tic
% temp_name = strsplit(pwd,'in-context-bldc');
% savepath = fullfile(temp_name{1}, "in-context-bldc","data","simulated\CL_speed_matlab\");
now_string = string(datetime('now'),"yyyy-MM-dd_HH-mm-ss");

speed_loop = 0;
current_loop = 1;

% set_parameters_perturbed
set_parameters_real
T = 2;
Ts = 1e-4;
time = 0:Ts:T-Ts;

Min_value = -BLDC.CurrentMax;
Max_value = BLDC.CurrentMax;
Min_duration = 0.2;
Max_duration = 0.4;

reference_current = step_sequence(T, Ts, Min_value, Max_value, Min_duration, Max_duration);
% reference_current = ones(length(reference_current),1)*BLDC.CurrentMax;
% reference_current = reference_speed / 30 * pi; %in rad/s
% 
% P_list = [100,10,1,0.1,0.01];
% I_list = [100,10,1];

P_list = [6];
I_list = [150];
for P = P_list
    for I = I_list
        PID_current.p = P;
        PID_current.i = I;
        
        
        
        speed_input.time = time;
        speed_input.signals.values = zeros(length(time),1);
        load_input.time = time;
        load_input.signals.values = zeros(length(time),1);
        current_input.time = time;
        current_input.signals.values = reference_current;
        % current_input.signals.values = ones(length(time),1)*BLDC.CurrentMax;
        voltage_d_input.time = time;
        voltage_d_input.signals.values = zeros(length(time),1);
        voltage_q_input.time = time;
        voltage_q_input.signals.values = zeros(length(time),1);

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
        ax1 = subplot(3,1,1);
        hold on
        grid on
        plot(output.time, output.signals.values(:,3), "DisplayName","Omega ref")
        plot(output.time, output.signals.values(:,2), "DisplayName","Omega")
        legend(["Omega ref", "Omega"])
        tit = "P: " + P + ", I: " + I;
        title(tit)

        ax2 = subplot(3,1,2);
        hold on
        grid on
        plot(output.time, output.signals.values(:,6), "DisplayName","iq ref")
        plot(output.time, output.signals.values(:,5), "DisplayName","iq")
        plot(output.time, output.signals.values(:,4), "DisplayName","id")
        legend()
        
        ax3 = subplot(3,1,3);
        hold on
        grid on
        plot(output.time, output.signals.values(:,7), "DisplayName","vd")
        plot(output.time, output.signals.values(:,8), "DisplayName","vq")
        legend()
        linkaxes([ax1, ax2, ax3], 'x')
    end
end

% logsout_autotuned = logsout;
% save('AutotunedSpeed','logsout_autotuned')

