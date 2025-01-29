lclear
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
set_parameters_corrected
% BLDC.ViscousFrictionCoefficient = BLDC.ViscousFrictionCoefficient*1e-6;

real_data_path = fullfile(temp_name{1}, "in-context-bldc","data","CL_experiments\train\inertia13_ki-0.0061-kp-11.8427");
real_data_path = fullfile(real_data_path, "2024-10-16--10-57-42_exp  26.csv");
% real_data_path = fullfile(real_data_path, "2024-10-16--10-57-42_exp  93.csv");
real_data = readmatrix(real_data_path);
real_data = real_data(1:1000,:);

time = real_data(:,1);
T = time(end);
Ts =  time(2) - time(1);


reference_speed = real_data(:,7) / 30 * pi;
% BLDC.RotorVelocityInit = real_data(1,6)/i_omega;


PID_current.p = 50;
PID_current.i = 1;


P_list = [0.1];
I_list = [0.1];
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
        ax1 = subplot(1,1,1);
        hold on
        grid on
        plot(output.output.time, output.output.signals.values(:,3), "DisplayName","Omega ref")
        plot(output.output.time, output.output.signals.values(:,2), "DisplayName","Omega")
        plot(output.output.time, real_data(:,6), "DisplayName","Omega real")
        legend()
        tit = "P: " + P + ", I: " + I;
        title(tit)

        figure
        ax2 = subplot(1,1,1);
        hold on
        grid on
        plot(output.output.time, output.output.signals.values(:,6), "DisplayName","iq ref")
        plot(output.output.time, output.output.signals.values(:,5), "DisplayName","iq")
        % plot(output.output.time, output.output.signals.values(:,4), "DisplayName","id")
        plot(output.output.time, real_data(:,2), "DisplayName","iq real")
        legend()

        figure
        ax3 = subplot(1,1,1);
        hold on
        grid on
        % plot(output.output.time, output.output.signals.values(:,6), "DisplayName","iq ref")
        % plot(output.output.time, output.output.signals.values(:,5), "DisplayName","iq")
        plot(output.output.time, output.output.signals.values(:,4), "DisplayName","id")
        plot(output.output.time, real_data(:,3), "DisplayName","id real")
        legend()

        figure
        ax4 = subplot(1,1,1);
        hold on
        grid on
        % plot(output.output.time, output.output.signals.values(:,7), "DisplayName","vd")
        plot(output.output.time, output.output.signals.values(:,8), "DisplayName","vq")
        plot(output.output.time, real_data(:,4), "DisplayName","vq real")
        legend()

        figure
        ax5 = subplot(1,1,1);
        hold on
        grid on
        plot(output.output.time, output.output.signals.values(:,7), "DisplayName","vd")
        % plot(output.output.time, output.output.signals.values(:,8), "DisplayName","vq")
        plot(output.output.time, real_data(:,5), "DisplayName","vd real")
        legend()
        linkaxes([ax1, ax2, ax3, ax4, ax5], 'x')
    end
end

% logsout_autotuned = logsout;
% save('AutotunedSpeed','logsout_autotuned')

