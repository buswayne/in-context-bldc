clear
clc
close all

%%% starts the BLDC simulator multiple times and feeds it the reference
%%% speed from an experiment on the real motor. Simulated current and 
%%% voltage signals are compared with the real ones. Different combinations
%%% of Kp and Ki for the speed and current controller can be set.

tic
temp_name = strsplit(pwd,'in-context-bldc');
savepath = fullfile(temp_name{1}, "in-context-bldc","data","simulated\CL_speed_matlab\");
now_string = string(datetime('now'),"yyyy-MM-dd_HH-mm-ss");

speed_loop = 1;
current_loop = 1;

set_parameters



real_data_path = fullfile(temp_name{1}, "in-context-bldc","data","CL_experiments_double_sensor\train\inertia13_ki-0.0061-kp-11.8427");
exp_name = "2025-03-03--14-51-19_exp  10.csv";
real_data_path = fullfile(real_data_path, exp_name);

real_data = readmatrix(real_data_path);
real_data = real_data(1:1000,:);

time = real_data(:,1);
T = time(end);
Ts =  time(2) - time(1);


reference_speed = real_data(:,13) / 30 * pi;

P_list_current = [1];
I_list_current = [200];

P_list_speed = [0.1];
I_list_speed = [0.1];

for P_c = P_list_current
    for I_c = I_list_current
        
        for P_s = P_list_speed
            for I_s = I_list_speed
                PID_speed.p = P_s;
                PID_speed.i = I_s;
                PID_current.p = P_c;
                PID_current.i = I_c;
                
                
                
                speed_input.time = time;
                speed_input.signals.values = reference_speed;
                load_input.time = time;
                load_input.signals.values = zeros(length(time),1);
                current_input.time = time;
                current_input.signals.values = zeros(length(time),1);
                voltage_d_input.time = time;
                voltage_d_input.signals.values = zeros(length(time),1);
                voltage_q_input.time = time;
                voltage_q_input.signals.values = zeros(length(time),1);
        
                mdl = 'BLDC_simulator';
                
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
                toc
                
                figure
                ax1 = subplot(1,1,1);
                hold on
                grid on
                plot(output.output.time, output.output.signals.values(:,3), "DisplayName","Omega ref")
                plot(output.output.time, output.output.signals.values(:,2), "DisplayName","Omega")
                plot(output.output.time, real_data(:,11), "DisplayName","Omega real")
                legend()
                tit = "P_c: " + P_c + ", I_c: " + I_c + ", P_s: " + P_s + ", I_s: " + I_s;
                title(tit)
        
                figure
                ax2 = subplot(1,1,1);
                hold on
                grid on
                plot(output.output.time, output.output.signals.values(:,6), "DisplayName","iq ref")
                plot(output.output.time, output.output.signals.values(:,5), "DisplayName","iq")
                plot(output.output.time, real_data(:,2), "DisplayName","iq real")
                legend()
                tit = "P_c: " + P_c + ", I_c: " + I_c + ", P_s: " + P_s + ", I_s: " + I_s;
                title(tit)
        
                figure
                ax3 = subplot(1,1,1);
                hold on
                grid on
                plot(output.output.time, output.output.signals.values(:,4), "DisplayName","id")
                plot(output.output.time, real_data(:,3), "DisplayName","id real")
                legend()
                tit = "P_c: " + P_c + ", I_c: " + I_c + ", P_s: " + P_s + ", I_s: " + I_s;
                title(tit)
        
                figure
                ax4 = subplot(1,1,1);
                hold on
                grid on
                plot(output.output.time, output.output.signals.values(:,8), "DisplayName","vq")
                plot(output.output.time, real_data(:,4), "DisplayName","vq real")
                legend()
                tit = "P_c: " + P_c + ", I_c: " + I_c + ", P_s: " + P_s + ", I_s: " + I_s;
                title(tit)
        
                figure
                ax5 = subplot(1,1,1);
                hold on
                grid on
                plot(output.output.time, output.output.signals.values(:,7), "DisplayName","vd")
                plot(output.output.time, real_data(:,5), "DisplayName","vd real")
                legend()
                linkaxes([ax1, ax2, ax3, ax4, ax5], 'x')
                tit = "P_c: " + P_c + ", I_c: " + I_c + ", P_s: " + P_s + ", I_s: " + I_s;
                title(tit)
            end
        end
    end
end


