clear
clc
% close all
tic
temp_name = strsplit(pwd,'in-context-bldc');
savepath = fullfile(temp_name{1}, "in-context-bldc","data","simulated\CL_speed_matlab\");
now_string = string(datetime('now'),"yyyy-MM-dd_HH-mm-ss");

% real_data_path = fullfile(temp_name{1}, "in-context-bldc","data","CL_experiments\test\inertia04_ki-0.0061-kp-11.8427\2024-10-16--15-16-43_exp   1.csv");
% real_data_path = fullfile(temp_name{1}, "in-context-bldc","data","CL_experiments\test\inertia07_ki-0.0061-kp-11.8427\2024-10-16--16-31-18_exp   6.csv");
real_data_path = fullfile(temp_name{1}, "in-context-bldc","data","CL_experiments\train\inertia13_ki-0.0061-kp-11.8427\2024-10-16--10-57-42_exp  26.csv");
real_data = readmatrix(real_data_path);
real_data = real_data(1:1000,:);

vq_ref = real_data(:,4);
vd_ref = real_data(:,5);
time = real_data(:,1);
T = time(end);
Ts =  time(2) - time(1);

real_speed = real_data(:,6);

speed_loop = 0;
current_loop = 0;

% set_parameters_perturbed



speed_input.time = time;
speed_input.signals.values = zeros(length(time),1);
load_input.time = time;
load_input.signals.values = zeros(length(time),1);
current_input.time = time;
current_input.signals.values = zeros(length(time),1);

voltage_q_input.time = time;
voltage_q_input.signals.values = vq_ref;

voltage_d_input.time = time;
voltage_d_input.signals.values = vd_ref;

% i_r = 1.02768672700489;
% i_l = 4.60283692637795;
% i_f = 1.42590636378311*1.25;
% i_i = 57.8552740781578;
% i_omega = 2.89004415384155*1.34;

I_r = 0.992378217301769 * 0.5 * 0.8 * [0.9];
I_l = 4.94023123067063 * 0.5 * 0.8 * [0.7];
I_f = 1.47494219024451 * [0.8];
I_i = 29.0435907344057 * [1];
I_omega = 2.497;

mdl = 'BLDC_simulator_BO_alt';

for i_r = I_r
    for i_l = I_l
        for i_f = I_f
            for i_i = I_i
                for i_omega = I_omega


set_parameters_real

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

toc

figure
ax1 = subplot(4,1,1);
hold on
grid on
plot(output.output.time, real_data(:,7), "DisplayName","Omega ref")
plot(output.output.time, output.output.signals.values(:,2), "DisplayName","Omega")
plot(output.output.time, real_data(:,6), "DisplayName","Omega real")
legend()
title_str = sprintf('i_r %f, i_l %f, i_f %f, i_i %f, i_omega ',[i_r i_l i_f i_i i_omega]);
title(title_str)

ax2 = subplot(4,1,2);
hold on
grid on
% plot(output.output.time, output.output.signals.values(:,6), "DisplayName","iq ref")
plot(output.output.time, output.output.signals.values(:,5), "DisplayName","iq")
plot(output.output.time, real_data(:,2), "DisplayName","iq real")
legend()

ax3 = subplot(4,1,3);
hold on
grid on
plot(output.output.time, output.output.signals.values(:,4), "DisplayName","id")
plot(output.output.time, real_data(:,3), "DisplayName","id real")
legend()

ax4 = subplot(4,1,4);
hold on
grid on
plot(output.output.time, output.output.signals.values(:,7), "DisplayName","vd")
plot(output.output.time, output.output.signals.values(:,8), "DisplayName","vq")
legend()
linkaxes([ax1, ax2, ax3, ax4], 'x')
% 
% figure
% % ax1 = subplot(4,1,1);
% hold on
% grid on
% % plot(output.output.time, output.output.signals.values(:,3), "DisplayName","Omega ref")
% plot(output.output.time, output.output.signals.values(:,2), "DisplayName","Omega")
% plot(output.output.time, real_data(:,6), "DisplayName","Omega real")
% legend()
% title_str = sprintf('i_r %f, i_l %f, i_f %f, i_i %f, i_omega ',[i_r i_l i_f i_i i_omega]);
% title(title_str)
% 
% figure
% % ax2 = subplot(4,1,2);
% hold on
% grid on
% % plot(output.output.time, output.output.signals.values(:,6), "DisplayName","iq ref")
% plot(output.output.time, output.output.signals.values(:,5), "DisplayName","iq")
% plot(output.output.time, real_data(:,2), "DisplayName","iq real")
% legend()
% 
% figure
% % ax3 = subplot(4,1,3);
% hold on
% grid on
% plot(output.output.time, output.output.signals.values(:,4), "DisplayName","id")
% plot(output.output.time, real_data(:,3), "DisplayName","id real")
% legend()
% 
% % figure
% % % ax4 = subplot(4,1,4);
% % hold on
% % grid on
% % plot(output.output.time, output.output.signals.values(:,7), "DisplayName","vd")
% % plot(output.output.time, output.output.signals.values(:,8), "DisplayName","vq")
% % legend()
% % % linkaxes([ax1, ax2, ax3, ax4], 'x')

                end 
            end
        end
    end
end



% load handel.mat
% y = y(1:19000);
% player = audioplayer(y,Fs);
% play(player);