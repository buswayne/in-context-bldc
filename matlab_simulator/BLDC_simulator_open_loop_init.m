clear
clc
% close all
tic
temp_name = strsplit(pwd,'in-context-bldc');
savepath = fullfile(temp_name{1}, "in-context-bldc","data","simulated\CL_speed_matlab\");
now_string = string(datetime('now'),"yyyy-MM-dd_HH-mm-ss");

% real_data_path = fullfile(temp_name{1}, "in-context-bldc","data","CL_experiments\test\inertia04_ki-0.0061-kp-11.8427\2024-10-16--15-16-43_exp   1.csv");
% real_data_path = fullfile(temp_name{1}, "in-context-bldc","data","CL_experiments\test\inertia07_ki-0.0061-kp-11.8427\2024-10-16--16-31-18_exp   6.csv");
real_data_path = fullfile(temp_name{1}, "in-context-bldc","data","CL_experiments\train\inertia13_ki-0.0061-kp-11.8427");
exp_name = "2024-10-16--10-57-42_exp  16.csv";
real_data_path = fullfile(real_data_path, exp_name);

real_data = readmatrix(real_data_path);
real_data = real_data(1:300,:);

vq_ref = real_data(:,4);
vd_ref = real_data(:,5);
time = real_data(:,1);
T =time(end);
Ts =  time(2) - time(1);

real_speed = real_data(:,6);

speed_loop = 0;
current_loop = 0;

% set_parameters_perturbed
set_parameters_real
BLDC.RotorVelocityInit = real_data(1,6);


% 
% Min_value = -BLDC.CurrentMax*0;
% Max_value = BLDC.CurrentMax;
% Min_duration = 0.5;
% Max_duration = 1;



speed_input.time = time;
speed_input.signals.values = zeros(length(time),1);
load_input.time = time;
load_input.signals.values = zeros(length(time),1);
current_input.time = time;
current_input.signals.values = zeros(length(time),1);
% current_input.signals.values = ones(length(time),1)*BLDC.CurrentMax;
voltage_q_input.time = time;
voltage_q_input.signals.values = vq_ref;

voltage_d_input.time = time;
voltage_d_input.signals.values = vd_ref;

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
% 
% exp_name = "Experiment_" + now_string + ".csv";
% writetable(out_tab,fullfile(savepath,exp_name));
toc

figure
ax1 = subplot(4,1,1);
title(exp_name, Interpreter="none")
hold on
grid on
plot(output.time, output.signals.values(:,3), "DisplayName","Omega ref")
plot(output.time, output.signals.values(:,2), "DisplayName","Omega")
plot(output.time, real_data(:,6), "DisplayName","Omega real")
legend()
ax2 = subplot(4,1,2);
hold on
grid on
plot(output.time, output.signals.values(:,6), "DisplayName","iq ref")
plot(output.time, output.signals.values(:,5), "DisplayName","iq")
plot(output.time, real_data(:,2), "DisplayName","iq real")
legend()

ax3 = subplot(4,1,3);
hold on
grid on
plot(output.time, output.signals.values(:,4), "DisplayName","id")
plot(output.time, real_data(:,3), "DisplayName","id real")
legend()

ax4 = subplot(4,1,4);
hold on
grid on
plot(output.time, output.signals.values(:,7), "DisplayName","vd")
plot(output.time, output.signals.values(:,8), "DisplayName","vq")
legend()
linkaxes([ax1, ax2, ax3, ax4], 'x')

% logsout_autotuned = logsout;
% save('AutotunedSpeed','logsout_autotuned')
figure
plot(time, real_data(:,6)./output.signals.values(:,2))
ylim([0,10])
legend('real Omega / Omega')

