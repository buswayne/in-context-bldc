clear
clc
close all

%%% starts the BLDC simulator and feeds it with direct and quadrature
%%% voltage from a real experiment

tic
temp_name = strsplit(pwd,'in-context-bldc');
savepath = fullfile(temp_name{1}, "in-context-bldc","data","simulated\CL_speed_matlab\");
now_string = string(datetime('now'),"yyyy-MM-dd_HH-mm-ss");

real_data_path = fullfile(temp_name{1}, "in-context-bldc","data","CL_experiments_double_sensor\train\inertia13_ki-0.0061-kp-11.8427");
exp_name = "2025-03-03--14-51-19_exp  10.csv";
real_data_path = fullfile(real_data_path, exp_name);

real_data = readmatrix(real_data_path);
real_data = real_data(1:500,:);

vq_ref = real_data(:,4);
vd_ref = real_data(:,5);
time = real_data(:,1);
T =time(end);
Ts =  time(2) - time(1);

real_speed = real_data(:,6);

speed_loop = 0;
current_loop = 0;

set_parameters

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

% exp_name = "Experiment_" + now_string + ".csv";
% writetable(out_tab,fullfile(savepath,exp_name));
toc

figure
ax1 = subplot(4,1,1);
title(exp_name, Interpreter="none")
hold on
grid on
plot(output.output.time, output.output.signals.values(:,2), "DisplayName","Omega sim")
plot(output.output.time, real_data(:,11), "DisplayName","Omega real")
legend()
ax2 = subplot(4,1,2);
hold on
grid on
plot(output.output.time, output.output.signals.values(:,5), "DisplayName","iq sim")
plot(output.output.time, real_data(:,2), "DisplayName","iq real")
legend()

ax3 = subplot(4,1,3);
hold on
grid on
plot(output.output.time, output.output.signals.values(:,4), "DisplayName","id sim ")
plot(output.output.time, real_data(:,3), "DisplayName","id real")
legend()

ax4 = subplot(4,1,4);
hold on
grid on
plot(output.output.time, output.output.signals.values(:,7), "DisplayName","vd")
plot(output.output.time, output.output.signals.values(:,8), "DisplayName","vq")
legend()
linkaxes([ax1, ax2, ax3, ax4], 'x')

