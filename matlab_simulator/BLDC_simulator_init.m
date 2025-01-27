clear
clc
close all
tic
temp_name = strsplit(pwd,'in-context-bldc');
savepath = fullfile(temp_name{1}, "in-context-bldc","data","simulated\CL_speed_matlab\");
now_string = string(datetime('now'),"yyyy-MM-dd_HH-mm-ss");

set_parameters_perturbed
% set_parameters
T = 10;
Ts = 1e-2;
time = 0:Ts:T-Ts;

Min_value = 0;
Max_value = 2500; %in rpm
Min_duration = 1.5;
Max_duration = 3;

reference_speed = step_sequence(T, Ts, Min_value, Max_value, Min_duration, Max_duration);
reference_speed = reference_speed / 30 * pi; %in rad/s

speed_input.time = time;
speed_input.signals.values = reference_speed;
load_input.time = time;
load_input.signals.values = zeros(length(time),1);

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

%%
figure

subplot(311)
hold on
grid on
plot(out_tab.t, out_tab.v_q)
plot(out_tab.t, out_tab.v_d)

subplot(312)
hold on
grid on
plot(out_tab.t, out_tab.i_q)
plot(out_tab.t, out_tab.i_d)

subplot(313)
hold on
grid on
plot(out_tab.t, out_tab.r)
plot(out_tab.t, out_tab.omega)


% logsout_autotuned = logsout;
% save('AutotunedSpeed','logsout_autotuned')

