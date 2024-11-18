clear
clc
close all
tic
temp_name = strsplit(pwd,'in-context-bldc');
savepath = fullfile(temp_name{1}, "in-context-bldc","data","simulated\CL_speed_matlab\");
now_string = string(datetime('now'),"yyyy-MM-dd_HH-mm-ss");

set_parameters
T = 10;
Ts = 1e-4;
time = 0:Ts:T-Ts;

Min_value = 0;
Max_value = 1000; %in rpm
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


output_clean.Time = output.time;
output_clean.theta = output.signals.values(:,1);
output_clean.omega = output.signals.values(:,2);
output_clean.omega_ref = output.signals.values(:,3);
output_clean.id = output.signals.values(:,4);
output_clean.iq = output.signals.values(:,5);
output_clean.iq_ref = output.signals.values(:,6);
output_clean.Vd = output.signals.values(:,7);
output_clean.Vq = output.signals.values(:,8);

out_tab = struct2table(output_clean);

exp_name = "Experiment_" + now_string + ".csv";
writetable(out_tab,fullfile(savepath,exp_name));
toc


% logsout_autotuned = logsout;
% save('AutotunedSpeed','logsout_autotuned')

