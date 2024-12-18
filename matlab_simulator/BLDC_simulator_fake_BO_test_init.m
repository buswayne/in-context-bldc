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
real_data = real_data(1:200,:);

vq_ref = real_data(:,4);
vd_ref = real_data(:,5);
time = real_data(:,1);
T =time(end);
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

i_r = 8.7155;
i_l = 44.241;
i_f = 0.016994;
i_i= 45.223;
i_bt = 1;
i_bv = 11.425;
i_a = 0.0051174;


mdl = 'BLDC_simulator_BO';


set_parameters_real
% BLDC.StatorPhaseResistance = BLDC.StatorPhaseResistance * i_r;
% BLDC.InductanceLd = BLDC.InductanceLd * i_l;
% BLDC.InductanceLq = BLDC.InductanceLd;
% BLDC.InductanceL0 = BLDC.InductanceLd;
% BLDC.FluxLinkage = BLDC.FluxLinkage * i_f;
% BLDC.Inertia = BLDC.Inertia * i_i;
% BLDC.BreakawayFrictionTorque = BLDC.BreakawayFrictionTorque * i_bt;
% BLDC.CoulombFrictionTorque = BLDC.BreakawayFrictionTorque;
% BLDC.ViscousFrictionCoefficient = BLDC.ViscousFrictionCoefficient * i_bv;

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

toc

figure
ax1 = subplot(4,1,1);
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

