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

i_r = 1.00488164382963;
i_l = 0.934332431272569/2;
i_f = 0.817897644833537/1.85;
i_i = 8.84599293023962/80;
% i_bt = 1; %% no
i_bv = 3.61947730564325;
i_a = 0.222788302473949; %%
% i_p = 7;


var.i_r = i_r;
var.i_l = i_l;
var.i_f = i_f;
var.i_i = i_i;
var.i_b = i_bv;
var.i_a = i_a;
% var.i_p = i_p;

% i_f = i_f*7/i_p; %% correcting the flux for the pole pairs

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

% error1 = cost_function(var)
% error2 = cost_function2(var)
% error3 = cost_function_CL(var)
% error4 = cost_function3(var)

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
title_str = sprintf('i_r %f, i_l %f, i_f %f, i_i %f, i_b %f, i_a %f',[i_r i_l i_f i_i i_bv i_a]);
title(title_str)

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
% figure
% plot(time, real_data(:,6)./output.signals.values(:,2))
% ylim([0,10])
% legend('real Omega / Omega')

load handel.mat
y = y(1:19000);
player = audioplayer(y,Fs);
play(player);