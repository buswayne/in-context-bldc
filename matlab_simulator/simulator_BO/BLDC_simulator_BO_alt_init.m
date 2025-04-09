clear
clc
close all
tic


I_r = 1;
I_l = 1;
I_f = 1;
I_i = 1;
I_omega = 1;

%%% last = ???

temp_name = strsplit(pwd,'in-context-bldc');
savepath = fullfile(temp_name{1}, "in-context-bldc","data","simulated\CL_speed_matlab\");
now_string = string(datetime('now'),"yyyy-MM-dd_HH-mm-ss");
name = now_string + "_BO_result.mat";
bayes_save_file_name = fullfile(temp_name{1}, "in-context-bldc", "matlab_simulator", name);

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


speed_loop = 0;
current_loop = 0;

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

set_parameters_real

I_r_var = optimizableVariable("i_r",[0.01,100],"Type","real");
I_l_var = optimizableVariable("i_l",[0.01,100],"Type","real");
I_f_var = optimizableVariable("i_f",[0.1,10],"Type","real");
I_i_var = optimizableVariable("i_i",[0.01,100],"Type","real");
I_omega_var = optimizableVariable("i_omega",[1,4],"Type","real");

fun = @(x)cost_function5_alt(x);


% result = bayesopt(fun, [I_r_var I_l_var I_f_var I_i_var I_b_var I_a_var, I_p_var],"Verbose",2, "AcquisitionFunctionName","expected-improvement-plus", NumSeedPoints=200, MaxObjectiveEvaluations=400, ExplorationRatio=0.7);
% result = bayesopt(fun, [I_r_var I_l_var I_f_var I_i_var I_b_var I_a_var],"Verbose",2, "AcquisitionFunctionName","expected-improvement-plus", NumSeedPoints=400, MaxObjectiveEvaluations=1000, ExplorationRatio=0.7);
result = bayesopt(fun, [I_r_var I_l_var I_f_var I_i_var I_omega_var],"Verbose",2, "AcquisitionFunctionName","expected-improvement-plus", NumSeedPoints=200, MaxObjectiveEvaluations=400, ExplorationRatio=0.7);
save(bayes_save_file_name, "result")


i_r = result.XAtMinObjective.i_r;
i_l = result.XAtMinObjective.i_l;
i_f = result.XAtMinObjective.i_f;
i_i = result.XAtMinObjective.i_i;
i_omega = result.XAtMinObjective.i_omega;
% i_bt = 1; %% no
% i_bv = result.XAtMinObjective.i_b;
% i_a = result.XAtMinObjective.i_a; %%
% i_p = 7;


real_data = readmatrix(real_data_path);
real_data = real_data(1:1000,:);

vq_ref = real_data(:,4);
vd_ref = real_data(:,5);
time = real_data(:,1);
T = time(end);
Ts =  time(2) - time(1);

mdl = 'BLDC_simulator_BO_alt';
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


figure
ax1 = subplot(4,1,1);
hold on
grid on
plot(output.output.time, output.output.signals.values(:,3), "DisplayName","Omega ref")
plot(output.output.time, output.output.signals.values(:,2), "DisplayName","Omega")
plot(output.output.time, real_data(:,6), "DisplayName","Omega real")
legend()
ax2 = subplot(4,1,2);
hold on
grid on
plot(output.output.time, output.output.signals.values(:,6), "DisplayName","iq ref")
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