clear
clc
% close all
tic


I_r = 1;
I_l = 1;
I_f = 1;
I_i= 1;
I_bv = 1;

%%% last = ???

temp_name = strsplit(pwd,'in-context-bldc');
savepath = fullfile(temp_name{1}, "in-context-bldc","data","simulated\CL_speed_matlab\");
now_string = string(datetime('now'),"yyyy-MM-dd_HH-mm-ss");
name = now_string + "_BO_CL_result.mat";
bayes_save_file_name = fullfile(temp_name{1}, "in-context-bldc", "matlab_simulator", name);

% real_data_path = fullfile(temp_name{1}, "in-context-bldc","data","CL_experiments\test\inertia04_ki-0.0061-kp-11.8427\2024-10-16--15-16-43_exp   1.csv");
% real_data_path = fullfile(temp_name{1}, "in-context-bldc","data","CL_experiments\test\inertia07_ki-0.0061-kp-11.8427\2024-10-16--16-31-18_exp   6.csv");
real_data_path = fullfile(temp_name{1}, "in-context-bldc","data","CL_experiments\train\inertia13_ki-0.0061-kp-11.8427\2024-10-16--10-57-42_exp  26.csv");
real_data = readmatrix(real_data_path);
real_data = real_data(1:200,:);

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
% 
% var=[I_r, I_l, I_f, I_i, I_bv];
% 
% error = cost_function(var)

I_r_var = optimizableVariable("i_r",[0.01,100],"Type","real");
I_l_var = optimizableVariable("i_l",[0.01,100],"Type","real");
I_f_var = optimizableVariable("i_f",[0.01,100],"Type","real");
I_i_var = optimizableVariable("i_i",[0.01,100],"Type","real");
I_b_var = optimizableVariable("i_b",[0.01,100],"Type","real");

fun = @(x)cost_function(x);


result = bayesopt(fun, [I_r_var I_l_var I_f_var I_i_var I_b_var],"Verbose",2, "AcquisitionFunctionName","expected-improvement-plus", NumSeedPoints=100, MaxObjectiveEvaluations=400, ExplorationRatio=0.7);
save(bayes_save_file_name, "result")



% mdl = 'BLDC_simulator_BO';
% 
% simIn = Simulink.SimulationInput(mdl);
% simIn = setVariable(simIn,'i_r', I_r);
% simIn = setVariable(simIn,'i_l', I_l);
% simIn = setVariable(simIn,'i_f', I_f);
% simIn = setVariable(simIn,'i_i', I_i);
% simIn = setVariable(simIn,'i_bv', I_bv);
% simout = sim(simIn)